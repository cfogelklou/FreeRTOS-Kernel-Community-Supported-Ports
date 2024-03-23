/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "port_counting_sem.hpp"
#include <chrono>
#include <condition_variable>

#include <iostream>
#include <map>
#include <mutex>
#include <stdint.h>
#include <thread>

#if defined(_MSC_VER)
#include <windows.h>
#define portDELETE_SELF_THREAD_PRIORITY                                        \
  THREAD_PRIORITY_TIME_CRITICAL /* Must be highest. */
#define portSIMULATED_INTERRUPTS_THREAD_PRIORITY THREAD_PRIORITY_TIME_CRITICAL
#define portSIMULATED_TIMER_THREAD_PRIORITY THREAD_PRIORITY_HIGHEST
#define portTASK_THREAD_PRIORITY THREAD_PRIORITY_ABOVE_NORMAL
#elif defined(__APPLE__) || defined(__linux__) || defined(__posix__)
#define PORT_POSIX_THREADS 1
#endif

#if defined(__APPLE__)
#include <mach/mach.h>
#endif

#if (PORT_POSIX_THREADS > 0)
#include <pthread.h>
#include <sched.h>
#define portDELETE_SELF_THREAD_PRIORITY sched_get_priority_max(SCHED_FIFO)
#define portSIMULATED_INTERRUPTS_THREAD_PRIORITY                               \
  sched_get_priority_max(SCHED_FIFO)
#define portSIMULATED_TIMER_THREAD_PRIORITY sched_get_priority_max(SCHED_FIFO)
#define portTASK_THREAD_PRIORITY (sched_get_priority_max(SCHED_FIFO) / 2)
#endif

#define portMAX_INTERRUPTS                                                     \
  ((uint32_t)sizeof(uint32_t) * 8UL) /* The number of bits in an uint32_t. */

#if (PORT_POSIX_THREADS > 0)
static void SetThreadPriority(std::thread::native_handle_type h, int priority) {
  // Set the priority of the thread
  sched_param sch;
  int policy = 0;
  pthread_getschedparam(h, &policy, &sch);
  sch.sched_priority = priority;
  if (pthread_setschedparam(h, policy, &sch)) {
    std::cerr << "Failed to set thread priority" << std::endl;
  }
}
#endif

/*
 * Created as a high priority thread, this function uses a timer to simulate
 * a tick interrupt being generated on an embedded target.  In this Windows
 * environment the timer does not achieve anything approaching real time
 * performance though.
 */
static uint16_t prvSimulatedPeripheralTimer(void *lpParameter);

/*
 * Process all the simulated interrupts - each represented by a bit in
 * ulPendingInterrupts variable.
 */
static void prvProcessSimulatedInterrupts(void);

/*
 * Interrupt handlers used by the kernel itself.  These are executed from the
 * simulated interrupt handler thread.
 */
static uint32_t prvProcessYieldInterrupt(void);
static uint32_t prvProcessTickInterrupt(void);

/*
 * Exiting a critical section will cause the calling task to block on yield
 * event to wait for an interrupt to process if an interrupt was pended while
 * inside the critical section.  This variable protects against a recursive
 * attempt to obtain pvInterruptEventMutex if a critical section is used inside
 * an interrupt handler itself.
 */
volatile BaseType_t xInsideInterrupt = pdFALSE;

#define SIZE_T size_t

/*
 * Called when the process exits to let Windows know the high timer resolution
 * is no longer required.
 */
static bool prvEndProcess(uint16_t dwCtrlType);

/*-----------------------------------------------------------*/

/* The WIN32 simulator runs each task in a thread.  The context switching is
 * managed by the threads, so the task stack does not have to be managed
 * directly, although the task stack is still used to hold an xThreadState
 * structure this is the only thing it will ever hold.  The structure indirectly
 * maps the task handle to a thread handle. */
typedef struct ThreadState {
  /* Handle of the thread that executes the task. */
  std::thread *pvThread;

  TaskFunction_t pxCode;

  /* Event used to make sure the thread does not execute past a yield point
   * between the call to SuspendThread() to suspend the thread and the
   * asynchronous SuspendThread() operation actually being performed. */
  // void *pvYieldEvent;
  CountingSemaphore *pvYieldEvent;

  void *pvParameters;

  bool isSuspended;

  bool isTerminated;

  bool hasTerminated;

  size_t xThreadId;

  ThreadState()
      : pvThread(nullptr), pvYieldEvent(nullptr), isSuspended(false),
        isTerminated(false), hasTerminated(false) {}

} ThreadState_t, TCB_t;

/*
 * Map of thread IDs to thread state structures.  This is used to obtain the
 * thread state structure from the thread ID when a thread is created.
 */
static std::hash<std::thread::id> hasher;
static std::map<size_t, ThreadState_t *> xThreadStateMap;

/* Simulated interrupts waiting to be processed.  This is a bit mask where each
 * bit represents one interrupt, so a maximum of 32 interrupts can be simulated.
 */
static volatile uint32_t ulPendingInterrupts = 0UL;

/* An event used to inform the simulated interrupt processing thread (a high
 * priority thread that simulated interrupt processing) that an interrupt is
 * pending. */
// static std::condition_variable vInterruptEvent;
// static std::condition_variable *pvInterruptEvent = &vInterruptEvent;

/* Mutex used to protect all the simulated interrupt variables that are accessed
 * by multiple threads. */
// static std::mutex vInterruptEventMutex;
// static std::mutex *pvInterruptEventMutex = &vInterruptEventMutex;

static CountingSemaphore vInterrupt(0, 1);
static CountingSemaphore *pvInterruptEvent = &vInterrupt;

#define ReleaseMutex(x) x.unlock()

/* The critical nesting count for the currently executing task.  This is
 * initialised to a non-zero value so interrupts do not become enabled during
 * the initialisation phase.  As each task has its own critical nesting value
 * ulCriticalNesting will get set to zero when the first task runs.  This
 * initialisation is probably not critical in this simulated environment as the
 * simulated interrupt handlers do not get created until the FreeRTOS scheduler
 * is started anyway. */
static volatile uint32_t ulCriticalNesting = 9999UL;

/* Handlers for all the simulated software interrupts.  The first two positions
 * are used for the Yield and Tick interrupts so are handled slightly
 * differently, all the other interrupts can be user defined. */
static uint32_t (*ulIsrHandler[portMAX_INTERRUPTS])(void) = {0};

/* Pointer to the TCB of the currently executing task. */
extern void *volatile pxCurrentTCB;

/* Used to ensure nothing is processed during the startup sequence. */
static BaseType_t xPortRunning = pdFALSE;
static BaseType_t xPortStarted = pdTRUE;

/*-----------------------------------------------------------*/
static TickType_t getMs(void) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// -----------------------------------------------------------
static void doSleep(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/*-----------------------------------------------------------*/

static uint16_t prvSimulatedPeripheralTimer(void *lpParameter) {

  TickType_t xMinimumWindowsBlockTime = (TickType_t)10;

  /* Just to prevent compiler warnings. */
  (void)lpParameter;

  while (xPortRunning == pdTRUE) {
    /* Wait until the timer expires and we can access the simulated interrupt
     * variables.  *NOTE* this is not a 'real time' way of generating tick
     * events as the next wake time should be relative to the previous wake
     * time, not the time that doSleep() is called.  It is done this way to
     * prevent overruns in this very non real time simulated/emulated
     * environment. */
    if (portTICK_PERIOD_MS < xMinimumWindowsBlockTime) {
      doSleep(xMinimumWindowsBlockTime);
    } else {
      doSleep(portTICK_PERIOD_MS);
    }

    if (xPortRunning == pdTRUE) {
      configASSERT(xPortRunning);

      /* Can't proceed if in a critical section as pvInterruptEventMutex won't
       * be available. */
      // WaitForSingleObject(pvInterruptEventMutex, INFINITE);
      pvInterruptEvent->lock_mutex();

      /* The timer has expired, generate the simulated tick event. */
      ulPendingInterrupts |= (1 << portINTERRUPT_TICK);

      /* The interrupt is now pending - notify the simulated interrupt
       * handler thread.  Must be outside of a critical section to get here so
       * the handler thread can execute immediately pvInterruptEventMutex is
       * released. */
      configASSERT(ulCriticalNesting == 0UL);
      // SetEvent(pvInterruptEvent);
      pvInterruptEvent->notify();

      /* Give back the mutex so the simulated interrupt handler unblocks
       * and can access the interrupt handler variables. */
      // ReleaseMutex(pvInterruptEventMutex);
      pvInterruptEvent->unlock_mutex();
    }
  }

  return 0;
}
/*-----------------------------------------------------------*/

static bool prvEndProcess(uint16_t dwCtrlType) {
  /* Remove compiler warnings. */
  (void)dwCtrlType;

  return pdFALSE;
}

/*-----------------------------------------------------------*/
static void xLocalThreadRunner(ThreadState_t *pxThreadState) {

  const size_t hsh = hasher(std::this_thread::get_id());

  // Get current thread ID
  pxThreadState->xThreadId = hsh;

  // Save this thread ID
  xThreadStateMap[hsh] = pxThreadState;

  pxThreadState->isSuspended = true;
  pxThreadState->pvYieldEvent->wait();
  // We are suspended
  pxThreadState->isSuspended = false;

  while ((xPortStarted || xPortRunning) && (!pxThreadState->isTerminated)) {

    // The thread will have to yield itself to the FreeRTOS scheduler
    // This should never return.
    pxThreadState->pxCode(pxThreadState->pvParameters);

    pxThreadState->isTerminated = true;
  }

  pxThreadState->hasTerminated = true;
  pxThreadState->pvThread = nullptr;
};

/*-----------------------------------------------------------*/

StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack,
                                   TaskFunction_t pxCode, void *pvParameters) {

  int8_t *const pcTopOfStack = (int8_t *)pxTopOfStack;

  /* In this simulated case a stack is not initialised, but instead a thread
   * is created that will execute the task being created.  The thread handles
   * the context switching itself.  The ThreadState_t object is placed onto
   * the stack that was created for the task - so the stack buffer is still
   * used, just not in the conventional way.  It will not be used for anything
   * other than holding this structure. */
  const size_t ceilSize = ((sizeof(ThreadState_t) + 7) / 8) * 8;
  configASSERT(ceilSize >= sizeof(ThreadState_t));
  ThreadState_t *const pxThreadState =
      (ThreadState_t *)(pcTopOfStack - ceilSize);

  pxThreadState->isTerminated = false;
  pxThreadState->hasTerminated = false;
  pxThreadState->pxCode = pxCode;
  pxThreadState->pvParameters = pvParameters;
  pxThreadState->isSuspended = false;

  /* Create the event used to prevent the thread from executing past its yield
   * point if the SuspendThread() call that suspends the thread does not take
   * effect immediately (it is an asynchronous call). */
  // pxThreadState->pvYieldEvent =
  //     CreateEvent(NULL,  /* Default security attributes. */
  //                 FALSE, /* Auto reset. */
  //                 FALSE, /* Start not signalled. */
  //                 NULL); /* No name. */
  pxThreadState->pvYieldEvent = new CountingSemaphore(0, 1);

  /* Create the thread itself. */
  // pxThreadState->pvThread = CreateThread(
  //     NULL, xStackSize, (LPTHREAD_START_ROUTINE)pxCode, pvParameters,
  //     CREATE_SUSPENDED | STACK_SIZE_PARAM_IS_A_RESERVATION, NULL);
  // Create a thread, but in suspended state.
  pxThreadState->pvThread = new std::thread(xLocalThreadRunner, pxThreadState);

  std::thread::native_handle_type h = pxThreadState->pvThread->native_handle();

#if defined(_MSC_VER)
  // Get the native handle
  SetThreadAffinityMask(h, 0x01);
  SetThreadPriorityBoost(h, TRUE);
#endif
  SetThreadPriority(h, portTASK_THREAD_PRIORITY);

  // Let it loose, it will get stuck on the first yield
  pxThreadState->pvThread->detach();

  // Put ourselves to sleep while it starts
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  configASSERT(
      pxThreadState
          ->pvThread); /* See comment where TerminateThread() is called. */

  return (StackType_t *)pxThreadState;
}

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler(void) {
  std::thread *pvHandle = NULL;
  int32_t lSuccess;
  ThreadState_t *pxThreadState = NULL;
  // SYSTEM_INFO xSystemInfo;

  /* This port runs windows threads with extremely high priority.  All the
   * threads execute on the same core - to prevent locking up the host only
   * start if the host has multiple cores. */
  // GetSystemInfo(&xSystemInfo);

  if (false) {
  } else {
    lSuccess = pdPASS;

    /* The highest priority class is used to [try to] prevent other Windows
     * activity interfering with FreeRTOS timing too much. */

    /* Install the interrupt handlers used by the scheduler itself. */
    vPortSetInterruptHandler(portINTERRUPT_YIELD, prvProcessYieldInterrupt);
    vPortSetInterruptHandler(portINTERRUPT_TICK, prvProcessTickInterrupt);
  }

  if (lSuccess == pdPASS) {
    /* Start the thread that simulates the timer peripheral to generate
     * tick interrupts.  The priority is set below that of the simulated
     * interrupt handler so the interrupt event mutex is used for the
     * handshake / overrun protection. */
    // pvHandle = CreateThread(NULL, 0, prvSimulatedPeripheralTimer, NULL,
    //                        CREATE_SUSPENDED, NULL);
    pvHandle = new std::thread(prvSimulatedPeripheralTimer, nullptr);

    {}

    if (pvHandle != NULL) {
      // SetThreadPriority(pvHandle, portSIMULATED_TIMER_THREAD_PRIORITY);
      // SetThreadPriorityBoost(pvHandle, TRUE);
      // SetThreadAffinityMask(pvHandle, 0x01);
      // ResumeThread(pvHandle);
      pvHandle->detach();
      std::thread::native_handle_type h = pvHandle->native_handle();
#ifdef _MSC_VER
      SetThreadPriorityBoost(h, TRUE);
      SetThreadAffinityMask(h, 0x01);
#endif
      SetThreadPriority(h, portSIMULATED_TIMER_THREAD_PRIORITY);
    }

    /* Start the highest priority task by obtaining its associated thread
     * state structure, in which is stored the thread handle. */
    pxThreadState = (ThreadState_t *)*((uintptr_t *)pxCurrentTCB);
    ulCriticalNesting = 0;

    /* Start the first task. */
    // ResumeThread(pxThreadState->pvThread);
    // pxThreadState->pvThread->detach();
    /* The scheduler is now running. */
    xPortRunning = pdTRUE;

    pxThreadState->pvYieldEvent->notify();

    /* Handle all simulated interrupts - including yield requests and
     * simulated ticks. */
    {
      std::thread pvInterruptsHandle =
          std::thread(prvProcessSimulatedInterrupts);
      std::thread::native_handle_type h = pvInterruptsHandle.native_handle();
      SetThreadPriority(h, portSIMULATED_INTERRUPTS_THREAD_PRIORITY);
#ifdef _MSC_VER
      SetThreadPriorityBoost(h, TRUE);
      SetThreadAffinityMask(h, 0x01);
#endif
      pvInterruptsHandle.detach();
    }

    while (1) {
      doSleep(1000);
    }
  }

  return 0;
}
/*-----------------------------------------------------------*/

static uint32_t prvProcessYieldInterrupt(void) {
  /* Always return true as this is a yield. */
  return pdTRUE;
}
/*-----------------------------------------------------------*/

static uint32_t prvProcessTickInterrupt(void) {
  uint32_t ulSwitchRequired;

  /* Process the tick itself. */
  configASSERT(xPortRunning);
  ulSwitchRequired = (uint32_t)xTaskIncrementTick();

  return ulSwitchRequired;
}
/*-----------------------------------------------------------*/

static void prvProcessSimulatedInterrupts(void) {
  uint32_t ulSwitchRequired = pdFALSE;
  ThreadState_t *pxThreadState = nullptr;

  uint16_t xResult = pdFAIL;
  const uint16_t xTimeoutMilliseconds = 1000;

  /* Create a pending tick to ensure the first task is started as soon as
   * this thread pends. */
  ulPendingInterrupts |= (1 << portINTERRUPT_TICK);

  // Wake up the interrupt event
  pvInterruptEvent->notify();

  while (xPortRunning) {
    xInsideInterrupt = pdFALSE;

    pvInterruptEvent->wait();

    pvInterruptEvent->lock_mutex();

    /* Cannot be in a critical section to get here.  Tasks that exit a
     * critical section will block on a yield mutex to wait for an interrupt
     * to process if an interrupt was set pending while the task was inside
     * the critical section.  xInsideInterrupt prevents interrupts that
     * contain critical sections from doing the same. */
    xInsideInterrupt = pdTRUE;

    /* Used to indicate whether the simulated interrupt processing has
     * necessitated a context switch to another task/thread. */
    ulSwitchRequired = pdFALSE;

    if (ulPendingInterrupts != 0) {
      /* For each interrupt we are interested in processing, each of which is
       * represented by a bit in the 32bit ulPendingInterrupts variable. */
      for (uint32_t i = 0; i < portMAX_INTERRUPTS; i++) {
        /* Is the simulated interrupt pending? */
        if ((ulPendingInterrupts & (1UL << i)) != 0) {
          /* Is a handler installed? */
          if (ulIsrHandler[i] != nullptr) {
            /* Run the actual handler.  Handlers return pdTRUE if they
             * necessitate a context switch. */
            if (ulIsrHandler[i]() != pdFALSE) {
              /* A bit mask is used purely to help debugging. */
              ulSwitchRequired |= (1 << i);
            }
          }

          /* Clear the interrupt pending bit. */
          ulPendingInterrupts &= ~(1UL << i);
        }
      }
    }

    if (ulSwitchRequired) {
      void *const pvOldCurrentTCB = pxCurrentTCB;

      /* Select the next task to run. */
      vTaskSwitchContext();

      /* If the task selected to enter the running state is not the task
       * that is already in the running state. */
      if (pvOldCurrentTCB != pxCurrentTCB) {

        /* Suspend the old thread.  In the cases where the (simulated)
         * interrupt is asynchronous (tick event swapping a task out rather
         * than a task blocking or yielding) it doesn't matter if the
         * 'suspend' operation doesn't take effect immediately - if it
         * doesn't it would just be like the interrupt occurring slightly
         * later.  In cases where the yield was caused by a task blocking
         * or yielding then the task will block on a yield event after the
         * yield operation in case the 'suspend' operation doesn't take
         * effect immediately.  */
        pxThreadState = (ThreadState_t *)*((size_t *)pvOldCurrentTCB);
        std::thread::native_handle_type h =
            pxThreadState->pvThread->native_handle();
#ifdef _MSC_VER
        SuspendThread(h);
#elif defined(__APPLE__)
        // To suspend a thread
        kern_return_t kr;
        kr = thread_suspend(pthread_mach_thread_np(h));
        if (kr != KERN_SUCCESS) {
          // handle error
        }

        // To resume a thread
        // kr = thread_resume(pthread_mach_thread_np(h));
        // if (kr != KERN_SUCCESS) {
        // handle error
        //}
#else
#endif

        /* Obtain the state of the task now selected to enter the
         * Running state. */
        pxThreadState = (ThreadState_t *)(*(size_t *)pxCurrentTCB);

        configASSERT(pxThreadState->pvThread != NULL);
        configASSERT(pxThreadState->isSuspended);
        configASSERT(!pxThreadState->isTerminated);
        configASSERT(!pxThreadState->hasTerminated);

        /* pxThreadState->pvThread can be NULL if the task deleted
         * itself - but a deleted task should never be resumed here. */
        configASSERT(pxThreadState->pvThread != NULL);
        // ResumeThread(pxThreadState->pvThread);

        // Unyield the thread
        // pxThreadState->pvYieldEvent->notify();
      }
    }

    /* If the thread that is about to be resumed stopped running
     * because it yielded then it will wait on an event when it resumed
     * (to ensure it does not continue running after the call to
     * SuspendThread() above as SuspendThread() is asynchronous).
     * Signal the event to ensure the thread can proceed now it is
     * valid for it to do so.  Signaling the event is benign in the case that
     * the task was switched out asynchronously by an interrupt as the event
     * is reset before the task blocks on it. */
    pxThreadState = (ThreadState_t *)(*(size_t *)pxCurrentTCB);
    // SetEvent(pxThreadState->pvYieldEvent);
    pxThreadState->pvYieldEvent->notify();

    // ReleaseMutex(pvInterruptEventMutex);
    pvInterruptEvent->unlock_mutex();
  }
}
/*-----------------------------------------------------------*/

void vPortDeleteThread(void *pvTaskToDelete) {

  /* Find the handle of the thread being deleted. */
  ThreadState_t *pxThreadState = (ThreadState_t *)(*(size_t *)pvTaskToDelete);

  /* Check that the thread is still valid, it might have been closed by
   * vPortCloseRunningThread() - which will be the case if the task associated
   * with the thread originally deleted itself rather than being deleted by a
   * different task. */
  if (pxThreadState->pvThread != NULL) {
    // WaitForSingleObject(pvInterruptEventMutex, INFINITE);

    /* !!! This is not a nice way to terminate a thread, and will eventually
     * result in resources being depleted if tasks frequently delete other
     * tasks (rather than deleting themselves) as the task stacks will not be
     * freed. */
    // ulErrorCode = TerminateThread(pxThreadState->pvThread, 0);
    pxThreadState->isTerminated = true;

    // Wait for the thread to terminate itself
    pxThreadState->pvYieldEvent->notify();

    // while (!pxThreadState->hasTerminated) {
    //   doSleep(10);
    // }
  }
}

/*-----------------------------------------------------------*/

void vPortCloseRunningThread(void *pvTaskToDelete,
                             volatile BaseType_t *pxPendYield) {

  uint32_t ulErrorCode;

  /* Remove compiler warnings if configASSERT() is not defined. */
  (void)ulErrorCode;

  /* Find the handle of the thread being deleted. */
  ThreadState_t *pxThreadState = (ThreadState_t *)(*(size_t *)pvTaskToDelete);
  std::thread *pvThread = pxThreadState->pvThread;

  /* Raise the Windows priority of the thread to ensure the FreeRTOS scheduler
   * does not run and swap it out before it is closed.  If that were to happen
   * the thread would never run again and effectively be a thread handle and
   * memory leak. */
  // SetThreadPriority(pvThread, portDELETE_SELF_THREAD_PRIORITY);

  /* This function will not return, therefore a yield is set as pending to
   * ensure a context switch occurs away from this thread on the next tick. */
  *pxPendYield = pdTRUE;

  /* Mark the thread associated with this task as invalid so
   * vPortDeleteThread() does not try to terminate it. */
  pxThreadState->pvThread = NULL;

  /* Close the thread. */
  // ulErrorCode = CloseHandle(pvThread);
  configASSERT(ulErrorCode);
  pxThreadState->isTerminated = true;
  pxThreadState->pvYieldEvent->notify();

  /* This is called from a critical section, which must be exited before the
   * thread stops. */
  taskEXIT_CRITICAL();
  // CloseHandle(pxThreadState->pvYieldEvent);
  // ExitThread(0);
}
/*-----------------------------------------------------------*/

void vPortEndScheduler(void) { xPortRunning = pdFALSE; }
/*-----------------------------------------------------------*/

void vPortGenerateSimulatedInterrupt(uint32_t ulInterruptNumber) {
  ThreadState_t *pxThreadState = (ThreadState_t *)*((size_t *)pxCurrentTCB);

  configASSERT(xPortRunning);

  if (ulInterruptNumber < portMAX_INTERRUPTS) {
    // WaitForSingleObject(pvInterruptEventMutex, INFINITE);
    pvInterruptEvent->lock_mutex();
    ulPendingInterrupts |= (1 << ulInterruptNumber);

    /* The simulated interrupt is now held pending, but don't actually
     * process it yet if this call is within a critical section.  It is
     * possible for this to be in a critical section as calls to wait for
     * mutexes are accumulative.  If in a critical section then the event
     * will get set when the critical section nesting count is wound back
     * down to zero. */
    if (ulCriticalNesting == 0) {
      // SetEvent(pvInterruptEvent);
      pvInterruptEvent->notify();

      /* Going to wait for an event - make sure the event is not already
       * signaled. */
      // ResetEvent(pxThreadState->pvYieldEvent);
      pxThreadState->pvYieldEvent->reset();
    }

    // ReleaseMutex(pvInterruptEventMutex);
    pvInterruptEvent->unlock_mutex();

    if (ulCriticalNesting == 0) {
      /* An interrupt was pended so ensure to block to allow it to
       * execute.  In most cases the (simulated) interrupt will have
       * executed before the next line is reached - so this is just to make
       * sure. */
      // WaitForSingleObject(pxThreadState->pvYieldEvent, INFINITE);
      taskENTER_CRITICAL();
      taskEXIT_CRITICAL(); // This will yield the thread
    }
  }
}
/*-----------------------------------------------------------*/

void vPortSetInterruptHandler(uint32_t ulInterruptNumber,
                              uint32_t (*pvHandler)(void)) {
  if (ulInterruptNumber < portMAX_INTERRUPTS) {

    // WaitForSingleObject(pvInterruptEventMutex, INFINITE);
    pvInterruptEvent->lock_mutex();
    ulIsrHandler[ulInterruptNumber] = pvHandler;
    // ReleaseMutex(pvInterruptEventMutex);
    pvInterruptEvent->unlock_mutex();
  }
}
/*-----------------------------------------------------------*/

void vPortEnterCritical(void) {
  if (!xPortRunning) {
    return;
  }

  if (xPortRunning == pdTRUE) {
    /* The interrupt event mutex is held for the entire critical section,
     * effectively disabling (simulated) interrupts. */
    // WaitForSingleObject(pvInterruptEventMutex, INFINITE);
    pvInterruptEvent->lock_mutex();
  }

  ulCriticalNesting++;
}
/*-----------------------------------------------------------*/

void vPortExitCritical(void) {
  if (!xPortRunning) {
    return;
  }

  int32_t lMutexNeedsReleasing = pdTRUE;

  /* The interrupt event mutex should already be held by this thread as it was
   * obtained on entry to the critical section. */

  if (ulCriticalNesting > 0) {
    ulCriticalNesting--;

    /* Don't need to wait for any pending interrupts to execute if the
     * critical section was exited from inside an interrupt. */
    if ((ulCriticalNesting == 0) && (xInsideInterrupt == pdFALSE)) {
      /* Were any interrupts set to pending while interrupts were
       * (simulated) disabled? */
      if (ulPendingInterrupts != 0UL) {
        ThreadState_t *pxThreadState =
            (ThreadState_t *)*((size_t *)pxCurrentTCB);

        configASSERT(xPortRunning);

        /* The interrupt won't actually executed until
         * pvInterruptEventMutex is released as it waits on both
         * pvInterruptEventMutex and pvInterruptEvent.
         * pvInterruptEvent is only set when the simulated
         * interrupt is pended if the interrupt is pended
         * from outside a critical section - hence it is set
         * here. */
        // SetEvent(pvInterruptEvent);
        pvInterruptEvent->notify();

        /* The calling task is going to wait for an event to ensure the
         * interrupt that is pending executes immediately after the
         * critical section is exited - so make sure the event is not
         * already signaled. */
        // ResetEvent(pxThreadState->pvYieldEvent);
        pxThreadState->pvYieldEvent->reset();

        /* Mutex will be released now so the (simulated) interrupt can
         * execute, so does not require releasing on function exit. */
        lMutexNeedsReleasing = pdFALSE;
        // ReleaseMutex(pvInterruptEventMutex);

        // WaitForSingleObject(pxThreadState->pvYieldEvent, INFINITE);
        pxThreadState->isSuspended = true;
        pvInterruptEvent->unlock_mutex();

        // Here is where we block
        pxThreadState->pvYieldEvent->wait();

        // We were unblocked by someone...
        pvInterruptEvent->lock_mutex();
        pxThreadState->isSuspended = false;
        pvInterruptEvent->unlock_mutex();
      }
    }
  }

  if (lMutexNeedsReleasing == pdTRUE) {
    configASSERT(xPortRunning);
    // ReleaseMutex(pvInterruptEventMutex);
    pvInterruptEvent->unlock_mutex();
  }
}
/*-----------------------------------------------------------*/
