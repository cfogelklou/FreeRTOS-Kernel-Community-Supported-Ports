# A Port of FreeRTOS using modern C++ to emulate the OS.

The point of this project is to make emulation on Windows/macOS/Linux easier, but using modern cross-platform C++ primitives like std::thread.

## Design

The default operating mode does not support preemption. This is compatible with Windows/Linux/macOS.

macOS and Windows support thread suspension, so in these ports, preemption IS supported.

### Design without preemption

```plantuml
@startuml

participant main as main
participant "ISR Thread" as isr
participant "High Prio Thread" as h
participant "Low Prio Thread" as l
participant "Kernel" as k

== Startup ==

note over h
*this.sem* for each thread is a native OS counting semaphore, not a FreeRTOS semaphore.
ISR Thread is also a native OS thread (std::thread), not FreeRTOS.
end note

main->isr: Start
isr->isr: is_waiting=true\nis_running=false\nsem_wait(this.sem)


main->h: Start
h->h: is_waiting=true\nis_running=false\nsem_wait(this.sem)


main->l: Start
l->l: is_waiting=true\nis_running=false\nsem_wait(this.sem)

== Kernel Start ==

note over main
No tasks currently running.
ASSERT: No tasks running.
end note
main->isr: notify(isr.sem)
activate isr
  isr<->k: pxCurrentTCB\n\t:=vTaskSwitchContext()

  note over isr
  Only the ISR can clear is_waiting and set is_running.
  The task should toggle them back before notifying the ISR.

  end note
  isr->isr:assert((h.is_waiting)&&\n\t(h.is_running == false))
  isr->h: h.is_waiting=false\nh.is_running=true\nnotify(h.sem)
activate h
  isr->isr: wait(isr.sem)
deactivate isr
  h->h:run
  h->h:EnterCritical()
  h->h:ExitCritical()
  h->h:h.is_waiting=true\nh.is_running=false\n
  h->isr:notify(isr.sem)
activate isr
  h->h: sem_wait(this.sem)
deactivate h

  isr<->k: pxCurrentTCB\n\t:=vTaskSwitchContext()


  isr->l: l.is_waiting=false\nl.is_running=true\nnotify(l.sem)
activate l
  isr->isr: wait(isr.sem)
deactivate isr
  l->l:run
  l->l:EnterCritical()
  l->l:ExitCritical()
  l->l:l.is_waiting=true\nl.is_running=false\n
  l->isr:notify(isr.sem)
activate isr
  l->l: sem_wait(this.sem)
deactivate l




@enduml
```
