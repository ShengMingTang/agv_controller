# Version : tircgo1107
# Manual : https://hackmd.io/2yVRbpZSQiuUu7M4j-Bghg?view

# Add action to provide programming interface

# routine:
## sudo chmod a+rw /dev/input/js0
## sudo chmod 666 /dev/ttyUSB0
## source tircgo/devel/setup.bash

# nodes available
## If running a node any below with nonempty cmd line args, the first one must be its name [mendatory]
## Use "" to pass an empty string to the process

## tircgo_controller/ticgo_c... for controller node
> main control
## tircgo_controller/Agent.py
> for UART history
## tircgo_controller/Scheduler.py
> for Schedule, written in setup() and loop() using shot hand function above

# For testing
## tircgo_controller/Imm_wifi.py
> Emulating wifi, the first 2 args is the sender and the receiver, matched with the one you launch controller with

```python
# In tircgo_controller/src/scripts/Scheduler.py
# customize the 2 function above using ONLY
# Work(R, N)
# Drive(Linear, Angular, duration)
# Delay(N) for 0.1*N sec
def setup():
    return
def loop():
    return
```