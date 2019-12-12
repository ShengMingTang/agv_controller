# Version : tircgo1107
# Manual : https://hackmd.io/2yVRbpZSQiuUu7M4j-Bghg?view

# Add action to provide programming interface

# routine:
## sudo chmod a+rw /dev/input/js0
## sudo chmod 666 /dev/ttyUSB0
## source tircgo/devel/setup.bash
### rosrun tircgo_controller tircgo_controller_node ""
### rosrun tircgo_uart tircgo_uart_encoder_decoder_node
### rosrun tircgo_controller Agent.py ""
### rosrun tircgo_controller Scheduler.py ""

# nodes available
## If running a node any below with nonempty cmd line args, the first one must be its name [mendatory]
## Use "" to pass an empty string to the process

## tircgo_controller/ticgo_controller_node [name]
> main control
## tircgo_controller/Agent.py [name]
> for UART history
## tircgo_controller/Scheduler.py [name]
> for Schedule, written in setup() and loop() using shot hand function above

# For testing
## tircgo_controller/Imm_wifi.py [from, to]
> Emulating wifi, the first 2 args is the sender and the receiver, matched with the one you launch controller with
## tircgo_controller/Imm_joy.py [name]
> Separating single joystick input to separate controllers

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