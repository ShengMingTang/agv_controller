# Version : tircgo1107
# Manual : https://hackmd.io/2yVRbpZSQiuUu7M4j-Bghg?view

# Add action to provide programming interface

# routine:
## sudo chmod a+rw /dev/input/js0
## sudo chmod 666 /dev/ttyUSB0
## source tircgo/devel/setup.bash

# nodes available
## tircgo_controller/ticgo_c... for controller node
## ticgo_controller/tircgo_s... for schduler node [deprecated]

## Note that instruction are written in schedule.py
## rosrun scheduler.py to execute this schedule

```python
# Go(R, N), go to specified node N in route R
# Delay(n), delay 0.1n sec
# setup() only runs once in the start of this schedule
# loop() will loop over the life of this schedule
from scheduler import *
def setup():
    # any code
    return
def loop():
    # any code
    return
```