# Version : tircgo1107
# Manual : https://hackmd.io/2yVRbpZSQiuUu7M4j-Bghg?view

# Add action to provide programming interface

# routine:
## sudo chmod a+rw /dev/input/js0
## sudo chmod 666 /dev/ttyUSB0
## source tircgo/devel/setup.bash

# nodes available
## tircgo_controller/ticgo_c... for controller node
## ticgo_controller/tircgo_s... for schduler node

```c++
// template for scheduler.cpp
// put it under tircgo/src/tircgo_controller/src
// $ source path_to_tircgo/devel/setup.bash
// $ cd ~/tircgo
// $ catkin_make
// $ roslaunch tircgo_controller background.launch
#include "scheduler.h"

void setup()
{
    
}

void loop()
{

}
```