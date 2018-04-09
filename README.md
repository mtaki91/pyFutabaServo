# pyFutabaServo
Using Futaba's servo motor from Python

## Installation
Install python-serial, and add $USERNAME into dialout.
```
sudo apt-get install python-serial
sudo gpasswd -a $USERNAME dialout
```

## Setting Up 
### After booting up
```
sudo ./startup.sh
```

### How To Use
#### Import 
```
from robot_servo import *
```
Serial converter
```
ttyUSB0 = serial_converter('/dev/ttyUSB0', 115200)
```
Servo
```
servo_list = [futaba_servo(ttyUSB0, id) for id in [1,2,3,4,5,6]]
```
Servo cluster(e.g., hand, arm....)
```
activate_sequence = [[0,3],[1,4],[2,5]]

#it means that when starting up,
#first, servo_list[0] and servo_list[3] are move to initial position.
#next, [1] and [4] are move.
#finnaly, [2] and [5] are move.

zero_position = [0,-200,0,0,-200,0]
hand = servo_cluster(servo_list, activate_sequence, zero_position)
```

#### start and stop
start and go initial posture
```
hand.activate()
```
stop
```
hand.deactivate()
```

#### setting torque

off   : 0  
on    : 1  
brake : 2  

```
# all servo
hand.torque(1)
# one by one
hand.torque([0,0,0,1,1,1])
```

#### move servo
1[deg] -> 10  
1[sec] -> 100  

```
angles = [-200, -100, 0, 0, 100, 200] # -20, -10, 0, 0, 10, 20 [deg]
move_time = 100 # 1sec
hand.move(angles, move_time)
```

#### get servo status
```
hand.get_status()
```
