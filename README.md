This is a tutorial on using ROS with a Donkey Car.

## Installation

* Follow instructions on https://downloads.ubiquityrobotics.com/pi.html and [Ubuiquity Robot network instructions](https://learn.ubiquityrobotics.com/connect_network) to get ROS running on your Donkey Car.

## Accessing on home wifi (eg, Gwifi)

Assumes it is in the mode where it connects to a home wifi (see switch_to_wifi_mode.sh below)

1. Turn on donkey car
2. `ssh ubuntu@ubiquityrobot.local` (password: `ubuntu`)

## Accessing in field

Assumes it is in the mode where it provides an access point (see switch_to_ap_mode.sh below)

1. See https://learn.ubiquityrobotics.com/connecting

## Switching between home wifi and access point mode

### switch_to_ap_mode.sh

```
# To run this:
#   sudo bash
#   nohup ./switch_to_ap_mode.sh

echo "y" | pifi remove GWifi
echo "Removed GWifi"
# sleep 30  # just to see if a reboot even needed
reboot
```

### switch_to_wifi_mode.sh

```
# To run this:
#   sudo bash
#   ./switch_to_wifi_mode.sh

pifi add GWifi YOUR_WIFI_PASSWORD
echo "Rebooting, wait 30s and ssh ubuntu@ubiquityrobot.local"
reboot
```

## Running motor ECU driver (i2cpwm_board)

In order to control the motor and steering, you go through the i2cpwm_board ros node. This assumes that you've already installed https://gitlab.com/bradanlane/ros-i2cpwmboard.

Start the ros node by running:

```
$ rosrun i2cpwm_board i2cpwm_board
```

and you should see output:

```
[ INFO] [1571265047.816527752]: I2C bus opened on /dev/i2c-1
[ INFO] [1571265047.832949932]: Setting PWM frequency to 50 Hz
```

## Sending low level control messages to motor/steering

This step is useful to validate that the i2cpwm_board is properly working.

To set the throttle to make the wheels spin:

```
$ rostopic pub /servos_absolute i2cpwm_board/ServoArray <TAB> <TAB>
```

and change the default values to:

```
$ rostopic pub /servos_absolute i2cpwm_board/ServoArray "servos:
- servo: 1
  value: 366.0"
```

To set it back to idle, redo the above command but use `333.0` as the value.

## Driving via teleop twist

### Launch donkey_llc node

This is the bridge between teleop twist messages and the i2cpwm_board controls.  It reads teleop twist messages from the `cmd_vel` topic and controls the motor accordingly.  It assumes you've downloaded the code, see Appendix / Build donkey_llc below.

In a seperate terminal, run:

```
$ cd ~/catkin_ws/src/donkey_llc
$ rosrun donkey_llc low_level_control.py
```

### Launch teleop twist

```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Now you can type `i` to accelerate and `k` to stop.  

## Viewing Camera images

TODO

## Appendix

### Build donkey_llc

```
  135  git clone https://github.com/tizianofiorenzani/ros_tutorials.git tizianofiorenzani_ros_tutorials
  136  cd tizianofiorenzani_ros_tutorials/
  137  ls
  138  cd donkey_car/
  139  ls
  140  cd src/
  141  ls
  142  rosrun donkey_llc low_level_control.py
  143  cd
  144  cd catkin_ws/
  145  ls
  146  cd src/
  147  catkin_create_pkg donkey_llc
  148  cd donkey_llc/
  149  ls
  150  cd ..
  151  rm -rf donkey_llc/
  152  catkin_create_pkg donkey_llc rospy
  153  cd donkey_llc/src/
  154  ls
  155  cp ~/tizianofiorenzani_ros_tutorials/donkey_car/src/low_level_control.py
  156  cp ~/tizianofiorenzani_ros_tutorials/donkey_car/src/low_level_control.py .
  157  rosrun donkey_llc low_level_control.py
```

## References

* [ROS and Raspberry Pi for Beginners | Tutorial #0 - Topics Packages RosMaster](https://www.youtube.com/watch?v=iLiI_IRedhI)
* [Ubuiquity Robot network instructions](https://learn.ubiquityrobotics.com/connect_network)
