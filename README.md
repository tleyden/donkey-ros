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

This step is useful to validate that the i2cpwm_board is properly working.  You probably want to put your donkey car on a "rack" so that you don't end up chasing it.

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

After hitting enter, the wheels on your donkey car should be spinning.

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

### Capture and publish to topic

This assumes you've built the https://github.com/dganbold/raspicam_node module.  See Appendix / Build raspicam_node.

```
$ roslaunch raspicam_node camera_module_v2_640x480_5fps_autocapture.launch
```

### Display on a laptop running ROS

On a linux laptop with ROS installed:

```
$ export ROS_MASTER_URI=http://ubiquityrobot.local:11311
$ rosrun rqt_image_view rqt_image_view
```

and you should see a window popup that shows the video stream from the donkey car camera.

### Display on a laptop via HTTP streaming

On the raspberry pi do the following one-time install:

```
$ sudo apt-get install ros-kinetic-web-video-server
```

Then start the streaming server via:

```
$ rosrun web_video_server web_video_server
```

and in your browser open [http://ubiquityrobot.local:8080/stream_viewer?topic=/raspicam_node/image_raw](http://ubiquityrobot.local:8080/stream_viewer?topic=/raspicam_node/image_raw).


## Wrapping up into a single launch file

```
$ cat donkey_ros.launch
<launch>

  <include file="$(find i2cpwm_board)/launch/i2cpwm_node.launch"/>

  <include file="$(find raspicam_node)/launch/camera_module_v2_640x480_5fps_autocapture.launch"/>

  <node pkg="donkey_llc" name="donkey_llc" type="low_level_control.py" output="screen" >
  </node>
  
  <node pkg="web_video_server" name="web_video_server" type="web_video_server">
  </node>
  
</launch>
$ roslaunch donkey_ros.launch
```

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

### Build raspicam_node

```
  237  git clone https://github.com/dganbold/raspicam_node.git
  238  catkin_make --pkg raspicam_node
  239  ls
  240  cd ..
  241  catkin_make --pkg raspicam_node
  242  source devel/setup.bash
```

## References

* [ROS and Raspberry Pi for Beginners | Tutorial #0 - Topics Packages RosMaster](https://www.youtube.com/watch?v=iLiI_IRedhI)
* [Ubuiquity Robot network instructions](https://learn.ubiquityrobotics.com/connect_network)
* [dganbold/raspicam_node](https://github.com/dganbold/raspicam_node)
* [How to publish Image Stream in ROS Raspberry Pi](https://www.theconstructsim.com/publish-image-stream-ros-kinetic-raspberry-pi/)
* [ROS web-video-server](https://wiki.ros.org/web_video_server)