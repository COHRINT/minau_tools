# minau_tools
## Hardware testing documentation
### On the subs
There are a bunch of aliases that are important in the bashrc of the subs.
- main -> this launches pretty much all onboard software
- home -> this zeros the location and orientation of the sub
- sonar -> this will start the ping 360 on the sub
After you start these you will have to arm the motors with the arm_control rosservice.
### Topside
On all terminals you will need to connect to one of the subs rosmasters. To do so run in minau_tools/scripts:
```bash
  source set_remote_rosmaster.sh ip
```
To start up the rviz visualizer, on guppy run:
```bash
  roslaunch minau_tools guppy_test.launch
```
and on bruce run:
```bash
  roslaunch minau_tools bruce_test.launch
```
This visualizer has a model of a pool in it. The subs are represented as a green cube. If you click on 2D Nav Goal on the top menu bar of you can now click at any location and it will send a waypoint goal to the subs and they will go there.

Currently the detection algorithm is ran topside because the pis are not powerful enought to run yolo onboard. To run yolo on your computer, set the remote rosmaster like above and then run (just change X to the corresponding asset):
```bash
  roslaunch sonar_yolo_detector sonar_yolo.launch __ns:=bluerov2_X
```
Also currently the IMUs have a significant amount of drift. To temporarily counteract this we can manually send corrections to the filter. To correct by XX.xx degrees run:
```bash
  rostopic pub /bluerov2_X/imu/correction std_msgs/Float64 "data: XX.xx"
```
