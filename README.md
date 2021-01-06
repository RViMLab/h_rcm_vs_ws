# Homography-based 2D Visual Servo with Remote Center of Motion Workspace
This workspace implements an exemplary use cases for homography-based 2D visual servoing with remote center of motion constraint. It is based upon 4 main packages
- [rcom](https://github.com/RViMLab/rcom), implements an abstract task remote center of motion gain controller
- [h_vs](https://github.com/RViMLab/h_vs), implements the desired homography generation, and the visual servo to compute the desired camera frame twist velocity
- [h_rcom_vs](https://github.com/RViMLab/h_rcom_vs), implements the specific task for the abstract [rcom](https://github.com/RViMLab/rcom) package
- [fri_ros](https://github.com/KCL-BMEIS/fri_ros), handles the communication to the KUKA LBR Med via the fast robot interface

To build this code, do
```shell
mkdir -p calibration_pattern_h_rcom_vs/src && cd calibration_pattern_h_rcom_vs/src
git clone --recursive https://github.com/RViMLab/h_rcom_vs_ws && cd ..
catkin_make && source devel/setup.bash
```

All examples are explained below.

## Calibration Pattern
This visual servo computes a desired homography from a calibration pattern. To run this example, open 3 terminals and do
```shell
roslaunch lbr_stroz_endoscope_moveit moveit_planning_execution.launch sim:=true  # initializes robot
```
where `sim` can be set to `false` for use on the real robot. In case of a real setup, also launch a camera, for example
```shell
roslaunch h_rcom_vs decklink_storz_endoscope.launch
```
In the 2nd terminal do
```shell
roslaunch h_rcom_vs rcom_init.launch image_topic:=/lbr/storz_endoscope_camera/image_raw  # initializes random endoscope position
```
where `image_topic` is `/decklink/crop/image_raw` for the real setup. In a 3rd terminal do
```shell
roslaunch h_rcom_vs h_rcom_calibration_pattern_vs.launch image_topic:=/lbr/storz_endoscope_camera/image_raw  # launches the visual servo
```
The `h_vs_node` and `h_gen_calibration_pattern_node` nodes within this launch file also load a calibration file, therefore, for a real setup do
```shell
roslaunch h_rcom_vs h_rcom_calibration_pattern_vs.launch \
    image_topic:=/decklink/crop/image_raw \
    cname:=decklink url:=package://h_rcom_vs/config/decklink_storz_endoscope_calibrationdata/ost.yaml  # launches the visual servo
```

## Deep Servos
- different servos with h_gen_node.py, and flags

## Known Limitations
- There are some incompatabilities with Pyhton3, ROS, and OpenCV, see [stack overflow](https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3). It can be solved by performing an extended sourcing.
