# Homography-based 2D Visual Servo with Remote Center of Motion Workspace
This workspace implements an exemplary use cases for homography-based 2D visual servoing with remote center of motion constraint. It is based upon 4 main packages
- [rcm](https://github.com/RViMLab/rcm), implements an abstract task remote center of motion gain controller
- [h_vs](https://github.com/RViMLab/h_vs), implements the desired homography generation, and the visual servo to compute the desired camera frame twist velocity
- [h_rcm_vs](https://github.com/RViMLab/h_rcm_vs), implements the specific task for the abstract [rcm](https://github.com/RViMLab/rcm) package
- [fri_ros](https://github.com/KCL-BMEIS/fri_ros), handles the communication to the KUKA LBR Med via the fast robot interface

To build this code, do
```shell
mkdir -p calibration_pattern_h_rcm_vs/src && cd calibration_pattern_h_rcm_vs/src
git clone --recursive https://github.com/RViMLab/h_rcm_vs_ws && cd ..
```
Then, for the simulated setup do
```shell
catkin_make -DCATKIN_BLACKLIST_PACKAGES="decklink_ros" && source devel/setup.bash
```
For the real setup do
```shell
catkin_make -DDECKLINK_SDK_DIR="path/to/Blackmagic_DeckLink_SDK" && source devel/setup.bash
```
All examples are explained below.

## Calibration Pattern
This visual servo computes a desired homography from a calibration pattern. To run this example, open 3 terminals and do
```shell
roslaunch lbr_storz_endoscope_moveit moveit_planning_execution.launch sim:=true  # initializes robot
```
where `sim` can be set to `false` for use on the real robot. In case of a real setup, also launch a camera, for example, open a 4th terminal and do
```shell
roslaunch h_rcm_vs decklink_storz_endoscope.launch
```
In the 2nd terminal do
```shell
roslaunch h_rcm_vs rcm_init.launch image_topic:=/lbr/storz_endoscope_camera/image_raw  # initializes random endoscope position
```
For the real setup, do
```shell
roslaunch h_rcm_vs rcm_init.launch \
    image_topic:=/decklink/crop/image_raw \
    url:=package://h_rcm_vs/config/decklink_storz_endoscope_calibrationdata/ost.yaml  # initializes random endoscope position
```
In a 3rd terminal, do
```shell
roslaunch h_rcm_vs h_rcm_calibration_pattern_vs.launch image_topic:=/lbr/storz_endoscope_camera/image_raw  # launches the visual servo
```
The `h_vs_node` and `h_gen_calibration_pattern_node` or `h_gen_endoscopy_calibration_pattern_node` nodes within this launch file also load a calibration file, therefore, for a real setup do
```shell
roslaunch h_rcm_vs h_rcm_endoscopy_calibration_pattern_vs.launch \
    image_topic:=/decklink/crop/image_raw \
    cname:=decklink url:=package://h_rcm_vs/config/decklink_storz_endoscope_calibrationdata/ost.yaml  # launches the visual servo
```

## Stored Views
### Endoscope
In this mode, the user has the ability to move the robot via a GUI and to capture images along the way. In the process, a graph with images as nodes is built that can be used for servoing. To run, open 2 terminals and do
```shell
roslaunch lbr_storz_endoscope_moveit moveit_planning_execution.launch sim:=true  # initializes robot
```
where `sim` can be set to `false` for use on the real robot. In case of a real setup, also launch a camera, for example, open a 3rd terminal and do
```shell
roslaunch h_rcm_vs decklink_storz_endoscope.launch
```
In the second terminal do
```shell
roslaunch h_rcm_vs h_rcm_endoscopy_stored_views_vs.launch
```
For the real setup do
```shell
roslaunch h_rcm_vs h_rcm_endoscopy_stored_views_vs.launch \
    image_topic:=/decklink/crop/image_raw \
    cname:=decklink url:=package://h_rcm_vs/config/decklink_storz_endoscope_calibrationdata/ost.yaml \
    sim:=false  # launches the visual servo
```

### Exoscope
In this mode, the user has the ability to move the robot via a GUI and to capture images along the way. In the process, a graph with images as nodes is built that can be used for servoing. To run, open 2 terminals and do
```shell
roslaunch lbr_storz_exoscope_moveit moveit_planning_execution.launch sim:=true  # initializes robot
```
where `sim` can be set to `false` for use on the real robot. In case of a real setup, also launch a camera, for example, open a 3rd terminal and do
```shell
roslaunch h_pose_vs decklink_storz_exoscope.launch
```
In the second terminal do
```shell
roslaunch h_pose_vs h_pose_exoscope_stored_views_vs.launch
```
For the real setup do
```shell
roslaunch h_pose_vs h_pose_exoscope_stored_views_vs.launch \
    image_topic:=/decklink/crop/image_raw \
    cname:=decklink url:=package://h_pose_vs/config/decklink_storz_exoscope_calibrationdata/ost.yaml \
    sim:=false  # launches the visual servo
```

## Deep Servos
- different servos with h_gen_node.py, and flags

## Known Limitations
- There are some incompatabilities with Pyhton3, ROS, and OpenCV, see [stack overflow](https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3). It can be solved by performing an extended sourcing.
