# Pose Listener
The `pose_listener` package allows you to read a ROS topic and saves poses to a file. Poses are stored in TUM format, but code is straightforward and it is easily extensible to other formats.
It currently supports:
* `nav_msgs::Odometry`
* `geometry_msgs::PoseStamped`
* `geometry_msgs::PoseWithCovarianceStamped`

## Dependencies
* ROS

## Build
Clone this repository to your workspace and `catkin_make` it.

## Usage
Run:
```
rosrun pose_listener pose_listener _topic:=<TOPIC_NAME> _type:=<TOPIC_TYPE> [_output_file:=<PATH_TO_FILE>]
```
where 
* `TOPIC_NAME` is the name of the topic, e.g. `/firefly_sbx/vio/odom`. Run `rostopic list` to print a list of topic names.
* `TOPIC_TYPE` is the type of the topic. It must be one of the following values:
  * `O` if the type is `nav_msgs::Odometry`
  * `PS` if the type is `geometry_msgs::PoseStamped`
  * `PCS` if the type is `geometry_msgs::PoseWithCovarianceStamped`
* `PATH_TO_FILE` is a path to the output file. `_output_file` is an optional argument. Default value: `trajectory.txt`

## TODO
* The type of the topic should be queried in runtime.
* Path of the output file should be taken as a command line parameter.
* Support KITTI formats (and many others)
* Support different ROS msgs.
