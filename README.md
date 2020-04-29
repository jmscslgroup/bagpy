# bagpy
__A Python package to facilitate the reading of a rosbag file based on semantic datatypes.__

__`bagpy`__ provides a wrapper class `bagreader` written in python that provides an easy to use interface for reading 
[bag files](http://wiki.ros.org/Bags) recorded by `rosbag record` command. This wrapper class uses ROS's python API `rosbag`
internally to perform all operations. One of the interesting features about using __`ROSBagReader`__ is that a user doesn't 
need to supply [rostopic name](http://wiki.ros.org/rostopic) to extract relevant data. One can extract data based on the 
type of data the user is seeking.

## Usage principle
The philosophy behind developing this project is to make everything as simple and less confusing as possible. 
As a result, there are not too many options or freedom of usage being provided by __`ROSBagReader`__ class. 
If you need a wide variety of options, users can directly use ROS's `rosbag` python package or 
Robotics System Toolbox APIs provided by MATLAB. However, if you are looking for simplicity, __`bagpy`__ is an
elegant choice.
