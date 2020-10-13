About bagpy
------------

**bagpy** provides a wrapper class ``bagreader`` written in python that provides an easy to use interface for reading bag files recorded by rosbag record command. This wrapper class uses ROS's python API rosbag internally to perform all operations. One of the interesting features about using bagreader is that a user doesn't need to supply rostopic name to extract relevant data. One can extract data based on the type of data the user is seeking. However, ``bagpy`` also allow to decode messages by rostopic types. ``bagpy`` has proven to decode custom ROS messages as well.
