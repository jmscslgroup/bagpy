<img src="https://raw.githubusercontent.com/jmscslgroup/bagpy/master/banner_bagpy.png" alt="Strym Logo" align="center"/>

[![Build Status](https://travis-ci.com/jmscslgroup/bagpy.svg?branch=master)](https://travis-ci.com/jmscslgroup/bagpy)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/jmscslgroup/bagpy/graphs/commit-activity)
[![made-with-python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)
[![made-with-sphinx-doc](https://img.shields.io/badge/Made%20with-Sphinx-1f425f.svg)](https://www.sphinx-doc.org/)
[![PyPI version shields.io](https://img.shields.io/pypi/v/bagpy.svg)](https://pypi.python.org/pypi/bagpy/)
[![PyPI license](https://img.shields.io/pypi/l/bagpy.svg)](https://pypi.python.org/pypi/bagpy/)
[![Downloads](https://pepy.tech/badge/bagpy)](https://pepy.tech/project/bagpy)



# bagpy
__A Python package to facilitate the reading of a rosbag file based on semantic datatypes.__

__`bagpy`__ provides a wrapper class `bagreader` written in python that provides an easy to use interface for reading 
[bag files](http://wiki.ros.org/Bags) recorded by `rosbag record` command. This wrapper class uses ROS's python API `rosbag`
internally to perform all operations. One of the interesting features about using __`bagreader`__ is that a user doesn't 
need to supply [rostopic name](http://wiki.ros.org/rostopic) to extract relevant data. One can extract data based on the 
type of data the user is seeking.

## Requirements
- Ubuntu 18.04 or later
- Python 3.6 or higher. **Now, the preferred version is Python 3.9**. With other versions, there are some dependency issues as how pip works has changed.
- Python 2.x support not available. Python 3.x virtual environment is recommended for pip installation.
- Works with Windows as well, how, I don't provide support for Windows. See my comment on an issue here: https://github.com/jmscslgroup/bagpy/issues/2#issuecomment-710095649
- **Note: it is not compatible with ROS 2.**

## Installation
```
pip install bagpy
```

## Usage principle
The philosophy behind developing this project is to make everything as simple and less confusing as possible. 
As a result, there are not too many options or freedom of usage being provided by __`bagreader`__ class. 
If you need a wide variety of options, users can directly use ROS's `rosbag` python package or 
Robotics System Toolbox APIs provided by MATLAB. However, if you are looking for simplicity, __`bagpy`__ is an
elegant choice.

## Examples
See Notebooks and python scripts at https://github.com/jmscslgroup/bagpy/tree/master/notebook for examples.
