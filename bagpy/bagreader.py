#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved

import subprocess
import yaml
import os
import time

import rosbag
from std_msgs.msg import String, Header
from geometry_msgs.msg  import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

import numpy  as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sea
import plotnine as pn


class bagreader:
    '''
    `bagreader` class provides API to read rosbag files in an effective easy manner with significant hassle.
    This class is reimplementation of its MATLAB equivalent that can be found at https://github.com/jmscslgroup/ROSBagReader

    Parameters
    ----------------
    bagfile: `string`
        Bagreader constructor takes name of a bag file as an  argument. name of the bag file can be provided as the full  qualified path, relative path or just the file name.

    Attributes
    --------------
    bagfile: `string`
        Full path of the bag  file, e.g `/home/ece446/2019-08-21-22-00-00.bag`
    filename: `string`
        Name of the bag file, e.g. `2019-08-21-22-00-00.bag`
    dir: `string`
        Directory where bag file is located
    reader: `rosbag.Bag`
        rosbag.Bag object that 

    topic: `pandas dataframe`
        stores the available topic from bag file being read as a table
    n_messages: `integer`
        stores the number of messages
    message_types:`list`, `string`
        stores all the available message types
    datafolder: `string`
        stores the path/folder where bag file is present - may be relative to the bag file or full-qualified path.

        E.g. If bag file is at `/home/ece446/2019-08-21-22-00-00.bag`, then datafolder is `/home/ece446/2019-08-21-22-00-00/`

    message_dictionary: `dictionary`
        message_dictionary will be a python dictionary to keep track of what datafile have been generated mapped by types

    graycolordark: `tuple`
        dark gray color for timeseries plots

    graycolorlight: `tuple`
        light gray color for timeseries plots
    
    linecolor: `tuple`
        a set of line color for timeseries plots
    markercolor: `tuple`
        a set of marker color for timeseries plots

    Example
    ---------
    >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 

    '''

    def __init__(self, bagfile):
        self.bagfile = bagfile
        
        slashindices = find(bagfile, '/')
        
        # length of slashindices list will be zero if a user has pass only bag file name , e.g. 2020-03-04-12-22-42.bag
        if  len(slashindices) > 0:
            self.filename =bagfile[slashindices[-1]:]
            self.dir = bagfile[slashindices[0]:slashindices[-1]]
        else:
            self.filename = bagfile
            self.dir = './'

        self.reader = rosbag.Bag(self.bagfile)

        info = self.reader.get_type_and_topic_info() 
        self.topic_tuple = info.topics.values()
        self.topics = info.topics.keys()

        self.message_types = []
        for t1 in self.topic_tuple: self.message_types.append(t1.msg_type)

        self.n_messages = []
        for t1 in self.topic_tuple: self.n_messages.append(t1.message_count)

        self.frequency = []
        for t1 in self.topic_tuple: self.frequency.append(t1.frequency)

        self.topic_table = pd.DataFrame(list(zip(self.topics, self.message_types, self.n_messages, self.frequency)), columns=['Topics', 'Types', 'Message Count', 'Frequency'])

        self.start_time = self.reader.get_start_time()
        self.end_time = self.reader.get_end_time()

        self.datafolder = bagfile[0:-4]

        if os.path.exists(self.datafolder):
            print("[INFO]  Data folder {1} already exists. Not creating.".format(self.datafolder))
        else:
            try:
                os.mkdir(self.datafolder)
            except OSError:
                print("[ERROR] Failed to create the data folder {1}.".format(self.datafolder))
            else:
                print("[INFO]  Successfully created the data folder {1}.".format(self.datafolder))


        

    def laser_data(self, **kwargs):
        '''
        Class method `laser_data` extracts laser data from the given file, assuming laser data is of type `sensor_msgs/LaserScan`.

        Parameters
        -------------
        kwargs
            variable keyword arguments

            tstart: `double`
                The starting time from where to extract the laser data

                Default Value: None
            
            tend: `double`
                The end time upto where to extract the laser data

                Default Value: None

        Returns
        ---------
        `dictionary`
            A dictionary containing two keys: _csv_ and _pickle_. Corresponding values are a list laser data filepath extracted in the form of .csv and .pickle format.

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> laserdatafile = b.laser_data(tstart =0.2, tend = 230.0)
        >>> print(laserdatafile)

        '''

        tstart =None
        tend = None
        try:
            tstart = kwargs['tstart']
            if tstart < self.start_time:
                tstart = self.start_time
                print("[WARNING] desired start time `tstart` is less than start time of ROS bag, using start time of ROS bag" )
        except KeyError as e:
            pass
        
        try:
            tend = kwargs['tend']
            if tend < self.end_time:
                tend = self.end_time
                print("[WARNING] desired end time `tend` is greater than end time of ROS bag, using end time of ROS bag" )
        except KeyError as e:
            pass
        
        type_to_look ="sensor_msgs/LaserScan"
        table_rows = self.topic_table[self.topic_table['Types']==type_to_look]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        all_msg = []
        for i in range(len(table_rows)):
            msg_list = [LaserScan() for count in range(message_counts[i])]
            k = 0
            for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend): 
                msg_list[k] = msg

            all_msg.append(msg_list)

    def vel_data(self, **kwargs):
        pass

    def std_data(self, **kwargs):
        pass

    def compressed_images(self, **kwargs):
        pass

    def odometry_data(self, **kwargs):
        pass

    def wrench_data(self, **kwargs):
        pass

    def  clock_data(self, **kwargs):
        pass

    def pointcloud_data(self, **kwargs):
        pass

    def ts_plot_vel(self):
        pass

    def ts_plot_std(self):
        pass

    def ts_plot_odometry(self):
        pass

    def ts_wrench_vel(self):
        pass

    def ts_plot_vel(self):
        pass
    
    def animate_laser(self):
        pass

    def animate_pointcloud(self):
        pass

def find(s, ch):
    '''
    Function `find` returns indices all the occurence of `ch` in `s` 

    Parameters
    -------------
    s: `string`
        String or a setence where to search for occurrences of the character `ch`

    s: `char`
        Character to look for

    Returns
    ---------
    `list`
        List of indices of occurrences of character `ch` in the string `s`.

    '''
    return [i for i, ltr in enumerate(s) if ltr == ch]



    
    
    