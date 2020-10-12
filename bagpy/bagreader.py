#!/usr/bin/env python
# coding: utf-8

# Author : Rahul Bhadani
# Initial Date: March 2, 2020
# About: bagreader class to read  ros bagfile and extract relevant data
# License: MIT License

#   Permission is hereby granted, free of charge, to any person obtaining
#   a copy of this software and associated documentation files
#   (the "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject
#   to the following conditions:

#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.

#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
#   ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
#   TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#   PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
#   SHALL THE AUTHORS, COPYRIGHT HOLDERS OR ARIZONA BOARD OF REGENTS
#   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
#   AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
#   OR OTHER DEALINGS IN THE SOFTWARE.

__maintainer__ = 'Rahul Bhadani'
__email__  = 'rahulbhadani@email.arizona.edu'

import sys
import ntpath
import os
import time
from io import BytesIO
import csv
import inspect

import rosbag
from std_msgs.msg import String, Header
from geometry_msgs.msg  import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan


import numpy  as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sea
import pickle

class bagreader:
    '''
    `bagreader` class provides API to read rosbag files in an effective easy manner with significant hassle.
    This class is reimplementation of its MATLAB equivalent that can be found at https://github.com/jmscslgroup/ROSBagReader

    Parameters
    ----------------
    bagfile: `string`
        Bagreader constructor takes name of a bag file as an  argument. name of the bag file can be provided as the full  qualified path, relative path or just the file name.

    verbose: `bool`
        If True, prints some relevant information. Default: `True`
    
    tmp: `bool`
        If True, creates directory in /tmp folder. Default: `False`

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

    Example
    ---------
    >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 

    '''

    def __init__(self, bagfile, verbose=True, tmp = False):
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

        if tmp:
            self.datafolder = '/tmp/' + bagfile.split('/')[-1][0:-4]

        self.verbose = verbose

        if os.path.exists(self.datafolder):
            if self.verbose:
                print("[INFO]  Data folder {0} already exists. Not creating.".format(self.datafolder))
        else:
            try:
                os.mkdir(self.datafolder)
            except OSError:
                print("[ERROR] Failed to create the data folder {0}.".format(self.datafolder))
            else:
                if self.verbose:
                    print("[INFO]  Successfully created the data folder {0}.".format(self.datafolder))
 
    def message_by_topic(self, topic):
        '''
        Class method `message_by_topic` to extract message from the ROS Bag by topic name `topic`

        Parameters
        ---------------
        topic: `str`
            
            Topic from which to extract the message.

        Returns
        ---------
        `list`
            A list of messages extracted from the bag file by the topic name

        Example
        -----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> msg = b.message_by_topic(topic='/catvehicle/vel')

        '''

        msg_list = []
        tstart =None
        tend = None
        for topic, msg, t in self.reader.read_messages(topics=topic, start_time=tstart, end_time=tend): 
            msg_list.append(msg)

        return msg_list

    def laser_data(self, **kwargs):
        '''
        Class method `laser_data` extracts laser data from the given file, assuming laser data is of type `sensor_msgs/LaserScan`.

        Parameters
        -------------
        kwargs
            variable keyword arguments

        Returns
        ---------
        `list`
            A list of strings. Each string will correspond to file path of CSV file that contains extracted data of laser scan type

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> laserdatafile = b.laser_data()
        >>> print(laserdatafile)

        '''
        tstart =None
        tend = None
        
        type_to_look ="sensor_msgs/LaserScan"
        table_rows = self.topic_table[self.topic_table['Types']==type_to_look]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        column_names = ["Time",
                                "header.seq", 
                                "header.frame_id", 
                                "angle_min" , 
                                "angle_max", 
                                "angle_increment", 
                                "time_increment", 
                                "scan_time", 
                                "range_min", 
                                "range_max"]

        for p in range(0, 182):
            column_names.append("ranges_" + str(p))
        for p in range(0, 182):
            column_names.append("intensities_" + str(p))

        all_msg = []
        csvlist = []
        for i in range(len(table_rows)):
            tempfile = self.datafolder + "/" + topics_to_read[i].replace("/", "-") + ".csv"
            file_to_write = ntpath.dirname(tempfile) + '/' + ntpath.basename(tempfile)[1:]
            #msg_list = [LaserScan() for count in range(message_counts[i])]
            k = 0

            if sys.hexversion >= 0x3000000:
                opencall = open(file_to_write, "w", newline='')
            else:
                opencall = open(file_to_write, 'wb')

            with opencall as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(column_names) # write the header
                for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend): 
                    #msg_list[k] = msg
                    
                    new_row = [t.secs + t.nsecs*1e-9, 
                                            msg.header.seq, 
                                            msg.header.frame_id, 
                                            msg.angle_min,
                                            msg.angle_max, 
                                            msg.angle_increment, 
                                            msg.time_increment, 
                                            msg.scan_time,  
                                            msg.range_min, 
                                            msg.range_max]

                    ranges = [None]*182
                    intensities = [None]*182

                    for ir, ran in enumerate(msg.ranges):
                        ranges[ir] = ran

                    for ir, ran in enumerate(msg.intensities):
                        intensities[ir] = ran

                    new_row  = new_row + ranges
                    new_row = new_row + intensities
                    writer.writerow(new_row)
                
                k = k + 1

            csvlist.append(file_to_write)
        return csvlist

    def vel_data(self, **kwargs):
        '''
        Class method `vel_data` extracts velocity data from the given file, assuming laser data is of type `geometry_msgs/Twist`.

        Parameters
        -------------
        kwargs
            variable keyword arguments

        Returns
        ---------
        `list`
            A list of strings. Each string will correspond to file path of CSV file that contains extracted data of geometry_msgs/Twist type

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> veldatafile = b.vel_data()
        >>> print(veldatafile)

        '''
        tstart = None
        tend = None
        
        type_to_look ="geometry_msgs/Twist"
        table_rows = self.topic_table[self.topic_table['Types']==type_to_look]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        column_names = ["Time",
                                "linear.x", 
                                "linear.y", 
                                "linear.z" , 
                                "angular.x", 
                                "angular.y", 
                                "angular.z"]

        csvlist = []
        for i in range(len(table_rows)):
            tempfile = self.datafolder + "/" + topics_to_read[i].replace("/", "-") + ".csv"
            file_to_write = ntpath.dirname(tempfile) + '/' + ntpath.basename(tempfile)[1:]
            k = 0

            if sys.hexversion >= 0x3000000:
                opencall = open(file_to_write, "w", newline='')
            else:
                opencall = open(file_to_write, 'wb')

            with opencall as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(column_names) # write the header
                for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend):
                    
                    new_row = [t.secs + t.nsecs*1e-9, 
                                            msg.linear.x, 
                                            msg.linear.y,
                                            msg.linear.z,
                                            msg.angular.x,
                                            msg.angular.y,
                                            msg.angular.z]

                    writer.writerow(new_row)
                
                k = k + 1

            csvlist.append(file_to_write)
        return csvlist

    def std_data(self, **kwargs):
        '''
        Class method `std_data` extracts velocity data from the given file, assuming laser data is of type `std_msgs/{bool, byte, Float32, Float64, Int16, Int32, Int8, UInt16, UInt32, UInt64, UInt8}` of 1-dimension.

        Parameters
        -------------
        kwargs
            variable keyword arguments

        Returns
        ---------
        `list`
            A list of strings. Each string will correspond to file path of CSV file that contains extracted data of `std_msgs/{bool, byte, Float32, Float64, Int16, Int32, Int8, UInt16, UInt32, UInt64, UInt8}`

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> stddatafile = b.std_data()
        >>> print(stddatafile)

        '''
        tstart = None
        tend = None
        
        type_to_look =["std_msgs/Bool", "'std_msgs/Byte", "std_msgs/Float32", "std_msgs/Float64",
                                    "std_msgs/Int8", "std_msgs/Int16", "std_msgs/Int32",
                                    "std_msgs/Uint8", "std_msgs/Uint16", "std_msgs/Uint32"]

        table_rows = self.topic_table[self.topic_table['Types'].isin(type_to_look)]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        column_names = ["Time", "data"]

        csvlist = []
        for i in range(len(table_rows)):
            tempfile = self.datafolder + "/" + topics_to_read[i].replace("/", "-") + ".csv"
            file_to_write = ntpath.dirname(tempfile) + '/' + ntpath.basename(tempfile)[1:]
            k = 0

            if sys.hexversion >= 0x3000000:
                opencall = open(file_to_write, "w", newline='')
            else:
                opencall = open(file_to_write, 'wb')

            with opencall as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(column_names) # write the header
                for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend):
                    
                    new_row = [t.secs + t.nsecs*1e-9, 
                                            msg.data]

                    writer.writerow(new_row)
                
                k = k + 1

            csvlist.append(file_to_write)
        return csvlist

    def compressed_images(self, **kwargs):
        raise NotImplementedError("To be implemented")

    def odometry_data(self, **kwargs):
        '''
        Class method `odometry_data` extracts velocity data from the given file, assuming laser data is of type `nav_msgs/Odometry`.

        Parameters
        -------------
        kwargs
            variable keyword arguments

        Returns
        ---------
        `list`
            A list of strings. Each string will correspond to file path of CSV file that contains extracted data of nav_msgs/Odometry type

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> odomdatafile = b.odometry_data()
        >>> print(odomdatafile)

        '''
        tstart = None
        tend = None
        
        type_to_look ="nav_msgs/Odometry"
        table_rows = self.topic_table[self.topic_table['Types']==type_to_look]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        column_names = ["Time",
                                "header.seq", 
                                "header.frame_id", 
                                "child_frame_id",
                                "pose.x" , 
                                "pose.y", 
                                "pose.z", 
                                "orientation.x", 
                                "orientation.y", 
                                "orientation.z",
                                "orientation.w",
                                "linear.x",
                                "linear.y",
                                "linear.z",
                                "angular.x",
                                "angular.y",
                                "angular.z"]

        csvlist = []
        for i in range(len(table_rows)):
            tempfile = self.datafolder + "/" + topics_to_read[i].replace("/", "-") + ".csv"
            file_to_write = ntpath.dirname(tempfile) + '/' + ntpath.basename(tempfile)[1:]
            k = 0

            if sys.hexversion >= 0x3000000:
                opencall = open(file_to_write, "w", newline='')
            else:
                opencall = open(file_to_write, 'wb')

            with opencall as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(column_names) # write the header
                for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend):
                    new_row = [t.secs + t.nsecs*1e-9, 
                                            msg.header.seq, 
                                            msg.header.frame_id,
                                            msg.child_frame_id,
                                            msg.pose.pose.position.x,
                                            msg.pose.pose.position.y,
                                            msg.pose.pose.position.z,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w,
                                            msg.twist.twist.linear.x,
                                            msg.twist.twist.linear.y,
                                            msg.twist.twist.linear.z]

                    writer.writerow(new_row)
                
                k = k + 1

            csvlist.append(file_to_write)
        return csvlist

    def wrench_data(self, **kwargs):
        '''
        Class method `wrench_data` extracts velocity data from the given file, assuming laser data is of type `geometry_msgs/Wrench`.

        Parameters
        -------------
        kwargs
            variable keyword arguments

        Returns
        ---------
        `list`
            A list of strings. Each string will correspond to file path of CSV file that contains extracted data of geometry_msgs/Wrench type

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> wrenchdatafile = b.wrench_data()
        >>> print(wrenchdatafile)

        '''
        tstart = None
        tend = None
        
        type_to_look ="geometry_msgs/Wrench"
        table_rows = self.topic_table[self.topic_table['Types']==type_to_look]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        column_names = ["Time",
                                "force.x", 
                                "force.y", 
                                "force.z" , 
                                "torque.x", 
                                "torque.y", 
                                "torque.z"]

        csvlist = []
        for i in range(len(table_rows)):
            tempfile = self.datafolder + "/" + topics_to_read[i].replace("/", "-") + ".csv"
            file_to_write = ntpath.dirname(tempfile) + '/' + ntpath.basename(tempfile)[1:]
            k = 0

            if sys.hexversion >= 0x3000000:
                opencall = open(file_to_write, "w", newline='')
            else:
                opencall = open(file_to_write, 'wb')

            with opencall as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(column_names) # write the header
                for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend):
                    
                    new_row = [t.secs + t.nsecs*1e-9, 
                                            msg.force.x, 
                                            msg.force.y,
                                            msg.force.z,
                                            msg.torque.x,
                                            msg.torque.y,
                                            msg.torque.z]

                    writer.writerow(new_row)
                
                k = k + 1

            csvlist.append(file_to_write)
        return csvlist

    def  clock_data(self, **kwargs):
        '''
        Class method `vel_data` extracts velocity data from the given file, assuming laser data is of type `rosgraph_msgs/Clock`.

        Parameters
        -------------
        kwargs
            variable keyword arguments

        Returns
        ---------
        `list`
            A list of strings. Each string will correspond to file path of CSV file that contains extracted data of rosgraph_msgs/Clock type

        Example
        ----------
        >>> b = bagreader('/home/ivory/CyverseData/ProjectSparkle/sparkle_n_1_update_rate_100.0_max_update_rate_100.0_time_step_0.01_logtime_30.0_2020-03-01-23-52-11.bag') 
        >>> clockdatafile = b.clock_data()
        >>> print(clockdatafile)

        '''
        tstart = None
        tend = None
        
        type_to_look ="rosgraph_msgs/Clock"
        table_rows = self.topic_table[self.topic_table['Types']==type_to_look]
        topics_to_read = table_rows['Topics'].values
        message_counts = table_rows['Message Count'].values
        
        column_names = ["Time",
                                "clock.secs", 
                                "clock.nsecs"]

        csvlist = []
        for i in range(len(table_rows)):
            tempfile = self.datafolder + "/" + topics_to_read[i].replace("/", "-") + ".csv"
            file_to_write = ntpath.dirname(tempfile) + '/' + ntpath.basename(tempfile)[1:]

            k = 0

            if sys.hexversion >= 0x3000000:
                opencall = open(file_to_write, "w", newline='')
            else:
                opencall = open(file_to_write, 'wb')

            with opencall as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(column_names) # write the header
                for topic, msg, t in self.reader.read_messages(topics=topics_to_read[i], start_time=tstart, end_time=tend):
                    new_row = [t.secs + t.nsecs*1e-9, 
                                            msg.clock.secs, 
                                            msg.clock.nsecs]

                    writer.writerow(new_row)
                
                k = k + 1

            csvlist.append(file_to_write)
        return csvlist

    def pointcloud_data(self, **kwargs):
        raise NotImplementedError("To be implemented")

    def plot_vel(self, save_fig = False):
        '''
        `plot_vel` plots the timseries velocity data
        
        Parameters
        -------------
        save_fig: `bool`

        If `True` figures are saved in the data directory.

        '''
        import IPython 
        shell_type = IPython.get_ipython().__class__.__name__

        if shell_type == 'ZMQInteractiveShell':
            IPython.get_ipython().run_line_magic('matplotlib', 'inline')

        csvfiles = self.vel_data()
        
        dataframes = [None]*len(csvfiles)

        # read the csvfiles into pandas dataframe
        for i, csv in enumerate(csvfiles):
            df = pd.read_csv(csv)
            dataframes[i] = df

        fig, axs = create_fig(len(csvfiles))

        for i, df in enumerate(dataframes):
            axs[i].scatter(x = 'Time', y='linear.x', data=df, marker='D',  linewidth=0.3, s = 9, color="#2E7473")
            axs[i].scatter(x = 'Time', y='linear.y', data=df, marker='s',  linewidth=0.3, s = 9, color="#EE5964")
            axs[i].scatter(x = 'Time', y='linear.z', data=df, marker='p',  linewidth=0.3, s = 9, color="#ED9858")
            axs[i].scatter(x = 'Time', y='angular.x', data=df, marker='P',  linewidth=0.3, s = 9, color="#1c54b2")
            axs[i].scatter(x = 'Time', y='angular.y', data=df, marker='*',  linewidth=0.3, s = 9, color="#004F4A")
            axs[i].scatter(x = 'Time', y='angular.z', data=df, marker='8',  linewidth=0.3, s = 9, color="#4F4A00")
            axs[i].legend(df.columns.values[1:])

            if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=16)
                axs[i].set_xlabel('Time', fontsize=14)
                axs[i].set_ylabel('Messages', fontsize=14)
            else:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=12)
                axs[i].set_xlabel('Time', fontsize=10)
                axs[i].set_ylabel('Messages', fontsize=10)
        fig.tight_layout()
        if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
            fig.suptitle("Velocity Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 14, y = 1.02)
        else:
             fig.suptitle("Velocity Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 10, y = 1.02)

        if save_fig:
            current_fig = plt.gcf()
            fileToSave = self.datafolder + "/" + _get_func_name()

            with open(fileToSave + ".pickle", 'wb') as f:
                pickle.dump(fig, f) 
            current_fig.savefig(fileToSave + ".pdf", dpi = 100) 
            current_fig.savefig(fileToSave + ".png", dpi = 100) 

        plt.show()

    def plot_std(self, save_fig = False):
        '''
        `plot_std` plots the timseries standard Messages such as  `std_msgs/{bool, byte, Float32, Float64, Int16, Int32, Int8, UInt16, UInt32, UInt64, UInt8}` of 1-dimension
        
        Parameters
        -------------
        save_fig: `bool`

        If `True` figures are saved in the data directory.
        '''
        import IPython 
        shell_type = IPython.get_ipython().__class__.__name__

        if shell_type == 'ZMQInteractiveShell':
            IPython.get_ipython().run_line_magic('matplotlib', 'inline')

        csvfiles = self.std_data()
        
        dataframes = [None]*len(csvfiles)

        # read the csvfiles into pandas dataframe
        for i, csv in enumerate(csvfiles):
            df = pd.read_csv(csv)
            dataframes[i] = df

        fig, axs = create_fig(len(csvfiles))

        if len(csvfiles) == 0:
            print("No standard data found")
            return

        for i, df in enumerate(dataframes):
            axs[i].scatter(x = 'Time', y='data', data=df, marker='D',  linewidth=0.3, s = 9, color="#2E7473")
            axs[i].legend(df.columns.values[1:])
            if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=16)
                axs[i].set_xlabel('Time', fontsize=14)
                axs[i].set_ylabel('Messages', fontsize=14)
            else:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=12)
                axs[i].set_xlabel('Time', fontsize=10)
                axs[i].set_ylabel('Messages', fontsize=10)

        if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
            fig.suptitle("Standard Messages Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 14, y = 1.02)
        else:
             fig.suptitle("Standard Messages Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 10, y = 1.02)
        fig.tight_layout()
        if save_fig:
            current_fig = plt.gcf()
            fileToSave = self.datafolder + "/" + _get_func_name()

            with open(fileToSave + ".pickle", 'wb') as f:
                pickle.dump(fig, f) 
            current_fig.savefig(fileToSave + ".pdf", dpi = 300) 
        
        plt.show()

    def plot_odometry(self, save_fig = False):
        '''
        `plot_odometry` plots the timseries odometry data
        
        Parameters
        -------------
        save_fig: `bool`

        If `True` figures are saved in the data directory.
        '''
        import IPython 
        shell_type = IPython.get_ipython().__class__.__name__

        if shell_type == 'ZMQInteractiveShell':
            IPython.get_ipython().run_line_magic('matplotlib', 'inline')

        csvfiles = self.odometry_data()
        
        dataframes = [None]*len(csvfiles)

        # read the csvfiles into pandas dataframe
        for i, csv in enumerate(csvfiles):
            df = pd.read_csv(csv)
            dataframes[i] = df

        fig, axs = create_fig(len(csvfiles))     
      
        for i, df in enumerate(dataframes):
            axs[i].scatter(x = 'Time', y='pose.x', data=df, marker='D',  linewidth=0.3,s = 9, color="#2E7473")
            axs[i].scatter(x = 'Time', y='pose.y', data=df, marker='D',  linewidth=0.3, s = 9, color="#EE5964")
            axs[i].scatter(x = 'Time', y='pose.z', data=df, marker='D',  linewidth=0.3, s = 9, color="#ED9858")
            axs[i].scatter(x = 'Time', y='orientation.x', data=df, marker='*',  linewidth=0.3, s = 9, color="#1c54b2")
            axs[i].scatter(x = 'Time', y='orientation.y', data=df, marker='*',  linewidth=0.3, s = 9, color="#004F4A")
            axs[i].scatter(x = 'Time', y='orientation.z', data=df, marker='8',  linewidth=0.3, s = 9, color="#4F4A00")
            axs[i].scatter(x = 'Time', y='orientation.w', data=df, marker='8',  linewidth=0.3, s = 9, color="#004d40")
            axs[i].scatter(x = 'Time', y='linear.x', data=df, marker='s',  linewidth=0.3, s = 9, color="#ba68c8")
            axs[i].scatter(x = 'Time', y='linear.y', data=df, marker='s',  linewidth=0.3, s = 9, color="#2C0C32")
            axs[i].scatter(x = 'Time', y='linear.z', data=df, marker='P',  linewidth=0.3, s = 9, color="#966851")
            axs[i].scatter(x = 'Time', y='angular.x', data=df, marker='P', linewidth=0.3, s = 9, color="#517F96")
            axs[i].scatter(x = 'Time', y='angular.y', data=df, marker='p', linewidth=0.3, s = 9, color="#B3C1FC")
            axs[i].scatter(x = 'Time', y='angular.z', data=df, marker='p', linewidth=0.3, s = 9, color="#FCEFB3")
            axs[i].legend(df.columns.values[4:])
            if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=16)
                axs[i].set_xlabel('Time', fontsize=14)
                axs[i].set_ylabel('Messages', fontsize=14)
            else:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=12)
                axs[i].set_xlabel('Time', fontsize=10)
                axs[i].set_ylabel('Messages', fontsize=10)

        if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
            fig.suptitle("Odometry Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 14, y = 1.02)
        else:
             fig.suptitle("Odometry Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 10, y = 1.02)
        fig.tight_layout()
        if save_fig:
            current_fig = plt.gcf()
            fileToSave = self.datafolder + "/" + _get_func_name()

            with open(fileToSave + ".pickle", 'wb') as f:
                pickle.dump(fig, f) 
            current_fig.savefig(fileToSave + ".pdf", dpi = 100) 
            current_fig.savefig(fileToSave + ".png", dpi = 100) 

        plt.show()

    def plot_wrench(self, save_fig = False):
        '''
        `plot_wrench` plots the timseries wrench data
        
        Parameters
        -------------
        save_fig: `bool`

        If `True` figures are saved in the data directory.
        '''

        import IPython 
        shell_type = IPython.get_ipython().__class__.__name__

        if shell_type == 'ZMQInteractiveShell':
            IPython.get_ipython().run_line_magic('matplotlib', 'inline')

        csvfiles = self.wrench_data()
        
        dataframes = [None]*len(csvfiles)

        # read the csvfiles into pandas dataframe
        for i, csv in enumerate(csvfiles):
            df = pd.read_csv(csv)
            dataframes[i] = df

        fig, axs = create_fig(len(csvfiles))

        for i, df in enumerate(dataframes):
            axs[i].scatter(x = 'Time', y='force.x', data=df, marker='D',  linewidth=0.3, s = 9, color="#2E7473")
            axs[i].scatter(x = 'Time', y='force.y', data=df, marker='s',  linewidth=0.3, s = 9, color="#EE5964")
            axs[i].scatter(x = 'Time', y='force.z', data=df, marker='*',  linewidth=0.3, s = 9, color="#ED9858")
            axs[i].scatter(x = 'Time', y='torque.x', data=df, marker='P',  linewidth=0.3, s = 9, color="#1c54b2")
            axs[i].scatter(x = 'Time', y='torque.y', data=df, marker='p',  linewidth=0.3, s = 9, color="#004F4A")
            axs[i].scatter(x = 'Time', y='torque.z', data=df, marker='8',  linewidth=0.3, s = 9, color="#4F4A00")
            axs[i].legend(df.columns.values[1:])
            if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=16)
                axs[i].set_xlabel('Time', fontsize=14)
                axs[i].set_ylabel('Messages', fontsize=14)
            else:
                axs[i].set_title(ntpath.basename(csvfiles[i]), fontsize=12)
                axs[i].set_xlabel('Time', fontsize=10)
                axs[i].set_ylabel('Messages', fontsize=10)

        if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:
            fig.suptitle("Wrench Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 14, y = 1.02)
        else:
             fig.suptitle("Wrench Timeseries Plot\n"+ntpath.dirname(csvfiles[0]), fontsize = 10, y = 1.02)
        fig.tight_layout()
        if save_fig:
            current_fig = plt.gcf()
            fileToSave = self.datafolder + "/" + _get_func_name()

            with open(fileToSave + ".pickle", 'wb') as f:
                pickle.dump(fig, f) 
            current_fig.savefig(fileToSave + ".pdf", dpi = 300) 

        plt.show()

    def animate_laser(self):
        raise NotImplementedError("To be implemented")

    def animate_pointcloud(self):
        raise NotImplementedError("To be implemented")


def _get_func_name():
    return inspect.stack()[1][3]

def animate_timeseries(time, message, **kwargs):
    '''
    `animate_timeseries` will animate a time series data. Time and Message pandas series are expected
    
    
    Parameters
    ----------
    
    time: `pandas.core.series.Series`
        Time Vector in the form of Pandas Timeseries
        
    message: `pandas.core.series.Series`
        Message Vector in the form of Pandas Timeseries
        
    
    kwargs: variable keyword arguments
            
        title: `str`

            Title of the plot. By Default, it is `Timeseries Plot`
            
    '''
    
    
    import IPython 
    shell_type = IPython.get_ipython().__class__.__name__
    
    
    assert (len(time) == len(message)), ("Time and Message Vector must be of same length. Current Length of Time Vector: {0}, Current Length of Message Vector: {0}".format(len(time), len(message)))
    
    plot_title = 'Timeseries Plot'
    try:
        plot_title = kwargs["title"]
    except KeyError as e:
        pass

    fig, ax = create_fig(1)
    ax = ax[0]
    plt.style.use('ggplot')
    plt.rcParams['figure.figsize'] = [15, 10]
    plt.rcParams['font.size'] = 16.0
    plt.rcParams['legend.fontsize'] = 14.0
    plt.rcParams['xtick.labelsize'] = 14.0
    plt.rcParams['ytick.labelsize'] = 14.0
    plt.rcParams['legend.markerscale']  = 2.0

    if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:

        if shell_type == 'ZMQInteractiveShell':
            IPython.get_ipython().run_line_magic('matplotlib', 'inline')
        
        print('Warning: Animation is being executed in IPython/Jupyter Notebook. Animation may not be real-time.')
        l, = ax.plot([np.min(time),np.max(time)],[np.min(message),np.max(message)], alpha=0.6, 
                     marker='o', markersize=5, linewidth=0, markerfacecolor='#275E56')


        def animate(i):

            l.set_data(time[:i], message[:i])
            ax.set_xlabel('Time', fontsize=15)
            ax.set_ylabel('Message', fontsize=15)
            ax.set_title(plot_title, fontsize=16)

        for index in range(len(message)-1):
            animate(index)
            IPython.display.clear_output(wait=True)
            display(fig)
            plt.pause(time[index + 1] - time[index])

    else:
        for index in range(0, len(message)-1):
            ax.clear()
            if index < 500:
                sea.lineplot(time[:index], message[:index],  linewidth=2.0, color="#275E56")
            else:
                sea.lineplot(time[index - 500:index], message[index - 500:index],  linewidth=2.0, color="#275E56")
            ax.set_title(plot_title, fontsize=16)
            ax.set_xlabel('Time', fontsize=15)
            ax.set_ylabel('Message', fontsize=15)
            plt.draw()
            plt.pause(time[index + 1] - time[index])
            

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

def timeindex(df, inplace=False):
    '''
    Convert multi Dataframe of which on column must be 'Time'  to pandas-compatible timeseries where timestamp is used to replace indices

    Parameters
    --------------

    df: `pandas.DataFrame`
        A pandas dataframe with two columns with the column names "Time" and "Message"

    inplace: `bool`
        Modifies the actual dataframe, if true, otherwise doesn't.

    Returns
    -----------
    `pandas.DataFrame`
        Pandas compatible timeseries with a single column having column name "Message" where indices are timestamp in hum  an readable format.
    '''
    
    if inplace:
        newdf = df
    else:
        newdf =df.copy(deep = True)

    newdf['Time'] = df['Time']
    Time = pd.to_datetime(newdf['Time'], unit='s')
    newdf['Clock'] = pd.DatetimeIndex(Time)
    
    if inplace:
        newdf.set_index('Clock', inplace=inplace)
    else:
        newdf = newdf.set_index('Clock')
    return newdf

def _setplots(**kwargs):
    import IPython 
    
    shell_type = IPython.get_ipython().__class__.__name__

    ncols = 1
    nrows= 1
    if kwargs.get('ncols'):
        ncols = kwargs['ncols']

    if kwargs.get('nrows'):
        nrows = kwargs['nrows']

    if shell_type in ['ZMQInteractiveShell', 'TerminalInteractiveShell']:

        plt.style.use('default')
        plt.rcParams['figure.figsize'] = [15*ncols, 6*nrows]
        plt.rcParams['font.size'] = 22.0 + 3*(ncols-1)+ 2*(nrows - 1)
        plt.rcParams['figure.facecolor'] = '#ffffff'
        #plt.rcParams[ 'font.family'] = 'Roboto'
        #plt.rcParams['font.weight'] = 'bold'
        plt.rcParams['xtick.color'] = '#828282'
        plt.rcParams['xtick.minor.visible'] = True
        plt.rcParams['ytick.minor.visible'] = True
        plt.rcParams['xtick.labelsize'] = 16 + 2*(ncols-1)+ 2*(nrows - 1)
        plt.rcParams['ytick.labelsize'] = 16 + 2*(ncols-1)+ 2*(nrows - 1)
        plt.rcParams['ytick.color'] = '#828282'
        plt.rcParams['axes.labelcolor'] = '#000000'
        plt.rcParams['text.color'] = '#000000'
        plt.rcParams['axes.labelcolor'] = '#000000'
        plt.rcParams['grid.color'] = '#cfcfcf'
        plt.rcParams['axes.labelsize'] = 20+ 3*(ncols-1)+ 2*(nrows - 1)
        plt.rcParams['axes.titlesize'] = 25+ 3*(ncols-1)+ 2*(nrows - 1)
        #plt.rcParams['axes.labelweight'] = 'bold'
        #plt.rcParams['axes.titleweight'] = 'bold'
        plt.rcParams["figure.titlesize"] = 30.0 + 4*(ncols-1) + 2*(nrows - 1)
        #plt.rcParams["figure.titleweight"] = 'bold'

        plt.rcParams['legend.markerscale']  = 2.0
        plt.rcParams['legend.fontsize'] = 10.0 + 3*(ncols-1)+ 2*(nrows - 1)
        plt.rcParams["legend.framealpha"] = 0.5
        
    else:
        plt.style.use('default')
        plt.rcParams['figure.figsize'] = [18*ncols, 6*nrows]
        plt.rcParams['font.size'] = 12.0
        plt.rcParams['figure.facecolor'] = '#ffffff'
        #plt.rcParams[ 'font.family'] = 'Roboto'
        #plt.rcParams['font.weight'] = 'bold'
        plt.rcParams['xtick.color'] = '#828282'
        plt.rcParams['xtick.minor.visible'] = True
        plt.rcParams['ytick.minor.visible'] = True
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['ytick.color'] = '#828282'
        plt.rcParams['axes.labelcolor'] = '#000000'
        plt.rcParams['text.color'] = '#000000'
        plt.rcParams['axes.labelcolor'] = '#000000'
        plt.rcParams['grid.color'] = '#cfcfcf'
        plt.rcParams['axes.labelsize'] = 10
        plt.rcParams['axes.titlesize'] = 10
        #plt.rcParams['axes.labelweight'] = 'bold'
        #plt.rcParams['axes.titleweight'] = 'bold'
        plt.rcParams["figure.titlesize"] = 24.0
        #plt.rcParams["figure.titleweight"] = 'bold'
        plt.rcParams['legend.markerscale']  = 1.0
        plt.rcParams['legend.fontsize'] = 8.0
        plt.rcParams["legend.framealpha"] = 0.5
        

def create_fig(num_of_subplots=1, **kwargs):

    import IPython 
    shell_type = IPython.get_ipython().__class__.__name__


    nrows = num_of_subplots
    ncols = 1
    
    if kwargs.get('ncols'):
        ncols = kwargs['ncols']
    
    if kwargs.get('nrows'):
        nrows = kwargs['nrows']
    
    _setplots(ncols=ncols, nrows=nrows)
    fig, ax = plt.subplots(ncols=ncols, nrows=nrows)
    

    if nrows == 1 and ncols == 1:
        ax_ = []
        ax_.append(ax)
        ax = ax_
    else:
        ax = ax.ravel()

    if sys.hexversion >= 0x3000000:
        for a in ax:
            a.minorticks_on()
            a.grid(which='major', linestyle='-', linewidth='0.25', color='dimgray')
            a.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
            a.patch.set_facecolor('#efefef')
            a.spines['bottom'].set_color('#828282')
            a.spines['top'].set_color('#828282')
            a.spines['right'].set_color('#828282')
            a.spines['left'].set_color('#828282')
    else:
        for a in ax:
            a.minorticks_on()
            a.grid(True, which='both')
            
    fig.tight_layout(pad=1.0*nrows)
    return fig, ax


def set_colorbar(fig, ax, im, label):
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes
    axins1 = inset_axes(ax,
                width="50%",  # width = 50% of parent_bbox width
                height="3%",  # height : 5%
                loc='upper right')
    cbr = fig.colorbar(im, ax=ax, cax=axins1, orientation="horizontal")
    cbr.set_label(label, fontsize = 20)

    return cbr