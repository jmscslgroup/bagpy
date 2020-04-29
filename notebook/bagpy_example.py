#!/usr/bin/env python
# coding: utf-8

# In[1]:


from bagpy import bagreader
import bagpy
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
b = bagreader('/home/ivory/CyverseData/JmscslgroupData/ARED/2016-07-28/data_by_test/Bag Files/test3_09-23-59.bag')
csv = b.laser_data()
df = pd.read_csv(csv[0])
ms = b.vel_data()
vel = pd.read_csv(ms[0])
s = b.std_data()
data = pd.read_csv(s[0])
odom = b.odometry_data()
odomdata = pd.read_csv(odom[0])
w = b.wrench_data()
wdata = pd.read_csv(w[0])       

bagpy.animate_timeseries(vel['Time'], vel['linear.x'], title='Velocity Timeseries Plot')



