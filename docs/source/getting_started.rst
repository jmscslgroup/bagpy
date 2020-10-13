Getting Started
---------------

The philosophy behind developing this project is to make everything as simple and less confusing as possible. If you are looking for simplicity, bagpy is an elegant choice for decoding 
ROS messages from bagfiles



bagpy workflow at a glance
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Import bagpy::

    import bagpy
    from bagpy import bagreader
    import pandas as pd
    import seaborn as sea
    import matplotlib.pyplot as plt
    import numpy as np

Read your data
''''''''''''''
Read your data file (.bag) using::


    b = bagreader('OS1-64_city1.bag') 
    

The object ``b`` stores data from bag file.


Specialized Message Retrieval
''''''''''''''''''''''''''''''

Some types such geometry_msgs/Twist that have common use in propagating velocity information can retrieved as::

    ms = b.vel_data()
    vel = pd.read_csv(ms[0])
    fig, ax = bagpy.create_fig(1)
    ax[0].scatter(x='Time', y='linear.x', data=vel)
    plt.show()
    
    
Specialized Message Visualization
''''''''''''''''''''''''''''''''''''''

Visualize some common messages as follows::

    b.plot_vel(save_fig=True)
    

Generic message retrieval
''''''''''''''''''''''''''''

ROS messages of any type and topic can be decoded as follows::

    csvfiles = []
    for t in b.topics:
        data = b.message_by_topic(t)
        csvfiles.append(data)
        
    print(csvfiles[0])
    data = pd.read_csv(csvfiles[0])


``data`` can be further used for downstream analysis.

