|PyPI| |PyPIDownloads| |Travis|

bagpy - Reading rosbag files simplified
============================================================

.. image::  https://raw.githubusercontent.com/jmscslgroup/bagpy/master/banner_bagpy.png
   :width: 500px
   :align: left

**bagpy**, a python package provides specialized class ``bagreader`` to read and decode ROS messages from bagfiles in just a few lines of code.
**bagpy** further allows to decode custom ROS messages which other is difficult without the custom ros message definition.


|
|
|
|

Contributing to the project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Feel free to submit an `issue <https://github.com/jmscslgroup/bagpy/issues/new/choose>`_
or send us an `email <mailto:rahulbhadani@email.arizona.edu>`_. 
If you like to contribute to this project, please fork this repository to your GitHub account,
create a new branch for yourself and send a pull request for the merge. After reviewing the changes,
we will decide if this is a good place to add your changes.
Your help to improve **bagpy** is highly appreciated.

Licensing
^^^^^^^^^^

| License: MIT License 
| Copyright Rahul Bhadani, Jonathan Sprinkle, Arizona Board of Regents
| Initial Date: Nov 12, 2019
| Permission is hereby granted, free of charge, to any person obtaining 
| a copy of this software and associated documentation files 
| (the "Software"), to deal in the Software without restriction, including
| without limitation the rights to use, copy, modify, merge, publish,
| distribute, sublicense, and/or sell copies of the Software, and to 
| permit persons to whom the Software is furnished to do so, subject 
| to the following conditions:

| The above copyright notice and this permission notice shall be 
| included in all copies or substantial portions of the Software.

| THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF 
| ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
| TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
| PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT 
| SHALL THE AUTHORS, COPYRIGHT HOLDERS OR ARIZONA BOARD OF REGENTS
| BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN 
| AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
| OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
| OR OTHER DEALINGS IN THE SOFTWARE.

.. toctree::
   :caption: Main
   :maxdepth: 1
   :hidden:
    
   about
   getting_started
   installation

.. toctree::
   :caption: Tutorials
   :maxdepth: 1
   :hidden:

   Reading_bagfiles_from_cloud
   bagpy_example
   
.. toctree::
   :caption: API
   :maxdepth: 1
   :hidden:

   api_bagreader
   
.. toctree::
   :caption: Example Datasets
   :maxdepth: 1
   :hidden:

.. |PyPI| image:: https://img.shields.io/pypi/v/bagpy.svg
   :target: https://pypi.org/project/bagpy

.. |PyPIDownloads| image:: https://pepy.tech/badge/bagpy
   :target: https://pepy.tech/project/bagpy

.. |Travis| image:: https://travis-ci.com/jmscslgroup/bagpy.svg?branch=master
   :target: https://travis-ci.com/jmscslgroup/bagpy/
   

