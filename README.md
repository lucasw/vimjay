vimjay
======

A graph based image processing and generation tool.

Nodes are processing blocks with inputs and outputs.  Currently inputs are not distinguished from outputs and all can be either, though in some nodes certain ios will be overwritten every update.

The current model is to only update nodes that are input ancestors to an active output, and that have dirty inputs.

Originally hosted on http://code.google.com/p/binarymillenium/source/browse/trunk/opencv/camthing on svn.

standalone tools
================

Launch a webcam:

```
rosrun libuvc_camera camera_node
```

rqt_image_view crashes when it tries to view /image_raw from this.

rosrun image_view image_view does work though.

image_deque
-----------

```
rosrun vimjay image_deque image:=/image_raw
rostopic pub -1 /single std_msgs/Bool True
```

ROS conversion
--------------

```
  :s/VLOG(\(.\{-}\)) << \(.*\);/ROS_DEBUG_STREAM_COND(log_level > \1, \2);/
```

Match across two lines (need to replace all the single line VLOGs first)

```
  :s/VLOG(\(.\{-}\)) << \(.*\n.*\);/ROS_DEBUG_STREAM_COND(log_level > \1, \2);/
```

Three lines (not sure how to generalize):
```
:s/VLOG(\(.\{-}\)) << \(.*\n.*\n.*\);/ROS_DEBUG_STREAM_COND(log_level > \1, \2);/
```

Replace errors:
```
:s/LOG(ERROR) << \(.*\);/ROS_ERROR_STREAM(\1);/
:s/LOG(INFO) << \(.*\);/ROS_INFO_STREAM(\1);/
```
