vimjay
======

A graph based image processing and generation tool.

Nodes are processing blocks with inputs and outputs.  Currently inputs are not distinguished from outputs and all can be either, though in some nodes certain ios will be overwritten every update.

The current model is to only update nodes that are input ancestors to an active output, and that have dirty inputs.

Originally hosted on http://code.google.com/p/binarymillenium/source/browse/trunk/opencv/camthing on svn.

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
