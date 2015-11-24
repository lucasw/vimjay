# vimjay

A graph based image processing and generation tool.

Nodes are processing blocks with inputs and outputs.  Currently inputs are not distinguished from outputs and all can be either, though in some nodes certain ios will be overwritten every update.

The current model is to only update nodes that are input ancestors to an active output, and that have dirty inputs.

Originally hosted on http://code.google.com/p/binarymillenium/source/browse/trunk/opencv/camthing on svn.

## standalone tools

### USB camera input

#### libuvc_camera

Tried removing scanning_mode and now auto_exposure has a bad number in it.
```
rosrun dynamic_reconfigure dynparam get /usb_cam/uvc_camera
```

#### usb-cam vs. libuvc_camera

The former uses v4l rather than uvc, so won't implement uvc controls (it would be great to have standalone for that).
usb_cam seems more stable so far, doesn't require editing usb dev permissions and doesn't take down the linux usb camera modules (how to restore them?).

ros-jade-usb-cam

Launch a webcam:

```
rosrun libuvc_camera camera_node
```

rqt_image_view crashes when it tries to view /image_raw from this.

rosrun image_view image_view does work though.

### Ros-ified v4l2ucp

Using usb_cam seems fine, but would like to control it through ros with every control exposed just like v4l2ucp does.

The libuvc_camera cfg file has every possible control in it?
Can the controls actually use be set at run-time?

v4l2ucp source code:
http://sourceforge.net/p/v4l2ucp/git/ci/master/tree/src/

### image_deque

```
rosrun vimjay image_deque image:=/image_raw
rostopic pub -1 /single std_msgs/Bool True
```

## ROS conversion

This project is in the process of being converted to ros, which may substantially alter it to the point of being unrecognizable and it probably ought to be renamed.

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
