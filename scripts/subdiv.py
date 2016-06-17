#!/usr/bin/env python
# use input points to create subdivision shapes in 2D
# output them as an image, or rviz marker, or leave that
# to other node.

import numpy
import rospy

from geometry_msgs.msg import Point
from opencv_apps.msg import Point2D, Point2DArray
from visualization_msgs.msg import Marker

class SubDiv:
    def __init__(self):
        # TODO(lucasw) replace with dr
        self.subdivisions = rospy.get_param('~subdivisions', 2)
        self.curve_pub = rospy.Publisher('curve_points', Point2DArray, queue_size=1)
        self.marker_pub = rospy.Publisher('curve_points_marker', Marker, queue_size=1)
        self.point_sub = rospy.Subscriber('control_points', Point2DArray,
                                          self.control_points_callback, queue_size=1)

    def sub_point(self, p1, p2, t):
        pt = Point2D()
        pt.x = p1.x + (p2.x - p1.x) * t
        pt.y = p1.y + (p2.y - p1.y) * t
        return pt

    def subdivide(self, point_array):
        sub_points = Point2DArray()
        # TODO(lucasw) replace with numpy operations
        for i1 in range(len(point_array.points)):
            i2 = (i1 + 1) % len(point_array.points)
            p1 = point_array.points[i1]
            p2 = point_array.points[i2]
            sub_points.points.append(self.sub_point(p1, p2, 0.25))
            sub_points.points.append(self.sub_point(p1, p2, 0.75))
        return sub_points

    def control_points_callback(self, msg):
        subdiv_points = msg
        for i in range(self.subdivisions):
            subdiv_points = self.subdivide(subdiv_points)
        self.curve_pub.publish(subdiv_points)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "subdivision"
        marker.id = 0  # self.subdivisions
        marker.action = Marker.ADD
        marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = str(self.subdivisions)
        for pt in subdiv_points.points:
            gpt = Point()
            gpt.x = pt.x
            gpt.y = pt.y
            marker.points.append(gpt)
        marker.points.append(marker.points[0])
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('sub_div')
    sub_div = SubDiv()
    rospy.spin()
