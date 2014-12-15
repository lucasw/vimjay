import rospy
from std_msgs.msg import String

def key_callback(msg):
    print msg.data

rospy.init_node('test_key')
sub = rospy.Subscriber('/vimjay/key', String, key_callback, queue_size=1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
