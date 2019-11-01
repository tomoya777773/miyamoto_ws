from std_msgs.msg import Int16
import rospy

def callback(msg):
    print msg.data

rospy.init_node('listener')
sub = rospy.Subscriber("/hockey_2/predict/place", Int16, callback)
rospy.spin()