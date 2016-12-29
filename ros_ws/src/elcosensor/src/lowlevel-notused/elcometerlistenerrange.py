#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range

def callback(data):
    rospy.loginfo( rospy.get_caller_id()+" measured %f", data.range*1000 )
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("elcometer", Range, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
