#!/usr/bin/env python
from elcosensor.srv import *
import roslib;
import rospy;
import re;
from sensor_msgs.msg import Range
import serial

#global ds2
global ds2 
ds2 = Range();

#subscriber callback, modifies ds2 as soon as new data is published on the elcometer topic 
#i.e. (requires elcometertalkerrange.py to be active)
def listen_to_elcometer( data ):
	global ds2;
	rospy.loginfo( rospy.get_caller_id()+" measured %f", data.range );
	rospy.logwarn( rospy.get_caller_id()+" measured %f", data.range );
	ds2 = data;

#service callback, returns the latest measured value.
def handle_elcometer_server( Request ):
	print Request;
	rospy.loginfo( 'MeasureThickness: ' + str( Request ) + str( ds2.range ) );
	return ds2

def server():
	#set up a service
	rospy.init_node( 'elcometer_server', anonymous=True );
	s = rospy.Service( 'elcometerserver', MeasureThickness, handle_elcometer_server )
	print "elcometer server is active"
	rospy.loginfo( "elcometer server is active" );
	#try to integrate a listener 
	#rospy.init_node('listener', anonymous=True)
    	rospy.Subscriber("elcometer", Range, listen_to_elcometer)
	#spin it
	rospy.spin();

if __name__ == '__main__':
	try:
		server();
	except rospy.ROSInterruptException:
		pass



