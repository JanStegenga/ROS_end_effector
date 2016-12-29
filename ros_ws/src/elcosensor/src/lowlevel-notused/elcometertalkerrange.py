#!/usr/bin/env python
import roslib;
import rospy;
import re;
from sensor_msgs.msg import Range
import serial

ser = serial.Serial( '/dev/ttyACM0', 9600 );
ser.setTimeout( 1 );
ser.write( '-' );	#puts elcometer in ASCII mode it seems
rospy.sleep(1.0);
ser.write( '*' );	#put elcometer into data transfer mode
rospy.sleep(1.0);
ser.write( '*' );	#gives identification string
rospy.sleep(1.0);
data_str = ser.readline();
print data_str		#print the identification string on screen
rospy.sleep(1.0);
ser.write( chr(27) );	#left arrow escapes from the data transfer screen
rospy.sleep(1.0);

def talker():
	pub = rospy.Publisher( 'elcometer', Range, queue_size = 10 );
	rospy.init_node( 'elcometer_talker', anonymous=True );
	while not rospy.is_shutdown():
		data_str = ser.readline();
		if 'mm' in data_str :
			data_num = float( data_str[0:data_str.find('mm')-1] );
			#print data_num
			ds = Range();
			#ds.header.seq = 0;
			ds.header.frame_id = '0';
			ds.header.stamp = rospy.Time.now();
			ds.radiation_type = Range.ULTRASOUND;
			ds.field_of_view = 0.1;
			ds.min_range = 0.0000000;
			ds.max_range = 0.0015000;
			ds.range = data_num/1000.0;
			rospy.loginfo(ds);
			pub.publish(ds);
		rospy.sleep(1.0);
	
	ser.close();

if __name__ == '__main__':
	try:
		talker();
	except rospy.ROSInterruptException:
		pass
