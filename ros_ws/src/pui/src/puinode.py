#!/usr/bin/env python
import roslib
import rospy
import socket	
import zmq
import time	
import sys		
import rsClientRequest_pb2
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from rosgraph_msgs.msg import Log
from elcosensor.srv import *
from end_effector.srv import *
#from highlevel.srv import *


#try to do this using classes
global pubmouseleft 	   
global pubmouseright 	 
global pubbuttonleft 	  
global pubbuttonright	
pubmouseleft 	   	= rospy.Publisher( 'puileftmouse', TwistStamped, queue_size = 10 )
pubmouseright 	   	= rospy.Publisher( 'puirightmouse', TwistStamped, queue_size = 10 )
pubbuttonleft 	   	= rospy.Publisher( 'puileftbutton', Vector3Stamped, queue_size = 10 )
pubbuttonright	   	= rospy.Publisher( 'puirightbutton', Vector3Stamped, queue_size = 10 )

#define a response:
global response
Response 				     = rsClientRequest_pb2.ServerResponse()
Response.id				     = 'empty'
Response.status.currentSystemMode            = 2
Response.status.vehicleBatteryHealth         = 100
Response.status.vehicleClampStatus           = True
Response.status.vehicleCameraStatus          = 2
Response.status.manipulatorMagnetStatus      = True
Response.status.manipulatorMagnetLatchStatus = True
Response.status.layerThicknessMeasurement    = 100
Response.status.vehiclePositionAlongRail     = 500
Response.status.vehicleSpeed                 = 10
Response.status.errorId                      = 4
Response.vehicleOrientation.manipulator1Angle= 60	#ID 10    rotation at base in degrees
Response.vehicleOrientation.manipulator2Angle= 45	#ID 20&21 arm lifting motors (shoulder)
Response.vehicleOrientation.manipulator3Angle= 0	#ID 30    rotation of upper arm
Response.vehicleOrientation.manipulator4Angle= 270	#ID 40    elbow
Response.vehicleOrientation.manipulator5Angle= 90	#non existent
Response.vehicleOrientation.manipulator6Angle= 180	#ID 50    rotation of lower arm
Response.vehicleOrientation.manipulator7Angle= 90	#ID 60    wrist

#Generate a protobuf message & display it on screen
global Request
Request = rsClientRequest_pb2.ClientRequest()

#-------
def Response_callback( Request ):
	global Response
#	if Request.requestStatus:
#		pass
	#error gets written in a separate handler using /rosout#
	#Get the current measurement			-> Elcosensor node
	try:
		locMeasureThickness = rospy.ServiceProxy( 'MeasureThickness', MeasureThickness )
		Response.status.layerThicknessMeasurement = locMeasureThickness( '' )
	except: 
		Response.status.layerThicknessMeasurement = 0
	#Get the current position			-> Xsens; Faulhaber_driver node?
	#Get the current angles - large and small arm	-> Gumstix node
	try:
		locGetAngles = rospy.ServiceProxy( 'GetAngles', GetAngles )
	except:
		pass
	#Get the current clamp  status			-> Clamp node
	#try:
	#	locClampSendCommand = rospy.ServiceProxy( 'ClampSendCommand', ClampSendCommand )
	#	Response.status.vehicleClampStatus
	#Get the current magnet status			-> Gumstix?
	#Get the current camera status			-> Camera controller?
	#Get the current system status			-> state machine controller?
	try: 
		rospy.ServiceProxy( 'get_state', get_state )
	except:
		pass
	
#-------
def Request_callback( Request ):
	for Mouse in Request.mouse3d:
		#create message
		#cg rospy.logwarn("puinode got mouse data")
		pubmsg = TwistStamped()
		pubmsg.header.stamp    = rospy.Time.now()
		pubmsg.twist.linear.x  = Mouse.x
		pubmsg.twist.linear.y  = Mouse.y
		pubmsg.twist.linear.z  = Mouse.z
		pubmsg.twist.angular.x = Mouse.rx
		pubmsg.twist.angular.y = Mouse.ry
		pubmsg.twist.angular.z = Mouse.rz
		if ( Mouse.id == 2 ):		#left (2)
			pubmouseleft.publish( pubmsg )
		else:				#right (1)
			pubmouseright.publish( pubmsg )
	for Button in Request.buttons:
		#create message if button was pressed (which is probably generated when button state switches from 0 to 1)
		if (Button.pressed):
			pubmsg = Vector3Stamped()
			pubmsg.header.stamp = rospy.Time.now()
			pubmsg.vector.x = Button.id+10 if ((Button.id%3) == 0) else 0
			pubmsg.vector.y = Button.id+10 if ((Button.id%3) == 1) else 0
			pubmsg.vector.z = Button.id+10 if ((Button.id%3) == 2) else 0
			if ( Button.id < 3 ):		#left (0,1,2)
				pubbuttonleft.publish( pubmsg )
			else:				#right (3,4,5)
				pubbuttonright.publish( pubmsg )
#-------
#When a message with level >= 8 (error and fatal messages) is posted on /rosout, 
#change the global response error
def handle_Log( msg ):
	global Response
	if ( msg.level >= 8 ):
		Response.status.errorId = 3
		Response.error   	= msg.name + ': ' + msg.msg
		#cg rospy.logwarn( Response ) 	  
#-------
#The server receives Requests (aka mouse inputs) and sends Responses (system Status)
def protobuftalker():
	rospy.init_node( 'protobuf1_server', anonymous=True );
	rospy.Subscriber( 'rosout', Log, handle_Log )
	
	

	#get parameters
	HOST 	= '192.168.2.6'        # 127.0.0.1 = local host
	PORT 	= 55557                  # Arbitrary non-privileged port
	if rospy.has_param( '/pui_host' ):
	     HOST = rospy.get_param( '/pui_host' );
	if rospy.has_param( '/pui_port' ):
	     PORT = rospy.get_param( '/pui_port' );

	#setup the server
	try:
		context = zmq.Context();
		s	= context.socket( zmq.REP );
		s.bind( "tcp://%s:%s" % (HOST, PORT) )
		#cg rospy.logwarn( s );
	except:
		rospy.logerr( 'Failed to connect to PUI on: ' + HOST + ':' + str( PORT ) )
		rospy.signal_shutdown( "Error while connecting to PUI" );
		return
	
	i = 0
	while not rospy.is_shutdown():
		data = s.recv( )			#this waits until data is received		
		Request.ParseFromString( data )
		req = str( Request )
		Request_callback( Request )
		#cg rospy.logwarn(Request.id)
		#pub.publish(req);
		Response.id = 'ros report ' + str( i )
		s.send( Response.SerializeToString() ) 	#send a Response to the Client
		#cg rospy.logwarn( Response.id )
		rospy.sleep(0.01);
		i+=1;

	s.close();

#-------
if __name__ == '__main__':
	try:
		protobuftalker();
	except rospy.ROSInterruptException:
		pass

