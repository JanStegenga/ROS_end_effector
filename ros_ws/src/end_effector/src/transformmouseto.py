#!/usr/bin/env python
from gumstix.srv import * 
import roslib
import rospy
import numpy as np
import numpy.matlib
import numpy.linalg
#import select
#import mtdevice
#import math
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import TwistStamped
# transform Euler angles or matrix into quaternions
from math import pi, radians
#from tf.transformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix

#------------------------------------------------------------------------------------
global Ha
Ha = np.dstack( ( np.eye(4), np.eye(4), np.eye(4) ) )	#3 stacked diagonals
Ha = np.dstack( (Ha, Ha) );				#6 stacked diagonals
Ha = np.dstack( (Ha, np.eye(4) ) );
Ha[3, 0:3, 0] = [ 0.00,  0.00,  0.025];
Ha[3, 0:3, 1] = [ 0.00, -0.08,  0.026];
Ha[3, 0:3, 2] = [ 0.00,  0.00,  0.125];
Ha[3, 0:3, 3] = [ 0.00,  0.00,  0.274];
Ha[3, 0:3, 4] = [-0.05,  0.00,  0.125];
Ha[3, 0:3, 5] = [ 0.00,  0.00,  0.338];
Ha[3, 0:3, 6] = [ 0.00,  0.00,  0.100];

global Hc
Hc = np.copy( Ha )

global Ta
Ta = np.matrix( [  [ 0,  0,  1,  0,  0,  0], 
		   [-1,  0,  0,  0,  0,  0], 
		   [ 0,  0,  1,  0,  0,  0],
		   [-1,  0,  0,  0,  0,  0],
		   [ 0,  0,  1,  0,  0,  0],
		   [-1,  0,  0,  0,  0,  0]  ] ).T;

global J


global q
q = np.matrix( [0, -pi/6, pi/2, pi/2, 0, 0] ).T

global qlim
qlim = np.matrix( [ [-270, 270], [-170, 20], [-180, 180], [-140, 170], [-180, 180], [-110, 110] ] )*pi/180

#------------------------------------------------------------------------------------
def expT( T, qq ):
	w       = T[0:3]
	v       = T[3:6]
	expRFwq = expRF(w,qq)
	renorm	= 1/pow( np.linalg.norm(w,2), 2 )
	val     = [ expRFwq, renorm*( np.eye(3) - expRFwq )*(skew(w)*v) + w.T*v*w ]
	return  np.concatenate( val, [[0, 0, 0, 1]], axis = 1 ) 
#------------------------------------------------------------------------------------
def expRF( w, qq ):
	skeww = skew(w);
	return np.eye(3) + skeww*sin(qq) + skeww**2*(1 - cos(qq))	
#------------------------------------------------------------------------------------
def updateJ( ):
	global J
	J = np.copy( Hc );
	for i in range (0,5):
		J[:,i] = adjoint( Hc[:,:,i] ) * Ta[:,i]
#------------------------------------------------------------------------------------
def updateHc( ):
	global Hc
	Hc = np.copy( Ha );
	Hc[:,:,0] = Ha[:,:,0]*expT( Ta, q[0] )
	for i in range 0,5):	#6 
		Hc[:,:,i] = Hc[:,:,i-1] * Ha[:,:,i]*expT( Ta[:,i], q[i] )
	Hc[:,:,-1] = Hc[:,:,-2]*Ha[:,:,-1]
#------------------------------------------------------------------------------------
def skew( x ):
	#return a 3x3 matrix, input is 3x1
	return np.matlib.matrix( [[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]] )
#------------------------------------------------------------------------------------
def adjoint( x ):
	#return a 6x6 matrix, input is 4x4 (homogeneous) matrix
	R = x[0:3,0:3]	#rotation
	p = x[:,3]	#position
	return np.bmat( [[R, np.zeros( (3,3) )], [skew(p)*R, R]] ) #the 4 3x3 matrices put together
#------------------------------------------------------------------------------------
def inputTwist( msg ):
	global q
	#some 'constants' that will need to be loaded or initialized in some way:
	Hmouseroot = np.eye(4, k=0, dtype=float)	#k=0: main diagonal
	c1	   = 1
	lamb       = 0.2

	#calculation:
	v 	   = np.dot( Hmouseroot[0:3,0:3], np.transpose([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]) )
	w 	   = np.dot( Hmouseroot[0:3,0:3], np.transpose([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]) )
	v_norm 	   = np.linalg.norm( v )
	w_norm 	   = np.linalg.norm( w )
	if (v_norm == 0 ):			#pure rotation
		T = np.bmat( [w, np.array([0, 0, 0]) ] )
	elif (w_norm == 0):			#pure translation
		T = np.bmat( [np.array([0, 0, 0]), v ] )
	else:					#twist
                r=np.dot( skew(w), v)/(w_norm**2)   
                labda = np.dot(np.transpose(w), v)/(w_norm**2)
                T=np.bmat( [w, np.dot(skew(r),w)+np.dot(labda,w)] )
	Jpseudo = np.linalg.solve( J.T, np.dot( J, J.T )  + pow(lamb,2)*np.eye(6, k=0, dtype=float) )
	Htwistattachpoint = np.concatenate( (   np.eye(3, k=0, dtype=float), np.asmatrix(Hc[:3, 3, -1])   ), axis=0 )
	Htwistattachpoint = np.concatenate( (   Htwistattachpoint, np.asmatrix([0,0,0,1]).T ), axis = 1 )

	qdot = 0.01*np.dot( Jpseudo, adjoint( Htwistattachpoint ) )*T.T 
	qnew = q + qdot*c1;

	#adhere to limits
	q = np.minimum( np.maximum( qnew, qlim[:,0] ), qlim[:,1] )
	#update state matrices
	updateJ
	updateHc
	#apply new setpoints
	req = Float32MultiArray()
	req.data = q
	locSetAngleAbs( req )
	rospy.logwarn( q )

	# return qnew
#------------------------------------------------------------------------------------
def transform3Dmouse():
	rospy.init_node( 'transform3Dmouse', anonymous=True )
	pub = rospy.Publisher( 'tf_talking', TwistStamped, queue_size = 1 )
	sub = rospy.Subscriber( 'tf_listening', TwistStamped, inputTwist )
	#create service proxys:
	rospy.wait_for_service( 'GetID' )
	global locSetAngleAbs;	
	locGetID       = rospy.ServiceProxy( 'GetID', GetID )
	locGetAngleAbs = rospy.ServiceProxy( 'GetAnglesAbs', GetAnglesAbs )
	locSetAngleAbs = rospy.ServiceProxy( 'SetAnglesAbs', SetAnglesAbs )
	updateJ( )	
	while not rospy.is_shutdown():
		request = String()
		request.data = 'whatever'
		response = locGetAngleAbs( request ) 
		#rospy.logwarn( response.angles.data )		
		rospy.logwarn( q )		
		rospy.logwarn( '---------' )
		#pub.publish(ds)
		rospy.sleep(10);
	
#------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:
		transform3Dmouse();
	except rospy.ROSInterruptException:
		pass

#------------------------------------------------------------------------------------
