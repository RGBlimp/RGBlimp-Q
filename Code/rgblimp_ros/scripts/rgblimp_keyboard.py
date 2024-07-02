#!/usr/bin/env python

# RGBlimp-Q Keyboard
# Hao Cheng 20231217

import rospy
import time
import sys, select, tty, termios
from std_msgs.msg import UInt64
from std_msgs.msg import String

Throttle = 170
Delta = 10
delta = 1
Limit = 300

def ctrl(key):
    #rospy.loginfo(key)

    if key==' ':
        rospy.loginfo("\033[1;31m EMERGENCY STOP !!!\033[0m")
        pub.publish( UInt64(data=0) )
        return
		

    # Gripper Open
    if key=='o' or key=='O':
        rospy.loginfo("\033[1;33m Gripper Open  ( 0 deg) \033[0m")
        pub.publish( UInt64(data = 0 + 2*(2**60)) )
        return
	# Gripper Close
    if key=='p' or key=='P':
        rospy.loginfo("\033[1;33m Gripper Close (90 deg) \033[0m")
        pub.publish( UInt64(data = 90 + 2*(2**60)) )
        return
		
		
    # Gondola x+
    if key=='e' or key=='E':
        pub.publish( UInt64( data = 1000 +  500*(2**10) + 3*(2**60) ) ) 
        time.sleep(0.2)
        pub.publish( UInt64( data =  500 +  500*(2**10) + 3*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Gondola x+ \033[0m")
        return
    # Gondola x-
    if key=='d' or key=='D':
        pub.publish( UInt64( data =    0 +  500*(2**10) + 3*(2**60) ) ) 
        time.sleep(0.2)
        pub.publish( UInt64( data =  500 +  500*(2**10) + 3*(2**60) ) ) 
        rospy.loginfo("\033[1;33m Gondola x- \033[0m")
        return
    # Gondola y-
    if key=='s' or key=='S':
        pub.publish( UInt64( data =  500 +    0*(2**10) + 3*(2**60) ) ) 
        time.sleep(0.2)
        pub.publish( UInt64( data =  500 +  500*(2**10) + 3*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Gondola y- \033[0m")
        return
    # Gondola y+
    if key=='f' or key=='F':
        pub.publish( UInt64( data =  500 + 1000*(2**10) + 3*(2**60) ) ) 
        time.sleep(0.2)
        pub.publish( UInt64( data =  500 +  500*(2**10) + 3*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Gondola y+ \033[0m")
        return

    # Gondola pid
    if key=='x' or key=='X':
        # (A,B)=(Dx,Dy); (C,D,E)=(Kp,Ki*10,Kd*10)
        pub.publish( UInt64( data =  (0 +500) +  (0 +500)*(2**10) + 0*(2**20) + 0*(2**30) + 0*(2**40)+ 0*(2**50) + 4*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Gondola PID \033[0m")
        return
    # Gondola pid Tuner
    if key=='t':
        # (A,B)=(Dx,Dy); (C,D,E)=(Kp,Ki*10,Kd*10)
        pub.publish( UInt64( data =  (0 +500) +  (0 +500)*(2**10) + 30*(2**20) + 50*(2**30) + 10*(2**40)+ 0*(2**50) + 4*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Gondola PID Tuner \033[0m")
        return
		
    # Feedback Control -- Straight Flight
    if key=='c' or key=='C':
        # (A,B)=(Kp,Kd*10); Aymax=C 
        pub.publish( UInt64( data = 60 + 8*(2**10) + 45*(2**20) + 1*(2**50) + 4*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Straight Flight \033[0m")
        return

    # Feedback Control -- Eular Spiral
    if key=='v' or key=='V':
        pub.publish( UInt64( data = ( -3 +500) +  0*(2**10) + 2*(2**50) + 4*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Eular Spiral \033[0m")
        return

    # Feedback Control -- Square End Motion Trajectory of the Continuum Arm
    if key=='b':
        # Ax=A [mm], Ay=B [mm]; Ta(Dy=Ay)=C/10 [s], Tb(Dx=Ax)=D/10; E?LOOP:SINGLE;
        pub.publish( UInt64( data = 40 + 40*(2**10) + 50*(2**20)+ 50*(2**30)+ 0*(2**40) + 3*(2**50) + 4*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Square End Motion \033[0m")
        return
    # Feedback Control -- Triangular End Motion Trajectory of the Continuum Arm
    if key=='B':
        # Ax=A [mm], Ay=B [mm]; Ta(Dy=Ay)=C/10 [s], Tb=D/10 [s]; E?LOOP:SINGLE;
        pub.publish( UInt64( data = 40 + 50*(2**10) + 400*(2**20)+ 50*(2**30)+ 0*(2**40) + 4*(2**50) + 4*(2**60) ) ) 
        rospy.loginfo("\033[1;32m Triangular End Motion \033[0m")
        return


    # Throttle Manual Setting with Step Delta
    if key=='Q':
        Throttle = int(( (Throttle+Delta) if (Throttle+Delta<Limit) else Limit )%1000)
    if key=='A':
        Throttle = 0
    if key=='Z':
        Throttle = int(( (Throttle-Delta) if (Throttle-Delta>0) else 0 )%1000)
    # Throttle Manual Setting with Step delta
    if key=='q':
        Throttle = int(( (Throttle+delta) if (Throttle+delta<Limit) else Limit )%1000)
    if key=='a':
        Throttle = 0
    if key=='z':
        Throttle = int(( (Throttle-delta) if (Throttle-delta>0) else 0 )%1000)
		
		
    # Throttle Running
    if key=='m' or key=='M':
        pub.publish( UInt64( data = Throttle + 1*(2**60) ) ) 
        rospy.loginfo("\033[1;36m Throttle Running \033[0m" )
    if key=='l' or key=='L':
        pub.publish( UInt64( data = 0 + 1*(2**60) ) ) 
        rospy.loginfo("\033[1;36m Throttle HOLD ON \033[0m" )
		

    rospy.loginfo("\033[1;34m Throttle = \033[0m" + str(Throttle))

if __name__=='__main__':
    rospy.init_node('rgblimp_keyboard', anonymous=True)
    pub = rospy.Publisher('command', UInt64, queue_size=1)

    # Keyboard Hook
    rate = rospy.Rate(100) # freq
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno()) 
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]: 
           ctrl(sys.stdin.read(1))
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

