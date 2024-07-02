#!/usr/bin/env python

# RGBlimp-Q PID Tuner
# Hao Cheng 20240122

import rospy
import time
import math
from std_msgs.msg import UInt64
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D

class imu:
    def __init__(self):
        self.q = Quaternion()
        self.phi   = 0.0
        self.theta = 0.0
        self.psi   = 0.0


imuA = imu()
imuB = imu()
q    = Quaternion()

Dx = 0.0
Dy = 0.0
deltaDx = 0.0
deltaDy = 0.0

def getConfig():
    global Dx, Dy, deltaDx, deltaDy
    tmpA = Quaternion()
    tmpA.w = math.cos(imuA.phi/2)*math.cos(imuA.theta/2)
    tmpA.x = math.sin(imuA.phi/2)*math.cos(imuA.theta/2)
    tmpA.y = math.cos(imuA.phi/2)*math.sin(imuA.theta/2)
    tmpA.z =-math.sin(imuA.phi/2)*math.sin(imuA.theta/2)

    tmpB = Quaternion()
    tmpB.w = math.cos(imuB.phi/2)*math.cos(imuB.theta/2)
    tmpB.x = math.sin(imuB.phi/2)*math.cos(imuB.theta/2)
    tmpB.y = math.cos(imuB.phi/2)*math.sin(imuB.theta/2)
    tmpB.z =-math.sin(imuB.phi/2)*math.sin(imuB.theta/2)

    tmp   = Quaternion()
    tmp.w = tmpB.w*tmpA.w + tmpB.x*tmpA.x + tmpB.y*tmpA.y + tmpB.z*tmpA.z
    tmp.x = tmpB.w*tmpA.x - tmpB.x*tmpA.w - tmpB.y*tmpA.z + tmpB.z*tmpA.y
    tmp.y = tmpB.w*tmpA.y + tmpB.x*tmpA.z - tmpB.y*tmpA.w - tmpB.z*tmpA.x
    tmp.z = tmpB.w*tmpA.z - tmpB.x*tmpA.y + tmpB.y*tmpA.x - tmpB.z*tmpA.w

    q.w = math.sqrt(tmp.w*tmp.w +tmp.z*tmp.z)
    q.x = 1/q.w*( tmp.w*tmp.x + tmp.z*tmp.y)
    q.y = 1/q.w*(-tmp.z*tmp.x + tmp.w*tmp.y)
    q.z = 0.0
    #rospy.loginfo("q.[w,x,y,z] = [" + str(q.w) + "," + str(q.x) + "," + str(q.y) + "," + str(q.z) + "]")

    tmpx = 0.0 if q.y==0 else (1 if q.y>0 else -1)*2*40*math.acos(q.w)/math.sqrt(1+q.x**2/q.y**2)
    tmpy = 0.0 if q.y==0 else -q.x/q.y*tmpx
    Dx = tmpx - deltaDx #deltaDx
    Dy = tmpy - deltaDy #deltaDy
    #rospy.loginfo("D.[x,y] = [" + str(Dx) + "," + str(Dy) + "]")


# Gondola
def imuAUpdate(msg):
    imuA.q = msg
    #rospy.loginfo("IMUA.[w,x,y,z] = [" + str(msg.w) + "," + str(msg.x) + "," + str(msg.y) + "," + str(msg.z) + "]")
    imuA.phi   = math.atan2(2*(msg.w*msg.x + msg.y*msg.z), 1 - 2*(msg.x**2 + msg.y**2))
    imuA.theta = math.asin(2*(msg.w*msg.y - msg.x*msg.z))
    imuA.psi   = math.atan2(2*(msg.w*msg.z + msg.x*msg.y), 1 - 2*(msg.y**2 + msg.z**2))


# Body
def imuBUpdate(msg):
    imuB.q = msg
    #rospy.loginfo("IMUB.[w,x,y,z] = [" + str(msg.w) + "," + str(msg.x) + "," + str(msg.y) + "," + str(msg.z) + "]")
    imuB.phi   = math.atan2(2*(msg.w*msg.x + msg.y*msg.z), 1 - 2*(msg.x**2 + msg.y**2))
    imuB.theta = math.asin(2*(msg.w*msg.y - msg.x*msg.z))
    imuB.psi   = math.atan2(2*(msg.w*msg.z + msg.x*msg.y), 1 - 2*(msg.y**2 + msg.z**2))

def paramUpdate(msg):
    global Dx, Dy, deltaDx, deltaDy
    deltaDx =  (float(msg.data%256)-120)/40.0
    deltaDy = (float((msg.data>>8)%256)-120)/40.0
    rospy.loginfo("delta.[x,y] = [" + str(deltaDx) + "," + str(deltaDy) + "]")

if __name__=='__main__':
    rospy.init_node('rgblimp_tuner', anonymous=True)
    rospy.Subscriber("IMUA", Quaternion, imuAUpdate)
    rospy.Subscriber("IMUB", Quaternion, imuBUpdate)
    rospy.Subscriber("param", UInt64, paramUpdate)

    pub = rospy.Publisher('command', UInt64, queue_size=1)
    pub_config = rospy.Publisher('Config', Pose2D, queue_size=1)
    time.sleep(1)
    pub.publish( UInt64( data = 0 + 10*(2**60) ) )  # Request the calibration information

    rate = rospy.Rate(50) # freq
    while not rospy.is_shutdown():
        print("IMUB.[phi,theta,psi] = [" + '%.1f'%(imuB.phi*180/3.14) + ","  + '%.1f'%(imuB.theta*180/3.14) + ","  + '%.1f'%(imuB.psi*180/3.14) + "] deg; [Dx,Dy] = [" + '%.2f'%(Dx) + "," + '%.2f'%(Dy) + "] mm; [vphi, gamma] = [" + '%.1f'%(math.atan2(Dy,Dx+1e-5)*180/3.14) + ","  + '%.1f'%(math.sqrt(Dx**2+Dy**2)/40*180/3.14) + "] deg")
        #print("IMUA.[phi,theta,psi] = [" + '%.1f'%(imuA.phi*180/3.14) + ","  + '%.1f'%(imuA.theta*180/3.14) + ","  + '%.1f'%(imuA.psi*180/3.14) + "] deg" + "IMUB.[phi,theta,psi] = [" + '%.1f'%(imuB.phi*180/3.14) + ","  + '%.1f'%(imuB.theta*180/3.14) + ","  + '%.1f'%(imuB.psi*180/3.14) + "] deg")
        getConfig()
        pub_config.publish( Pose2D(x = Dx, y = Dy) ) 
        rate.sleep()
    
