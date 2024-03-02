#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

flag = 0
theta=0
y=0
x=0
f=0
s=0
def callback(data):
    global flag
    global y
    global x
    global f
    global s
    global theta 
    theta=2*math.degrees(math.acos(data.pose.pose.orientation.w))
#    if(data.pose.pose.orientation.z<0):
#        theta=360-theta
    if flag == 0:
        s = data.pose.pose.position.y
        f = data.pose.pose.position.x
        flag = 1
    else:
        y = data.pose.pose.position.y
        x = data.pose.pose.position.x
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("odom", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
 #   rospy.spin()



def talker():
    global flag
    global y
    global x
    global f
    global s
    global theta 
    pub = rospy.Publisher('cmd_vel', Float64, queue_size=10)
    pub_2 = rospy.Publisher('/brakes', Float64, queue_size=10)
    pub_3 = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    targetTheta=30
#    if targetTheta<0:
#        targetTheta+360
    steering=0
    ThetaHead=0
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while y ==0:
        pass
    while y-s<25:
            pub.publish(0.1)

    while f-x<3:
        pub_3.publish(targetTheta-theta)
    s=y
    while ThetaHead-theta<-0.128585:
        pub_3.publish(ThetaHead-theta)
    while y-s<25:
        pub.publish(0.1)
    while not rospy.is_shutdown():
        pub.publish(0)
        pub_2.publish(1)
        pub_3.publish(0)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass