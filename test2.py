#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

flag = 0
flag2 = 0
y=[0,0]
s=[0,0]
def callback(data):
    global flag
    global y
    global s
    if flag == 0:
        s = [round(data.pose.pose.position.x,6),round(data.pose.pose.position.y,6)]
        flag = 1
    else:
        y = [round(data.pose.pose.position.x,6),round(data.pose.pose.position.y,6)]
        f=math.sqrt((s[1]-y[1])**2+(s[0]-y[0])**2)
        rospy.loginfo(f)

    
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
#    global s
#    global y
    pub = rospy.Publisher('cmd_vel', Float64, queue_size=10)
    pub_2 = rospy.Publisher('/brakes', Float64, queue_size=10)
    pub_3 = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    global flag2
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #f=math.sqrt((s[1]-y[1])**2+(s[0]-y[0])**2)
        #print(s[1])
        #rospy.loginfo(f)
        pub.publish(0.2)
        pub_3.publish(21.6)
        pub_2.publish(0)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass