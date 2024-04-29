#!/usr/bin/env python
# move forward 75m v1 code:
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
############################################ To debug velocities and accelerations ############################## 
flag = 0
flag_2 = 0
old_pose=0
new_pose=0
start_time=0
end_time=0
vel_end=0
vel_start=0
counter=0
acc=0

Throttle=0.05
Stearing=21.0
Brake=0
def perpendicular( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def normalize(a):
    a = np.array(a)
    return a/np.linalg.norm(a)

def callback(data):
    global flag
    global flag_2
    global start_time
    global end_time
    global vel_end
    global vel_start
    global old_pose
    global new_pose
    global counter
    global acc

    global Throttle
    global Stearing
    global Brake
    if flag == 0:#to get 2 poses before calc vel
        y_new_pose = data.pose.pose.position.y
        x_new_pose = data.pose.pose.position.x
        r=math.sqrt((x_new_pose+6)**2+y_new_pose**2)
        theta=2*math.degrees(math.acos(data.pose.pose.orientation.w))
        if(data.pose.pose.orientation.z<0):
            theta=360-theta
        theta=theta+90
        if theta>=360:
            theta-=360
            
        a=[x_new_pose+6,y_new_pose]
        ang=perpendicular(normalize(a))
        angle = math.degrees(np.arctan2(ang[1],ang[0]))
        if angle<=0:
            angle+=360

        update=angle-theta

        if update<-180:
            update+=360

#        Stearing+=0.1*update

        if Stearing>25:
            Stearing=25
        if Stearing<20:
            Stearing=20

#        if update>=10:
#            Brake=1
#            Stearing=0
#            Throttle=0

        #Stearing+=angle-theta
        rospy.loginfo(r)
        #x_vel=data.twist.twist.linear.x
        #y_vel=data.twist.twist.linear.y
        #vel=math.sqrt(x_vel**2+y_vel**2)

        #if x_new_pose<=-11:
            #Throttle=0.01
            #rospy.loginfo("hi")
            #flag=1

        #end_time=time.time()
        #flag = 1
    else:
        old_pose = new_pose
        start_time =end_time
        end_time=time.time()
        new_pose = data.pose.pose.position.y
        Time=end_time-start_time
        if flag_2==0: #to get 2 vel before get acc
            vel_end=(new_pose-old_pose)/(Time)
            flag_2=1
        else:
            vel_start=vel_end
            vel_end=(new_pose-old_pose)/(Time)
            counter+=1
            acc+=(vel_end-vel_start)/Time
            if counter==50:
                #rospy.loginfo(acc/50)
                acc=0
                counter=0

        #rospy.loginfo(vel_end)
##############################################################################################################
    
def listener():

#listen to odom tobic for debuggibg only
    rospy.Subscriber("odom", Odometry, callback)


def move_forward():

    global Throttle
    global Stearing
    global Brake

    Throttle_pub = rospy.Publisher('cmd_vel', Float64, queue_size=10)#cmd_vel here is throttle not twist msg
    Brake_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
    Stearing_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)

    Max_vel=15  #m/s
    Max_acc=1   #m/s**2
    noBreak=0
    Break=1

    Time_phase1=6 #s (to achieve velocity 6m/s)  eqation: v = v0 + at
    Throttle_phase1=1

    Time_phase2=7 #s (to achieve 42 m)  eqation: f = v * t
    Throttle_phase2=0.4

    Throttle_phase3=0 #decelerate until the end (deceleration=1m/s**2)

    rospy.init_node('move_forward', anonymous=True)#initiate node call move_forward to subscribe and publish to topics
    rate = rospy.Rate(10) # 10hz
    init_time=time.time()
    while not rospy.is_shutdown():
        Stearing_pub.publish(Stearing)
        Throttle_pub.publish(Throttle)
        Brake_pub.publish(Brake)

        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
        move_forward()
    except rospy.ROSInterruptException:
        pass


#todo : more accuracy by add more throttles
    

'''
phase1:
    
    f=(v**2)/(2*Max_acc*Throttle_phase1)=(6**2)/(2*1*1)=18m
    t=v/(Max_acc*Throttle_phase1)=6s

phase2:
    f=totalDistance-f(phase1)-f(deceleration)                     note:acc_pahse1=Max_acc*Throttle_phase1=1=deceleration
    f=39m
    t=f/v=6.5~7s

phase3:
    f=f_phase1=18m
    t=t_phase1=6s

'''