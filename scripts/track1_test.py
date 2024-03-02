#!/usr/bin/env python
# move forward 75m v1 code:
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import time
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
    if flag == 0:#to get 2 poses before calc vel
        new_pose = data.pose.pose.position.y
        end_time=time.time()
        flag = 1
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

    Throttle_pub = rospy.Publisher('cmd_vel', Float64, queue_size=10)#cmd_vel here is throttle not twist msg
    Break_pub = rospy.Publisher('/brakes', Float64, queue_size=10)

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
        if time.time()-init_time<=Time_phase1:
            Break_pub.publish(noBreak)
            Throttle_pub.publish(Throttle_phase1)

        elif time.time()-init_time<=Time_phase1+Time_phase2:
            Throttle_pub.publish(Throttle_phase2)

        else:    
            Throttle_pub.publish(Throttle_phase3)
            Break_pub.publish(Break)

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