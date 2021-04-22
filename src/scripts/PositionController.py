#!/usr/bin/env python
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import PoseStamped

import threading

import rospy

integratorActive = False

isSlamActive = False

x_setpoint = 0
y_setpoint = -3
z_setpoint = 0
phi_setpoint = 0

t_slam_prev = 0

x_slam_prev = 0
y_slam_prev = 0
z_slam_prev = 0

t_slam = 0

x_slam = 0
y_slam = 0
z_slam = 0

vx_slam = 0
vy_slam = 0
vz_slam = 0

phi_imu = 0

pub = rospy.Publisher

kp_y = 2
ki_y = 0.1
kd_y = 0.02

kp_vy = 5
ki_vy = 0.5
kd_vy = 0.02


prevStamp_nsec = 0


throttle_pub = rospy.Publisher("/autothrottle", Float32, queue_size = 10)

def orb_pose_callback(msg):

    global isSlamActive

    if isSlamActive == False:
        isSlamActive = True

    global x_slam_prev
    global y_slam_prev
    global z_slam_prev

    global x_slam
    global y_slam
    global z_slam

    global prevStamp_nsec

    global vx_slam
    global vy_slam
    global vz_slam


    pose = msg.pose
    x_slam = pose.position.x
    y_slam = pose.position.y
    z_slam = pose.position.z

    stamp_nsec = msg.header.stamp.to_nsec()
    dt = (stamp_nsec-prevStamp_nsec)*10e-9

    vx_slam = (x_slam - x_slam_prev)/dt
    vy_slam = (y_slam - y_slam_prev)/dt
    vz_slam = (x_slam - x_slam_prev)/dt

    x_slam_prev = x_slam
    y_slam_prev = y_slam
    z_slam_prev = z_slam

    prevStamp_nsec = stamp_nsec



def imu_callback(msg):

    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w

    quat = [x, y, z, w]

    r = R.from_quat(quat)

    euler = r.as_euler('zyx', degrees = True)

    global phi_imu
    phi_imu = euler[0]

    #print(phi_imu)

def joy_callback(msg):

    print(msg)

    
    axes = msg.axes
    modeSwitch = axes[5]

    
    if modeSwitch == 1.0:
        print("integrator active")
        integratorActive = True
    else:
        print("integrator inactive")
        integratorActive = False

def controlLoop():

    global isSlamActive

    loopTime = 5.0
    dt = 1/loopTime

    rate = rospy.Rate(loopTime)

    y_error_prev = 0
    y_error_intg = 0

    vy_error_prev = 0
    vy_error_intg = 0

    y_input = 1

    desiredVelocity = 0

    while not rospy.is_shutdown():

        global phi_imu
        global x_slam
        global y_slam
        global z_slam
        global vy_slam

        global integratorActive

        if isSlamActive == True:
            x_error = x_setpoint - x_slam
            y_error = y_setpoint - y_slam
            z_error = z_setpoint - z_slam


            phi_error = phi_setpoint - phi_imu
            
            if integratorActive == True:
                y_error_intg += y_error*dt
            else:
                y_error_intg = 0

            
            y_error_deriv = (y_error - y_error_prev)/dt

            y_error_prev = y_error

            desiredVelocity = (kp_y*y_error + ki_y*y_error_intg + kd_y*y_error_deriv)/100

            print "slam position: ", y_slam, "slam velocity: ", vy_slam, "slam position setpoint: ", y_setpoint , "slam velocity setpoint: ", desiredVelocity


            vy_error = desiredVelocity - vy_slam

            if integratorActive == True:
                vy_error_intg += vy_error
            else:
                vy_error_intg = 0

            vy_error_deriv = (vy_error - vy_error_prev)/dt

            vy_error_prev = vy_error

            throttle_cmd = 1 + (kp_vy*vy_error + ki_vy*vy_error_intg + kd_vy*vy_error_deriv)

            throttle_pub.publish(throttle_cmd)



        '''
        print(x_error)
        print(y_error)
        print(z_error)
        print(phi_error)
        '''
        
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('PositionController')

    joySub  = rospy.Subscriber('/joy', Joy, joy_callback)
    poseSub = rospy.Subscriber('/Mono_Inertial/orb_pose', PoseStamped, orb_pose_callback)
    imuSub  = rospy.Subscriber('/imu', Imu, imu_callback)

    worker = threading.Thread(target = controlLoop)
    worker.start()

    rospy.spin()