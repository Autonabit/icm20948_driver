#!/usr/bin/env python3

import rospy
import board
import busio
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
from adafruit_icm20x import ICM20948,AccelRange,GyroRange,MagDataRate

def icm20948_node():

    raw_pub = rospy.Publisher('icm20948/raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('icm20948/mag', MagneticField, queue_size=10)
    rospy.init_node('icm20948')

    i2c = busio.I2C(board.SCL, board.SDA)
    icm = ICM20948(i2c)

    if hasattr(AccelRange, rospy.get_param('~acc_g')):
        rospy.loginfo("Setting accel range to %s" %rospy.get_param('~acc_g'))
        icm.accelerometer_range = getattr(AccelRange, rospy.get_param('~acc_g'))
    else:
        rospy.logerr("%s is not a valid accel range" %rospy.get_param('~acc_g'))
        return

    if hasattr(GyroRange, rospy.get_param('~gyro_dps')):
        rospy.loginfo("Setting gyro range to %s" %rospy.get_param('~gyro_dps'))
        icm.gyro_range = getattr(GyroRange, rospy.get_param('~gyro_dps'))
    else:
        rospy.logerr("%s is not a valid gyro range" %rospy.get_param('~gyro_dps'))
        return

    
    # Set up IMU sensors for 100hz operation
    icm.accelerometer_data_rate_divisor = 10  # ~102.27Hz
    icm.gyro_data_rate_divisor = 10 # 100Hz
    icm.magnetometer_data_rate = MagDataRate.RATE_100HZ
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        messurement_time = rospy.Time.now()
        acc_data = icm.acceleration
        gyr_data = icm.gyro
        mag_data = tuple(i*1e-6 for i in icm.magnetic) #convert from uT to T

        raw_msg = Imu()
        raw_msg.header.stamp = messurement_time
	            
        raw_msg.orientation.w = 0
        raw_msg.orientation.x = 0
        raw_msg.orientation.y = 0
        raw_msg.orientation.z = 0
            
        raw_msg.linear_acceleration.x = acc_data[0]
        raw_msg.linear_acceleration.y = acc_data[1]
        raw_msg.linear_acceleration.z = acc_data[2]
            
        raw_msg.angular_velocity.x = gyr_data[0]
        raw_msg.angular_velocity.y = gyr_data[1]
        raw_msg.angular_velocity.z = gyr_data[2]
            
        raw_msg.orientation_covariance[0] = -1
        raw_msg.linear_acceleration_covariance[0] = -1
        raw_msg.angular_velocity_covariance[0] = -1
        raw_pub.publish(raw_msg)
        
        mag_msg = MagneticField()
        mag_msg.header.stamp = messurement_time
        mag_msg.magnetic_field.x = mag_data[0]
        mag_msg.magnetic_field.y = mag_data[1]
        mag_msg.magnetic_field.z = mag_data[2]
        mag_msg.magnetic_field_covariance[0] = -1
        mag_pub.publish(mag_msg)

        rate.sleep()   
    
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node finished")

if __name__ == '__main__':
    try:
        icm20948_node()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node exited with exception.")