#!/usr/bin/env python3

import rospy
import board
import busio
import numpy as np
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
from adafruit_icm20x import ICM20948,AccelRange,GyroRange,MagDataRate

class ICM20948_NODE(object):

    def __init__(self):
        rospy.init_node('icm20948')
        self.raw_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.icm = ICM20948(self.i2c)
        self.initialize_imu()

        try:
            self.soft_iron = np.array(rospy.get_param("~soft_iron")).reshape((3,3)) 
            rospy.loginfo("Loaded soft-iron calibration")
            rospy.loginfo(self.soft_iron)
        except:
            rospy.logerr("Unable to load soft-iron calibration")
            self.soft_iron = np.identity(3)

        try:
            self.hard_iron = np.array(rospy.get_param("~hard_iron"))
            rospy.loginfo("Loaded hard-iron calibration")
            rospy.loginfo(self.hard_iron)
        except:
            rospy.logerr("Unable to load hard-iron calibration")
            self.hard_iron = np.zeros(3)

    def initialize_imu(self):
        if hasattr(AccelRange, rospy.get_param('~acc_g')):
            rospy.loginfo("Setting accel range to %s" %rospy.get_param('~acc_g'))
            self.icm.accelerometer_range = getattr(AccelRange, rospy.get_param('~acc_g'))
        else:
            rospy.logerr("%s is not a valid accel range" %rospy.get_param('~acc_g'))
            return

        if hasattr(GyroRange, rospy.get_param('~gyro_dps')):
            rospy.loginfo("Setting gyro range to %s" %rospy.get_param('~gyro_dps'))
            self.icm.gyro_range = getattr(GyroRange, rospy.get_param('~gyro_dps'))
        else:
            rospy.logerr("%s is not a valid gyro range" %rospy.get_param('~gyro_dps'))
            return

        # Set up IMU sensors for 100hz operation
        self.icm.accelerometer_data_rate_divisor = 10  # ~102.27Hz
        self.icm.gyro_data_rate_divisor = 10 # 100Hz
        self.icm.magnetometer_data_rate = MagDataRate.RATE_100HZ

    @property
    def magnetic(self):
        mag = np.array(self.icm.magnetic)
        calib_mag = self.soft_iron @ (mag - self.hard_iron)
        calib_mag *= 1e-6 # convert uT to T
        return tuple(calib_mag) 


    def run(self):        
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            measurement_time = rospy.Time.now()
            acc_data = self.icm.acceleration
            gyr_data = self.icm.gyro
            mag_data = self.magnetic

            linear_acceleration_covariance = [
            0.017 , 0 , 0,
            0 , 0.017, 0,
            0 , 0 , 0.017
            ]

            angular_velocity_covariance = [
            0.04, 0 , 0,
            0 , 0.04, 0,
            0 , 0 , 0.04
            ]

            raw_msg = Imu()
            raw_msg.header.stamp = measurement_time
            raw_msg.header.frame_id = "imu_link"
                    
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
            raw_msg.linear_acceleration_covariance = linear_acceleration_covariance
            raw_msg.angular_velocity_covariance = angular_velocity_covariance
            self.raw_pub.publish(raw_msg)
            
            mag_msg = MagneticField()
            mag_msg.header.stamp = measurement_time
            mag_msg.header.frame_id = "imu"
            mag_msg.magnetic_field.x = mag_data[0]
            mag_msg.magnetic_field.y = -mag_data[1]
            mag_msg.magnetic_field.z = -mag_data[2]
            mag_msg.magnetic_field_covariance[0] = 0.5e-6
            mag_msg.magnetic_field_covariance[4] = 0.5e-6
            mag_msg.magnetic_field_covariance[8] = 0.5e-6
            self.mag_pub.publish(mag_msg)

            rate.sleep()   
        
        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node finished")

if __name__ == '__main__':
    try:
        icm =  ICM20948_NODE()
        icm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node exited with exception.")
