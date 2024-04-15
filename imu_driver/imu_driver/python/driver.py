#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# confirm if we should add libraries as dependencies
import rospy
import serial
from imu_driver.msg import Vectornav
from imu_driver.srv import *
import argparse

# Define Argument parser and add an argument called port to take address of serial port
parser = argparse.ArgumentParser(description = "Driver code for GPS puck that collects data from serial port." )
parser.add_argument("port", help="Address of the serial port where the GPS is connected", default = "/dev/ttyUSB0")
port_address,unknown = parser.parse_known_args()
port_address = port_address.port

def get_quaternion(r,p,y):
    rospy.wait_for_service('convert_to_quaternion')
    try:
        service = rospy.ServiceProxy('convert_to_quaternion',convert_to_quaternion) 
        response = service(r,p,y)
        return [response.w,response.x,response.y,response.z]
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        rospy.logwarn("Service call failed: %s"%e)

if __name__ == '__main__':
    SENSOR_NAME = "imu1_Frame"
    config_string = '$VNWRG,6,0\r\n'

    rospy.init_node("imu_driver", anonymous = True)
    
    serial_port = port_address #"/dev/pts/2" #"/dev/ttyUSB0" # modify to get from paramater
    #serial_port = "/dev/ttyACM1"
    serial_baud = 115200
    
    port = serial.Serial(serial_port,serial_baud, timeout= 3.)
    port.write(config_string.encode())
    rospy.logdebug("Using IMU sensor on port: " + serial_port + "at" + str(serial_baud))

    imu_data = Vectornav()
    rate = rospy.Rate(40)
    # Constant since we are receiving data from single GPS puck
    imu_data.header.frame_id = SENSOR_NAME
    imu_data.imu.header.frame_id = SENSOR_NAME
    imu_data.mag_field.header.frame_id = SENSOR_NAME
    # Increases by 1 for every valid data received
    imu_data.header.seq = 0
    imu_data.imu.header.seq = 0
    imu_data.mag_field.header.seq = 0
    # Publish data under the topic '/gps'
    imu_pub = rospy.Publisher("/imu", Vectornav, queue_size = 10)

    try:
        while not rospy.is_shutdown():
            serial_data = port.readline().decode()  
            time = rospy.Time.now()
            covariance_unknown = [0,0,0,0,0,0,0,0,0]
            #serial_data = serial_data.decode()
            #print(serial_data)

            # $VNYMR,+164.618,+022.062,-003.757,-00.3611,-00.0797,+00.2916,+03.553,+00.595,-08.826,+00.004000,-00.000843,+00.000141*64
            # $VNYMR, yaw, pitch, roll, mag x, mag y, mag z, acc x, acc y, acc z, ar x, ar y, ar z

            if serial_data == '':
                rospy.logwarn("No Data")
            else:
                data = list(serial_data.split(","))
                # Process the data if the gps data is of type "$VNYMR"
                if "$VNYMR" in data[0]:

                    if '' in data:
                        rospy.logwarn("Needed values missing")
                        continue 
                    quaternions = get_quaternion(float(data[3]),float(data[2]),float(data[1]))
                    # Assing and publsih the data to the /imu topic
                    imu_data.header.seq += 1
                    imu_data.imu.header.seq += 1
                    imu_data.mag_field.header.seq += 1
                    imu_data.header.stamp = time
                    imu_data.imu.header.stamp = time
                    imu_data.mag_field.header.stamp = time

                    imu_data.raw_imu = serial_data

                    imu_data.imu.orientation.w = quaternions[0]
                    imu_data.imu.orientation.x = quaternions[1]
                    imu_data.imu.orientation.y = quaternions[2]
                    imu_data.imu.orientation.z = quaternions[3]
                    
                    imu_data.imu.angular_velocity.x = float(data[10])
                    imu_data.imu.angular_velocity.y = float(data[11])
                    imu_data.imu.angular_velocity.z = float(data[12].split("*")[0])

                    imu_data.imu.linear_acceleration.x = float(data[7])
                    imu_data.imu.linear_acceleration.y = float(data[8])
                    imu_data.imu.linear_acceleration.z = float(data[9])

                    imu_data.mag_field.magnetic_field.x = float(data[4])
                    imu_data.mag_field.magnetic_field.y = float(data[5])
                    imu_data.mag_field.magnetic_field.z = float(data[6])

                    imu_data.imu.orientation_covariance = covariance_unknown
                    imu_data.imu.angular_velocity_covariance = covariance_unknown
                    imu_data.imu.linear_acceleration_covariance = covariance_unknown
                    imu_data.mag_field.magnetic_field_covariance = covariance_unknown
                    
                    imu_pub.publish(imu_data)

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down node.")

