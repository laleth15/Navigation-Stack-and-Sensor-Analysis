import rospy
import serial
import numpy as np
from lab4_driver.msg import Vectornav as imu_msg
import time
from lab4_driver.srv import convert_to_quaternion, convert_to_quaternionRequest

import argparse

# Define Argument parser and add an argument called port to take address of serial port
# parser = argparse.ArgumentParser(description = "Driver code for GPS puck that collects data from serial port." )
# parser.add_argument("port", help="Address of the serial port where the GPS is connected", default = "/dev/ttyUSB0")
# port_address,unknown = parser.parse_known_args()
# port_address = port_address.port

def imu_client():
	rospy.init_node('driver')
	serial_port = rospy.get_param('imu_port','/dev/pts/99')
 	

	rospy.logdebug("Using IMU on port "+serial_port+" at "+str(115200))
	print("Using IMU on port "+serial_port+" at "+str(115200))

	port = serial.Serial(serial_port, 115200)
	port.write(b'$VNWRG,07,40*XX\r')

	values_pub = rospy.Publisher("/imu", imu_msg, queue_size=5)
	
	msg = imu_msg()
	rate = rospy.Rate(40)
	msg.Header.frame_id = "IMU1_Frame"
	seq = 0
	print("Starting the conversion...")
	rospy.wait_for_service('euler_to_quaternion')
	
	while not rospy.is_shutdown():

		line = port.readline()
		
		decoded_line = line.decode('utf-8')
		if "VNYMR" in decoded_line:
					print(decoded_line)
					parts = decoded_line.split(",")
					
					current_time = time.time()
					secs = int(current_time)
					nsecs = int((current_time - secs) * 10**9)

					msg.Header.seq = seq
					msg.Header.stamp.secs = secs
					msg.Header.stamp.nsecs = nsecs

					yaw_radians = float(parts[1]) * (np.pi/180)
					pitch_radians = float(parts[2]) * (np.pi/180)
					roll_radians = float(parts[3]) * (np.pi/180)
					
					euler_to_quaternion = rospy.ServiceProxy('euler_to_quaternion', convert_to_quaternion)
					req = convert_to_quaternionRequest()
					req.yaw = yaw_radians
					req.pitch = pitch_radians
					req.roll = roll_radians

					req = euler_to_quaternion(req)
					
					msg.imu.orientation.w = req.w
					msg.imu.orientation.x = req.x
					msg.imu.orientation.y = req.y
					msg.imu.orientation.z = req.z

					msg.mag_field.magnetic_field.x = float(parts[4])
					msg.mag_field.magnetic_field.y = float(parts[5])
					msg.mag_field.magnetic_field.z = float(parts[6])

					msg.imu.linear_acceleration.x = float(parts[7])
					msg.imu.linear_acceleration.y = float(parts[8])
					msg.imu.linear_acceleration.z = float(parts[9])

					msg.imu.angular_velocity.x = float(parts[10])
					msg.imu.angular_velocity.y = float(parts[11])
					msg.imu.angular_velocity.z = float(parts[12][:-5])

					msg.imu_raw_data = str(decoded_line)

					seq = seq + 1
					values_pub.publish(msg)
					rate.sleep() 

if __name__ == '__main__':
	try:
		imu_client()
	except rospy.ROSInterruptException:
		pass
