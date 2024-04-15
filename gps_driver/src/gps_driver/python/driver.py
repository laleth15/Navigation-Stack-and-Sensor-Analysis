#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# confirm if we should add libraries as dependencies
import rospy
import serial
from gps_driver.msg import gps_msg
import utm
import argparse

# Define Argument parser and add an argument called port to take address of serial port
parser = argparse.ArgumentParser(description = "Driver code for GPS puck that collects data from serial port." )
parser.add_argument("port", help="Address of the serial port where the GPS is connected", default = "/dev/ttyUSB0")
port_address,unknown = parser.parse_known_args()
port_address = port_address.port

if __name__ == '__main__':
    SENSOR_NAME = "GPS1_Frame"

    rospy.init_node("gps_driver", anonymous = True)
    
    serial_port = port_address #"/dev/pts/2" #"/dev/ttyUSB0" # modify to get from paramater
    serial_baud = 4800
    
    port = serial.Serial(serial_port,serial_baud, timeout= 3.)
    rospy.logdebug("Using GPS sensor on port: " + serial_port + "at" + str(serial_baud))

    gps_data = gps_msg()
    # Constant since we are receiving data from single GPS puck
    gps_data.Header.frame_id = SENSOR_NAME
    # Increases by 1 for every valid data received
    gps_data.Header.seq = 0
    # Publish data under the topic '/gps'
    gps_pub = rospy.Publisher("/gps", gps_msg, queue_size = 10)

    try:
        while not rospy.is_shutdown():
            serial_data = port.readline().decode('utf-8')
            #serial_data = serial_data.decode()

            if serial_data == '':
                rospy.logwarn("No Data")
            else:
                serial_data = list(serial_data.split(","))

                # Process the data if the gps data is of type "GPGGA"
                if "$GPGGA" in serial_data[0]:

                    # Check for missing data at indeices 1, 2, 3, 4 ,5, 8, 9, 10. If missing proceed to next loop 
                    if '' in serial_data[1:6] or '' in serial_data [8:11]:
                        rospy.logwarn("Needed values missing")
                        continue

                    time_stamp = float(serial_data[1])    

                    #Convert sentence latitude to decimal (DDMM.MMMM to Deg)
                    lat_dd = int(float(serial_data[2])/100)
                    lat_ss = float(serial_data[2]) - (lat_dd*100)
                    lat_in_deg = lat_dd + (lat_ss/60)

                    if serial_data[3] == "S":
                        lat_in_deg = -lat_in_deg

                    #Convert sentence longitude to decimal (DDDDMM.MMMM to Deg)
                    lon_dd = int(float(serial_data[4])/100)
                    lon_ss = float(serial_data[4]) - (lon_dd*100)
                    lon_in_deg = lon_dd + (lon_ss/60)

                    if serial_data[5] == "W":
                        lon_in_deg = -lon_in_deg

                    # Convert lat lon data to utm format
                    utm_converted = utm.from_latlon(lat_in_deg,lon_in_deg)
                    
                    # Assing and publsih the data to the /gps topic
                    gps_data.Header.seq += 1
                    gps_data.Header.stamp.secs = int(time_stamp) # Confirm datatypes and logic
                    gps_data.Header.stamp.nsecs = int(round(time_stamp%int(time_stamp),2)*100)
                    gps_data.Latitude = lat_in_deg # Confirm
                    gps_data.Longitude = lon_in_deg
                    gps_data.Altitude = serial_data[9]+" "+serial_data[10]
                    gps_data.HDOP = round(float(serial_data[8]),2)
                    gps_data.UTM_easting = utm_converted[0]
                    gps_data.UTM_northing = utm_converted[1]
                    gps_data.Zone = utm_converted[2]
                    gps_data.Letter = utm_converted[3]
                    gps_data.UTC = time_stamp # Confirm

                    gps_pub.publish(gps_data)

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down node.")

