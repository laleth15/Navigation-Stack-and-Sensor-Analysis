#!/usr/bin/env python3

import rospy
import serial
import utm
from lab4_driver.msg import gps_msg


def convert_to_utm(gps_data_str):

    gps_str_split = gps_data_str

    if gps_data_str == '' or gps_str_split[1] == '' or gps_str_split[2] == '' or gps_str_split[3] == '' or gps_str_split[4] =='' :
        rospy.logwarn("Incomplete data recieved from GPS.")
        return None 
   

    time_stamp = float(gps_str_split[1])
    lat_str = float(gps_str_split[2])
    long_str = float(gps_str_split[4])

    lat_dd = int(lat_str/100)
    lat_mm = lat_str- lat_dd*100
    lat = lat_dd + (lat_mm/60)

    longit_dd = int(long_str/100)
    longit_mm = long_str- longit_dd*100
    long = longit_dd + (longit_mm/60) 

    time_stamp_hh = int(int(time_stamp)/10000)
    time_stamp_mm = (int((int(time_stamp)/10000 - time_stamp_hh)*100))
    time_stamp_ss = (((int(time_stamp)/10000 - time_stamp_hh)*100)%1)*100
    time_stamp_secs = 3600 * time_stamp_hh + 60*time_stamp_mm + time_stamp_ss

    time_stamp_nsecs = ((time_stamp) % 1) * (10 ** 9)

    time_stamp = (time_stamp_secs, time_stamp_nsecs)
    altitude = float(gps_data_str[9])
    hdop = float(gps_str_split[8])
    UTC = float(gps_str_split[1])
    if(gps_str_split[3] == 'S'):
        lat = -lat
    if(gps_str_split[5] == 'W'):
        long = -long
    print("latitude:", lat, " longitude:", long)
    (easting, northing, zone_number, zone_letter) = utm.from_latlon(lat, long)
    print("easting, northing, zone_number, zone_letter, altitude, hdop, UTC", easting, northing, zone_number, zone_letter, altitude, hdop, UTC)

    return (time_stamp, easting, northing, zone_number, zone_letter, lat, long, altitude, hdop, UTC)


def utm_to_rosmsg(utm_data):
    
    msg = gps_msg()

    msg.Header.stamp.secs = int(utm_data[0][0])
    msg.Header.stamp.nsecs = int(utm_data[0][1])   #nsecs wont matter as we will be using it at 1hz
    msg.Header.frame_id = "GPS1_Frame"
    msg.Altitude = utm_data[7]
    msg.Latitude = utm_data[5]
    msg.Longitude = utm_data[6]
    msg.UTM_easting = utm_data[1]
    msg.UTM_northing = utm_data[2]
    msg.UTC = utm_data[9]
    msg.Zone = utm_data[3]
    msg.Letter = utm_data[4]
    msg.HDOP = utm_data[8]

    return msg


if __name__ == '__main__':
    rospy.init_node('gps_data')
    serial_port = rospy.get_param('gps_port','/dev/pts/44')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug("Using GPS on port "+serial_port+" at "+str(serial_baud))

    sleep_time = 0.2

    gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            msg = gps_msg()
            
            gps_data= port.readline()
            print(gps_data)
            if gps_data == '':
                rospy.logwarn("NO INCOMING GPS DATA")
            else:
                gps_data_str = gps_data.decode("utf-8") #converts bytes to string
                print(gps_data_str)
                gps_str_split = gps_data_str.split(",")
                if ('$GPGGA' in gps_str_split[0]):
                    utm_coords = convert_to_utm(gps_str_split)
                    if not utm_coords is None:
                        ros_data_gps = utm_to_rosmsg(utm_coords)
                        print(ros_data_gps)
                        gps_pub.publish(ros_data_gps)

            rospy.sleep(0.2)   

    except rospy.ROSInterruptException:
        port.close()
    



