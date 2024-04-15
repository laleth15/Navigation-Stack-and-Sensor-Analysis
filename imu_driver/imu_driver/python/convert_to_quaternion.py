#!/usr/bin/env python3
# -*- coding: utf-8 -*-

 
from imu_driver.srv import convert_to_quaternion,convert_to_quaternionResponse
import rospy
import math

def handle_request(req):
   # req.x, req.y, req.z -> a, b, c
    cr = math.cos(req.x * 0.5)
    sr = math.sin(req.x * 0.5)
    cp = math.cos(req.y * 0.5)
    sp = math.sin(req.y * 0.5)
    cy = math.cos(req.z * 0.5)
    sy = math.sin(req.z * 0.5)

    res = convert_to_quaternionResponse()

    res.w = cr * cp * cy + sr * sp * sy
    res.x = sr * cp * cy - cr * sp * sy
    res.y = cr * sp * cy + sr * cp * sy
    res.z = cr * cp * sy - sr * sp * cy

    return res
 
def convert_to_quaternion_server():
    rospy.init_node('convert_to_quaternion')
    s = rospy.Service('convert_to_quaternion', convert_to_quaternion, handle_request)
    rospy.spin()
  
if __name__ == "__main__":
    convert_to_quaternion_server()