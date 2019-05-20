#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from mav_msgs.msg import RateThrust
from flightgoggles.msg import IRMarker, IRMarkerArray

def IRMarkerArrayCallback(data):
    global gates
    valid = {'Gate1':[0,0],'Gate2':[0,0],'Gate3':[0,0],'Gate4':[0,0],
            'Gate5':[0,0],'Gate6':[0,0],'Gate7':[0,0],'Gate8':[0,0],
            'Gate9':[0,0],'Gate10':[0,0],'Gate11':[0,0],'Gate12':[0,0],
            'Gate13':[0,0],'Gate14':[0,0],'Gate15':[0,0],'Gate16':[0,0],
            'Gate17':[0,0],'Gate18':[0,0],'Gate19':[0,0],'Gate20':[0,0],
            'Gate21':[0,0],'Gate22':[0,0],'Gate23':[0,0]}
    for marker in data.markers:
        gates[marker.landmarkID.data][int(marker.markerID.data)-1] = (int(marker.x), int(marker.y))
        valid[marker.landmarkID.data][0] += 1
        valid[marker.landmarkID.data][1] = 1

    reprojection = np.zeros((758, 1024, 3), np.uint8)
    for name, corners in gates.items():
        if valid[name][0] == 4 and valid[name][1] == 1:
            cv2.line(reprojection, corners[0], corners[1], (255,128,0), 1)
            cv2.line(reprojection, corners[1], corners[2], (255,128,0), 1)
            cv2.line(reprojection, corners[2], corners[3], (255,128,0), 1)
            cv2.line(reprojection, corners[3], corners[0], (255,128,0), 1)
    cv2.imshow("Reprojection", reprojection)
    cv2.waitKey(1)
    
    rospy.loginfo(valid)


def pubRateThrust():
    # pub = rospy.Publisher('output/rateThrust', RateThrust, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        thr_msg = RateThrust()
        thr_msg.header.frame_id = "uav/imu"
        thr_msg.header.stamp = rospy.Time.now()

        marker = IRMarker()

        pitch = 0.0;
        roll = -0.1;
        yaw = 0.5;
        vertical = 11;

        thr_msg.angular_rates.x = roll;
        thr_msg.angular_rates.y = pitch;
        thr_msg.angular_rates.z = yaw;
        thr_msg.thrust.z = vertical;

        # rospy.loginfo(thr_msg)
        # pub.publish(thr_msg)
        rate.sleep()

global gates
if __name__ == '__main__':
    gates = {"Gate1":[(0,0),(0,0),(0,0),(0,0)],
    "Gate2":[(0,0),(0,0),(0,0),(0,0)],
    "Gate3":[(0,0),(0,0),(0,0),(0,0)],
    "Gate4":[(0,0),(0,0),(0,0),(0,0)],
    "Gate5":[(0,0),(0,0),(0,0),(0,0)],
    "Gate6":[(0,0),(0,0),(0,0),(0,0)],
    "Gate7":[(0,0),(0,0),(0,0),(0,0)],
    "Gate8":[(0,0),(0,0),(0,0),(0,0)],
    "Gate9":[(0,0),(0,0),(0,0),(0,0)],
    "Gate10":[(0,0),(0,0),(0,0),(0,0)],
    "Gate11":[(0,0),(0,0),(0,0),(0,0)],
    "Gate12":[(0,0),(0,0),(0,0),(0,0)],
    "Gate13":[(0,0),(0,0),(0,0),(0,0)],
    "Gate14":[(0,0),(0,0),(0,0),(0,0)],
    "Gate15":[(0,0),(0,0),(0,0),(0,0)],
    "Gate16":[(0,0),(0,0),(0,0),(0,0)],
    "Gate17":[(0,0),(0,0),(0,0),(0,0)],
    "Gate18":[(0,0),(0,0),(0,0),(0,0)],
    "Gate19":[(0,0),(0,0),(0,0),(0,0)],
    "Gate20":[(0,0),(0,0),(0,0),(0,0)],
    "Gate21":[(0,0),(0,0),(0,0),(0,0)],
    "Gate22":[(0,0),(0,0),(0,0),(0,0)],
    "Gate21":[(0,0),(0,0),(0,0),(0,0)]}
    try:
        rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, IRMarkerArrayCallback)
        pubRateThrust()
    except rospy.ROSInterruptException:
        pass
