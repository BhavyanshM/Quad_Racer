#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from mav_msgs.msg import RateThrust
from flightgoggles.msg import IRMarker, IRMarkerArray

def arrangeCorners(corners):
    ulx, uly, urx, ury, llx, lly, lrx, lry = 0,0,0,0,0,0,0,0
    for i in range(4):
        corner_x, corner_y = corners[i][0], corners[i][1]
        midpoint_x = sum([corner[0] for corner in corners])/4
        midpoint_y = sum([corner[1] for corner in corners])/4
        if(corner_x < midpoint_x and corner_y < midpoint_y):
          ulx = corner_x
          uly = corner_y
        elif(corner_x > midpoint_x and corner_y < midpoint_y):
          urx = corner_x
          ury = corner_y
        elif(corner_x > midpoint_x and corner_y > midpoint_y):
          lrx = corner_x
          lry = corner_y
        elif(corner_x < midpoint_x and corner_y > midpoint_y):
          llx = corner_x
          lly = corner_y
    # print(midpoint_x, midpoint_y)
    return llx, lly, ulx, uly, urx, ury, lrx, lry, midpoint_x, midpoint_y

def getObjectCorners(name):
    ulx, uly, urx, ury, llx, lly, lrx, lry = 0,0,0,0,0,0,0,0    
    nominal_location = rospy.get_param('/uav/'+name+'/nominal_location')
    b = np.linalg.norm(np.subtract(nominal_location[0],nominal_location[1]))
    a = np.linalg.norm(np.subtract(nominal_location[0],nominal_location[3]))
    return max(a,b), min(a,b)

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
            llx, lly, ulx, uly, urx, ury, lrx, lry, mx, my = arrangeCorners(corners)
            length, breadth = getObjectCorners(name)
            # print("L,B:", length, breadth)
            objPts = np.float32([[0, 0, 0], [length, 0, 0], [0, breadth, 0], [length, breadth, 0]])
            imgPts = np.float32([[llx, lly],[lrx, lry],[ulx, uly],[urx, ury]])
            axisPts = np.float32([[0, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]])
            fx, cx, fy, cy = 548.4088134765625, 512.0, 548.4088134765625, 384.0
            dist_coef = np.zeros(4)
            K = np.float64([[fx, 0.0, cx],[0.0, fy, cy],[0.0, 0.0, 1.0]])
            _ret, rvec, tvec = cv2.solvePnP(objPts, imgPts, K, dist_coef)
            cv2.line(reprojection, (ulx,uly), (urx,ury), (255,128,0), 1)
            cv2.line(reprojection, (urx,ury), (lrx,lry), (255,128,0), 1)
            cv2.line(reprojection, (lrx,lry), (llx,lly), (255,128,0), 1)
            cv2.line(reprojection, (llx,lly), (ulx,uly), (255,128,0), 1)            
            cv2.circle(reprojection, (mx, my), 5, (255,128,128), -1)
            projPts = cv2.projectPoints(axisPts, rvec, tvec, K, dist_coef)[0].reshape(-1, 2)
            cv2.line(reprojection, (projPts[0][0],projPts[0][1]), (projPts[1][0],projPts[1][1]), (128,255,255), 1)
            cv2.line(reprojection, (projPts[0][0],projPts[0][1]), (projPts[2][0],projPts[2][1]), (255,128,255), 1)
            cv2.line(reprojection, (projPts[0][0],projPts[0][1]), (projPts[3][0],projPts[3][1]), (255,255,128), 1)

            print(tvec)
            rotMatrix = cv2.Rodrigues(rvec)
            H = cv2.hconcat(rotMatrix, tvec)
            Z = np.float64([0,0,0,1])
            V = cv2.vconcat(H, Z)
            # cv::Mat trans = V.inv();
            # rospy.loginfo(projPts)

    cv2.imshow("Reprojection", reprojection)
    cv2.waitKey(1)
    


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
    "Gate23":[(0,0),(0,0),(0,0),(0,0)]}
    try:
        rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, IRMarkerArrayCallback)
        pubRateThrust()
    except rospy.ROSInterruptException:
        pass
