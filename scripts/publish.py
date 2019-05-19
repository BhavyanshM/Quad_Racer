#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from mav_msgs.msg import RateThrust
from flightgoggles.msg import IRMarker, IRMarkerArray
from subscribe import IRMarkerArrayCallback

def pubRateThrust():
    pub = rospy.Publisher('output/rateThrust', RateThrust, queue_size=10)
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
        pub.publish(thr_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, IRMarkerArrayCallback)
        pubRateThrust()
    except rospy.ROSInterruptException:
        pass
