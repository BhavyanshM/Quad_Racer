import rospy


def IRMarkerArrayCallback(markers):
	rospy.loginfo(markers)