import rospy


from geometry_msgs.msg import Vector3, Quaternion, Pose
from sensor_msgs.msg import JointState


class LivelyNode:
    pass


if __name__ == "__main__":
    rospy.init_node("lively_node")
    node = LivelyNode()
    rospy.spin()