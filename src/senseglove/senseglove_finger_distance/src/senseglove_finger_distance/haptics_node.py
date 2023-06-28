import rospy
from senseglove_shared_resources.msg import SenseGloveState, FingerDistanceFloats
import time
from std_msgs.msg import Time
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rospy.init_node('senseglove_haptics_node')
    rospy.loginfo("initialize haptics node")
    hap_pub = rospy.Publisher('/senseglove/0/lh/controller/trajectory/command', JointTrajectory, queue_size=5)
    joint_list = ['empty']
    if rospy.has_param('/senseglove/0/lh/controller/trajectory/joints'):
        joint_list = rospy.get_param('/senseglove/0/lh/controller/trajectory/joints')
    publish_rate = 60
    if rospy.has_param('/senseglove/0/lh/controller/hand_state/publish_rate'):
        publish_rate = rospy.get_param('/senseglove/0/lh/controller/hand_state/publish_rate')

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        hap_cmd = JointTrajectory()
        hap_cmd.header = Header()
        hap_cmd.header.stamp = rospy.Time.now()
        hap_cmd.joint_names = joint_list
        print("joint list: ", hap_cmd.header.stamp)
        point = JointTrajectoryPoint()
        ff_cmd = [100, 100, 0, 0, 0]  # [thumb_brake, index_brake, middle_brake, ring_brake, pinky_brake]
        bz_cmd = [80, 80, 0, 0, 0]  # [thumb_cmc, index_mcp, middle_mcp, ring_mcp, pinky_mcp]
        point.positions = ff_cmd + bz_cmd
        point.time_from_start = rospy.Duration(0.02)
        hap_cmd.points.append(point)
        hap_pub.publish(hap_cmd)
        rate.sleep()
    if rospy.is_shutdown():
        return

    rospy.spin()

if __name__ == '__main__':
    main()
