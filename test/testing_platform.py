from polhemus_ros_driver.knuckle_position_calibration_action_server import SrGloveCalibration
from polhemus_ros_driver.msg import CalibrateGoal, CalibrateActionGoal, CalibrateAction
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
import rosbag
import rospy
import actionlib

rospy.init_node('test')


files = ['calibration_1.bag', 'calibration_2.bag']
fingers = ('ff', 'mf', 'rf', 'lf')

side_prefix = 'lh'
calibration = SrGloveCalibration()
calibration._hand_side = side_prefix
calibration._index = 1
calibration._base = 'polhemus_base_1'

calibration._initialize([side_prefix])
calibration._initialize_finger_data()

rospy.logwarn(calibration._hand_side)
rospy.logwarn(calibration._index)
rospy.logwarn(calibration._base)


for file in files:
    rospy.loginfo(f"Testing {file}")
    bag = rosbag.Bag(file)

    for topic, msg, t in bag.read_messages(topics='/tf'):
        for transform in msg.transforms:
            #print(transform)
            for finger in fingers:
                #rospy.logerr(calibration._finger_data[side_prefix][finger]['polhemus_tf_name'])
                #print(transform.child_frame_id, calibration._finger_data[side_prefix][finger]['polhemus_tf_name'])
                if transform.child_frame_id == calibration._finger_data[side_prefix][finger]['polhemus_tf_name']:
                    #print(transform.child_frame_id, calibration._finger_data[side_prefix][finger]['polhemus_tf_name'])
                    data = transform.transform.translation
                    position = [data.x, data.y, data.z]
                    calibration._finger_data[calibration._hand_side][finger]['data'].append(position)

    result = calibration._get_knuckle_positions(side_prefix)
    for finger in fingers:
        #print(finger, result[finger]['polhemus_tf_name'])
        x = result[finger]['center'].pose.position.x
        y = result[finger]['center'].pose.position.y
        z = result[finger]['center'].pose.position.z
        rospy.logerr(f"Knuckle position for {finger} at ({x},{y},{z})")

    rospy.loginfo("Finished")




