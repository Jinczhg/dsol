import cv2
from pathlib import Path
from tqdm import tqdm

import rosbag
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
import ros_numpy
import sensor_msgs

def get_img_from_ros_image_msg(msg):
    """ Returns image as a numpy array. 
    Note: can be used with any topic of message type 'sensor_msgs/Image'
    """
    msg.__class__ = sensor_msgs.msg.Image
    return ros_numpy.numpify(msg)

if __name__ == '__main__':
    bag_file = "../../../dataset/20220307_172336.bag"
    bag_name = "indoor_1"
    bag = rosbag.Bag(bag_file, "r")
    #bridge = CvBridge()
    count = 0
    topics = [
        "/device_0/sensor_0/Infrared_1/image/data",
        "/device_0/sensor_0/Infrared_2/image/data"
    ]

    data_dir = Path("../../../dataset/realsense") / bag_name
    left_dir = data_dir / "infra1"
    right_dir = data_dir / "infra2"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    for topic, msg, t in tqdm(bag.read_messages(topics=topics)):
        count = t.to_nsec() // 1000000

        #cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_img =  get_img_from_ros_image_msg(msg)
        if topic == topics[0]:
            cv2.imwrite(str(left_dir / f"image{count:08d}.png"), cv_img)
        elif topic == topics[1]:
            cv2.imwrite(str(right_dir / f"image{count:08d}.png"), cv_img)
    bag.close()
