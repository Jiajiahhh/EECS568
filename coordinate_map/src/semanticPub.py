from mmseg.apis import inference_segmentor, init_segmentor
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class VideoStreamPublisher:
    def __init__(self):
        self.node_name = "SemanticNode"
        rospy.init_node(self.node_name, anonymous=True)
        
        self.bridge = CvBridge()
     
        self.input_topic = "/camera/rgb/image_raw"  ##for simulation
        # self.input_topic = "/d435/color/image_raw"  ##for rosbag
        self.img_size = (600,375)
        self.raw_size = (640,480)
        
        self.output_topic = "/Semantic_info"
        
        config_path = '/home/wmc/rob599/SegFormer/local_configs/rellis_New/ours_std.py'
        check_path = '/home/wmc/rob599/SegFormer/work_dirs/b1_ch384_emb250_ar32.pth'
        self.model = init_segmentor(config_path, check_path, device='cpu')

        self.subscriber = rospy.Subscriber(self.input_topic, Image, self.callback)
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=10)
        # self.g = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            cv_image = cv2.resize(cv_image,self.img_size)
            # self.g+=1
            # if self.g % 15 ==0:
            #     cv2.imwrite('/home/wmc/eecs568_ws/test/'+str(int(self.g / 15))+'.jpg',cv_image)
            #     print(int(self.g / 15))
            
        except CvBridgeError as e:
            print(e)

        result = inference_segmentor(self.model, cv_image)
        result = cv2.resize(result,self.raw_size,interpolation=cv2.INTER_NEAREST)
        # cv2.imshow('tmp',result[0].astype(np.uint8)*50)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(result[0].astype(np.uint8), "mono8")
            ros_image.header = data.header
            print('xxxx')
            self.publisher.publish(ros_image)
        except CvBridgeError as e:
            print(e)
        # cv2.waitKey(0)

if __name__ == '__main__':
    VideoStreamPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down video stream publisher node.")
