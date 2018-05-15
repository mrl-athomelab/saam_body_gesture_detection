import rospy
import cv_bridge
import json
from saam_pose_estimation.srv import PoseEstimator


class PoseEstimation:
    def __init__(self, service_name='saam_pose_estimation'):
        rospy.wait_for_service(service_name)
        self.service = rospy.ServiceProxy(service_name, PoseEstimator)
        self.bridge = cv_bridge.CvBridge()

    def detect(self, image):
        image_h, image_w = image.shape[:2]

        encoded_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        resp = self.service(input_image=encoded_image)
        resp = resp.objects_json.data
        resp = json.loads(resp)
        for i in range(len(resp)):
            for j in resp[i].keys():
                resp[i][j][0] = int(image_w * resp[i][j][0])
                resp[i][j][1] = int(image_h * resp[i][j][1])
        return resp
