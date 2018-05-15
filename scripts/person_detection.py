import rospy
import cv_bridge
from saam_object_detection.srv import WhatAmILookingAt
import json
import numpy as np


class PersonDetection:
    def __init__(self, service_name='what_am_i_looking_at'):
        rospy.wait_for_service(service_name)
        self.service = rospy.ServiceProxy(service_name, WhatAmILookingAt)
        self.bridge = cv_bridge.CvBridge()

    @staticmethod
    def json_str_to_dict(json_str):
        data = json.loads(json_str)
        objects_dict = dict()
        # based on saam-object-detection document
        objects_dict['num_detections'] = int(data['num_detections'])
        objects_dict['detection_boxes'] = np.array(data['detection_boxes'])
        objects_dict['detection_classes'] = np.array(data['detection_classes'])
        objects_dict['detection_scores'] = np.array(data['detection_scores'])
        return objects_dict

    def detect(self, image):
        image_h, image_w = image.shape[:2]

        encoded_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        resp = self.service(input_image=encoded_image)
        resp = resp.objects_json
        resp = self.json_str_to_dict(resp.data)
        persons = []
        for i in range(resp["num_detections"]):
            if resp["detection_classes"][i] != "person" or resp["detection_scores"][i] < 0.25:
                continue

            y1, x1, y2, x2 = resp['detection_boxes'][i]
            x1 = int(image_w * x1)
            y1 = int(image_h * y1)
            x2 = int(image_w * x2)
            y2 = int(image_h * y2)
            item = [(x1, y1, x2, y2), resp["detection_scores"][i]]
            persons.append(item)

        return persons
