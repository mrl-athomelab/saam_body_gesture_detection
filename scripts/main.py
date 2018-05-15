#!/usr/bin/env python

import rospy
import cv2
import time
from person_detection import PersonDetection
from pose_estimation import PoseEstimation
from camera_reader import CameraStreamer
from persons import Persons

# color constants
RED_COLOR = (0, 0, 255)
BLUE_COLOR = (255, 0, 0)
WHITE_COLOR = (255, 255, 255)
DEFAULT_POINTS = {'1', '2', '3', '4', '5', '6', '7'}


def visualize_points(image, points):
    for key in points:
        point = points[key]
        cv2.putText(image, key, (point[0] - 5, point[1] - 5), cv2.FONT_HERSHEY_PLAIN,
                    1.0, RED_COLOR)
        cv2.circle(image, tuple(point), 2, WHITE_COLOR, -1)


def limit_points(points):
    output = {}
    for point in points:
        if point in DEFAULT_POINTS:
            output[point] = points[point]
    return output


class WaveDetector:
    def __init__(self):
        rospy.loginfo("Connecting to person detection service ...")
        self.pd = PersonDetection(service_name='what_am_i_looking_at')

        rospy.loginfo("Connecting to pose estimation service ...")
        self.pe = PoseEstimation()

        rospy.loginfo("Connecting to camera ...")
        self.camera = CameraStreamer(image_topic="/usb_cam/image_raw", scale=1.0)

        self.persons = Persons()

    def process(self):
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)

        while True:
            start_time = time.time()

            ret, image = self.camera.read()
            if not ret:
                rospy.logwarn("Frame skipped !")
                continue

            bodies = self.filter_human_bodies(self.pd.detect(image), self.pe.detect(image))
            for body in bodies:
                person = body["person"]
                if person[1] < 0.6:
                    continue

                points = body["points"]
                if '5' not in points:
                    continue
                points = limit_points(points)

                selected_person, exists = self.persons.find_nearest(points['1'])
                cv2.circle(image, (selected_person.x, selected_person.y), 4, BLUE_COLOR, -1)

                selected_person.add_point(points)

                if not exists:
                    self.persons.add(selected_person)
                else:
                    self.persons.update(selected_person.person_id, points['1'])

                visualize_points(image, points)

                x1, y1, x2, y2 = person[0]
                text = "{:.2f}, {}, {}".format(person[1], selected_person.person_id, selected_person.get_wave_label())
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_PLAIN, 1.0, 1)
                cv2.rectangle(image, (x1 - 1, y1), (x1 + text_size[0][0], y1 - text_size[0][1] - 15), RED_COLOR, -1)
                cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_PLAIN, 1.0, WHITE_COLOR)
                cv2.rectangle(image, (x1, y1), (x2, y2), RED_COLOR, 2)

            process_time = time.time() - start_time
            cv2.putText(image, "Process time : {:.2f}".format(process_time), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1.0,
                        RED_COLOR)
            cv2.imshow("image", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.logwarn("The q key has been pressed !")
                break

    @staticmethod
    def filter_human_bodies(persons_list, humans_list):
        output = []
        for person in persons_list:
            x1, y1, x2, y2 = person[0]

            for body_parts in humans_list:
                for part in body_parts:
                    x, y = body_parts[part]
                    if x1 < x < x2 and y1 < y < y2:
                        output.append({"person": person, "points": body_parts})
                        break
        return output


if __name__ == "__main__":
    rospy.init_node("human_wave_detection", anonymous=True)

    instance = WaveDetector()
    try:
        instance.process()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting done ...")
    finally:
        rospy.loginfo("Exiting ...")
