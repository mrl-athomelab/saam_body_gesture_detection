import time
import random


class Person:
    def __init__(self, x, y, person_id=None, queue_limit=5, capture_time=0.5):
        self.x = x
        self.y = y
        self.z = 0
        self.queue_limit = queue_limit
        self.capture_time = capture_time

        if person_id is None:
            person_id = random.randrange(10000, 99999)

        self.person_id = person_id

        self.point_queue = []
        self.latest_time = time.time()

    def __str__(self):
        return "x: {}, y: {}, id: {}".format(self.x, self.y, self.person_id)

    def add_point(self, point):
        current_time = time.time()
        if current_time - self.latest_time < self.capture_time:
            return

        self.latest_time = current_time

        self.point_queue.append(point)
        if len(self.point_queue) > self.queue_limit:
            self.point_queue = self.point_queue[-self.queue_limit:]

    def get_wave_label(self):
        return "UNKNOWN"
