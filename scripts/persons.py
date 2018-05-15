import math
from person import Person


class Persons:
    def __init__(self, threshold=20):
        self.list = {}
        self.threshold = threshold

    @staticmethod
    def distance(a, b):
        return math.sqrt(math.pow(a.x - b.x, 2) + math.pow(a.y - b.y, 2) + math.pow(a.z - b.z, 2))

    def find_nearest(self, point):
        x, y = point
        person = Person(x, y)
        latest_value = self.threshold
        for p in self.list:
            current_value = self.distance(self.list[p], person)
            print current_value
            if current_value < latest_value:
                latest_value = current_value
                person = self.list[p]

        if latest_value != self.threshold:
            return person, True

        return person, False

    def add(self, person):
        self.list[person.person_id] = person

    def update(self, person_id, point):
        self.list[person_id].x = point[0]
        self.list[person_id].y = point[1]
