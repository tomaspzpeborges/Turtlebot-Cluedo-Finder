class Point(object):

    def __init__(self, quaternion, distance):
        self.quaternion = quaternion
        self.distance = distance

    def get_quaternion(self):
        return self.quaternion

    def get_distance(self):
        return self.distance

    def __str__(self):
        return str(self.quaternion) + ' \n' + str(self.distance)
