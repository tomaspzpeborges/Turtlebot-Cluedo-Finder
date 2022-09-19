import math

class Laser:

    def __init__(self,data):
        self.min_angle = data.angle_min;
        self.max_angle = data.angle_max;
        self.increments = data.angle_increment;
        self.max = data.range_max;
        self.min = data.range_min;
        self.values = [];

        self.full_range = abs(data.angle_min) + abs(data.angle_max);

        self.left = self.min_angle/self.increments;
        self.total = self.full_range/self.increments;
        self.right = self.total-self.left;

    def add_value(self,values):
        self.values = values;

    def get_value_degree(self,degree):# -29.88363825366 , 30.038820215052688 - 0.0937753606165639
        self.aux = degree/self.increments;
        number_values = self.values/self.full_range;
        half_number_values = number_values/2;
        return self.values[self.aux-int(half_number_values):self.aux+int(half_number_values)];

        