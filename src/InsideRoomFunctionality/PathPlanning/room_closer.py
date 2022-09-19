import math
from .BaseStructures.point import Point
from .BaseStructures.line import Line

class RoomCloser:
    # Constructors
    def __init__(self, map, distance, angle, entrance):
        angle = angle % 360;

        self.map = map;
        self.A = Point(global_map=map);
        self.A.add_real_points(entrance[0],entrance[1]);
        self.AB = self.compute_AB(angle);
        self.A_wall = self.compute_wall_para_A();
        self.B = self.compute_B(distance, angle);
        self.B_wall = self.compute_wall_B();

    def compute_block(self, block_value):
        points = self.find_points(self.B_wall);
        self.draw_pixel_line_arr(points[0].get_global(), points[1].get_global(), block_value);

    # Matrix operations

    # Line square intersections
    # minX(x0),maxY(x1)           1              maxX(x1),maxY(y1)
    #  |_________________________________________________|
    #  |                                                 |
    #  |\                                                |
    #  | -----------                                     |
    # 4|            \                                    |2
    #  |             ---------                           |
    #  |                      \                          |
    #  |                       --------------------      |
    #  |___________________________________________\_____|
    # minX(x0),maiY(x0)           3              maxX(x1),minY(y0)

    def find_points(self, main_line):
        width = (self.map.width-1) * self.map.resolution;
        height = (self.map.height-1) * self.map.resolution;
        minX, minY = self.map.origin;
        maxX, maxY = minX+width, minY+height;
        lines = self.square_lines(minX, minY, maxX, maxY);
        points_inter = []
        for i in range(len(lines)):
            point = lines[i].get_line_intersection(main_line);
            if point != [None,None]:
                points_inter.append(point);
        return self.square_points(points_inter, minX, minY, maxX, maxY);

    # Compute line equation for the square
    def square_lines(self,x0,y0,x1,y1):
        p1 = Point(global_map=self.map);
        p2 = Point(global_map=self.map);
        p3 = Point(global_map=self.map);
        p4 = Point(global_map=self.map);
        p1.add_real_points(x0, y1);
        p2.add_real_points(x1, y1);
        p3.add_real_points(x1, y0);
        p4.add_real_points(x0, y0);
        line1 = Line();
        line1.add_line_points(p1, p2);
        line2 = Line()
        line2.add_line_points(p2, p3);
        line3 = Line()
        line3.add_line_points(p3, p4);
        line4 = Line()
        line4.add_line_points(p4, p1);
        return [line1, line2, line3, line4];

    # Check if the intersection pointy are withing the rectangle
    def square_points(self, points, x0, y0, x1, y1):
        final_points = [];

        if points[0][0]>=x0 and points[0][0]<=x1:
            point = Point(global_map=self.map);
            point.add_real_points(points[0][0],points[0][1])
            final_points.append(point);
        if points[1][1]>=y0 and points[1][1]<=y1:
            point = Point(global_map=self.map);
            point.add_real_points(points[1][0],points[1][1])
            final_points.append(point);
        if len(points) > 2:
            if points[2][0]>=x0 and points[2][0]<=x1:
                point = Point(global_map=self.map);
                point.add_real_points(points[2][0],points[2][1])
                final_points.append(point);
        if len(points) > 3:
            if points[3][1]>=y0 and points[3][1]<=y1:
                point = Point(global_map=self.map);
                point.add_real_points(points[3][0],points[3][1])
                final_points.append(point);
        if len(final_points)!= 2:
            if len(final_points) > 2:
                for i in range(len(final_points)):
                    for j in range(i+1,len(final_points)):
                        if final_points[i].get_real() == final_points[j].get_real():
                            j -= 1;
                            final_points.pop(j);
                if len(final_points) == 2:
                    return final_points;
                else:
                    return [];
            else:
                return [];
        else:
            return final_points;

    # Other
    def draw_pixel_line_arr(self,point1, point2, block_value):
        self.draw_pixel_line(point1[0], point1[1], point2[0], point2[1],block_value);

    def draw_pixel_line(self, x0, y0, x1, y1, block_value):
        step_number  = int(abs(x0-x1) + abs(y0-y1))
        step_size = 1.0/step_number;
        p = [];
        t = 0.0;
        for i in range(step_number):
            p.append([x0 * t + x1 * (1 - t), y0 * t + y1 * (1 - t)]);
            t+=step_size;
        for i in range(len(p)):
            self.map.set_element(int(round(p[i][0])), int(round(p[i][1])), block_value);

    # Compute lines and points
    def compute_AB(self, angle, type_a="d"):
        if type_a == "r":
            angle = math.degrees(angle);
        A = self.A.get_real();
        if angle == 0 or angle == 180:
            mAB = 0;
            nAB = A[1];
        elif angle == 90 or angle == 270:
            mAB = "undef"
            nAB = A[0]  # this represents the line x = A[0]
        else:
            angle = math.radians(angle);
            mAB = math.tan(angle);
            nAB = A[1] - mAB * A[0];
        return Line(mAB, nAB);

    def compute_B(self,distance, angle, type_a="d"):
        if type_a == "d":
            angle = math.radians(angle);
        A = self.A.get_real();
        x = A[0] + distance * math.cos(angle);
        y = A[1] + distance * math.sin(angle);
        #x = A[0] + distance * math.sin(angle);
        #y = A[1] + distance * math.cos(angle);
        B = Point(global_map=self.map);
        B.add_real_points(x, y);
        return B;

    def compute_wall_B(self):
        B = self.B.get_real();
        if self.AB.get_m() == 0:
            mB = "undef";
            nB = B[0];  # this represents the line x = B[0]
        elif self.AB.get_m() == "undef":
            mB = 0;
            nB = B[1];
        else:
            mB = -1 / self.AB.get_m();
            nB = B[1] - mB * B[0];
        return Line(mB, nB);

    def compute_wall_para_A(self):
        A = self.A.get_real();
        if self.AB.get_m() == 0:
            return Line('undef',A[0])
        elif self.AB.get_m()=='undef':
            return Line(0,A[1]);
        else:
            mA = -1 / self.AB.get_m();
            nA = A[1] - mA * A[0];
            return Line(mA, nA);
