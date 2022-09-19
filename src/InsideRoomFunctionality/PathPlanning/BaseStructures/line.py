
class Line:
    def __init__(self, m=None, n=None):
        if m!=None and n!=None:
            self.not_null = True;
        else:
            self.not_null = False;
        self.m = m;
        self.n = n;

    # Setters
    def add_line_points(self, point1, point2):
        self.not_null=True;
        if point1.real_y - point2.real_y==0:
            self.m = 0;
            self.n = point1.real_y - point1.real_x * self.m;
        else:
            if point1.real_x - point2.real_x==0:
                self.m='undef';
                self.n = point1.real_x;
            else:
                self.m = abs(point1.real_y - point2.real_y) / abs(point1.real_x - point2.real_x);
                self.n = point1.real_y - point1.real_x * self.m;

    # Getters
    def get_m(self):
        return self.m;

    def get_n(self):
        return self.n;

    def get_line(self):
        return self.m, self.n;

    # Functionality
    def get_line_intersection(self, line):
        if self.m=='undef' or line.get_m() == 'undef':
            if self.m=='undef' and line.get_m() == 'undef':
                return [None, None];
            if self.m == 'undef':
                return [self.n,line.get_m()*self.n + line.get_n()];
            else:
                return [line.get_n(),self.m*line.get_n() + self.n];

        if self.m - line.get_m() != 0:
            x = (line.get_n() - self.n) / (self.m - line.get_m());
            y = self.m * x + self.n;
            return [x, y];
        else:
            return [None, None];

    def check_point(self, point):
        p = point.get_real();
        if self.m*p[0] + self.n == p[1]:
            return True;
        return False;




