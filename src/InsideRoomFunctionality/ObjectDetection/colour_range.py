class ColourRange:

    # Constructors
    def __init__(self, colour1, colour2, name=""):
        self.name = name;
        self.colour1 = colour1;
        self.colour2 = colour2;

    # Getters
    def get_colour1(self):
        return self.colour1;

    def get_colour2(self):
        return self.colour2;

    # Checks
    def is_within_range(self, colour):
        res = False;
        if colour.h <= self.colour1.h and colour.s <= self.colour1.s and colour.v <= self.colour1.v and colour.h >= self.colour2.h and colour.s >= self.colour2.s and colour.v >= self.colour2.v:
            res = True;
        if colour.h <= self.colour2.h and colour.s <= self.colour2.s and colour.v <= self.colour2.v and colour.h >= self.colour1.h and colour.s >= self.colour1.s and colour.v >= self.colour1.v:
            res = True;
        return res;
