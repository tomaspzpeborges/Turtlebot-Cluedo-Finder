

class Initialpoints:
    def __init__(self, path, filename):
        self.filename = filename;
        self.path = path;
        self.error = False;
        self.setup();

    def setup(self):
        self.f = open(self.path + self.filename, 'r')
        lines = self.f.readlines();
        self.f.close();
        for line in lines:
            elems = line.split(":");
            if len(elems) == 2:
                elems[0] = elems[0].replace(' ', '');
                elems[1] = elems[1].replace(' ', '');
                elems[1] = elems[1].replace('[', '');
                elems[1] = elems[1].replace(']', '');
                aux = elems[1].split(",");
                if elems[0] == "room1_entrance_xy":
                    self.entrance1 = [float(aux[0]), float(aux[1])];
                elif elems[0] == "room2_entrance_xy":
                    self.entrance2 = [float(aux[0]), float(aux[1])];
                elif elems[0] == "room1_centre_xy":
                    self.center1 = [float(aux[0]), float(aux[1])];
                elif elems[0] == "room2_centre_xy":
                    self.center2 = [float(aux[0]), float(aux[1])];

    def test_input(self):
        print(self.entrance1);
        print(self.entrance2);
        print(self.center1);
        print(self.center2);