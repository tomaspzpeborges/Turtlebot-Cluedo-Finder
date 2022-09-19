from .point import Point

class Location:
    #Constructors
    def __init__(self,point,arr=[]):

        #Default valuess
        #           0
        #-45_______________________ 45
        #  |           |-1         |
        #  |           |           |
        #  |           |           |
        #-90___________|___________|90
        #  |-1         |          1|
        #  |           |           |
        #  |           |1          |
        #-135______________________|135
        #             180
        self.x_directions = {"positive":0,"negative":180};
        self.y_directions = {"positive":90,"negative":-90};
        self.main_diagonal_dierctions = {"positive":135,"negative":-45};
        self.secoundary_diagonal_directions = {"positive":-135,"negative":45}; #Secoundary diagonal has the directions the other way around(sorry)

        self.direction_value = {"positive":1,"negative":-1};
    
        self.point = point;
        self.directions=0;
        self.direction=False;
        self.queue = [];
        self.rqueue = [];
        if len(arr)==0:
            self.add_directions();
        else:
            self.setup(arr);

    def setup(self,arr):
        self.x = arr[0];
        self.y = arr[1];
        self.mxy = arr[2];
        self.sxy = arr[3];
        for i in range(4):
            if arr[i]!=0:
                self.directions +=1;
        if self.directions>0:
            self.create_rotation_queue();
            self.direction=True;

    #Setters
    def add_directions(self,x=0,y=0,mxy=0,syx=0):
        self.setup([x,y,mxy,syx]);

    def add_directions_arr(self,arr):
        self.setup(arr);

    def remove_rotation(self,angle):
        #print(angle)
        for i in range(len(self.rqueue)):
            if self.rqueue[i] ==angle:
                self.rqueue.pop(i);
                #TO DO: Add more remove options
                break;

    #Getters
    def get_point(self):
        return self.point;

    def get_rotation_queue(self):
        return self.queue;

    def get_angle_queue(self):
        return self.rqueue;

    #Core functionality
    def create_rotation_queue(self):
        self.queue = [];
        
        if self.x==2:
            self.queue.append(["x",-1]);
            self.queue.append(["x",1]);
        elif self.x!=0:
            self.queue.append(["x",self.x]);

        if self.y==2:
            self.queue.append(["y",-1]);
            self.queue.append(["y",1]);
        elif self.y!=0:
            self.queue.append(["y",self.y]);

        if self.mxy==2:
            self.queue.append(["mxy",-1]);
            self.queue.append(["mxy",1]);
        elif self.mxy!=0:
            self.queue.append(["mxy",self.mxy]);
        
        if self.sxy==2:
            self.queue.append(["sxy",-1]);
            self.queue.append(["sxy",1]);
        elif self.sxy!=0:
            self.queue.append(["sxy",self.sxy]);

        self.create_angle_queue();

    def create_angle_queue(self):
        self.rqueue = [];
        for i in self.queue:
            if i[0]=="x":
                if i[1]==self.direction_value["negative"]:
                    self.rqueue.append(self.x_directions["negative"])
                else:
                    self.rqueue.append(self.x_directions["positive"]);
            if i[0]=="y":
                if i[1]==self.direction_value["negative"]:
                    self.rqueue.append(self.y_directions["negative"])
                else:
                    self.rqueue.append(self.y_directions["positive"]);
            if i[0]=="mxy":
                if i[1]==self.direction_value["negative"]:
                    self.rqueue.append(self.main_diagonal_dierctions["negative"])
                else:
                    self.rqueue.append(self.main_diagonal_dierctions["positive"]);
            if i[0]=="sxy":
                if i[1]==self.direction_value["negative"]:
                    self.rqueue.append(self.secoundary_diagonal_directions["negative"])
                else:
                    self.rqueue.append(self.secoundary_diagonal_directions["positive"]);
            
        
    #Static functionality
    @staticmethod  
    def order_locations_proximity_strat_0(locations):
        after = [];
        after.append(locations.pop(0));
        while(len(locations)!=0):
            aux_distance = 99999999999999999;
            for i in range(len(locations)):
                distance = Point.get_distance(after[-1].get_point(),locations[i].get_point(),"local");
                if distance<aux_distance:
                    aux_distance = distance;
                    save_i = i;
            after.append(locations.pop(save_i));
        return after;     
