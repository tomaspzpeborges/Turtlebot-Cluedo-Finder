import math

class Point:
    def __init__(self,local_map=None,global_map=None,conversion=None):
        if local_map!=None:
            self.local_map = local_map;
            self.global_map = global_map;
            self.conversion = conversion;
        else:
            self.global_map = global_map;
            self.conversion = None;

    #Setters
    def add_global_points(self,x,y):
        self.global_x = x;
        self.global_y = y;
        if self.conversion!=None:
            self.local_x = self.global_x-self.conversion[0];
            self.local_y = self.global_y-self.conversion[1];
        self.real_x,self._real_y = self.global_map.pixel_to_world(self.global_x,self.global_y);
       
    def add_local_points(self,x,y):
        if self.conversion != None:
            self.local_x = x;
            self.local_y = y;
            self.global_x = self.local_x+self.conversion[0];
            self.global_y = self.local_y+self.conversion[1];
            self.real_x,self.real_y = self.global_map.pixel_to_world(self.global_x,self.global_y);

    def add_real_points(self,x,y):
        self.real_x = x;
        self.real_y = y;
        self.global_x,self.global_y = self.global_map.world_to_pixel(self.real_x, self.real_y);
        if self.conversion != None:
            self.local_x = self.global_x-self.conversion[0];
            self.local_y = self.global_y-self.conversion[1];

    #Getters
    def get_global(self):
        return [self.global_x,self.global_y];

    def get_local(self):
        if self.conversion != None:
            return [self.local_x,self.local_y];

    def get_real(self):
        return [self.real_x,self.real_y];

    #Static functionality
    @staticmethod   
    def get_distance(point1, point2, type):
        if type== "global":
            return math.hypot(point2.global_x - point1.global_x, point2.global_y - point1.global_y);
        elif type=="local":
            return math.hypot(point2.local_x - point1.local_x, point2.local_y - point1.local_y);
        elif type== "real":
            return math.hypot(point2.real_x - point1.real_x, point2.real_y - point1.real_y);
        
    @staticmethod  
    def order_points_proximity_strat_0(points):
        after = [];
        after.append(points.pop(0));
        while(len(points)!=0):
            aux_distance = 99999999999999999;
            for i in range(len(points)):
                distance = Point.get_distance(after[-1],points[i],"local");
                if distance<aux_distance:
                    aux_distance = distance;
                    save_i = i;
            after.append(points.pop(save_i));
        return after;
