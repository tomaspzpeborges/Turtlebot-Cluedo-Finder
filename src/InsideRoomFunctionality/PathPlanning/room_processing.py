import numpy as np

class RoomProcessing:
    def __init__(self, room):
        #Configure the room parameters
        self.distance_from_wall = 20;
        self.robot_clearence = 10;
        self.room = room;

        #Main map values
        self.wall = 100;

        #Proximity values
        self.proximity = {"robot_non_clearence_point": -2, "minimum_wall_distance_point": -1, "free_space_point": 0, "robot_position_point": 1};

        #Rotation index values
        self.information_index = {"proximity": 0, "x_axis_direction": 1, "y_axis_direction": 2, "x_y_main_diag_direction": 3, "x_y_sec_diag_direction": 4};

        #Rotation direction values
        self.direction = {"to_0": -1, "from_0": 1, "both": 2};


    def compute_points(self, room_map, robot_clearence=10, distance_from_wall=20, fill_path=True):
        # Configure variables
        self.distance_from_wall = distance_from_wall;
        self.robot_clearence = robot_clearence;
        self.n,self.m = room_map.shape;
        self.room_map_points = np.zeros((self.n,self.m,5,));

        # Y axis
        self.up_down(room_map);

        # X axis
        self.left_right(room_map);

        # Diagonals
        self.compute_digonals(room_map);

        # Fill the empty gaps
        if(fill_path):
            self.final_pass();
        
        return self.room_map_points;
        

    def before_after(self,indexes,max_vals, dynamic_index,room_map):
        next = 0;
        coverred = False;
        before = 0;
        after = 0;
        while(indexes[dynamic_index]<max_vals[dynamic_index] and before<self.distance_from_wall):
            if room_map[indexes[0]][indexes[1]]==self.wall:
                coverred = True;
                next = before+after;
                break;
            #______________________________________#
            if self.room_map_points[indexes[0]][indexes[1]][self.information_index["proximity"]]!=self.proximity["robot_non_clearence_point"]:
                if before<self.robot_clearence:
                    self.room_map_points[indexes[0]][indexes[1]][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                else:
                    self.room_map_points[indexes[0]][indexes[1]][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
            #______________________________________#
            indexes[dynamic_index]+=1;
            before+=1;
        if before==self.distance_from_wall:
            while(indexes[dynamic_index]<max_vals[dynamic_index] and after<self.robot_clearence):
                if room_map[indexes[0]][indexes[1]]==self.wall:
                    coverred = True;
                    next = before+after;
                    break;
                indexes[dynamic_index]+=1;
                after+=1;
            if after==self.robot_clearence: #All good
                if dynamic_index==0:
                    if self.room_map_points[indexes[0]-after][indexes[1]][self.information_index["proximity"]] == self.proximity["free_space_point"]:
                        self.room_map_points[indexes[0]-after][indexes[1]][self.information_index["proximity"]] = self.proximity["robot_position_point"];
                        if self.room_map_points[indexes[0]-after][indexes[1]][self.information_index["x_axis_direction"]]==self.direction["from_0"]:
                            self.room_map_points[indexes[0]-after][indexes[1]][self.information_index["x_axis_direction"]] = self.direction["both"];
                        else:
                            self.room_map_points[indexes[0]-after][indexes[1]][self.information_index["x_axis_direction"]] = self.direction["to_0"];
                else:
                    if self.room_map_points[indexes[0]][indexes[1]-after][self.information_index["proximity"]] == self.proximity["free_space_point"]:
                        self.room_map_points[indexes[0]][indexes[1]-after][self.information_index["proximity"]] = self.proximity["robot_position_point"];
                        if self.room_map_points[indexes[0]][indexes[1]-after][self.information_index["y_axis_direction"]] == self.direction["from_0"]:
                            self.room_map_points[indexes[0]][indexes[1]-after][self.information_index["y_axis_direction"]] = self.direction["both"];
                        else:
                            self.room_map_points[indexes[0]][indexes[1]-after][self.information_index["y_axis_direction"]] = self.direction["to_0"];
                coverred = True;
                next = before+after;
            else: #Not enough clearence / maybe closer to the wall but more than clearence?
                pass; 
            after = 0;
        else: #Black before wall distance
            if before>self.robot_clearence:
                pass; #Move robot closer to the wall?
            else:
                pass;#Robot doesnt fit
        before = 0;
        return indexes[dynamic_index],next,coverred;

    def before_after_diag(self,i,j,pos,room_map,diag_type):
        next = 0;
        coverred = False;
        before = 0;
        after = 0;
        while(i<self.n and i>=0 and j<self.m and before<self.distance_from_wall):
            if room_map[i][j]==self.wall:
                coverred = True;
                next = before+after;
                break;
            #______________________________________#
            if self.room_map_points[i][j][self.information_index["proximity"]]!=self.proximity["robot_non_clearence_point"]:
                if before<self.robot_clearence:
                    self.room_map_points[i][j][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                else:
                    self.room_map_points[i][j][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
            #______________________________________#
            i+=pos;
            j+=1;
            before+=1;
        if before==self.distance_from_wall:
            while(i<self.n and i>=0 and j<self.m and after<self.robot_clearence):
                if room_map[i][j]==self.wall:
                    coverred = True;
                    next = before+after;
                    break;
                i+=pos;
                j+=1;
                after+=1;
            if after==self.robot_clearence: #All good
                if self.room_map_points[i-pos*after][j-after][self.information_index["proximity"]] == self.proximity["free_space_point"]:
                    self.room_map_points[i-pos*after][j-after][self.information_index["proximity"]] = self.proximity["robot_position_point"];
                    if self.room_map_points[i-pos*after][j-after][diag_type] == self.direction["from_0"]:
                        self.room_map_points[i-pos*after][j-after][diag_type] = self.direction["both"];
                    else:
                        self.room_map_points[i-pos*after][j-after][diag_type] = self.direction["to_0"];
                coverred = True;
                next = before+after;
            else: #Not enough clearence / maybe closer to the wall but more than clearence?
                pass; 
            after = 0;
        else: #Black before wall distance
            if before>self.robot_clearence:
                pass; #Move robot closer to the wall?
            else:
                pass;#Robot doesnt fit
        before = 0;
        return i,j,next,coverred;

    def after_diag(self,i,j,next,pos,diag_type):
        if next>self.distance_from_wall:
            if self.room_map_points[i-pos*self.distance_from_wall][j-self.distance_from_wall][self.information_index["proximity"]] == self.proximity["free_space_point"]:
                self.room_map_points[i-pos*self.distance_from_wall][j-self.distance_from_wall][self.information_index["proximity"]] = self.proximity["robot_position_point"];
                if  self.room_map_points[i-pos*self.distance_from_wall][j-self.distance_from_wall][diag_type] == self.direction["to_0"]:
                    self.room_map_points[i-pos*self.distance_from_wall][j-self.distance_from_wall][diag_type] = self.direction["both"];
                else:
                    self.room_map_points[i-pos*self.distance_from_wall][j-self.distance_from_wall][diag_type] = self.direction["from_0"];
            #______________________________________#
            for z in range(1,self.distance_from_wall):
                if self.room_map_points[i-pos*z][j-z][self.information_index["proximity"]]!=self.proximity["robot_non_clearence_point"]:
                    if z<self.robot_clearence:
                        self.room_map_points[i-pos*z][j-z][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                    else:
                        self.room_map_points[i-pos*z][j-z][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
            #______________________________________#
        else: 
            if next>self.robot_clearence:
                pass; #Move robot closer to the wall?
            #______________________________________#
            for z in range(1,next):
                if self.room_map_points[i-pos*z][j-z][self.information_index["proximity"]]!=self.proximity["robot_non_clearence_point"]:
                    if z<self.robot_clearence:
                        self.room_map_points[i-pos*z][j-z][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                    else:
                        self.room_map_points[i-pos*z][j-z][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
            #______________________________________#
        
    def up_down(self,room_map):
        #Up-Down
        for i in range(self.n):
            next = 0;
            coverred = False;
            j = 0;
            while(j<self.m):
                if room_map[i][j]==0 and coverred==False:
                    j,next,coverred = self.before_after([i,j],[self.n,self.m],1,room_map);
                if room_map[i][j]==self.wall and coverred==True:
                    if next>self.distance_from_wall:
                        self.room_map_points[i][j-self.distance_from_wall][self.information_index["proximity"]] = self.proximity["robot_position_point"];
                        if self.room_map_points[i][j-self.distance_from_wall][self.information_index["y_axis_direction"]] == self.direction["to_0"]:
                            self.room_map_points[i][j-self.distance_from_wall][self.information_index["y_axis_direction"]] = self.direction["both"];
                        else:
                            self.room_map_points[i][j-self.distance_from_wall][self.information_index["y_axis_direction"]] = self.direction["from_0"];
                        #Add close regions before black line
                        #______________________________________#
                        for z in range(1,self.distance_from_wall):
                            if z<self.robot_clearence:
                                self.room_map_points[i][j-z][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                            else:
                                self.room_map_points[i][j-z][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
                        #______________________________________#
                    else: 
                        if next>self.robot_clearence:
                            pass; #Move robot closer to the wall?
                        #Add close regions before black line
                        #______________________________________#
                        for z in range(1,next):
                            if z<self.robot_clearence:
                                self.room_map_points[i][j-z][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                            else:
                                self.room_map_points[i][j-z][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
                        #______________________________________#
                    coverred = False; 
                next+=1;
                j+=1;

    def left_right(self,room_map):
        #Left-Right
        for j in range(self.m):
            next = 0;
            coverred = False;
            i = 0;
            while(i<self.n):
                if room_map[i][j]==0 and coverred==False:
                    i,next,coverred = self.before_after([i,j],[self.n,self.m],0,room_map);
                if room_map[i][j]==self.wall and coverred==True:
                    if next>self.distance_from_wall:
                        if self.room_map_points[i-self.distance_from_wall][j][self.information_index["proximity"]] > -1:
                            self.room_map_points[i-self.distance_from_wall][j][self.information_index["proximity"]] = self.proximity["robot_position_point"];
                            if self.room_map_points[i-self.distance_from_wall][j][self.information_index["x_axis_direction"]] == self.direction["to_0"]:
                                self.room_map_points[i-self.distance_from_wall][j][self.information_index["x_axis_direction"]] = self.direction["both"];
                            else:
                                self.room_map_points[i-self.distance_from_wall][j][self.information_index["x_axis_direction"]] = self.direction["from_0"];
                        #______________________________________#
                        for z in range(1,self.distance_from_wall):
                            if self.room_map_points[i-z][j][self.information_index["proximity"]]!=self.proximity["robot_non_clearence_point"]:
                                if z<self.robot_clearence:
                                    self.room_map_points[i-z][j][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                                else:
                                    self.room_map_points[i-z][j][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
                        #______________________________________#
                    else: 
                        if next>self.robot_clearence:
                            pass; #Move robot closer to the wall?
                        #______________________________________#
                        for z in range(1,next):
                            if self.room_map_points[i-z][j][self.information_index["proximity"]]!=self.proximity["robot_non_clearence_point"]:
                                if z<self.robot_clearence:
                                    self.room_map_points[i-z][j][self.information_index["proximity"]] = self.proximity["robot_non_clearence_point"];
                                else:
                                    self.room_map_points[i-z][j][self.information_index["proximity"]] = self.proximity["minimum_wall_distance_point"];
                        #______________________________________#
                    coverred = False;    
                next+=1;
                i+=1;
        


    def compute_digonals(self,room_map):
        
        #Main Diagonals
        for p in range(self.n):
            j = 0;
            i = p;
            next = 0;
            coverred = False;
            while i<self.n and j<self.m:
                if room_map[i][j]==0 and coverred==False:
                    i,j,next,coverred = self.before_after_diag(i,j,1,room_map,self.information_index["x_y_main_diag_direction"]);
                if room_map[i][j]==self.wall and coverred==True:
                    self.after_diag(i,j,next,1,self.information_index["x_y_main_diag_direction"]);
                    coverred =False;
                next+=1;
                i+=1;
                j+=1;                

        for k in range(self.m):
            j = k;
            i = 0;
            next = 0;
            coverred = False;
            while i<self.n and j<self.m:
                if room_map[i][j]==0 and coverred==False:
                    i,j,next,coverred = self.before_after_diag(i,j,1,room_map,self.information_index["x_y_main_diag_direction"]);
                if room_map[i][j]==self.wall and coverred==True:
                    self.after_diag(i,j,next,1,self.information_index["x_y_main_diag_direction"]);
                    coverred =False;
                next+=1;
                i+=1;
                j+=1;        


        #Secoundary Diagonals
        for p in range(self.n-1,0,-1):
            j = 0;
            i = p;
            next = 0;
            coverred = False;
            while i>=0 and j<self.m:
                if room_map[i][j]==0 and coverred==False:
                    i,j,next,coverred = self.before_after_diag(i,j,-1,room_map,self.information_index["x_y_sec_diag_direction"]);
                if room_map[i][j]==self.wall and coverred==True:
                    self.after_diag(i,j,next,-1,self.information_index["x_y_sec_diag_direction"]);
                    coverred =False;
                next+=1;
                i-=1;
                j+=1;                

        for k in range(self.m):
            j = k;
            i = self.n-1;
            next = 0;
            coverred = False;
            while i>=0 and j<self.m:
                if room_map[i][j]==0 and coverred==False:
                    i,j,next,coverred = self.before_after_diag(i,j,-1,room_map,self.information_index["x_y_sec_diag_direction"]);
                if room_map[i][j]==self.wall and coverred==True:
                    self.after_diag(i,j,next,-1,self.information_index["x_y_sec_diag_direction"]);
                    coverred = False;
                next+=1;
                i-=1;
                j+=1;     

    def final_pass(self):
        for i in range(self.robot_clearence,self.n-self.robot_clearence):
            for j in range(self.robot_clearence,self.m-self.robot_clearence):
                if self.room_map_points[i][j][self.information_index["proximity"]]==self.proximity["free_space_point"]:
                    if self.room_map_points[i-1][j][self.information_index["proximity"]]==self.proximity["minimum_wall_distance_point"]:
                        self.room_map_points[i][j][self.information_index["proximity"]]=self.proximity["robot_position_point"];
                        if self.room_map_points[i][j][self.information_index["y_axis_direction"]] == self.direction["from_0"]:
                            self.room_map_points[i][j][self.information_index["y_axis_direction"]]=self.direction["both"];
                        else:
                            self.room_map_points[i][j][self.information_index["y_axis_direction"]]=self.direction["to_0"];
                    if self.room_map_points[i+1][j][self.information_index["proximity"]]==self.proximity["minimum_wall_distance_point"]:
                        self.room_map_points[i][j][self.information_index["proximity"]]=self.proximity["robot_position_point"];
                        if self.room_map_points[i][j][self.information_index["y_axis_direction"]] == self.direction["to_0"]:
                            self.room_map_points[i][j][self.information_index["y_axis_direction"]]=self.direction["both"];
                        else:
                            self.room_map_points[i][j][self.information_index["y_axis_direction"]]=self.direction["from_0"];
                    if self.room_map_points[i][j-1][self.information_index["proximity"]]==self.proximity["minimum_wall_distance_point"]: 
                        self.room_map_points[i][j][self.information_index["proximity"]]=self.proximity["robot_position_point"];
                        if self.room_map_points[i][j][self.information_index["x_axis_direction"]] == self.direction["from_0"]:
                            self.room_map_points[i][j][self.information_index["x_axis_direction"]]=self.direction["both"];
                        else:
                            self.room_map_points[i][j][self.information_index["x_axis_direction"]]=self.direction["to_0"];
                    if self.room_map_points[i][j+1][self.information_index["proximity"]]==self.proximity["minimum_wall_distance_point"]:
                        self.room_map_points[i][j][self.information_index["proximity"]]=self.proximity["robot_position_point"];
                        if self.room_map_points[i][j][self.information_index["x_axis_direction"]] == self.direction["to_0"]:
                            self.room_map_points[i][j][self.information_index["x_axis_direction"]]=self.direction["both"];
                        else:
                            self.room_map_points[i][j][self.information_index["x_axis_direction"]]=self.direction["from_0"];
    