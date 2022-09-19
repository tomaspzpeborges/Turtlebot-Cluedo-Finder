import math

from .map import Map
from .room_processing import RoomProcessing
from .BaseStructures.point import Point
from .BaseStructures.location import Location
from .room_closer import RoomCloser
import numpy as np
import sys

class Room:

    # Constructors
    def __init__(self, center, entrance, angle, distance_from_entrance, robot_clearence, distance_from_wall, fill_path=True):
        self.center_real = center;
        self.entrance_real = entrance;
        self.angle_wall = angle;

        self.distance_from_entrance_real = distance_from_entrance;
        self.robot_clearence_real = robot_clearence;
        self.distance_from_wall_real = distance_from_wall;
        self.fill_path = fill_path;
        self.setup();

    def setup(self):
        self.map = Map();
        self.compute_block();
        self.extract_room_map();
        self.room_processing = RoomProcessing(self);
        self.extract_locations();

    # Extract movement points
    def get_locations(self):
        return self.locations;

    # Block the entrance to the room (Maybe add a fix adjustment to the room entrance)
    def compute_block(self):
        self.room_closer = RoomCloser(self.map, self.distance_from_entrance_real, self.angle_wall, self.entrance_real);
        self.room_closer.compute_block(100);


    # Point selection functionality
    def clean_angle(self,angles,clearence,index):
        #Add all points with that directions
        aux = [];
        for i in range(len(self.locations)):
            for j in self.locations[i].get_angle_queue():
                if j == angles[index]:
                    aux.append(i);
        #Add all points that should be removed
        remove = [];
        i=1;
        while i<len(aux):
            if aux[i] < aux[i - 1] + clearence:
                remove.append(aux.pop(i));
                i -= 1;
            i+=1;
        #Remove the points
        for r in remove:
            self.locations[r].remove_rotation(angles[index]);

    def compute_point_selection(self, basic=True, after=10, overlap=5):
        if basic:
            # Remove some points (should change based on more concrete evidence)
            for i in range(len(self.locations)):
                if i % after != 0:
                    self.locations[i] = None;

            i = 0;
            while (i < len(self.locations)):
                if self.locations[i] == None:
                    self.locations.pop(i);
                else:
                    i += 1;

        else:
            # Compute the point coverage
            t = math.tan(math.radians(30));
            p = self.robot_clearence + self.distance_from_wall;
            l = 2 * t * p;

            c = l-overlap;

            angles = {"+x":0,"-sxy":45,"+y":90,"+mxy":135,"-x":180,"-mxy":-45,"-y":-90,"+sxy":-135};

            # +x
            self.clean_angle(angles, c, "+x");
            # -x
            self.clean_angle(angles, c, "-x");
            # +y
            self.clean_angle(angles, c, "+y");
            # -y
            self.clean_angle(angles, c, "-y");
            # +mxy
            self.clean_angle(angles, c, "+mxy");
            # -mxy
            self.clean_angle(angles, c, "-mxy");
            # +sxy
            self.clean_angle(angles, c, "+sxy");
            # -sxy
            self.clean_angle(angles, c, "-sxy");

            #Clean of empty points
            i=0;
            while i < len(self.locations):
                if(len(self.locations[i].get_angle_queue())==0):
                    self.locations.pop(i);
                    i-=1;
                i+=1;

    # Extract movement points based on the room map
    def extract_locations(self):
        # Compute the pixel version
        self.robot_clearence = int(round(self.robot_clearence_real / self.room_map.resolution));
        self.distance_from_wall = int(round(self.distance_from_wall_real / self.room_map.resolution));

        # Extract a mask with proximity metrics
        self.room_map_points = self.room_processing.compute_points(self.room_map.get_numpy_maxtrix(), self.robot_clearence, self.distance_from_wall, self.fill_path);

        # Add points to an array that are on the edge of proximity
        locations = [];
        for i in range(self.n):
            for j in range(self.m):
                if self.room_map_points[i][j][0] == 1:
                    point = Point(self.room_map, self.map, [self.minN, self.minM]);
                    point.add_local_points(i, j);
                    locations.append(Location(point, self.room_map_points[i][j][1:5]));

        # Order the points such that the robot follows a linear path
        self.locations = Location.order_locations_proximity_strat_0(locations);

        # Clean points
        self.compute_point_selection(basic=False);

        # Add the points for visual representation
        for i in range(len(self.locations)):
            self.room_map.set_element_arr(self.locations[i].get_point().get_local(), -1);

        # Print points
        #for i in range(len(self.locations)):
            #print(str(self.locations[i].get_point().get_real()) + " - " + str(self.locations[i].get_angle_queue()))
        #     print(str(self.locations[i].get_point().get_real()) + " - " + str(self.locations[i].get_point().get_local()) + " - " + str(self.locations[i].get_point().get_global()))

        # Add the mask for visual representation
        """
        for i in range(self.n):
            for j in range(self.m):
                if self.room_map_points[i][j][0]==-1:
                    self.room_map.set_element(i,j,-1);
                elif self.room_map_points[i][j][0]==-2:
                    self.room_map.set_element(i,j,-2);
                elif self.room_map_points[i][j][0]==1:
                    self.room_map.set_element(i,j,100);
        """


    # Create the map of the room
    def extract_room_map(self):
        # Compute pixel center position - center of the room
        self.center = self.map.world_to_pixel_arr(self.center_real);
        # Create a visitation matrix
        aux_map = np.full((len(self.map.img),), -1);
        # Get new mapo boundries -  min stands for the minumu value of the ractangle that will fir the whole map with max being the maximum
        self.minM = self.map.height;
        self.minN = self.map.width;
        self.maxM = 0;
        self.maxN = 0;
        # Perform the recursive search
        sys.setrecursionlimit(999999999);
        self.search_point(aux_map, self.center[0], self.center[1]);
        sys.setrecursionlimit(1000);
        # Compute the size of the new map
        self.n = self.maxN - self.minN + 1;
        self.m = self.maxM - self.minM + 1;
        # Create a mapp array for the new map
        room_array = np.full((self.n * self.m,), 100);
        for i in range(self.n):
            for j in range(self.m):
                if aux_map[(self.minN + i) * self.map.height + (self.minM + j)] == 0:
                    room_array[i * self.m + j] = 0;
        # Create a map based on that array and other params
        self.room_map = Map(False, room_array, self.n, self.m, self.map.resolution);

    def search_point(self, aux_map, x, y):
        if aux_map[x * self.map.height + y] == -1 and x >= 0 and y < self.map.height and y >= 0 and x < self.map.width:
            if self.map.img[x * self.map.height + y] == 0:
                aux_map[x * self.map.height + y] = 0;
                # Up/Down/Left/Right
                self.search_point(aux_map, x + 1, y);
                self.search_point(aux_map, x - 1, y);
                self.search_point(aux_map, x, y + 1);
                self.search_point(aux_map, x, y - 1);
            else:
                aux_map[x * self.map.height + y] = 1;
            if y > self.maxM:
                self.maxM = y;
            if y < self.minM:
                self.minM = y;
            if x > self.maxN:
                self.maxN = x;
            if x < self.minN:
                self.minN = x;
