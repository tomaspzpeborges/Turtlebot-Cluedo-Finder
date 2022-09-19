from nav_msgs.msg import MapMetaData,OccupancyGrid
import rospy
import sys
import numpy as np
from matplotlib import pyplot as plt

class Map:

    #Construtors
    def __init__(self,create=True,array=[],width=0,height=0,resolution=0):
        self.setup(create,array,width,height,resolution);

    def setup(self,create,array,width,height,resolution):
        if create:
           self.create_map_from_ros();
        else:
            self.height = height;
            self.width = width;
            self.resolution = resolution;
            self.img = array;

    def create_map_from_ros(self):
        map_details = rospy.wait_for_message("/map_metadata",MapMetaData);
        self.height = map_details.height;
        self.width = map_details.width;
        self.resolution = round(map_details.resolution,8);
        self.origin = [map_details.origin.position.x,map_details.origin.position.y];
        self.origin_pixel = [int(round(abs(self.origin[0] / self.resolution))),int(round(abs(self.origin[1] / self.resolution)))];
        map_data = rospy.wait_for_message("/map",OccupancyGrid);
        self.img = np.array(map_data.data);
        self.img = np.reshape(self.img,(self.height,self.width,));
        self.img = self.img.T
        self.img = np.reshape(self.img,(self.height*self.width,));
        
    #Getters and setters
    def get_element(self,i,j):
        return self.img[i*self.height+j];

    def set_element(self,i,j,a):
        self.img[i*self.height+j] = a;

    def set_element_arr(self,arr,a):
        self.img[arr[0]*self.height+arr[1]] = a;

    def get_numpy_maxtrix(self):
        return np.reshape(self.img,(self.width,self.height,));

    #Converters
    def pixel_to_world(self,x,y):
        return (x-self.origin_pixel[0])*self.resolution,(y-self.origin_pixel[1])*self.resolution;

    def world_to_pixel(self,x,y):
        return int(round(self.origin_pixel[0]+x/self.resolution)),int(round(self.origin_pixel[1]+y/self.resolution));

    def pixel_to_world_arr(self,array):
        return (array[0]-self.origin_pixel[0])*self.resolution,(array[1]-self.origin_pixel[1])*self.resolution;

    def world_to_pixel_arr(self,array):
        return int(round(self.origin_pixel[0]+array[0]/self.resolution)),int(round(self.origin_pixel[1]+array[1]/self.resolution));

    #Print variables
    def print_variables(self):
        print("Width: " + str(self.width));
        print("Height: " + str(self.height));
        print("Resolution: " + str(self.resolution));
        print("Origin: " + str(self.origin));
        print("Origin_pixel: " + str(self.origin_pixel));

    #Print map array
    def print_map(self):
        aux_map = self.get_numpy_maxtrix();
        for i in range(len(aux_map)):
            for j in range(len(aux_map[i])):
                sys.stdout.write(str(aux_map[i][j]) + " ");
            sys.stdout.write("\n");

    #Show map using matplotlib
    def show_map(self):
        aux_map = self.get_numpy_maxtrix();
        for i in range(len(aux_map)):
            for j in range(len(aux_map[i])):
                if aux_map[i][j] == -1:
                    aux_map[i][j]=60;
                elif aux_map[i][j] == -2:
                    aux_map[i][j]=30;
                elif aux_map[i][j]== 100:
                    aux_map[i][j] = 0;
                elif aux_map[i][j]==0:
                    aux_map[i][j]= 100;
        plt.gray();
        plt.imshow(aux_map.T);
        plt.show();

    #Static show map using matplotlib(needs to be 2D array)
    @staticmethod
    def show_map_new(map):
        aux_map = map;
        for i in range(len(aux_map)):
            for j in range(len(aux_map[i])):
                if aux_map[i][j] == -1:
                    aux_map[i][j]=60;
                elif aux_map[i][j] == -2:
                    aux_map[i][j]=30;
                elif aux_map[i][j]== 100:
                    aux_map[i][j] = 0;
                elif aux_map[i][j]==0:
                    aux_map[i][j]= 100;
        plt.gray();
        plt.imshow(aux_map.T);
        plt.show();
