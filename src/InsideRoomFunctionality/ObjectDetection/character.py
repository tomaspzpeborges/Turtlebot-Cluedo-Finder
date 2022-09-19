import numpy as np
from matplotlib import pyplot as plt
import cv2 as cv

from .colour import Colour

class Character:

    def __init__(self,name,image_path):
        self.name=name;
        self.image_path = image_path;
        self.ranges = []
        self.read_image();
        self.distinc_colours();

    def read_image(self):
        self.image = cv.imread(self.image_path);

    def add_range(self,range):
        self.ranges.append(range);

    def distinct_hue(self):
        """
        self.different_hues = [];
        for i in range(len(self.image)):
            for j in range(len(self.image[i])):
                colour = Colour();
                colour.add_rgb_arr(self.image[i][j]);
                unique = True;
                for h in self.different_hues:
                    if round(h[0]/h[1])-20 >= colour.h and round(h[0]/h[1])+20 <=colour.h:
                        h[0]+=colour.h;
                        h[1]+=1;
                        unique = False;
                        break;
                if unique:
                    self.different_hues.append([colour.h,1]);
        """
        
    def print_lenght_hue(self):
        print(len(self.unique));

    def distinc_colours(self):
        self.distinct_colours = [];
        unique, counts = np.unique(self.image.reshape(-1, self.image.shape[2]), axis=0, return_counts=True)
        for i in range(len(unique)):
            self.distinct_colours.append([unique[i], counts[i],  counts[i]*100/len(unique)]);
        self.distinct_colours = sorted(self.distinct_colours, key=lambda x: x[1], reverse=False);

    def display_colour(self, number_colours, order="D"):
        print(len(self.distinct_colours))
        if number_colours<len(self.distinct_colours):
            if order=="D":
                n = len(self.distinct_colours) - 1;
                for i in range(number_colours):
                    colour = Colour();
                    colour.add_rgb_arr(self.distinct_colours[n - i][0]);
                    print(str(colour.get_rgb()) + " - " + str(self.distinct_colours[n - i][1]) + " - " + str(self.distinct_colours[n - i][2]));
            else: 
                for i in range(number_colours):
                    colour = Colour();
                    colour.add_rgb_arr(self.distinct_colours[i][0]);
                    print(str(colour.get_rgb()) + " - " + str(self.distinct_colours[i][1]));
               

    def display_image(self):
        bgr = cv.cvtColor(self.image, cv.COLOR_RGB2BGR);
        plt.imshow(bgr);
        plt.show();
