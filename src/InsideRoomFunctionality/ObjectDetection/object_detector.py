
from .character import Character
import multiprocessing as mp
import cv2 as cv

path1 = "/root/world_files/coursework/cluedo_images/"

class ObjectDetector:

    def __init__(self,sensory):
        #Sensor system
        self.sensory = sensory;
        #Pool
        #pool = mp.Pool(mp.cpu_count())
        #Characters
        character_data = [["Colonel Mustard",path1+"mustard.png"],["Mrs Peacock",path1+"peacock.png"],["Professor Plum",path1+"plum.png"],["Miss Scarlet",path1+"scarlet.png"]]
   

        #results = [pool.apply_async(Character, args=(char[0],char[1])) for char in character_data] 
        results = [Character(char[0],char[1]) for char in character_data]

        self.mustard = results[0];
        self.peacock = results[1];
        self.plum = results[2];
        self.scarlet = results[3];

        #pool.close()  

    def look_for_character(self):
        pass;

    def move_to_character(self):
        pass;

    def decide_character(self):
        pass;

    #Save outputs
    def take_screenshot(self,image,path,name="cluedo_character.png"):
        cv.imwrite(path+name,image);                   

    def save_name(self,character_name,path,name="cluedo_character.txt"):                                                                                                             
        fout = open(path+name, "w");
        fout.write(character_name);
        fout.close();