from __future__ import division
import cv2
import torch
import numpy as np

class Detector:
    #'../CNN/character_weights.pt'
    def __init__(self, model_path, weights_path):
        self.weights_path = weights_path;
        self.model = torch.hub.load(model_path, 'custom', path=weights_path, source='local');
        self.character_detected = False;

    def predict_img(self, img, params=None):
        self.original_img  = np.copy(img);

        img = img[..., ::-1] # Convert to RGB

        # Inference
        results = self.model(img, size=640)  # includes NMS
        tensor = results.xyxy[0]
        array = tensor.numpy()

        if len(array) == 0:
            return False
        data = array[0]
        

        self.conf_rating = data[4]
        self.bounding_box = {"x1": data[0], "y1": data[1], "x2": data[2], "y2": data[3]}
        self.class_index = data[5];

        width = (self.bounding_box["x2"] - self.bounding_box["x1"])
        area = width * (self.bounding_box["y2"] - self.bounding_box["y1"])

        if area < 2000 or ((640 - self.bounding_box["x2"]) < 50) or (self.bounding_box["x1"] < 50):   
            return False


        self.set_charater();


        if params!=None:
            if self.character_detected:
                #print("character detected")
                #print(self.conf_rating)
                if 100*self.conf_rating > params.get_rating():
                    #print("save character")
                    self.save_character(params.get_path());
                    return True;
                return False;
            return False;
        else:
            return True;

    def set_charater(self):
        if self.class_index == 0:
            self.character_detected = True;
        elif self.class_index == 1:
            self.character_detected = True;
        elif self.class_index == 2:
            self.character_detected = True;
        elif self.class_index == 3:
            self.character_detected = True;
        else:
            self.character_detected = False;

    def get_character_detected(self):
        return self.character_detected;

    def get_label(self):
        if self.class_index == 0:
            return "Colonel Mustard"
        elif self.class_index == 1:
            return "Miss Scarlet"
        elif self.class_index == 2:
            return "Mrs. Peacock"
        elif self.class_index == 3:
            return "Professor Plum"
        else:
            return "Unidentifeid"

    def get_image(self, img):
        # Using cv2.rectangle() method
        # Draw a rectangle with blue line borders of thickness of 2 px
        image_aux = img.copy();

        # If bounding box is empty output image
        try:
            self.bounding_box
        except AttributeError:
            return image_aux

        start_point = (int(self.bounding_box["x1"]), int(self.bounding_box["y1"]))
        end_point = (int(self.bounding_box["x2"]), int(self.bounding_box["y2"]))
        color = (255, 0, 0)
        thickness = 2

        label_position = (int(self.bounding_box["x2"]), int(self.bounding_box["y2"]) - 60)
        confidance_position = (int(self.bounding_box["x2"]) + 150, int(self.bounding_box["y2"]) - 60)

        image_aux = cv2.rectangle(image_aux, start_point, end_point, color, thickness);
        image_aux = cv2.putText(image_aux, self.get_label(), label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 4);
        image_aux = cv2.putText(image_aux, str(self.conf_rating), confidance_position, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 4);

        return image_aux;

    def get_rating(self):
        return self.conf_rating;

    def get_bounding_box(self):
        return self.bounding_box;

    def save_character(self, path):
        self.take_screenshot(path);
        self.save_name(path);

    # Save outputs
    def take_screenshot(self, path, name="cluedo_character.png"):
        cv2.imwrite(path + name, self.original_img);
        #cv2.imwrite(path + name, self.get_image(self.original_img));

    def save_name(self, path, name="cluedo_character.txt"):
        fout = open(path + name, "w");
        fout.write(self.get_label());
        fout.close();
