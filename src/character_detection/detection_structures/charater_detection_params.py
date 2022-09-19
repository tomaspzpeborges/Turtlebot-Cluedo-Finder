

class CharacterDetectionParams:
    # Constructors
    def __init__(self, save_path, rating):
        self.save_path = save_path;
        self.rating = rating;

    # Getters
    def get_path(self):
        return self.save_path;

    def get_rating(self):
        return self.rating;
