import yaml


class Yaml(object):
    __yaml_file_path = None
    __yaml_data = None

    def __init__(self, file_path):
        self.__yaml_file_path = file_path
        with open(file_path, "r") as stream:
            self.__yaml_data = yaml.safe_load(stream)

    def get_yaml_data(self):
        return self.__yaml_data

    def get_yaml_file_path(self):
        return self.__yaml_file_path

    def __str__(self):
        return 'Yaml object: ' + str(self.__yaml_file_path)
