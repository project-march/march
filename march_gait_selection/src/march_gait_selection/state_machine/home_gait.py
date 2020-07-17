from .gait_interface import GaitInterface


class HomeGait(GaitInterface):
    def __init__(self, name, position):
        self._name = 'home_{name}'.format(name=name)
        self._position = position

    def name(self):
        return self._name

    def starting_position(self):
        return self._position

    def final_position(self):
        return self._position
