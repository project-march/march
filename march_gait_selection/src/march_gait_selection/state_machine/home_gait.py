from .gait_interface import GaitInterface


class HomeGait(GaitInterface):
    def __init__(self, name, position):
        self._name = 'home_{name}'.format(name=name)
        self._position = position

    @property
    def name(self):
        return self._name

    @property
    def starting_position(self):
        return None

    @property
    def final_position(self):
        return self._position
