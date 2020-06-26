class Limits(object):

    def __init__(self, lower, upper, velocity, effort=None, k_position=None, k_velocity=None):
        self.lower = lower
        self.upper = upper
        self.velocity = velocity
        self.effort = effort
        self.k_position = k_position
        self.k_velocity = k_velocity

    def __eq__(self, other):
        return self.lower == other.lower and self.upper == other.upper and self.velocity == other.velocity and  \
            self.effort == other.effort and self.k_position == other.k_position and \
            self.k_velocity == other.k_velocity

    def __ne__(self, other):
        return not self == other
