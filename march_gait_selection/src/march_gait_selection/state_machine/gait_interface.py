class GaitInterface(object):
    def start(self, starting_position):
        """Called when the gait has been selected for execution.

        The gait should start at the given starting position.
        :param starting_position: starting positions of all joints
        """
        pass

    def starting_position(self):
        """Returns the starting position of all joints."""
        pass

    def final_position(self):
        """Returns the position of all the joints after the gait has ended."""
        pass

    def name(self):
        """Returns the name of the gait."""
        pass

    def update(self, elapsed_time):
        """Called in a loop with the elapsed time since the last update.

        :param float elapsed_time: Elapsed time in seconds since the last update
        :returns A pair of a trajectory and a flag. The trajectory that will be
                 set as the new goal for the controller, can be None. The flag
                 indicates whether the gait has finished.
        """
        return None, False

    def stop(self):
        """Called when the gait has been instructed to stop.

        :returns True when the stop action has been accepted, False otherwise.
        """
        return False

    def end(self):
        """Called when the gait has finished."""
        pass
