class GaitInterface(object):
    @property
    def name(self):
        """Returns the name of the gait."""
        return None

    @property
    def subgait_name(self):
        """Returns the name of the currently executing trajectory."""
        return None

    @property
    def version(self):
        """Returns the version of the currently executing trajectory."""
        return None

    @property
    def duration(self):
        """Returns the duration in seconds of the currently executing trajectory from the start of the gait."""
        return None

    @property
    def gait_type(self):
        """Returns a gait type of the currently executing trajectory."""
        return None

    @property
    def starting_position(self):
        """Returns the starting position of all joints."""
        return None

    @property
    def final_position(self):
        """Returns the position of all the joints after the gait has ended."""
        return None

    def start(self):
        """Called when the gait has been selected for execution and returns an optional starting trajectory."""
        return None

    def update(self, elapsed_time):
        """Called in a loop with the elapsed time since the last update.

        :param float elapsed_time: Elapsed time in seconds since the last update
        :returns A pair of a trajectory and a flag. The trajectory that will be
                 set as the new goal for the controller, can be None. The flag
                 indicates whether the gait has finished.
        """
        return None, True

    def transition(self, transition_request):
        """Requests a special transition.

        :param TransitionRequest transition_request: request on what special transition to perform
        :returns True when the request has been accepted, False otherwise.
        """
        return False

    def stop(self):
        """Called when the gait has been instructed to stop.

        :returns True when the stop action has been accepted, False otherwise.
        """
        return False

    def end(self):
        """Called when the gait has finished."""
        pass
