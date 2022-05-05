class ProportionalController:
    def __init__(self, Kp):
        self.Kp = Kp

    def step(self, e, dt=0):
        """ dt should be the time elapsed from the last time step was called """
        return self.Kp * e