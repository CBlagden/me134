# Helper class to hold state time

def ros_to_normal(cls, t, rostime):
    if (rostime):
        converted_t = t.seconds + (t.nanosec)*1e-9
    else:
        converted_t = t
    return converted_t

class StateClock():
    def __init__(self, t0, rostime=False):
        self.t0 = ros_to_normal(t0, rostime)
        self.paused = False

    # Get the delta time, including pauses
    def t_since_start(self, t, rostime=False):
        t = ros_to_normal(t, rostime)
        if (self.paused):
            self.paused = False
            self.t0 = t - self.paused_dt
    
        return t - self.t0

    # Stop the clock
    def pause(self, t, rostime=False):
        t = ros_to_normal(t, rostime)
        dt = self.t_since_start(t)

        self.paused = True
        self.paused_dt = dt
    
    # restart
    def restart(self, t0, rostime=False):
        self.t0 = ros_to_normal(t0, rostime)
        self.paused = False