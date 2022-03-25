import matplotlib.pyplot as plt
import numpy as np

class MotionState:
    def __init__(self, x, v, a):
        self.x = x
        self.v = v
        self.a = a

    def calculate(self, dt):
        return MotionState(self.x + self.v * dt + 0.5 * self.a * dt ** 2, self.v + self.a * dt, self.a)

    def integrate(self, dt):
        return self.v * dt + 0.5 * self.a * dt ** 2
    
    def __str__(self) -> str:
        return 'x: {}, v: {}, a: {}'.format(self.x, self.v, self.a)

class MotionConstraints:
    def __init__(self, v_max, a_max, d_max):
        self.v_max = v_max
        self.a_max = a_max
        self.d_max = d_max

    def __str__(self) -> str:
        return 'v_max: {}, a_max: {}, d_max: {}'.format(self.v_max, self.a_max, self.d_max)

class MotionProfile:
    def __init__(self, start_state: MotionState, end_state: MotionState, constraints: MotionConstraints):
        self.profileDuration = None
        self.dt1 = (constraints.v_max - start_state.v) / constraints.a_max
        self.dt3 = (end_state.v - constraints.v_max) / constraints.d_max
        self.accel_state = MotionState(start_state.x, start_state.v, constraints.a_max)
        self.deccel_state = MotionState(end_state.x, end_state.v, constraints.d_max).calculate(-self.dt3)

        # now calculate cruise state dt
        accel_end_state = self.accel_state.calculate(self.dt1)
        self.cruise_state = MotionState(accel_end_state.x, accel_end_state.v, 0)
        delta_x = end_state.x - start_state.x
        self.dt2 = (delta_x - self.accel_state.integrate(self.dt1) - self.deccel_state.integrate(self.dt3)) / constraints.v_max
        self.profileDuration = self.dt1 + self.dt2 + self.dt3
    

def plot(profile):
    time = np.arange(0, profile.profileDuration, 0.01)
    states = np.array([profile.get(t) for t in time])
    x_values = [state.x for state in states]
    v_values = [state.v for state in states]
    a_values = [state.a for state in states]
    print('dt1: {}\ndt2: {}\ndt3: {}'.format(profile.dt1, profile.dt2, profile.dt3))
    plt.plot(time, x_values, label='position')
    plt.plot(time, v_values, label='velocity')
    plt.plot(time, a_values, label='acceleration')
    plt.legend()
    plt.show()
    

