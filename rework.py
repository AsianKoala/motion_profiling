from math import sqrt
import matplotlib.pyplot as plt
import numpy as np


# note: a should be negative if deccel
class MotionState:
    def __init__(self, x, v, a) -> None:
        self.x = float(x)
        self.v = float(v)
        self.a = float(a)

    def get(self, t):
        return MotionState(self.x + self.v * t + 0.5 * self.a * pow(t, 2), self.v + self.a * t, self.a)

    def __str__(self) -> str:
        return "x: {}, v: {}, a: {}".format(self.x, self.v, self.a)

class MotionConstraints:
    def __init__(self, cruiseVel, accel, deccel) -> None:
        self.v_max = float(cruiseVel)
        self.accel = float(accel)
        self.deccel = -float(deccel)

class MotionPeriod:
    def __init__(self, start_state, dt) -> None:
        self.start_state = start_state
        self.end_state = start_state.get(dt)
        self.dt = abs(dt)
        self.dx = self.end_state.x - start_state.x

    def get(self, t):
        return self.start_state.get(t)

        
    def flipped(self):
        return MotionPeriod(MotionState(self.end_state.x, -self.end_state.v, self.end_state.a), -self.dt)

class MotionProfile:
    def __init__(self, *periods, is_reversed) -> None:
        self.duration = 0.0
        self.periods = periods
        for period in periods: self.duration += period.dt
        if is_reversed:
            self.periods = [x.flipped() for x in periods[::-1]]
        self.start_state = periods[0].start_state
        self.end_state = periods[-1].end_state

    def get(self, t):
        dt = 0.0
        for period in self.periods:
            if t <= period.dt + dt: return period.get(t - dt)
            dt += period.dt
        return self.end_state

def generate_trapezoidal(start_state: MotionState, end_state: MotionState, constraints: MotionConstraints):
    isReversed = False
    if end_state.x < start_state.x: # too lazy to deal with this properly
        temp = end_state
        end_state = start_state
        start_state = temp
        isReversed = True
        
    start_state.a = constraints.accel
    end_state.a = constraints.deccel

    accel_period = MotionPeriod(start_state, abs(constraints.v_max - start_state.v) / constraints.accel)

    deccel_time = abs((end_state.v - constraints.v_max) / constraints.deccel)
    deccel_start_state = MotionState(end_state.x, end_state.v, constraints.deccel).get(-deccel_time)
    deccel_period = MotionPeriod(deccel_start_state, deccel_time)

    dx = abs(end_state.x - start_state.x)
    cruise_dt = (dx - accel_period.dx - deccel_period.dx) / constraints.v_max
    cruise_period = MotionPeriod(MotionState(accel_period.end_state.x, accel_period.end_state.v, 0), cruise_dt)

    if cruise_dt < 0.0:
        print('wont reach cruise vel')
        cruise_period.dt = 0.0
        a = min(abs(constraints.accel), abs(constraints.deccel))
        dt = sqrt(dx / a)

        accel_period = MotionPeriod(MotionState(start_state.x, start_state.v, a), dt)
        deccel_period = MotionPeriod(MotionState(accel_period.end_state.x, accel_period.end_state.v, -a), dt)

    mp = MotionProfile(accel_period, cruise_period, deccel_period, is_reversed=isReversed)
    return mp

def plot(profile):
    time = np.arange(0, profile.duration, 0.01)
    states = np.array([profile.get(t) for t in time])
    x_values = [state.x for state in states]
    v_values = [state.v for state in states]
    a_values = [state.a for state in states]
    plt.plot(time, x_values, label='position')
    plt.plot(time, v_values, label='velocity')
    plt.plot(time, a_values, label='acceleration')
    plt.legend()
    plt.show()
    
def main():
    v_max = 40
    a_max = 40
    d_max = 20
    constraints = MotionConstraints(v_max, a_max, d_max)
    start_state = MotionState(100, 0, 0)
    end_state = MotionState(80, 0, 0)
    profile = generate_trapezoidal(start_state, end_state, constraints)
    plot(profile)

if __name__ == '__main__':
    main()

