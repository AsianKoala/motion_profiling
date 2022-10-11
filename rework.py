from math import sqrt
import matplotlib.pyplot as plt
import numpy as np


class MotionState:
    def __init__(self, x, v, a) -> None:
        self.x = x
        self.v = v
        self.a = a

    def get(self, t):
        return MotionState(self.x + self.v * t + 0.5 * self.a * pow(t, 2), self.v + self.a * t, self.a)

class MotionConstraints:
    def __init__(self, cruiseVel, accel, deccel) -> None:
        self.cruiseVel = cruiseVel
        self.accel = accel
        self.deccel = deccel

class MotionPeriod:
    def __init__(self, start_state, dt) -> None:
        self.start_state = start_state
        self.dt = dt
        self.end_state = start_state.get(dt)
        self.dx = self.end_state.x - start_state.x

    def get(self, t):
        return self.start_state.get(t)

class MotionProfile:
    def __init__(self, *periods) -> None:
        self.start_state = periods[0].start_state
        self.end_state = periods[-1].end_state
        self.periods = periods
        self.duration = 0.0
        for period in periods: self.duration += period.dt

    def get(self, t):
        dt = 0.0
        for period in self.periods:
            if t <= period.dt + dt: return period.get(t - dt)
            dt += period.dt
        return self.end_state

def absMin(a, b):
    if abs(a) < abs(b): return a
    return b

def generate_trapezoidal(start_state: MotionState, end_state: MotionState, constraints: MotionConstraints):
    accel_period = MotionPeriod(
        MotionState(start_state.x, start_state.v, constraints.accel),
        (abs((constraints.cruiseVel - start_state.v) / constraints.accel))
    )

    deccelTime = abs(end_state.v - constraints.cruiseVel) / constraints.cruiseVel
    deccelDx = end_state.v * -deccelTime + 0.5 * constraints.deccel * pow(deccelTime, 2)

    cruise_period = MotionPeriod(
        MotionState(accel_period.end_state, constraints.cruiseVel, 0.0),
        (end_state.x - start_state.x - accel_period.dx - deccelDx) / constraints.cruiseVel
    )

    deccel_period = MotionPeriod(
        MotionState(cruise_period.end_state, cruise_period.end_state.v, -constraints.deccel),
        deccelTime
            )

    if cruise_period.dt < 0.0:
        cruise_period.dt = 0.0
        accel_period = MotionPeriod(
                MotionState(accel_period.start_state.x, accel_period.start_state.v, absMin(accel_period.start_state.a, deccel_period.start_state.a)),
                sqrt(abs(end_state.x / accel_period.start_state.a))
                )
        deccel_period = MotionPeriod(MotionState(accel_period.end_state.x, accel_period.end_state.v, -accel_period.start_state.a))

    return MotionProfile(accel_period, cruise_period, deccel_period)

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
    a_max = 16
    v_max = 40
    d_max = 16
    constraints = MotionConstraints(v_max, a_max, d_max)
    start_state = MotionState(0, 0, 0)
    end_state = MotionState(80, 0, 0)
    profile = generate_trapezoidal(start_state, end_state, constraints)
    # print('dt1: {}\ndt2: {}\ndt3: {}\nprofile duration {}\n integral: {}'.format(
    #     profile.accel_time, profile.cruise_time, profile.deccel_time, profile.profileDuration, profile.total_integral))
    plot(profile)

if __name__ == '__main__':
    main()

