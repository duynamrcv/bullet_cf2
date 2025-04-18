import numpy as np
import pybullet as p

class PID():
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.reset()

    def reset(self):
        self.e_integral = 0
        self.e_prev = 0

    def compute_signal(self, x, x_target):
        # Calculate error
        e = x_target - x
        
        # PID control
        self.e_integral += e * self.dt
        derivative = (e - self.e_prev) / self.dt
        signal = self.Kp * e + self.Ki * self.e_integral + self.Kd * derivative

        self.e_prev = e
        return signal
    
class PIDController():
    def __init__(self, env, kp, kd, ki):
        self.max_action = env.action_space.high
        self.min_action = env.action_space.low
        self.reset(kp, ki, kd)

    def reset(self, kp, ki, kd):
        self.dt = p.getPhysicsEngineParameters()['fixedTimeStep']
        self.pid_pos = PID(kp, ki, kd, self.dt)
        self.pid_vel = PID(kp, ki, kd, self.dt)

    def choose_action(self, state, goal):
        pos = state[:3]
        vel = state[3:]

        vel_goal = self.pid_pos.compute_signal(pos, goal)
        signal = self.pid_vel.compute_signal(vel, vel_goal)
        
        signal = np.clip(signal, self.min_action, self.max_action)
        return signal