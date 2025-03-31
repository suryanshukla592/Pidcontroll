import numpy as np
import matplotlib.pyplot as plt

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
#Robot parameters
wheel_radius = 0.05 
distance_between_wheels = 0.2  
max_wheel_angular_velocity = 8.0#cause of the hardware limits
def differential_drive(v, w, dt, x, y, theta):
    theta += w*dt
    x += v*np.cos(theta)*dt
    y += v*np.sin(theta)*dt
    return x, y, theta
#Waypoints to navigate
waypoints = [(3, 8), (7, 3)]
x, y, theta = 0, 0, 0  
pid_linear = PIDController(Kp=0.7, Ki=0, Kd=0)
dt=0.1
trajectory = [(x, y)]
ideal_path = [(0, 0)] + waypoints
velocity = []

for waypoint in waypoints:
    target_x, target_y = waypoint
    while np.hypot(target_x - x, target_y - y) > 0.05:
        error_x = target_x - x
        error_y = target_y - y
        v = pid_linear.control(np.hypot(error_x, error_y), dt)
        theta = np.arctan2(error_y, error_x)
        w_l = (2*v - theta * distance_between_wheels) / (2 * wheel_radius)
        w_r = (2*v + theta * distance_between_wheels) / (2 * wheel_radius)
        w_l = np.clip(w_l, -max_wheel_angular_velocity, max_wheel_angular_velocity)#beacuse of probable hardware limits
        w_r = np.clip(w_r, -max_wheel_angular_velocity, max_wheel_angular_velocity)
        x, y, theta = differential_drive(v, theta, dt, x, y, theta)
        trajectory.append((x, y))
        velocity.append((v))
#Plot the path
trajectory = np.array(trajectory)
ideal_path = np.array(ideal_path)
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label="Robot Path")
plt.plot(ideal_path[:, 0], ideal_path[:, 1], 'g--', label="Ideal Path")
plt.scatter(*zip(*waypoints), color='r', marker='x', label="Waypoints")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
plt.title("PID-Controlled Robot Navigation")
plt.grid()
#Plot wheel velocities
time_steps = np.arange(len(velocity))*dt
plt.subplot(1, 2, 2)
plt.plot(time_steps,velocity, 'r-', label="Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.title("Velocity of Bot Over Time")
plt.grid()
plt.tight_layout()
plt.show()
