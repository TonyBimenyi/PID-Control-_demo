import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd, target):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.prev_error = 0
        self.integral = 0

    def update(self, current):
        error = self.target - current
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def simulate_dc_motor(controller, time_steps):
    current = 0
    set_points = np.zeros_like(time_steps)
    actual_points = np.zeros_like(time_steps)
    for i, t in enumerate(time_steps):
        set_points[i] = controller.target
        control_signal = controller.update(current)
        current += control_signal  # Simplified model for demonstration
        actual_points[i] = current
    return set_points, actual_points

# Parameters
kp = 0.1
ki = 0.3
kd = 0.2
target = 10  # Set point

# Create PID controller
controller = PIDController(kp, ki, kd, target)

# Simulation time
dt = 0.1
time_steps = np.arange(0, 20, dt)

# Simulate DC motor with PID control
set_points, actual_points = simulate_dc_motor(controller, time_steps)

# Plot results
plt.figure(figsize=(12, 6))

plt.subplot(3, 1, 1)
plt.plot(time_steps, set_points, label='Set Point')
plt.title('PID Control of DC Motor')
plt.ylabel('Set Point')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_steps, actual_points, label='Actual Point', color='orange')
plt.ylabel('Actual Point')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_steps, set_points, label='Set Point')
plt.plot(time_steps, actual_points, label='Actual Point', color='orange')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.tight_layout()
plt.show()
