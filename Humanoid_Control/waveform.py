import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Configuration ---
fs = 1000             # Sampling points
duration = 2          # Total time window (seconds)
t = np.linspace(0, duration, fs)
frequency = 1         # Hz
amplitude = 1         # Sine wave amplitude
phase_speed = 0.05     # Speed of phase shift

# --- Set up plot ---
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlim(t[0], t[-1])
ax.set_ylim(-1.5, 1.5)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Amplitude")

# --- Initialization function ---
def init():
    line.set_data([], [])
    return line,

# --- Update function for animation ---
def update(frame):
    phase = frame * phase_speed
    y = amplitude * np.sin(2 * np.pi * frequency * t + phase)
    line.set_data(t, y)
    return line,

# --- Run animation ---
ani = FuncAnimation(fig, update, frames=200, init_func=init, interval=50, blit=True)
plt.show()
