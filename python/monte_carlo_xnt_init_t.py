import numpy as np
import matplotlib.pyplot as plt

# Load data
data = np.loadtxt("../output/txt/datfil_mc_xnt_init_t.txt")

# Columns
time  = data[:, 0]
sigma = data[:, 1]
mean  = data[:, 2]

# Plot standard deviation
plt.figure(figsize=(8, 5))

plt.scatter(time, sigma)

plt.xlabel("Flight Time (s)")
plt.ylabel("Miss Distance Standard Deviation (ft)")
plt.title("mc_xnt: Miss Distance Standard Deviation vs Flight Time")

plt.grid(True)

plt.show()
