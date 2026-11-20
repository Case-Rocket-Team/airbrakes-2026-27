import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("readings.txt")

ODR_HZ = 200
t = np.arange(len(data)) / ODR_HZ

mean = data.mean()
std = data.std(ddof=1)
print(f"mean = {mean:.3f}")
print(f"std  = {std:.3f}")

plt.plot(t, data)
plt.axhline(mean, color="black")
plt.axhline(mean + std, color="red", linestyle="--")
plt.axhline(mean - std, color="red", linestyle="--")
plt.title(f"mean = {mean:.3f}, std = {std:.3f}")
plt.xlabel("time (s)")
plt.ylabel("altitude (m)")
plt.show()