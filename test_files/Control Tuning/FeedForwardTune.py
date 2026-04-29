import matplotlib.pyplot as plt
import numpy as np

# TEST DATA ONLY. WE WILL GET CSV AND THEN INPUT TO THIS FILE LATER
velocity  = [1,2,4,8]
voltage = [2,4,6,9]

# Linear Regression to get u = kS * sign(v) + kV * v
a,b = np.polyfit(velocity,voltage, deg = 1)

plt.scatter(velocity,voltage)
plt.xlabel("Velocity (cm/s)")
plt.ylabel("Voltage (milivolts)")

x_line = np.linspace(0, max(velocity), 100)
y_line = a * x_line + b

plt.plot(x_line, y_line)

plt.scatter(0, b, color='red', label='kS (intercept)')
plt.legend()

a = round(a, 3)
b = round(b, 3)

print(f"The line is y = {a}x + {b}")
print(f"kS = {b}")
print(f"kV = {a}")

plt.show()

