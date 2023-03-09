from openap import prop
from openap import Thrust, FuelFlow
import numpy as np
from itertools import combinations
import matplotlib.pyplot as plt
import os

aircraft = prop.aircraft('B748')
engine = prop.engine('RB211-524G')

T = Thrust(ac='B744', eng='PW4062')
fuelflow = FuelFlow(ac='B744', eng='PW4062')

# Path to save the figure and the csv
path = os.getcwd()

# Altitude definition
h = np.linspace(0, 35000, 15)
# Airspeed definition
tas = np.linspace(0, 300, 15)

# Reshape array to fit plot structure
tas, h = np.meshgrid(tas, h)

# Calculate Maximum thrust
T_out = T.cruise(alt=h, tas=tas) / 1000

# Calculate Fuel flow
FF = fuelflow.at_thrust(acthr=T_out, alt=h)

# Convert knots to m/s
tas *= 3600 / 1852

# Convert feet to m
h *= 0.3048

# Plot of the Maximum thrust available f(tas,h) and fuel flow f(T_max,h)
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

surf = ax.plot_surface(tas, h, T_out, antialiased=True)
ax.set_title(r'Maximum Thrust Available')
ax.set_xlabel('TAS (m/s)')
ax.set_ylabel('h (m)')
ax.set_zlabel('T (kN)')

plt.show()
fig.savefig(path + '/Thrust.eps', bbox_inches='tight', format='eps', dpi=1200)

fig1, ax1 = plt.subplots(subplot_kw={"projection": "3d"})

surf1 = ax1.plot_surface(T_out, h, FF, antialiased=True)
ax1.set_title('Fuel Flow')
ax1.set_xlabel('Tmax (kN)')
ax1.set_ylabel('h (m)')
ax1.set_zlabel('fuel flow (kg/s)')

plt.show()
fig1.savefig(path + '/FF.eps', bbox_inches='tight', format='eps', dpi=1200)

# Resize matrixes to fit matlab scatteredinterpolant function
T_out = np.resize(T_out, (len(T_out)*len(T_out)))
FF = np.resize(FF, (len(FF)*len(FF)))

np.savetxt(path + '/FF.csv', FF, delimiter=',')
np.savetxt(path + '/Thrust.csv', T_out, delimiter=',')

