from VLM.geometry.geometry_plotter import geometry_plotter
from VLM.geometry.geometry import geometry_generator
from VLM.VLM_.VLM import VLM
from VLM.utility.utilities import deg2rad
import numpy as np
import json
import os
import scipy.io as sio
import json
import matplotlib.pyplot as plt
import os


path = os.path.join(os.getcwd(), 'VLM/A320_TFM.json')
tornado = r'C:\Users\Usuario\Documents\MATLAB\TFM\tornado states'
path_out = os.path.join(os.getcwd(), 'comparison.json')

geometry_generator(path)

# reference case
geometry = np.load(os.path.join(os.getcwd(), 'VLM/A320_TFM.npy'), allow_pickle=True).item()

vlm = VLM(geometry)
alpha = np.linspace(-20, 20, 41)
beta = np.linspace(-20, 20, 41)
q = np.linspace(-10.0, 10.0, 21)
p = np.linspace(-10.0, 10.0, 21)
r = np.linspace(-10.0, 10.0, 21)
# TODO: Uncomment this section to generate the data
# data = []
# for i in alpha:
#     data.append(vlm.launch_calculation(geometry, 'WING', 50.0, 1.0, alpha=i))
#     data[-1]['alpha'] = i
#     data[-1]['beta'] = 0.0
#     data[-1]['p'] = 0.0
#     data[-1]['q'] = 0.0
#     data[-1]['r'] = 0.0
#
# for i in beta:
#     data.append(vlm.launch_calculation(geometry, 'WING', 50.0, 1.0, beta=i))
#     data[-1]['alpha'] = 0.0
#     data[-1]['beta'] = i
#     data[-1]['p'] = 0.0
#     data[-1]['q'] = 0.0
#     data[-1]['r'] = 0.0
#
# for i in p:
#     data.append(vlm.launch_calculation(geometry, 'WING', 50.0, 1.0, omega=[i, 0, 0]))
#     data[-1]['alpha'] = 0.0
#     data[-1]['beta'] = 0.0
#     data[-1]['p'] = i
#     data[-1]['q'] = 0.0
#     data[-1]['r'] = 0.0
#
# for i in q:
#     data.append(vlm.launch_calculation(geometry, 'WING', 50.0, 1.0, omega=[0, i, 0]))
#     data[-1]['alpha'] = 0.0
#     data[-1]['beta'] = 0.0
#     data[-1]['p'] = 0.0
#     data[-1]['q'] = i
#     data[-1]['r'] = 0.0
#
# with open(path_out, 'w') as f:
#     json.dump(data, f, indent=2)

# Load and separate cases

with open(path_out, 'r') as f:
    data = json.load(f)

alpha_tor = sio.loadmat(os.path.join(tornado, 'alpha.mat'))
alpha_VLM = data[:41]

# Alpha data
cD = [i['c_f_x'] for i in alpha_VLM]
cD_tor = [i[1] for i in alpha_tor['alpha']]
cL = [-i['c_fz'] for i in alpha_VLM]
cL_tor = [i[0] for i in alpha_tor['alpha']]
cM = [i['c_M_y'] for i in alpha_VLM]
cM_tor = [i[4] for i in alpha_tor['alpha']]

# Alpha plots
fig, ax = plt.subplots(3, sharex=True)
fig.suptitle(r'Variation with $\alpha$')
ax[0].plot(alpha, cD_tor)
ax[0].plot(alpha, cD)
ax[0].plot(alpha, 0.08901*deg2rad(alpha))
ax[1].plot(alpha, cL_tor)
ax[1].plot(alpha, cL)
ax[1].plot(alpha, 5.642*deg2rad(alpha))
ax[2].plot(alpha, cM_tor)
ax[2].plot(alpha, cM)
ax[2].plot(alpha, -0.20948*deg2rad(alpha))
ax[0].legend(['Tornado', 'VLM', 'A320 [4]'])
ax[0].set(ylabel='$C_D$')
ax[0].grid()
ax[1].set(ylabel='$C_L$')
ax[1].grid()
ax[2].set(ylabel='$C_m$', xlabel=r'$\alpha$ [rad]')
ax[2].grid()
fig.savefig(os.getcwd() + '/alpha.eps', bbox_inches='tight', format='eps', dpi=1200)
plt.close()

# Beta data
beta_tor = sio.loadmat(os.path.join(tornado, 'beta.mat'))
beta_VLM = data[41:82]
cY = [i['c_f_y'] for i in beta_VLM]
cY_tor = [i[2] for i in beta_tor['beta']]
cl = [-i['c_M_x'] for i in beta_VLM]
cl_tor = [i[3] for i in beta_tor['beta']]
cn = [i['c_M_y'] for i in beta_VLM]
cn_tor = [i[5] for i in beta_tor['beta']]

# Beta plots
fig, ax = plt.subplots(3, sharex=True)
fig.suptitle(r'Variation with $\beta$')
ax[0].plot(beta, cY_tor)
ax[0].plot(beta, cY)
ax[0].plot(beta, -0.5514*deg2rad(beta))
ax[1].plot(beta, cl_tor)
ax[1].plot(beta, cl)
ax[1].plot(beta, -0.0493*deg2rad(beta))
ax[2].plot(beta, cn_tor)
ax[2].plot(beta, cn)
ax[2].plot(beta, 0.10047*deg2rad(beta))
ax[0].legend(['Tornado', 'VLM', 'A320 [4]'])
ax[0].set(ylabel='$C_Y$')
ax[0].grid()
ax[1].set(ylabel='$C_l$')
ax[1].grid()
ax[2].set(ylabel='$C_n$', xlabel=r'$\beta$ [deg]')
ax[2].grid()
fig.savefig(os.getcwd() + '/beta.eps', bbox_inches='tight', format='eps', dpi=1200)
plt.close()

# P data
p_tor = sio.loadmat(os.path.join(tornado, 'p.mat'))
p_VLM = data[82:103]
cY = [i['c_f_y'] for i in p_VLM]
cY_tor = [i[2] for i in p_tor['p']]
cl = [-i['c_M_x'] for i in p_VLM]
cl_tor = [i[3] for i in p_tor['p']]
cn = [i['c_M_y'] for i in p_VLM]
cn_tor = [i[5] for i in p_tor['p']]

# P plots
fig, ax = plt.subplots(3, sharex=True)
fig.suptitle(r'Variation with $p$')
ax[0].plot(p, cY_tor)
ax[0].plot(p, cY)
ax[0].plot(p, np.zeros(p.shape))
ax[1].plot(p, cl_tor)
ax[1].plot(p, cl)
ax[1].plot(p, -0.41956*35.8/(2*231)*p)
ax[2].plot(p, cn_tor)
ax[2].plot(p, cn)
ax[2].plot(p, -0.017505*35.8/(2*231)*p)
ax[0].legend(['Tornado', 'VLM', 'A320 [4]'])
ax[0].set(ylabel='$C_Y$')
ax[0].grid()
ax[1].set(ylabel='$C_l$')
ax[1].grid()
ax[2].set(ylabel='$C_n$', xlabel=r'$p$ [rad/s]')
ax[2].grid()
fig.savefig(os.getcwd() + '/p.eps', bbox_inches='tight', format='eps', dpi=1200)
plt.close()

# Q data
q_tor = sio.loadmat(os.path.join(tornado, 'q.mat'))
q_VLM = data[103:124]
cD = [i['c_f_x'] for i in q_VLM]
cD_tor = [i[1] for i in q_tor['q']]
cL = [-i['c_fz'] for i in q_VLM]
cL_tor = [i[0] for i in q_tor['q']]
cM = [i['c_M_y'] for i in q_VLM]
cM_tor = [i[4] for i in q_tor['q']]

# Q plots
fig, ax = plt.subplots(3, sharex=True)
fig.suptitle(r'Variation with $q$')
ax[0].plot(p, cD_tor)
ax[0].plot(p, cD)
ax[0].plot(p, np.zeros(q.shape))
ax[1].plot(p, cL_tor)
ax[1].plot(p, cL)
ax[1].plot(p, np.zeros(q.shape))
ax[2].plot(p, cM_tor)
ax[2].plot(p, cM)
ax[2].plot(p, -7.03951*q*3.94/(2*231))
ax[0].legend(['Tornado', 'VLM', 'A320 [4]'])
ax[0].set(ylabel='$C_D$')
ax[0].grid()
ax[1].set(ylabel='$C_L$')
ax[1].grid()
ax[2].set(ylabel='$C_m$', xlabel=r'$q$ [rad/s]')
ax[2].grid()
fig.savefig(os.getcwd() + '/q.eps', bbox_inches='tight', format='eps', dpi=1200)
plt.close()

# R data
r_VLM = data[124:]
cY = [i['c_f_y'] for i in r_VLM]
cl = [-i['c_M_x'] for i in r_VLM]
cn = [i['c_M_y'] for i in r_VLM]

fig, ax = plt.subplots(3, sharex=True)
fig.suptitle(r'Variation with $r$')
ax[0].plot(r, cY)
ax[0].plot(r, r*35.8/(2*231)*0.28399)
ax[1].plot(p, cl)
ax[1].plot(p, 0.09915*35.8/(2*231)*p)
ax[2].plot(p, cn)
ax[2].plot(p, -0.22535*35.8/(2*231)*p)
ax[0].legend(['VLM', 'A320 [4]'])
ax[0].set(ylabel='$C_Y$')
ax[0].grid()
ax[1].set(ylabel='$C_l$')
ax[1].grid()
ax[2].set(ylabel='$C_n$', xlabel=r'$r$ [rad/s]')
ax[2].grid()
fig.savefig(os.getcwd() + '/r.eps', bbox_inches='tight', format='eps', dpi=1200)
plt.close()
