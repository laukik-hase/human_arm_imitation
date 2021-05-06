import numpy as np
import rbdl
from scipy import signal

np.set_printoptions(suppress=True)
urdf_filename = "manipulator4_meshed.urdf"
csv_filename = "datasets/demo2.csv"
model = rbdl.loadModel(urdf_filename, kwargs={"floating_base": False, "verbose": True})
print("DoF: ", model.q_size)
q_size = model.q_size
qdot_size = model.qdot_size

csv_data = np.genfromtxt(csv_filename, delimiter=',')
timeV = csv_data[:,0]
qV = csv_data[:,1:]*np.pi/180

# print(timeV)
# print(qV)

# Read in the experimental data
# Column  0: is time
# Columns 1 ,..., N: correspond to q0, ..., qN

n = len(timeV)

# Take numerical derivatives of q to get estimates qDot (generalized velocities)
# and qDDot (generalized accelerations)

qDotV = np.zeros(shape=(n, q_size), dtype=float)
qDDotV = np.zeros(shape=(n, q_size), dtype=float)
tauV = np.zeros(shape=(n, q_size), dtype=float)

# 3e. The values for qdot and qddot are formed using numerical derivatives
# for i in range(1,n):
#     qDotV[i] = (qV[i] - qV[i-1]) / (timeV[i] - timeV[i-1])
#     qDDotV[i] = (qDotV[i] - qDotV[i-1]) / (timeV[i] - timeV[i-1])

    # qDotV[:, i] = np.gradient(qV[:, i], timeV[:])

for i in range(0, model.q_size):
    qDotV[:, i] = np.gradient(qV[:, i], timeV[:])

for i in range(0, model.q_size):
    qDDotV[:, i] = np.gradient(qDotV[:, i], timeV[:])

# print(qDDotV)

q   = np.zeros(shape=(q_size),   dtype=float)
qd  = np.zeros(shape=(qdot_size),dtype=float)
qdd = np.zeros(shape=(qdot_size),dtype=float)
tau = np.zeros(shape=(qdot_size),dtype=float)

for i in range(0,n):
    for j in range(0, q_size):
        q[j] = qV[i, j]
    for j in range(0, qdot_size):
        qd[j] = qDotV[i, j]
        qdd[j] = qDDotV[i, j]
    # 3h. The inverse dynamics function in RBDL is called
    # print(qdd)
    
    rbdl.InverseDynamics(model, q, qd, qdd, tau)
    # 3i. The generalized force vector tau is copied to a matrix
    for j in range(0, qdot_size):
        tauV[i, j] = tau[j]

print(tauV)
np.savetxt('pytorque.csv', tauV, delimiter=',')
