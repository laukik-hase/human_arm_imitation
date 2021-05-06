import numpy as np
import rbdl
import sys

class rbdlpy:
    def __init__(self, _urdf_filename):
        self.model = rbdl.loadModel(_urdf_filename, kwargs={"floating_base": False, "verbose": False})
        self.q_size = self.model.q_size
        self.qdot_size = self.model.qdot_size

    def inverse_dynamics(self, _timeV, _qV):
        '''
        _timeV in seconds
        _qV in degrees
        '''
        n = len(_timeV)
        _qV = _qV * np.pi/180
        
        # Take numerical derivatives of q to get estimates qDot (generalized velocities)
        # and qDDot (generalized accelerations)

        qDotV = np.zeros(shape=(n, q_size), dtype=float)
        qDDotV = np.zeros(shape=(n, q_size), dtype=float)
        _tauV = np.zeros(shape=(n, q_size), dtype=float)

        # The values for qdot and qddot are formed using numerical derivatives

        for i in range(0, model.q_size):
            qDotV[:, i] = np.gradient(_qV[:, i], timeV[:])

        for i in range(0, model.q_size):
            qDDotV[:, i] = np.gradient(qDotV[:, i], timeV[:])

        q   = np.zeros(shape=(q_size),   dtype=float)
        qd  = np.zeros(shape=(qdot_size),dtype=float)
        qdd = np.zeros(shape=(qdot_size),dtype=float)
        tau = np.zeros(shape=(qdot_size),dtype=float)

        for i in range(0,n):
            for j in range(0, q_size):
                q[j] = _qV[i, j]
            for j in range(0, qdot_size):
                qd[j] = qDotV[i, j]
                qdd[j] = qDDotV[i, j]
            # The inverse dynamics function in RBDL is called
            rbdl.InverseDynamics(model, q, qd, qdd, tau)
            
            # The generalized force vector tau is copied to a matrix
            for j in range(0, qdot_size):
                _tauV[i, j] = tau[j]

        return _tauV

if __name__=="__main__":
    urdf_filename = sys.argv[1]
    my_rbdlpy = rbdlpy(urdf_filename)

    csv_filename = sys.argv[2]
    csv_data = np.genfromtxt(csv_filename, delimiter=',')
    timeV = csv_data[:,0]
    qV = csv_data[:,1:]
    
    tauV = my_rbdlpy.inverse_dynamics(timeV, qV)
    
    np.savetxt('pytorque.csv', tauV, delimiter=',')