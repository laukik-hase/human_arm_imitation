import numpy as np
import dartpy as dart

urdf_filename = ""
urdfParser = dart.utils.DartLoader()
model = urdfParser.parseSkeleton(urdf_filename)
dofs = model.getNumDofs()
ee = model
Kp = np.eye(3) * 50.0
Kd = np.eye(dofs) * 5.0
ee_offset = [0.05, 0, 0]

tf = ee.getTransform()
tf.pretranslate(ee_offset)
target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target", tf)

M = model.getMassMatrix()

J = ee.getLinearJacobian()
Jt = J.transpose()
JJt = np.matmul(J, Jt)
kI = 0.0025 * np.eye(3)
invJ = np.matmul(Jt, np.linalg.inv(JJt + kI))

dJ = ee.getLinearJacobianDeriv()
dJt = dJ.transpose()
dJdJt = np.matmul(dJ, dJt)
invdJ = np.matmul(dJt, np.linalg.inv(dJdJt + kI))

e = target.getTransform().translation() - ee.getTransform().translation()
de = -ee.getLinearVelocity()

cg = model.getCoriolisAndGravityForces()

tmp1 = np.matmul(np.matmul(invJ, Kp), de)
tmp2 = np.matmul(np.matmul(invdJ, Kp), e)

forces1 = np.matmul(M, tmp1 + tmp2)
forces2 = cg
forces3 = np.matmul(np.matmul(np.matmul(Kd, invJ), Kp), e)

forces = forces1 + forces2 + forces3