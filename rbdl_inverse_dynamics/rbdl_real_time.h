#include <iostream>
#include <vector>
#include <fstream>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

std::vector<RigidBodyDynamics::Math::VectorNd> get_torque(const char *urdf_filename, std::vector<double> timestamp, std::vector<RigidBodyDynamics::Math::VectorNd> angles);