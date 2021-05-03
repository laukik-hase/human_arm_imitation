
// #include <iostream>
// #include <vector>
// #include <fstream>
// #include <rbdl/rbdl.h>
// #include <rbdl/rbdl_utils.h>

// #ifndef RBDL_BUILD_ADDON_URDFREADER
// #error "Error: RBDL addon URDFReader not enabled."
// #endif

// #include <rbdl/addons/urdfreader/urdfreader.h>
#include "rbdl_real_time.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

std::vector<VectorNd> get_torque(const char *urdf_filename, std::vector<double> timestamp, std::vector<VectorNd> angles)
{
    rbdl_check_api_version(RBDL_API_VERSION);
    Model *model = new Model();
    if (!Addons::URDFReadFromFile(urdf_filename, model, false))
    {
        std::cout << "Error reading urdf\n";
    }

    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << Utils::GetModelDOFOverview(*model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << Utils::GetModelHierarchy(*model);

    std::vector<VectorNd> Q = angles;

    std::vector<VectorNd> QDot(Q.size(), VectorNd::Zero(model->q_size));
    for (int i = 1; i < Q.size(); i++)
    {
        QDot[i] = (Q[i] - Q[i - 1]) / (timestamp[i] - timestamp[i - 1]);
    }

    std::vector<VectorNd> QDDot(Q.size(), VectorNd::Zero(model->q_size));
    for (int i = 1; i < Q.size(); i++)
    {
        QDDot[i] = (QDot[i] - QDot[i - 1]) / (timestamp[i] - timestamp[i - 1]);
    }

    std::vector<VectorNd> Tau(Q.size(), VectorNd::Zero(model->q_size));

    std::cout << "Inverse Dynamics with q, qdot, qddot set to zero:" << std::endl;
    for (int i = 0; i < Q.size(); i++)
    {
        InverseDynamics(*model, Q[i], QDot[i], QDDot[i], Tau[i]);
    }

    delete model;
    return Tau;

}
