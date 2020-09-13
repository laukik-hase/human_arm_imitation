/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <vector>
#include <fstream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// takes a csv file with first column timestamps and the next <int columns> columns filled with joint angles in DEGREES
std::vector<VectorNd> load_csv (const std::string & path, int columns, std::vector<double>& timestamp) {
  std::vector<VectorNd> data;
  std::ifstream file;
  file.open(path);
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    VectorNd row = VectorNd::Zero(columns);
    uint column = 0;
    while (std::getline(lineStream, cell, ',')) {
      if (column==0){ // first column is timestamp
        timestamp.push_back(std::stod(cell));
      }
      else{
        row[column-1] = (std::stod(cell)*M_PI/180);
      }
      ++column;
    }
    data.push_back(row);  
  }
  file.close();
  return data;
}

void store_csv(const std::string & path, std::vector<VectorNd> data, std::vector<double>& timestamp){
  std::ofstream file;
  file.open(path);
  for(int i=0; i<data.size(); i++){
    file<<timestamp[i];
    for(int j=0; j<data[i].size(); j++){
      file<<","<<data[i][j];
    }
    file<<"\n";
  }
  file.close();
}

int main (int argc, char* argv[]) {
  rbdl_check_api_version (RBDL_API_VERSION);

  Model* model = new Model();

  if (argc != 3) {
    std::cerr << "Error: not right number of arguments." << std::endl;
    std::cerr << "usage: " << argv[0] << " <model.urdf>" << " <angles.csv>" << std::endl;
    exit(-1);
  }

  if (!Addons::URDFReadFromFile (argv[1], model, false)) {
    std::cerr << "Error loading model " << argv[1] << std::endl;
    abort();
  }
  std::cout << "Degree of freedom overview:" << std::endl;
  std::cout << Utils::GetModelDOFOverview(*model);

  std::cout << "Model Hierarchy:" << std::endl;
  std::cout << Utils::GetModelHierarchy(*model);

  std::vector<double> timestamp;
  std::cout<<model->q_size<<"\n";
  std::vector<VectorNd> Q = load_csv(argv[2], model->q_size, timestamp);

  std::vector<VectorNd> QDot(Q.size(), VectorNd::Zero (model->q_size));
  for(int i=1; i<Q.size(); i++){
    QDot[i] = (Q[i] - Q[i-1]) / (timestamp[i] - timestamp[i-1]);
  }

  std::vector<VectorNd> QDDot(Q.size(), VectorNd::Zero (model->q_size));
  for(int i=1; i<Q.size(); i++){
    QDDot[i] = (QDot[i] - QDot[i-1]) / (timestamp[i] - timestamp[i-1]);
  }

  std::vector<VectorNd> Tau(Q.size(), VectorNd::Zero (model->q_size));


  std::cout << "Inverse Dynamics with q, qdot, qddot set to zero:" << std::endl;
  for(int i=0; i<Q.size(); i++){
    InverseDynamics (*model, Q[i], QDot[i], QDDot[i], Tau[i]);
    // std::cout << Tau[i].transpose() << std::endl;
  }
  store_csv("QDot.csv", QDot, timestamp);
  store_csv("QDDot.csv", QDDot, timestamp);
  store_csv("torque.csv", Tau, timestamp);

  delete model;

  return 0;
}

