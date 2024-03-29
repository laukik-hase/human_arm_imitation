PROJECT (RBDLEXAMPLE CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

# Search for the RBDL include directory and library
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# Create an executable
ADD_EXECUTABLE (inv_dyn_from_urdf_and_angles_csv inv_dyn_from_urdf_and_angles_csv.cc)
ADD_EXECUTABLE (inv_dyn_urdf_test inv_dyn_urdf_test.cc)

# And link the library against the executable

TARGET_LINK_LIBRARIES ( inv_dyn_from_urdf_and_angles_csv
	${RBDL_LIBRARY}
	${RBDL_URDFReader_LIBRARY}
	)

TARGET_LINK_LIBRARIES ( inv_dyn_urdf_test
	${RBDL_LIBRARY}
	${RBDL_URDFReader_LIBRARY}
	)