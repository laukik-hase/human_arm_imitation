# Avitra Mobile Manipulator

<!-- TABLE OF CONTENTS -->

## Table of Contents

- [About the Project](#about-the-project)
- [Built With](#built-with)
- [Getting Started](#getting-started)
- [Prerequisites](#prerequisites)
- [Usage](#usage)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

<!-- ABOUT THE PROJECT -->

## About The Project

The 21st century has seen an upsurge in Robotics research and development. Autonomous mobile robots have been extensively studied and a lot of implementations have been created. Similarly, the Automation industry has made available a myriad of research on the topic of Dextrous Manipulators. We intend for AVITRA to combine research from both of these fields to create a platform for further research and projects based on autonomous mobile manipulators which are certainly the next step forward in robotics research.

The goal of this project is the development of an autonomous mobile manipulator. The project can be broadly divided into three aspects, the first is the Navigation and Localization System. the second is the motion planning and kinematics system of the manipulator and the third is the perception system. We have made use of the Robot Operating System framework for internal communication between these systems. Our hardware includes a 6-DOF manipulator, The RealSense D345i camera, The Intel NUC, and ESP32 as well as a 2-D LiDAR.


### Built With

- [ROS Kinetic](http://wiki.ros.org/)
- [MoveIt Motion Planning Framework](https://moveit.ros.org/)
- [YOLO: Real-Time Object Detection](https://pjreddie.com/darknet/yolo/)

### Prerequisites

- Python 2.7.x

```sh
sudo apt-get install python2
```

- ROS

```sh
sudo apt-get install ros-kinetic
```

- SLAM
```sh
sudo apt-get install ros-kinetic-depthimage-to-laserscan
sudo apt-get install ros-kinetic-slam-gmapping
sudo apt-get install ros-kinetic-navigation
```

- Manipulator
```sh
sudo apt-get install ros-kinetic-dynamixel-controllers
sudo apt install ros-kinetic-moveit
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
sudo apt-get install ros-kinetic-timed-roslaunch
```

- Perception
```sh
sudo apt-get install ros-kinetic-librealsense2
cd grasp_multiObject_multiGrasp
cd lib
make clean
make
cd ..
cd data
git clone https://github.com/pdollar/coco.git
cd coco/PythonAPI
make
cd ../../..
```

### Perception Prerequisites:
- Tensorflow 1.x
- Python 3
- Matlab/Octave (For data pre-processing)

For more details about perception part check this [repository](https://github.com/SRA-AVITRA/manipulator_ws/tree/updated-perception/grasp_multiObject_multiGrasp)

<!-- USAGE EXAMPLES -->

## Usage

- [Manipulator and Perception](https://github.com/SRA-AVITRA/manipulator_ws)
- [SLAM and Navigation](https://github.com/SRA-AVITRA/avitra_ws)
- [Low Level Control](https://github.com/SRA-AVITRA/avitra_esp) 

<!-- CONTACT -->

## Contact

Saaket Agashe - saaket.agashe@gmail.com
Shweta Kumaran - shwetakum98@gmail.com
Shashank Deshmukh - shshnk.sd@gmail.com
Shambhavi Kuthe - kutheshambhavi1@gmail.com
Aditya Gawali - adityagawali13@gmail.com
Hiten Kothari - hitenkothari08@gmail.com
Akshay Paralikar - akshay41298@gmail.com
Swapnil Mane - swapnilmane1393@gmail.com
Neil Menezes - neil5698@gmail.com 
Jigar Joisar - jedge2221@gmail.com 
Nitesh Gowda - gowdanitesh2598@gmail.com

<!-- Acknowledgements -->

## Acknowledgements

- [Avitra : 2017-2018](https://github.com/sachin0x18/Imitation)
- [Avitra : 2018-2019](https://github.com/atharvkhadtare/fyp_ws)


<!-- LICENSE -->

## License

Distributed under the MIT License. See `LICENSE` for more information.
