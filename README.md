Course : 3D Scanning & Motion Capture (IN2354)

This repository contains the solutions of the assignments of the above course

## Dependencies

### Install Eigen library

1. Download eigen's source code from http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
2. Extract the souce code
3. Build the source code using following commands 
 -> Go inside the source directory
 -> mkdir build && cd build
 -> cmake -D CMAKE_BUILD_TYPE=Release .. && make install


### Install FreeImage library

sudo apt-get install libfreeimage3 libfreeimage-dev libfreeimageplus3 libfreeimageplus-dev

### Install flann library

1. git clone git://github.com/mariusmuja/flann.git
2. mkdir flann/build
3. cd flann/build
4. cmake ..
5. make -j4
6. make install


### Install ceres library

>>> Install dependencies
1. sudo apt-get install libgoogle-glog-dev
2. sudo apt-get install libatlas-base-dev

>>> Compile Ceres
1. git clone https://ceres-solver.googlesource.com/ceres-solver
2. mkdir ceres-solver/build
3. cd ceres-solver/build
4. cmake ..
5. make -j3
6. make install


## Dataset

Download the recorded camera data from the TUM RGB-D SLAM Dataset
(https://vision.in.tum.de/data/datasets/rgbd-dataset). Use the Freiburg 1 dataset “fr1/xyz”, extract the tgz
archive to the data folder (make sure the variable filenameIn in main() is set respectively)
