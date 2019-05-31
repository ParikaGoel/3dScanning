Course : 3D Scanning & Motion Capture (IN2354)

This repository contains the solutions of the assignments of the above course

############# Dependencies ####################

>>> Install Eigen library

1. Download eigen's source code from http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
2. Extract the souce code
3. Build the source code using following commands 
 -> Go inside the source directory
 -> mkdir build && cd build
 -> cmake -D CMAKE_BUILD_TYPE=Release .. && make install


>>> Install FreeImage library

sudo apt-get install libfreeimage3 libfreeimage-dev libfreeimageplus3 libfreeimageplus-dev


############ Dataset ########################
Dataset required to be downloaded for Exercise 1
Download the recorded camera data from the TUM RGB-D SLAM Dataset
(https://vision.in.tum.de/data/datasets/rgbd-dataset). Use the Freiburg 1 dataset “fr1/xyz”, extract the tgz
archive to the data folder (make sure the variable filenameIn in main() is set respectively)
