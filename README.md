# Kinect Fusion project

## Setup

- Firstly, create data directory and download data to it. You can just use this [link](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz) or these commands:\
`mkdir data`\
`cd data`\
`wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz`\
`tar -xzf rgbd_dataset_freiburg1_xyz.tgz`\
`rm rgbd_dataset_freiburg1_xyz.tgz`

- Create build directory:\
`cmake -S . -B build`

- Compile the code:\
`cmake --build build`

To run the tests:\
`cd build && ctest`