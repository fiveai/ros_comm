# SHM on ROS
## General description
This repo contains the C++ implementation of the shared memory mechanism described in the _Smart Pointers and Shared Memory Synchronisation for Efficient Inter-process Communication in ROS on an Autonomous Vehicle_ paper along with detailed build and deployment instructions.

The implementation is based on ROS 1.12.14 but it can be easily ported to more modern ROS versions which support C++11 or later. The code can be built either directly on the host machine _or_ within a Docker container via  [the provided Docker script](https://github.com/costinior/ros_comm/blob/shm-on-ros/clients/benchmark/docker/Dockerfile) . The former assumes the installation of ROS 1.12.14, Boost 1.58.0 and python2 on the host machine. The latter requires Docker installation only on the host machine.

To support data gathering and ensure consistency across runs, a [suite of benchmarks](https://github.com/costinior/ros_comm/tree/shm-on-ros/clients/benchmark) has been put put in place. The suites or individual tests can be executed following the procedure described below. Currently, four protocols are supported: TCP, UDP, SHM and TZC.

[Boost.Process 1_65_0](https://www.boost.org/doc/libs/1_65_0/doc/html/process.html) has been imported into the source tree to support [unit tests](https://github.com/costinior/ros_comm/blob/shm-on-ros/test/test_roscpp/test/test_shm).

Bug fixes applied on top of ROS 1.12.14:
 - _roslaunch_ [bug fix](https://github.com/ros/ros_comm/pull/1115)
 - replace _boost::condition_variable_ variable with _std::condition_variable_ in [callback_queue.cpp](https://github.com/costinior/ros_comm/blob/shm-on-ros/clients/roscpp/src/libros/callback_queue.cpp)
## Cloning the repo and switching to the target branch
Note, the target folder structure must be the one shown below, or else the subsequent steps won't work.
```
git clone git@github.com:costinior/ros_comm.git $HOME/ros_comm/src/ros_comm
git checkout shm-on-ros
```
## Compile the entire workspace
```
cd $HOME/ros_comm
source /opt/ros/kinetic/setup.sh  # this assumes ROS kinetic 1.12.14 is already installed
catkin_make_isolated
  --source ./src/  \
  --build $HOME/ros_comm/build-release \
  --devel $HOME/ros_comm/devel-release \
  --install-space $HOME/ros_comm/install-release  \
  --install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Execute the benchmarks specified in launch.xml overriding some of the parameters
```
cd $HOME/ros_comm
source ./devel-release/setup.sh
roslaunch --screen -v  benchmark launch.xml   \
  use_case:=5p1s_same_host sub_stats_file_path:=$HOME \
  transport:=shm pub_queue_size:=200 pub_image_count:=200
```
## Execute the benchmark suite deploying the ROS nodes in separate docker images
```
cd ~/ros_comm/src/ros_comm/clients/benchmark/docker
docker-compose up

# on a different terminal attach to node1; note it must be node1!
docker attach node1

# once inside the container
cd /ros_comm
python2 ./src/ros_comm/clients/benchmark/execute.py --tcp=no --shm=yes --use_case=5p1s_separate_docker
```
## Build the docker image
```
cd ~/ros_comm/src/ros_comm/clients/benchmark/docker
export DOCKER_BUILDKIT=1
docker build --ssh default --tag YOUR_REGISTRY/shm-on-ros:2.4 .
```

## Benchmark execution command examples
Execute the benchmarks for 1p5s using TZC protocol, in separate docker, enforcing the subscribers start up order, allocating 16GB of shared memory to be used by the publishers, and telling the subscriber to wait 15 seconds before starting publishing messages, TCP_NODELAY enabled.
```
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=1p5s_separate_docker --no_pool=yes --pool=no     \
        --extra_params sub_enable_synch_startup:=true               \
                       pub_extra_delay_ms:=15000                    \
                       sub_stats_file_path:=/path/to/results        \
                       shm_size_mega_bytes:=
```


Execute the benchmarks for 5p1s using TZC protocol, in separate docker, with each publisher waiting for the subscriber to establish connection and with the subscriber start up delayed by 15secs, TCP_NODELAY enabled.
```
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        sub_extra_delay_ms:=15000                   \
                        sub_stats_file_path:=/path/to/results
```

Execute the benchmarks for 5p1s using TZC protocol, in separate docker, with each publisher waiting for the subscriber to establish connection and with the publishers start up order enforced, TCP_NODELAY enabled.
```
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        pub_enable_synch_startup:=true              \
                        sub_stats_file_path:=/path/to/results
```


Execute the benchmarks for 5p1s using SHM protocol, in separate docker, with each publisher waiting for the subscriber to establish connection and with the publishers start up order enforced, and image pools disabled.
```
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=yes --tzc=no --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        pub_enable_synch_startup:=true              \
                        sub_stats_file_path:=/path/to/results
```
