# SHM on ROS

## TO DO BEFORE RELEASE



## General description

This repository contains the C++ implementation of the shared memory mechanism described in the paper _Smart Pointers and Shared Memory Synchronisation for Efficient Inter-process Communication in ROS on an Autonomous Vehicle_ (IROS 2021) along with detailed build and deployment instructions.

The implementation is based on ROS (Kinetic) 1.12.14 but it can be easily ported to more modern ROS versions which support C++11 or later. The code can be built either directly on the host machine _or_ within a Docker container via [the provided Docker script](./clients/benchmark/docker/Dockerfile). The former assumes the installation of ROS 1.12.14, Boost 1.58.0 and python2 on the host machine. The latter only requires Docker installation on the host machine. See [the Docker website](https://docs.docker.com/get-docker/) for details of setting up Docker on the host machine.

A [suite of benchmarks](./clients/benchmark) has been provided. The entire suite or individual tests can be executed following the procedure described below. Currently, four protocols are supported: TCP, UDP, SHM and TZC.

When executed from within the Docker containers, the benchmarks results are generated under ${HOME}/fiveshm folder. Otherwise, environment variable SHM_ON_ROS_HOME dictates the results folder.

### Notes on provided code

[Boost.Process 1_65_0](https://www.boost.org/doc/libs/1_65_0/doc/html/process.html) has been imported into the source tree to support [unit tests](./test/test_roscpp/test/test_shm.cpp).

We apply the following bug fixes on top of ROS 1.12.14 in addition to our own code:
 - _roslaunch_ [bug fix](https://github.com/ros/ros_comm/pull/1115)
 - replace _boost::condition_variable_ variable with _std::condition_variable_ in [callback_queue.cpp](./clients/roscpp/src/libros/callback_queue.cpp)

For convenience of running the benchmarks, we have included TZC in this repository. TZC was developed by a group researchers afilliated to Tsinghua University, China and University of Maryland, USA and is described more fully in the _TZC: Efficient Inter-Process Communication for Robotics Middleware with Partial Serialization_. Its authors have released it under the [BSD](https://github.com/qboticslabs/tzc_transport/blob/master/package.xml) license.

## Building and running benchmarks natively

The following steps build the code directly on the host machine.

### Cloning the repo and switching to the target branch

Note, the target folder structure must be the one shown below, or else the subsequent steps won't work.

```
git clone git@github.com:costinior/ros_comm.git $HOME/ros_comm/src/ros_comm
git checkout fiveshm
```

### Compile the entire workspace

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

### Execute the benchmarks specified in launch.xml overriding some of the parameters

```
cd $HOME/ros_comm
source ./devel-release/setup.sh
roslaunch --screen -v  benchmark launch.xml   \
  use_case:=5p1s_same_host sub_stats_file_path:=$HOME \
  transport:=shm pub_queue_size:=200 pub_image_count:=200
```

## Building and running benchmarks within Docker

### Build the docker image

```
cd ~/ros_comm/src/ros_comm/clients/benchmark/docker
export DOCKER_BUILDKIT=1
docker build --ssh default --tag fiveshm:2.4 .
```

### Execute the benchmark suite deploying the ROS nodes in separate docker images

Assuming the Docker container built as above, with a tag of `fiveshm:2.4`:

```
cd ~/ros_comm/src/ros_comm/clients/benchmark/docker
docker-compose up

# on a different terminal attach to node1; note it must be node1!
docker attach node1
```

And then, from within the container, our benchmarks can be executed using commands of the form:

```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py --tcp=no --shm=yes --use_case=5p1s_separate_docker
```

The next sections provides examples of such commands.

### Benchmark execution command examples

Execute the benchmarks for 1p5s using TZC protocol, in separate Docker containers, enforcing the subscribers start up order, allocating 16GB of shared memory to be used by the publishers, and telling the subscriber to wait 15 seconds before starting publishing messages, TCP_NODELAY enabled.
```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=1p5s_separate_docker --no_pool=yes --pool=no     \
        --extra_params sub_enable_synch_startup:=true               \
                       pub_extra_delay_ms:=15000                    \
                       sub_stats_file_path:=/path/to/results        \
                       shm_size_mega_bytes:=
```


Execute the benchmarks for 5p1s using TZC protocol, in separate Docker containers, with each publisher waiting for the subscriber to establish connection and with the subscriber start up delayed by 15secs, TCP_NODELAY enabled.
```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        sub_extra_delay_ms:=15000                   \
                        sub_stats_file_path:=/path/to/results
```

Execute the benchmarks for 5p1s using TZC protocol, in separate Docker containers, with each publisher waiting for the subscriber to establish connection and with the publishers start up order enforced, TCP_NODELAY enabled.
```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        pub_enable_synch_startup:=true              \
                        sub_stats_file_path:=/path/to/results
```


Execute the benchmarks for 5p1s using SHM protocol, in separate Docker containers, with each publisher waiting for the subscriber to establish connection and with the publishers start up order enforced, and image pools disabled.
```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=yes --tzc=no --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        pub_enable_synch_startup:=true              \
                        sub_stats_file_path:=/path/to/results
```
