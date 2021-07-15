# FiveSHM

## General description

This repository contains the C++ implementation of the shared memory mechanism described in the paper _Smart Pointers and Shared Memory Synchronisation for Efficient Inter-process Communication in ROS on an Autonomous Vehicle_ (IROS 2021) along with detailed build and deployment instructions.

The implementation consists of a modified version of the `ros_comm` package from ROS Kinetic (v1.12.14) along with a [suite of benchmarks](./clients/benchmark).

The code can be built either directly on the host machine _or_ within a Docker container via [the provided Docker script](./clients/benchmark/docker/Dockerfile). The former assumes the installation of ROS 1.12.14, Boost 1.58.0 and python2 on the host machine. The latter only requires Docker installation on the host machine. See [the Docker website](https://docs.docker.com/get-docker/) for details of setting up Docker on the host machine.

The benchmarks - either the entire suite or individual tests - can be executed following the procedure described below. Currently, four protocols are supported: TCP, UDP, SHM and TZC.

When executed from within the Docker containers, the benchmarks results are generated under `${HOME}/fiveshm` folder. Otherwise, environment variable `FIVESHM_HOME` dictates the results folder.

### Notes on provided code

We apply the following bug fixes on top of ROS 1.12.14 in addition to our own code:
 - _roslaunch_ [bug fix](https://github.com/ros/ros_comm/pull/1115).
 - replace `boost::condition_variable` variable with `std::condition_variable` in [callback_queue.cpp](./clients/roscpp/src/libros/callback_queue.cpp).

### Third party code

[Boost.Process 1_65_0](https://www.boost.org/doc/libs/1_65_0/doc/html/process.html) has been imported into the source tree to support [unit tests](./test/test_roscpp/test/test_shm.cpp). We note that [Boost 1.65.0](https://github.com/boostorg/boost/tree/boost-1.65.0) is released under the [Boost Software License](http://www.boost.org/LICENSE_1_0.txt).

For convenience of running the benchmarks, we have also included [TZC](https://github.com/qboticslabs/tzc_transport) in this repository. TZC was developed by a group researchers afilliated to Tsinghua University, China and University of Maryland, USA and is described more fully in the _TZC: Efficient Inter-Process Communication for Robotics Middleware with Partial Serialization_. Its authors have released it under the [BSD](https://github.com/qboticslabs/tzc_transport/blob/master/package.xml) license.

## Cloning the repository and switching to the target branch

Note that we clone the repository into `$HOME/ros_comm/src/ros_comm` in this step. We assume this directory in subsequent steps. If you prefer to use a different working directory, the subsequent steps will need to be appropriately modified.

```
git clone git@github.com:fiveai/ros_comm.git $HOME/ros_comm/src/ros_comm
cd $HOME/ros_comm/src/ros_comm
git checkout fiveshm
```

## Building and running the benchmarks natively

The following steps build the code directly on the host machine.

### Compiling the entire workspace

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

### Executing the benchmarks specified in launch.xml overriding some of the parameters

```
cd $HOME/ros_comm
source ./devel-release/setup.sh
roslaunch --screen -v  benchmark launch.xml   \
  use_case:=5p1s_same_host sub_stats_file_path:=$HOME \
  transport:=shm pub_queue_size:=200 pub_image_count:=200
```

## Building and running benchmarks within Docker containers

### Building the Docker image

```
cd ~/ros_comm/src/ros_comm/clients/benchmark/docker
export DOCKER_BUILDKIT=1
docker build --ssh default --tag fiveshm .
```

### Executing the benchmark suite, deploying the ROS nodes in separate docker images

Assuming the Docker container built as above, with a tag of `fiveshm`:

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

1. Execute the benchmarks suite for 1p5s using TZC protocol, in separate Docker containers, enforcing the subscribers start up order, allocating 16GB of shared memory to be used by the publishers, and telling the subscriber to wait 15 seconds before starting publishing messages, `TCP_NODELAY` enabled.

```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py      \
        --tcp=no --shm=no --tzc=yes --udp=no                     \
        --use_case=1p5s_separate_docker --no_pool=yes --pool=no  \
        --extra_params sub_enable_synch_startup:=true            \
                       pub_extra_delay_ms:=15000                 \
                       shm_size_mega_bytes:=16000
```

2. Execute the benchmarks suite for 5p1s using TZC protocol, in separate Docker containers, with each publisher waiting for the subscriber to establish connection and with the subscriber start up delayed by 15secs, `TCP_NODELAY` enabled.

```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py      \
        --tcp=no --shm=no --tzc=yes --udp=no                     \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no  \
        --extra_params  pub_wait_for_subscribers:=true           \
                        sub_extra_delay_ms:=15000
```

3. Execute the benchmarks suite for 5p1s using TZC protocol, in separate Docker containers, with each publisher waiting for the subscriber to establish connection and with the publishers start up order enforced, `TCP_NODELAY` enabled, overriding the default results path.

```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py      \
        --tcp=no --shm=no --tzc=yes --udp=no                     \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no  \
        --extra_params  pub_wait_for_subscribers:=true           \
                        pub_enable_synch_startup:=true           \
                        sub_stats_file_path:=/path/to/results
```

4. Execute the benchmarks suite for 5p1s using SHM protocol, in separate Docker containers, with each publisher waiting for the subscriber to establish connection and with the publishers start up order enforced, and image pools disabled.

```
python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py      \
        --tcp=no --shm=yes --tzc=no --udp=no                     \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no  \
        --extra_params  pub_wait_for_subscribers:=true           \
                        pub_enable_synch_startup:=true
```

5. Execute _one single test_ using TCP protocol, in same docker container, with image pool disabled.

```
roslaunch --screen -v  benchmark launch.xml     \
        pub_image_count:=200                    \
        use_case:=1p5s_same_docker              \
        transport:=tcp                          \
        image_width_pixels:=512                 \
        image_height_pixels:=512                \
        pub_pool_size:=0
```
