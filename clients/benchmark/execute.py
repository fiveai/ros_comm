# ***************************************************************************************************
# * Copyright Five AI 2021.
# * All rights reserved.
# ***************************************************************************************************

# This script runs on top of roslaunch. Its purpose is to execute suites
# of benchmarks and it does so by generating sets of parameters
# that are then passed to roslaunch that in turn takes care of launching
# the ROS nodes specified in the launch.xml file. The launch.xml file takes
# in several parameters that can be overriden by this script via --extra_params
# argument.

import copy
import roslaunch
import rospy
import argparse

def profile(params):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    args = ['benchmark', 'launch.xml']
    args += params

    file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
    args = args[2:]

    launch_files = [(file, args)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files, verbose=True, force_screen=True)
    parent.start()
    parent.spin();
          
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Help')
    parser.add_argument('--pub_image_count', type=int, default=2000, help='The number of images broadcast by the publisher')
    parser.add_argument('--pub_pool_size', type=int, default=100, help='The number of images each publisher buffers before starting broadcasting')
    parser.add_argument('--extra_params', nargs='*', type=str, default="", help='Extra parameters in the form param:=value that are to be passed to roslaunch. Example: --extra_params transport:=tcp pub_hz:=10')
    parser.add_argument('--use_case', type=str, help="1p1s_same_host 1p5s_same_host 5p1s_same_host " +
                                                     "1p1s_same_docker 1p5s_same_docker 5p1s_same_docker " +
                                                     "1p1s_separate_docker 1p5s_separate_docker 5p1s_separate_docker")
    parser.add_argument('--tcp',      default='yes',  choices=['yes', 'no'], help="Executes TCP tests")
    parser.add_argument('--udp',      default='yes',  choices=['yes', 'no'], help="Executes UDP tests")
    parser.add_argument('--shm',      default='yes',  choices=['yes', 'no'], help="Executes SHM tests")
    parser.add_argument('--tzc',      default='yes',  choices=['yes', 'no'], help="Executes SHM tests")
    parser.add_argument('--pool',     default='yes',  choices=['yes', 'no'], help="Executes tests with the pool memory enabled")
    parser.add_argument('--no_pool',  default='yes',  choices=['yes', 'no'], help="Executes tests with the pool memory disabled")

    args = parser.parse_args()

    tcp_pool_params = [
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=4096", "image_height_pixels:=4096", "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=4096", "image_height_pixels:=2048", "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=4096", "image_height_pixels:=1024", "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=2048", "image_height_pixels:=1024", "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=1024", "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=512" , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=128" , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=32"  , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=8"   , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=4"   , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=2"   , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=1024", "image_height_pixels:=1"   , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=512",  "image_height_pixels:=1"   , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
        ["use_case:=" + args.use_case, "transport:=tcp", "image_width_pixels:=256",  "image_height_pixels:=1"   , "pub_image_count:=" + str(args.pub_image_count), "pub_hz:=30", "pub_pool_size:=" + str(args.pub_pool_size)],
    ]

    shm_pool_params = copy.deepcopy(tcp_pool_params)
    for L in shm_pool_params:
        L[1] = "transport:=shm"

    udp_pool_params = copy.deepcopy(tcp_pool_params)
    for L in udp_pool_params:
        L[1] = "transport:=udp"

    tzc_pool_params = copy.deepcopy(tcp_pool_params)
    for L in tzc_pool_params:
        L[1] = "transport:=tzc"

    tcp_no_pool_params = copy.deepcopy(tcp_pool_params)
    for L in tcp_no_pool_params:
        L[-1] = "pub_pool_size:=0"

    udp_no_pool_params = copy.deepcopy(udp_pool_params)
    for L in udp_no_pool_params:
        L[-1] = "pub_pool_size:=0"

    tzc_no_pool_params = copy.deepcopy(tzc_pool_params)
    for L in tzc_no_pool_params:
        L[-1] = "pub_pool_size:=0"

    shm_no_pool_params = copy.deepcopy(shm_pool_params)
    for L in shm_no_pool_params:
        L[-1] = "pub_pool_size:=0"

    # collect all params generated by this script

    all_params = list(list())
    if args.tcp == 'yes':
        if args.pool == 'yes':
            all_params += tcp_pool_params
        if args.no_pool == 'yes':
            all_params += tcp_no_pool_params
    if args.udp == 'yes':
        if args.pool == 'yes':
            all_params += udp_pool_params
        if args.no_pool == 'yes':
            all_params += udp_no_pool_params
    if args.tzc == 'yes':
        if args.pool == 'yes':
            all_params += tzc_pool_params
        if args.no_pool == 'yes':
            all_params += tzc_no_pool_params
    if args.shm == 'yes':
        if args.pool == 'yes':
            all_params += shm_pool_params
        if args.no_pool == 'yes':
            all_params += shm_no_pool_params

    # for each parameter set, append the command line extra parameters
    for L in all_params:
        L += args.extra_params

    for L in all_params:
        print(' '.join(L))

    for param_set in all_params:
        profile(param_set)
