
# Execute the benchmarks for 1p5s using TZC protocol, in separate docker,
# enforcing the subscribers start up order, allocating 16GB of shared memory
# to be used by the publishers, and telling the subscriber to wait
# 15 seconds before starting publishing messages, TCP_NODELAY enabled.
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=1p5s_separate_docker --no_pool=yes --pool=no     \
        --extra_params sub_enable_synch_startup:=true               \
                       pub_extra_delay_ms:=15000                    \
                       sub_stats_file_path:=/path/to/results        \
                       shm_size_mega_bytes:=16000


# Execute the benchmarks for 5p1s using TZC protocol, in separate docker,
# with each publisher waiting for the subscriber to establish connection and
# with the subscriber start up delayed by 15secs, TCP_NODELAY enabled.
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        sub_extra_delay_ms:=15000                   \
                        sub_stats_file_path:=/path/to/results


# Execute the benchmarks for 5p1s using TZC protocol, in separate docker,
# with each publisher waiting for the subscriber to establish connection and
# with the publishers start up order enforced, TCP_NODELAY enabled.
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=no --tzc=yes --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        pub_enable_synch_startup:=true              \
                        sub_stats_file_path:=/path/to/results


# Execute the benchmarks for 5p1s using SHM protocol, in separate docker,
# with each publisher waiting for the subscriber to establish connection and
# with the publishers start up order enforced, and image pools disabled.
python2 /ros_comm_fiveai/src/ros_comm/clients/benchmark/execute.py  \
        --tcp=no --shm=yes --tzc=no --udp=no                        \
        --use_case=5p1s_separate_docker --no_pool=yes --pool=no     \
        --extra_params  pub_wait_for_subscribers:=true              \
                        pub_enable_synch_startup:=true              \
                        sub_stats_file_path:=/path/to/results
