### Edge Detection with rosbag

1. 실행 방법

    ```bash
    # terminal A
    roscore

    # terminal B
    rosbag play cam_topic.bag
    rosbag play -l cam_topic.bag #무한 반복

    # terminal C
    rosrun my_cam edge_cam.py
    ```
