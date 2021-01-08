### create package and execute

```bash
$catkin_create_pkg ex_urdf rospy nav_msgs (이후 Rviz를 위한것들 추가 요망)
$roscore
$rospack profile
$rosrun odom_test odom_publisher_ex.py
```


### 코드 참고

- [해당 코드](https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf)에서  ``#!/usr/bin/env python`` 을 ``#!/usr/bin/env python2`` 로 변경했다.



```s
    $ rosrun 
    $ rostopic echo /odom
    header: 
    seq: 1
    stamp: 
        secs: 1610103318
        nsecs: 521123886
    frame_id: "odom"
    child_frame_id: "base_link"
    pose: 
    pose: 
        position: 
        x: 2.28659884145
        y: 0.263303990484
        z: 0.0
        orientation: 
        x: 0.0
        y: 0.0
        z: 0.813442027268
        w: 0.581645999105
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist: 
    twist: 
        linear: 
        x: 0.1
        y: -0.1
        z: 0.0
        angular: 
        x: 0.0
        y: 0.0
        z: 0.1
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ---

```

### rviz 실행

```s
roslaunch ex_urdf odom_pub.launch
```
