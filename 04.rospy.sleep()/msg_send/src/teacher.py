#!/usr/bin/env python

# 파이썬 노드를 선언할 때 위와 같이 파이썬 위치를 알려준다. 이는 파이썬 스크립트로써 현재 스크립트가 실행되어야함을 의미한다.

import rospy # 노드를 사용하기 위해서 import
from std_msgs.msg import String #std_msgs/String 타입을 이용하기 위해서 import

rospy.init_node('teacher') # teacher라는 노드 생성

pub = rospy.Publisher('my_topic',String) # String 타입의 my_topic 이라는 토픽을 전송하는 퍼블리셔 생성

rate = rospy.Rate(2) # 주기는 1초에 2번 보낸다.

while not rospy.is_shutdown(): # node가 죽지 않았을 때
    pub.publish('call me please') # `call me please`라는 내용이 담긴 토픽을 보낸다.
    rate.sleep() # 설정한 rate 주기만큼 쉰다.

# 1. `teacher.py`에서 `teacher` 노드를 생성한다.
# 2. 퍼블리셔로 명하고, node가 죽지 않을 때까지 String 타입의 `my_topic`을 보낸다.
# 3. `student.py`에서 `student` 노드를 생성한다
# 4. 서브스크라이버로 명명하고, `my_topic`을 받기를 기다리며 callback함수를 통해 받을 때마다 해당 메시지의 데이터를 출력한다.