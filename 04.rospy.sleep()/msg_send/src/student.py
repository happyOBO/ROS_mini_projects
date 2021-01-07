#!/usr/bin/env python
# 파이썬 노드를 선언할 때 위와 같이 파이썬 위치를 알려준다. 이는 파이썬 스크립트로써 현재 스크립트가 실행되어야함을 의미한다.

import rospy # 노드를 사용하기 위해서 import
from std_msgs.msg import String #std_msgs/String 타입을 이용하기 위해서 import

def callback(msg):
    print( msg.data )
    # 받은 토픽의 데이터 출력

rospy.init_node('student') # student라는 노드 생성

sub = rospy.Subscriber('my_topic',String,callback) # 'my_topic' 이라는 토픽을 받고, 그때마다 callback함수를 실행시킨다.

rospy.spin() # 토픽을 받을 때까지 기다린다.