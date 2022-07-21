#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import bluetooth


def calculate_vel(speed, direction):
    eps = 300
    vel_speed = 0
    if abs(speed - 2000) < eps:
        speed = 0
    else:
        vel_speed = (speed - 2000) / 4000
    return 1 if vel_speed > 1 else vel_speed, 0 if abs(direction - 2048) < 300 else (2048 - direction) / 2048


def talker():
    pub = rospy.Publisher('robot_diffdrive_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('robot_move_node', anonymous=True)
    # rate = rospy.Rate(100)

    bd_addr = "98:D3:C1:FD:C3:DF"
    port = 1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((bd_addr, port))

    move = Twist()
    buffer = bytearray()

    while not rospy.is_shutdown():
        data = sock.recv(1024)
        buffer.extend(data)
        if len(buffer) >= 6:
            speed = int.from_bytes(buffer[0:4], "little")
            direction = int.from_bytes(buffer[4:], "little")
            print("recv: ", speed, direction)
            vel_speed, vel_dir = calculate_vel(speed, direction)
            print(vel_speed, vel_dir)
            move.linear.x = vel_speed
            move.angular.z = vel_dir
            pub.publish(move)

            buffer.clear()

        # rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
