#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import bluetooth
import struct


def calculate_vel(speed, direction):
    eps = 300
    MAX_VAL = 4000
    MAX_SPEED = 1
    MIN_SPEED = -1
    vel_speed = 0
    if abs(speed - MAX_VAL / 2) > eps:
        vel_speed = (speed - MAX_VAL / 2) / (MAX_VAL / 2)
        if vel_speed > MAX_SPEED:
            vel_speed = MAX_SPEED
        elif vel_speed < MIN_SPEED:
            vel_speed = MIN_SPEED

    if direction > MAX_VAL:
        direction = MAX_VAL

    return vel_speed, 0 if abs(direction - MAX_VAL / 2) < eps else (MAX_VAL / 2 - direction) / (MAX_VAL / 2)


def activate_nodes():
    bd_addr = "98:D3:C1:FD:C3:DF"
    port = 1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((bd_addr, port))

    rospy.init_node('robot_move_node', anonymous=True)
    pub = rospy.Publisher('robot_diffdrive_controller/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('imu', Imu, list_callback, sock)

    move = Twist()
    buffer = bytearray()
    BUF_LEN = 4

    while not rospy.is_shutdown():
        data = sock.recv(1024)
        buffer.extend(data)
        if len(buffer) >= BUF_LEN:
            speed = int.from_bytes(buffer[0:2], "little")
            direction = int.from_bytes(buffer[2:], "little")
            print("received: (speed, rotation)", speed, direction)
            vel_speed, vel_dir = calculate_vel(speed, direction)
            print("set parmaters: (speed, rotation)", vel_speed, vel_dir)
            move.linear.x = vel_speed
            move.angular.z = vel_dir
            pub.publish(move)

            buffer.clear()


def list_callback(data, sock):
    byt = struct.pack('dd', data.linear_acceleration.x, data.angular_velocity.z)
    sock.send(byt)


if __name__ == '__main__':
    try:
        activate_nodes()
    except rospy.ROSInterruptException:
        pass
