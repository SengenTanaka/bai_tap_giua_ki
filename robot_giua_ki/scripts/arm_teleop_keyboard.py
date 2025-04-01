#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import sys
import tty
import termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def arm_controller():
    # Khởi tạo node ROS
    rospy.init_node('arm_teleop_keyboard', anonymous=True)

    # Publisher cho từng khớp
    part_1_pub = rospy.Publisher('/robot_giua_ki/part_1_joint_position_controller/command', Float64, queue_size=10)
    part_2_pub = rospy.Publisher('/robot_giua_ki/part_2_joint_position_controller/command', Float64, queue_size=10)
    part_3_pub = rospy.Publisher('/robot_giua_ki/part_3_joint_position_controller/command', Float64, queue_size=10)

    # Vị trí ban đầu của các khớp
    part_1_pos = 0.0  # part_1_joint (không có giới hạn)
    part_2_pos = 0.0  # part_2_joint (giới hạn từ -1.57 đến 1.57 rad)
    part_3_pos = 0.0  # part_3_joint (giới hạn từ -1.57 đến 1.57 rad)

    # Hướng dẫn điều khiển
    print("Điều khiển tay máy:")
    print("w: Tăng vị trí part_1_joint (quay quanh trục Z)")
    print("s: Giảm vị trí part_1_joint")
    print("a: Tăng vị trí part_2_joint (khớp nối part_1 và part_2)")
    print("d: Giảm vị trí part_2_joint")
    print("z: Tăng vị trí part_3_joint (khớp nối part_2 và part_3)")
    print("x: Giảm vị trí part_3_joint")
    print("q: Thoát")

    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'w':
            part_1_pos += 0.1  # Tăng vị trí part_1 (bước 0.1 rad)
        elif key == 's':
            part_1_pos -= 0.1  # Giảm vị trí part_1
        elif key == 'a':
            part_2_pos += 0.1  # Tăng vị trí part_2
            if part_2_pos > 1.57:  # Giới hạn theo URDF
                part_2_pos = 1.57
        elif key == 'd':
            part_2_pos -= 0.1  # Giảm vị trí part_2
            if part_2_pos < -1.57:
                part_2_pos = -1.57
        elif key == 'z':
            part_3_pos += 0.1  # Tăng vị trí part_3
            if part_3_pos > 1.57:  # Giới hạn theo URDF
                part_3_pos = 1.57
        elif key == 'x':
            part_3_pos -= 0.1  # Giảm vị trí part_3
            if part_3_pos < -1.57:
                part_3_pos = -1.57
        elif key == 'q':
            break

        # Publish vị trí khớp
        part_1_pub.publish(part_1_pos)
        part_2_pub.publish(part_2_pos)
        part_3_pub.publish(part_3_pos)

        # Log thông tin
        rospy.loginfo("Part 1 position: %.2f rad, Part 2 position: %.2f rad, Part 3 position: %.2f rad", 
                      part_1_pos, part_2_pos, part_3_pos)

        rate.sleep()

if __name__ == '__main__':
    try:
        arm_controller()
    except rospy.ROSInterruptException:
        pass