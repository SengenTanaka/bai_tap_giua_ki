#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from pynput import keyboard

# Khởi tạo node ROS
rospy.init_node('teleop_joint_control', anonymous=True)

# Publisher để gửi vận tốc khớp đến topic /joint_states
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

# Tốc độ tối đa của bánh xe (rad/s)
max_wheel_speed = 10.0  # Điều chỉnh nếu cần
wheel_radius = 0.025  # Bán kính bánh xe (m), giả định 0.025m, điều chỉnh nếu cần
wheel_distance = 0.2148557158515  # Khoảng cách từ tâm robot đến bánh xe (m), dựa trên URDF

# Biến để theo dõi trạng thái phím
linear_x = 0.0  # Tốc độ tiến/lùi (m/s)
linear_y = 0.0  # Tốc độ sang trái/phải (m/s)
angular_z = 0.0  # Tốc độ quay (rad/s)

# Hàm xử lý khi phím được nhấn
def on_press(key):
    global linear_x, linear_y, angular_z
    try:
        if key.char == 'w':  # Tiến lên
            linear_x = 0.2
        elif key.char == 's':  # Lùi lại
            linear_x = -0.2
        elif key.char == 'a':  # Sang trái
            linear_y = 0.2
        elif key.char == 'd':  # Sang phải
            linear_y = -0.2
        elif key.char == 'q':  # Quay trái
            angular_z = 0.5
        elif key.char == 'e':  # Quay phải
            angular_z = -0.5
    except AttributeError:
        pass

# Hàm xử lý khi phím được thả
def on_release(key):
    global linear_x, linear_y, angular_z
    try:
        if key.char in ['w', 's']:
            linear_x = 0.0
        elif key.char in ['a', 'd']:
            linear_y = 0.0
        elif key.char in ['q', 'e']:
            angular_z = 0.0
    except AttributeError:
        pass
    if key == keyboard.Key.esc:  # Thoát khi nhấn ESC
        return False

# Hàm tính vận tốc khớp cho bánh xe omnidrive
def compute_wheel_speeds(vx, vy, wz):
    # Góc của các bánh xe so với tâm robot (dựa trên URDF)
    theta1 = math.atan2(0.222141016151378, 0.214855715851501)  # omni_1
    theta2 = math.atan2(-0.0721410161513798, 0.2148557158515)  # omni_2
    theta3 = math.atan2(0.075000000000003, -0.0399999999999992)  # omni_3

    # Tính vận tốc tiếp tuyến của robot tại vị trí từng bánh xe
    # Vận tốc bánh xe = vận tốc tuyến tính + vận tốc quay
    w1 = (vx * math.cos(theta1) + vy * math.sin(theta1) + wz * wheel_distance) / wheel_radius
    w2 = (vx * math.cos(theta2) + vy * math.sin(theta2) + wz * wheel_distance) / wheel_radius
    w3 = (vx * math.cos(theta3) + vy * math.sin(theta3) + wz * wheel_distance) / wheel_radius

    # Điều chỉnh hướng quay dựa trên trục của bánh xe (dựa trên axis trong URDF)
    w1 = w1 * (0.500000000000001 * math.cos(theta1) - 0.866025403784438 * math.sin(theta1))
    w2 = w2 * (0.499999999999995 * math.cos(theta2) + 0.866025403784442 * math.sin(theta2))
    w3 = w3 * (-1.0 * math.cos(theta3))

    return w1, w2, w3

# Hàm gửi vận tốc khớp
def publish_joint_states():
    rate = rospy.Rate(10)  # 10 Hz
    joint_state = JointState()
    joint_state.name = ['omni_1_joint', 'omni_2_joint', 'omni_3_joint', 'part_1_joint', 'part_2_joint', 'part_3_joint']
    joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Vị trí khớp (không cần quan tâm)

    while not rospy.is_shutdown():
        # Tính vận tốc khớp cho các bánh xe
        w1, w2, w3 = compute_wheel_speeds(linear_x, linear_y, angular_z)

        # Gán vận tốc khớp
        joint_state.velocity = [w1, w2, w3, 0.0, 0.0, 0.0]  # Vận tốc khớp (chỉ bánh xe di chuyển)
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    print("Điều khiển robot bằng bàn phím:")
    print(" - W: Tiến lên")
    print(" - S: Lùi lại")
    print(" - A: Sang trái")
    print(" - D: Sang phải")
    print(" - Q: Quay trái")
    print(" - E: Quay phải")
    print(" - ESC: Thoát")

    # Bắt đầu lắng nghe phím bấm
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Gửi vận tốc khớp
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
