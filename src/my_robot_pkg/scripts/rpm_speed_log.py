#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import csv

# Informasi robot
diameter_roda = 0.15  # Diameter roda dalam meter
pulse_per_rev = 36  # Pulse per Rev

# Inisialisasi variabel untuk menghitung ticks
right_ticks = 0
left_ticks = 0
prev_right_ticks = 0
prev_left_ticks = 0

# Inisialisasi waktu sebelumnya
prev_time = rospy.Time.now()

# List untuk menyimpan data yang akan disimpan di file CSV
data_to_save = []

# Fungsi untuk menghitung RPM
def calculate_rpm(ticks, prev_ticks, time_elapsed):
    delta_ticks = ticks - prev_ticks
    return (delta_ticks / pulse_per_rev) / (time_elapsed.to_sec() / 60.0)

# Fungsi callback untuk topik right_ticks
def right_ticks_callback(data):
    global right_ticks, prev_right_ticks, prev_time
    right_ticks = data.data
    time_elapsed = rospy.Time.now() - prev_time
    if time_elapsed.to_sec() > 0:
        right_rpm = calculate_rpm(right_ticks, prev_right_ticks, time_elapsed)
        prev_right_ticks = right_ticks
        rospy.loginfo("RPM Right Wheel: %f", right_rpm)
    prev_time = rospy.Time.now()

# Fungsi callback untuk topik left_ticks
def left_ticks_callback(data):
    global left_ticks, prev_left_ticks, prev_time
    left_ticks = data.data
    time_elapsed = rospy.Time.now() - prev_time
    if time_elapsed.to_sec() > 0:
        left_rpm = calculate_rpm(left_ticks, prev_left_ticks, time_elapsed)
        prev_left_ticks = left_ticks
        rospy.loginfo("RPM Left Wheel: %f", left_rpm)
    prev_time = rospy.Time.now()

# Fungsi untuk menyimpan data ke dalam file CSV
def save_to_csv():
    with open('robot_data.csv', mode='w') as csv_file:
        fieldnames = ['Time', 'RPM Left Wheel', 'RPM Right Wheel', 'Robot Speed (m/s)']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        writer.writeheader()
        writer.writerows(data_to_save)

# Fungsi utama
if __name__ == '__main':
    rospy.init_node('wheel_speed_node', anonymous=True)
    rospy.Subscriber("right_ticks", Int16, right_ticks_callback)
    rospy.Subscriber("left_ticks", Int16, left_ticks_callback)

    # Perhitungan kecepatan bergerak robot (average dari kedua roda)
    rate = rospy.Rate(10)  # Frekuensi pembaruan (10 Hz)
    while not rospy.is_shutdown():
        time_elapsed = rospy.Time.now() - prev_time
        if time_elapsed.to_sec() > 0:
            robot_speed = (calculate_rpm(right_ticks, prev_right_ticks, time_elapsed) +
                           calculate_rpm(left_ticks, prev_left_ticks, time_elapsed)) * (diameter_roda / 2)
            rospy.loginfo("Robot Speed (m/s): %f", robot_speed)
            
            # Menyimpan data ke dalam list
            data_to_save.append({
                'Time': rospy.Time.now(),
                'RPM Left Wheel': calculate_rpm(left_ticks, prev_left_ticks, time_elapsed),
                'RPM Right Wheel': calculate_rpm(right_ticks, prev_right_ticks, time_elapsed),
                'Robot Speed (m/s)': robot_speed
            })

        rate.sleep()

        # Menyimpan data ke dalam file CSV jika tombol "q" ditekan
        if rospy.core.is_shutdown() or rospy.is_shutdown():
            if rospy.core.is_shutdown() or rospy.is_shutdown():
            if input("Press 'q' to save data and exit: ") == 'q':
                save_to_csv()
                break
