import os
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from turtlesim.msg import Pose as TurtlePose
from nrs_lv1_interfaces.msg import Znamenitost
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def main():
    # Putanja do bag datoteke
    bag_path = os.path.expanduser('~/ros2_ws/src/lv1_bringup/bag/bagfile_0')  # promijeni prema lokaciji

    # Inicijalizacija bag reader-a
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')  # bez konverzije
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Priprema za vizualizaciju
    x_positions = []
    y_positions = []
    oznacene_x = []
    oznacene_y = []
    oznacene_nazivi = []

    # Čitanje svih poruka
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/turtle1/pose':
            pose_msg = deserialize_message(data, TurtlePose)
            x_positions.append(pose_msg.x)
            y_positions.append(pose_msg.y)
        elif topic == '/znamenitost':
            zn_msg = deserialize_message(data, Znamenitost)
            oznacene_x.append(zn_msg.pose.x)
            oznacene_y.append(zn_msg.pose.y)
            oznacene_nazivi.append(zn_msg.naziv)

    # Plot putanje robota
    plt.figure(figsize=(8,8))
    plt.plot(x_positions, y_positions, 'b-', label='Putanja robota')
    plt.scatter(oznacene_x, oznacene_y, c='r', marker='x', s=100, label='Znamenitosti')

    # Dodavanje oznaka znamenitosti
    for i, naziv in enumerate(oznacene_nazivi):
        plt.text(oznacene_x[i]+0.1, oznacene_y[i]+0.1, naziv, fontsize=9, color='red')

    plt.title('Putanja robota i znamenitosti')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    main()
