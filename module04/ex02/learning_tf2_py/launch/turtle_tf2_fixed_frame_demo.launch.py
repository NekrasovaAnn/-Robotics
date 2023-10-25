import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.actions import SetParameter


# Создаем переменную, в которой будут хранитсься созданные узлы
def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        )

# Добавляем carrot1 кадр к черепахам
    return LaunchDescription([
        SetParameter(name='radius', value=4),
        SetParameter(name='direction_of_rotation', value=1),
        demo_nodes,
        Node(
            package='learning_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
    ])