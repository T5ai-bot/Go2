from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    mocap_dir = '/home/your_user/unitree_go2_ws/src/unitree-go2-ros2/champ/mocap'
    return LaunchDescription([
        Node(package='champ_bringup', executable='bringup', output='screen'),
        Node(package='champ_bringup',
             executable='mm_gait_node.py',
             name='mm_bridge',
             parameters=[{'db_path': mocap_dir}],
             output='screen')
    ])
