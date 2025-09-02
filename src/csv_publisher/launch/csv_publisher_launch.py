from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明CSV文件路径参数
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='/home/r/ros2_1/data.csv',
        description='Path to the CSV file to read'
    )
    
    # 定义节点
    csv_publisher_node = Node(
        package='csv_publisher',
        executable='csv_publisher_node',
        name='csv_publisher',
        parameters=[{
            'csv_file_path': '/home/r/ros2_1/data.csv' #LaunchConfiguration('csv_file')
        }],
        output='screen'
    )
    
    # 返回LaunchDescription
    return LaunchDescription([
        csv_file_arg,
        csv_publisher_node
    ])
