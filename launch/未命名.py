from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # -------------------------- 1. 声明参数（CSV文件路径，给csv_publisher用） --------------------------
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',  # 参数名
        default_value='/home/r/ros2_1/data.csv',
        description='Path to the CSV file to read'
    )

    # -------------------------- 2. 配置第一个包（csv_publisher）的节点 --------------------------
    csv_publisher_node = Node(
        package='csv_publisher',  # 包名（必须与实际包名一致）
        executable='csv_publisher_node',  # 节点可执行文件名（CMakeLists中install的名字）
        name='csv_publisher',  # 节点运行时的名称（可自定义，避免冲突）
        parameters=[{
            'csv_file_path': LaunchConfiguration('csv_file')  # 传递CSV路径参数
        }],
        output='screen'  # 节点日志输出到终端
    )

    # -------------------------- 3. 配置第二个包（data_subscriber）的节点 --------------------------
    data_processor_node = Node(
        package='data_processor',  # 第二个包的包名
        executable='data_processor_node',  # 第二个节点的可执行文件名
        name='data_processor',  # 节点运行时的名称
        output='screen'  # 日志输出到终端
    )

    # -------------------------- 4. 组合所有组件，返回启动描述 --------------------------
    return LaunchDescription([
        csv_file_arg,          # 先声明参数
        csv_publisher_node,    # 再启动发布节点
        data_processor_node   # 最后启动订阅节点（顺序可调整，ROS会自动等待话题）
    ])
