import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fishbot_bringup_dir = get_package_share_directory(
        'fishbot_bringup')
    lslidar_driver_dir = get_package_share_directory(
        'lslidar_driver')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    serial_bridge = launch_ros.actions.Node(
        package='fishbot_bringup',
        executable='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial_ttl',
            'baudrate': 115200,
        }]
    )

    lslidar_driver = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [lslidar_driver_dir, '/launch', '/lsn10_launch.py']),
    )
    
    return launch.LaunchDescription([
        urdf2tf,
        serial_bridge,
        lslidar_driver,
    ])
