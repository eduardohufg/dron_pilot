from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    node1 = Node(package='dron_pilot',
                       executable='detect_aruco',
                       name="detect_aruco",
                       )
        
    node2 = Node(package='dron_pilot',
                       executable='depth_camera',
                       name="depth_camera",
                       )
    
    node3 = Node(package='dron_pilot',
                       executable='cmd_dron',
                       name="cmd_dron",
                       )
    
    node4 = Node(package='dron_pilot',
                       executable='node_controller',
                       name="node_controller",
                       )
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_camera_bridge',
        arguments=['/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    bridge_camera_raw = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='x500_camera_bridge',
        arguments=[
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image', '/x500/camera/image_raw')
        ],
        output='screen'
    )
    
    l_d = LaunchDescription([node3, node2, node1, node4, bridge_node, bridge_camera_raw])

    return l_d