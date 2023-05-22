import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # """Launch file which brings up visual slam node configured for RealSense."""
    # static_tf_baselink_camera_link = Node(
    #     name='tf_static',
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['0.1', '0', '-0.05', '0', '0', '0', 'base_link', 'camera_link']
    # )

    happymoon_control = Node(
        name='happymoon_control',
        package='happymoon_quad_control',
        executable='happymoon_quad_control_node',
        output='screen',
        shell=True,
        parameters=[{
                    'kpxy': 3.0,
                    'kdxy': 3.0,

                    'kpz': 6.0,
                    'kdz': 5.0,
                    'krp': 8.0,
                    'kyaw': 5.0,

                    'refVelXYKp': 0.5,
                    'refVelZKp': 0.5,
                    'refVelHeadingKp': 0.5,
                    'refVelRateheadingKp': 0.5,

                    'pxy_error_max': 0.5,
                    'vxy_error_max': 0.5,
                    'pz_error_max': 0.5,
                    'vz_error_max': 0.5,
                    'yaw_error_max': 0.7,

                    'k_drag_x': 0.5,
                    'k_drag_y': 0.5,
                    'k_drag_z': 0.5,
                    'k_thrust_horz': 1.15,

                    'ref_pos_x': 0.0,
                    'ref_pos_y': 0.0,
                    'ref_pos_z': 1.0,
                    'ref_head': 0.0,
                    'ref_head_rate': 0.0
                    }]
    )


    return launch.LaunchDescription([happymoon_control])#,static_tf_baselink_camera_link