import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

config_dir = os.path.join(get_package_share_directory('px4_vision_hardware_cpp'), 'config')
apriltag_yaml = os.path.join(config_dir, 'apriltag.yaml')

def generate_launch_description():

    container = ComposableNodeContainer(
        name='apriltag_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',

        composable_node_descriptions=[
            # ------------------ Image Rectification Node ------------------
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='image_rectify_node',
                extra_arguments=[{'use_intra_process_comms': True}],

                remappings=[
                    ('image', 'x500/camera/image_raw'),
                    ('camera_info', 'x500/camera/camera_info'),
                    ('image_rect', 'x500/camera/image_rect'),
                ],
            ),

            # ------------------ AprilTag Detection Node ------------------
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag_node',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ('image_rect', 'x500/camera/image_rect'),
                    ('camera_info', 'x500/camera/camera_info'),
                ],

                parameters=[
                                {'use_sim_time': True},

                                # ---- Tag config ----
                                apriltag_yaml,

                                # ---- Pose ----
                                {'pose_estimation_method': 'pnp'},

                                # ---- Detector tuning ----
                                {'detector.threads': 2},
                                {'detector.decimate': 1.0},
                                {'detector.blur': 0.0},
                                {'detector.refine': True},
                                {'detector.sharpening': 0.25},
                                {'max_hamming': 1},

                            ],
            ),
        ]
    )

    return LaunchDescription([
        container,
    ])