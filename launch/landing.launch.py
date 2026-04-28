import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================================================================
    # Configuration & File Paths
    # =========================================================================
    pkg_dir = get_package_share_directory('px4_vision_hardware_cpp')
    
    # Path to the frames configuration (TF tree names)
    frames_yaml = os.path.join(pkg_dir, 'config', 'frames.yaml')
    
    # Path to the newly created mission parameters
    mission_yaml = os.path.join(pkg_dir, 'config', 'mission_params.yaml')

    # Extract specific frame names from frames.yaml to use in static TF publisher
    with open(frames_yaml, 'r') as f:
        frames_config = yaml.safe_load(f)
    
    frames_params = frames_config['/**']['ros__parameters']['frames']
    drone_frame = frames_params['drone']
    camera_frame = frames_params['camera']

    # =========================================================================
    # Global Launch Variables
    # =========================================================================
    # Set to False when deploying to physical hardware to use real system time instead of sim time
    USE_SIM_TIME = True

    # ----------------- Node Definitions -----------------

    # Static TF: Drone (base_link) -> Camera
    # Rotations align ROS (Z-up) to Camera Optical (Z-forward)
    static_camera_tf = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                name='camera_tf',
                                arguments=[
                                            '0', '0', '-0.10',
                                            '1.57079', '0', '3.14159',
                                            drone_frame,
                                            camera_frame        
                                        ],
                                parameters=[
                                                {'use_sim_time': USE_SIM_TIME}
                                            ],                      
                            )
    
    # --- State Estimation Nodes (Rover) ---
    gps_rover_enu_node = Node(
                                package='px4_vision_hardware_cpp',
                                executable='gps_rover_enu_publisher.py',
                                name='gps_rover_enu_publisher',
                                output='screen',
                                parameters=[
                                                frames_yaml, 
                                                mission_yaml,
                                                {'use_sim_time': USE_SIM_TIME}
                                            ],
                            )
    
    rover_state_ekf_node = Node(
                                    package='px4_vision_hardware_cpp',
                                    executable='rover_state_ekf_node.py',
                                    name='rover_state_ekf_node',
                                    output='screen',
                                    parameters=[
                                                    frames_yaml, 
                                                    mission_yaml,
                                                    {'use_sim_time': USE_SIM_TIME}
                                                ],
                                )
    
    rover_relative_state_node = Node(
                                        package='px4_vision_hardware_cpp',
                                        executable='rover_relative_state_node.py',
                                        name='rover_relative_state_node',
                                        output='screen',
                                        parameters=[
                                                        frames_yaml, 
                                                        mission_yaml,
                                                        {'use_sim_time': USE_SIM_TIME}
                                                    ],
                                    )
    
    # --- Perception Nodes ---
    # Include the external launch file for the AprilTag detection pipeline
    apriltag_pipeline_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'apriltag_pipeline.launch.py')
                )
            )

    # Landing Director: Switches between detecting the bundle vs single tag
    director_node = Node(
                            package='px4_vision_hardware_cpp',
                            executable='landing_director.py',
                            name='landing_director',
                            output='screen',
                            parameters=[
                                            frames_yaml, 
                                            mission_yaml,            # Injects tag_switch_altitude
                                            {'use_sim_time': USE_SIM_TIME}
                                        ],
                        )
                        
    # Computes relative pose of the drone to the AprilTag
    apriltag_relative_pose_node = Node(
        package='px4_vision_hardware_cpp',
        executable='apriltag_relative_pose.py',
        name='apriltag_relative_pose_node',
        output='screen',
        parameters=[
                        frames_yaml, 
                        mission_yaml,            # Injects bundle_ids
                        {'use_sim_time': USE_SIM_TIME},
                    ],
    )

    # --- Mission Management & Control Nodes ---
    # # High-level state machine and safety gatekeeper (Behavior Tree)
    # offboard_experiment_manager_node = Node(
    #                                             package='px4_vision_hardware_cpp',
    #                                             executable='offboard_experiment_manager',
    #                                             name='offboard_experiment_manager_node',
    #                                             output='screen',
    #                                             parameters=[
    #                                                             frames_yaml,
    #                                                             mission_yaml,            # Injects target_altitude, tag_loss_timeout, etc.
    #                                                             {'use_sim_time': USE_SIM_TIME},
    #                                                         ],
    #                                         )
    
    # # Low-level MPC controller for chasing the rover and landing
    # vision_guidance_controller_node = Node(
    #                                             package='px4_vision_hardware_cpp',
    #                                             executable='vision_guidance_controller',
    #                                             name='vision_guidance_controller_node',
    #                                             output='screen',
    #                                             parameters=[
    #                                                             frames_yaml,
    #                                                             mission_yaml,            # Injects mpc_horizon, descent_rate, max_rover_speed, etc.
    #                                                             {'use_sim_time': USE_SIM_TIME},
    #                                                         ],
    #                                         )
    
    mpc_solver_node = Node(
        package='px4_vision_hardware_cpp',
        executable='mpc_solver_node.py',
        name='mpc_solver_node',
        output='screen',
        parameters=[
            mission_yaml,
            {'use_sim_time': USE_SIM_TIME},
        ],
    )

    # C++ Node: The PX4 Interface (Executor + Vision Landing Mode)
    vision_landing_cpp_node = Node(
        package='px4_vision_hardware_cpp',
        executable='vision_landing_cpp_exec', 
        name='vision_landing_control_node',
        output='screen',
        parameters=[
            mission_yaml,
            {'use_sim_time': USE_SIM_TIME},
        ],
    )
                
    # --- Debugging & Visualization Nodes ---
    # tag_rviz_markers_node = Node(
    #                                 package='px4_vision_hardware_cpp',
    #                                 executable='tag_rviz_markers.py',
    #                                 name='tag_rviz_markers_node',
    #                                 output='screen',
    #                                 parameters=[
    #                                                 frames_yaml,
    #                                                 mission_yaml,
    #                                                 {'use_sim_time': USE_SIM_TIME},
    #                                             ],
    #                             )
        
    # Data logger for TF state + CSV output node
    # tf_state_logger_node = Node(
    #                         package='px4_vision_hardware_cpp',
    #                         executable='tf_state_logger',
    #                         name='tf_state_logger',
    #                         output='screen',
    #                         parameters=[
    #                             mission_yaml,
    #                             {'use_sim_time': USE_SIM_TIME},
    #                             {'log_dir': '/home/priyam22/px4_ros2_ws/src/px4_vision_hardware_cpp/px4_logs'}
    #                         ],
    #                     )
    
    # =========================================================================
    # Launch Description Return
    # =========================================================================
    return LaunchDescription([
                                # Start TF & Perception
                                static_camera_tf,
                                apriltag_pipeline_launch,
                                director_node,
                                apriltag_relative_pose_node,

                                # Start State Estimation
                                gps_rover_enu_node,
                                rover_state_ekf_node,
                                rover_relative_state_node,

                                # Start Mission Management & Control
                                # offboard_experiment_manager_node,
                                # vision_guidance_controller_node,
                                vision_landing_cpp_node,
                                mpc_solver_node,
                                # Debugging Nodes
                                # tag_rviz_markers_node,
                                # tf_state_logger_node,
                            ])