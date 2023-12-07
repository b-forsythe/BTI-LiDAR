import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='laylidar').find('laylidar')
    default_model_path = os.path.join(pkg_share, 'src/description/laylidar_desc.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
        # Define the path to the sensor.launch.xml file
    sensor_launch_file = os.path.join('/home/robot/ros2_ws/src/ouster-ros/ouster-ros/launch', 'driver.launch.py')
    
    spawn_entity = launch_ros.actions.Node(
        executable='spawn_entity.py',
        arguments=['-entity', 'laylidar', '-topic', 'robot_description'],
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': False}]
    )


    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
    )
    
    #slam_toolbox_node = launch_ros.actions.Node(
    #    package='slam_toolbox',
    #    executable='async_slam_toolbox_node',
    #    parameters=[{'use_sim_time': False}, 'src/laylidar/config/mapper_params_online_async.yaml'],
    #    output='screen',
    #)
    
    cartographer_occ = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.05', '-publish_period_sec', '0.1'],
        parameters=[{'use_sim_time':False}],
    )
    
    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        arguments=['-configuration_directory', 'src/laylidar/config/', '-configuration_basename', 'laylidar.lua'],
        remappings=[('/imu','/ouster/imu'),('points2','ouster/points')],
        parameters=[{'use_sim_time': False}],
    )
    
  #fake_encoder = launch_ros.actions.Node(
        #package='fake_encoder_sim',
       # executable='fake_encoder_sim_node',
    #)'''
    
    #robot_localization = launch_ros.actions.Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #   name='ekf_filter_node',
    #    output='screen',
    #    parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml'), {'use_sim_time':False}],
    # )
    
    delayed_cartographer_node = TimerAction(period=15.0, actions=[cartographer_node])
    delayed_rsp_node = TimerAction(period=15.0, actions=[robot_state_publisher_node])
    delayed_jsp_node = TimerAction(period=15.0, actions=[joint_state_publisher_node])
    delayed_cartOCC_node = TimerAction(period=15.0, actions=[cartographer_occ])
    #delayed_robotloc_node = TimerAction(period=15.0, actions=[robot_localization])
   



    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
                                            
               # Include the sensor.launch.xml file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_launch_file)
        ),
        
        delayed_cartographer_node,
        delayed_rsp_node,
        delayed_jsp_node,
        delayed_cartOCC_node,
        #delayed_robotloc_node,
        
        spawn_entity,
        
        #joint_state_publisher_node,
        #robot_state_publisher_node,
        #spawn_entity,
        #fake_encoder,
        #robot_localization,
        #cartographer_occ,
        #delayed_cartographer_node,
        
    ])
