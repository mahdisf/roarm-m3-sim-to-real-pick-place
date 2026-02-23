import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None

def generate_launch_description():
    
    # 1. Detect Hardware
    usb_port = '/dev/ttyUSB0' 
    is_real_robot = os.path.exists(usb_port)
    
    # 2. Paths
    roarm_moveit_pkg = get_package_share_directory('roarm_moveit')
    
    # USE THE CLEANER RVIZ CONFIG (from mtc_demo package)
    # This removes the "Visual Vector" / Interactive Marker
    mtc_demo_pkg = get_package_share_directory('roarm_moveit_mtc_demo')
    rviz_config_file = os.path.join(mtc_demo_pkg, 'rviz', 'mtc.rviz')

    # 3. Load Configs manually (for the Node)
    xacro_file = os.path.join(roarm_moveit_pkg, 'config', 'roarm_m3', 'roarm_m3.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    srdf_file = os.path.join(roarm_moveit_pkg, 'config', 'roarm_m3', 'roarm_m3.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}
    
    kinematics_yaml = load_yaml('roarm_moveit', 'config/roarm_m3/kinematics.yaml')

    # 4. Launch Logic
    launch_actions = []

    # Common Arguments for roarm_moveit.launch.py
    # We pass 'rviz_config' to override the default messy display
    moveit_args = {
        'use_sim_time': 'false' if is_real_robot else 'true',
        'use_fake_hardware': 'false' if is_real_robot else 'true',
        'rviz_config': rviz_config_file 
    }

    if is_real_robot:
        launch_actions.append(LogInfo(msg=f"✅ Real Robot Mode ({usb_port})"))
        # If you need to run roarm_driver separately, add it here.
        # Otherwise, assuming roarm_moveit handles the driver launch or it's running:
        
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(roarm_moveit_pkg, 'launch', 'roarm_moveit.launch.py')
            ),
            launch_arguments=moveit_args.items()
        )
        launch_actions.append(moveit_launch)

    else:
        launch_actions.append(LogInfo(msg="❌ Simulation Mode (Clean RViz)"))
        
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(roarm_moveit_pkg, 'launch', 'roarm_moveit.launch.py')
            ),
            launch_arguments=moveit_args.items()
        )
        launch_actions.append(moveit_launch)

    # 5. The Loop Node
    pick_place_node = Node(
        package='roarm_p',
        executable='simple_pick_place',
        name='factory_loop',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': not is_real_robot}
        ]
    )

    delayed_node = TimerAction(
        period=8.0, 
        actions=[pick_place_node]
    )
    launch_actions.append(delayed_node)

    return LaunchDescription(launch_actions)