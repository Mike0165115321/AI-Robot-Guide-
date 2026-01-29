#!/usr/bin/env python3
# ===================================================================================
# 🤖 robot_system.launch.py - Unified Robot System Launcher
# ===================================================================================
# ไฟล์นี้เป็น "Master Key" สำหรับเปิดระบบหุ่นยนต์ทั้งหมดในครั้งเดียว
# 
# ประกอบด้วย:
#   1. Micro-ROS Agent    - เชื่อมต่อกับ ESP32/Teensy (Motor Control)
#   2. Robot Core Odom    - คำนวณ Odometry และ TF
#   3. YDLidar Driver     - ขับเคลื่อน Lidar Sensor
#   4. QoS Relay          - แก้ปัญหา QoS Mismatch (Best Effort -> Reliable)
#   5. Nav2 Stack         - ระบบนำทางอัตโนมัติ
#   6. ROS2 Bridge        - เชื่อมต่อกับ Web UI
#
# การใช้งาน:
#   ros2 launch /path/to/robot_system.launch.py
#
# หมายเหตุ: สามารถปิดได้ด้วย Ctrl+C และทุก Node จะถูกปิดอย่างถูกต้อง
# ===================================================================================

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # =================================================================================
    # กำหนด Path ของไฟล์ Config
    # =================================================================================
    # Path ของโปรเจค AI-Robot-Guide (Backend)
    backend_hardware_dir = Path(__file__).parent.resolve()
    params_dir = backend_hardware_dir / 'params'
    components_dir = backend_hardware_dir / 'components'
    
    # Path ของ ROS 2 Packages
    ctrobot_dir = get_package_share_directory('ctrobot')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ไฟล์ Config
    lidar_params_file = str(params_dir / 'lidar_params.yaml')
    nav2_params_file = str(params_dir / 'nav2_params.yaml')
    map_file = os.path.join(ctrobot_dir, 'map', 'map3.yaml')
    
    # QoS Relay Script
    qos_relay_script = str(components_dir / 'qos_relay_node.py')
    ros2_bridge_script = str(backend_hardware_dir / 'ros2_bridge.py')
    ros2_bridge_script = str(backend_hardware_dir / 'ros2_bridge.py')
    odometry_script = str(backend_hardware_dir / 'odometry_node.py') # Added new script
    kinematics_script = str(backend_hardware_dir / 'kinematics_bridge.py') # Added kinematics bridge
    
    # =================================================================================
    # Launch Arguments
    # =================================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # =================================================================================
    # Node Definitions
    # =================================================================================
    
    # 1. Micro-ROS Agent - เชื่อมต่อกับบอร์ดฮาร์ดแวร์
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0'],
    )
    
    # 2. Robot Core Odom - คำนวณ Odometry จาก Encoder
    # robot_core_odom_node = Node(
    #     package='ctrobot',
    #     executable='robot_core_odom',
    #     name='navrobot_core_odom',
    #     output='screen',
    # )

    # 2. [NEW] Mecanum Odometry - ใช้ Python Script แทนตัวเดิมที่แก้ค่าไม่ได้
    odometry_process = ExecuteProcess(
        cmd=['/usr/bin/python3', odometry_script],
        name='odometry_node',
        output='screen',
    )
    
    # 3. Static TF: base_link -> laser
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
    )
    
    # 4. YDLidar Driver
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'x3_ydlidar_launch.py')
        ),
        launch_arguments={'params_file': lidar_params_file}.items(),
    )

    # 4.5 ROS Bridge Server (WebSocket) - Execute via command line because it's an XML launch file
    rosbridge_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9090'],
        name='rosbridge_websocket',
        output='screen',
    )
    
    # 5. QoS Relay Node - แก้ปัญหา Best Effort -> Reliable
    # (ใช้ ExecuteProcess แทน Node เพราะเป็น standalone script)
    qos_relay_process = ExecuteProcess(
        cmd=['/usr/bin/python3', qos_relay_script],
        name='qos_relay',
        output='screen',
    )
    
    # 6. ROS2 Bridge - เชื่อมต่อกับ Web UI ผ่าน UDP
    ros2_bridge_process = ExecuteProcess(
        cmd=['/usr/bin/python3', ros2_bridge_script],
        name='ros2_bridge',
        output='screen',
    )
    
    # 6.5 Kinematics Bridge - Converts cmd_vel to wheel_commands
    kinematics_process = ExecuteProcess(
        cmd=['/usr/bin/python3', kinematics_script],
        name='kinematics_bridge',
        output='screen',
    )
    
    # 7. Nav2 Stack - ระบบนำทาง
    # ใช้ GroupAction + SetRemap เพื่อให้ Nav2 ฟัง /scan_reliable แทน /scan (แก้ QoS Mismatch)
    # หมายเหตุ: ไม่ Remap /cmd_vel เพราะ Nav2 ต้องการควบคุมมอเตอร์ในโหมด Navigation
    nav2_group = GroupAction(
        actions=[
            SetRemap(src='/scan', dst='/scan_reliable'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                }.items(),
            ),
        ]
    )
    
    # =================================================================================
    # Launch Sequence
    # =================================================================================
    # ลำดับการ Launch (ปรับปรุงใหม่ให้รอ Micro-ROS เชื่อมต่อก่อน):
    # 1. Micro-ROS Agent - ทันที (ต้องเริ่มก่อนเพื่อรอการเชื่อมต่อ ESP32)
    # 2. Robot Core Odom - delay 3 วินาที (รอให้ Micro-ROS เชื่อมต่อ ESP32 ก่อน)
    # 3. Lidar + TF - ทันที
    # 4. ROS2 Bridge - ทันที
    # 5. QoS Relay - delay 2 วินาที
    # 6. Nav2 - delay 6 วินาที (รอให้ทุกอย่างพร้อม)
    
    # Delayed Nodes
    delayed_odom_node = TimerAction(
        period=3.0,  # รอให้ Micro-ROS Agent เชื่อมต่อกับ ESP32 ก่อน
        # actions=[robot_core_odom_node],
        actions=[odometry_process], # Use new node
    )
    
    delayed_qos_relay = TimerAction(
        period=2.0,
        actions=[qos_relay_process],
    )
    
    delayed_nav2 = TimerAction(
        period=6.0,  # เพิ่มเป็น 6 วินาทีให้ทุกอย่างพร้อม
        actions=[nav2_group],
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Stage 1: Micro-ROS Agent (ต้องเริ่มก่อนสุด)
        micro_ros_agent_node,
        
        # Stage 2: Hardware Foundation (Lidar, TF, Bridge - ทันที เพราะไม่ต้องรอ Micro-ROS)
        static_tf_base_laser,
        lidar_node,
        rosbridge_node, # Added rosbridge
        ros2_bridge_process,
        kinematics_process, # Added kinematics
        
        # Stage 3: Robot Core Odom (รอหลัง Micro-ROS connect)
        delayed_odom_node,
        
        # Stage 4: QoS Bridge (after lidar stabilizes)
        delayed_qos_relay,
        
        # Stage 5: Navigation (Moved to Web UI Map Manager)
        # delayed_nav2,
    ])
