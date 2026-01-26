#!/usr/bin/env python3
"""
🔍 Movement Debug Tool
=======================
ตรวจสอบทุกจุดในระบบควบคุมหุ่นยนต์:
1. Frontend API Endpoint
2. UDP Communication (hardware_service -> ros2_bridge)
3. ROS 2 cmd_vel Topic
4. ROS 2 wheel*_command Topics
5. ESP32/Micro-ROS Connection

รัน: python3 debug_movement.py
"""

import subprocess
import socket
import json
import time
import sys

# Colors
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
RESET = '\033[0m'

def print_header(text):
    print(f"\n{CYAN}{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}{RESET}\n")

def print_ok(text):
    print(f"  {GREEN}✅ {text}{RESET}")

def print_fail(text):
    print(f"  {RED}❌ {text}{RESET}")

def print_warn(text):
    print(f"  {YELLOW}⚠️  {text}{RESET}")

def print_info(text):
    print(f"  {CYAN}ℹ️  {text}{RESET}")


# ==================================================
# 1. Check if Backend API is running
# ==================================================
def check_backend_api():
    print_header("1. ตรวจสอบ Backend API")
    try:
        import requests
        resp = requests.get("http://127.0.0.1:8014/api/hardware/status", timeout=3)
        if resp.status_code == 200:
            print_ok(f"Backend API ทำงานปกติ (Status: {resp.status_code})")
            return True
        else:
            print_fail(f"Backend API Error: {resp.status_code}")
            return False
    except Exception as e:
        print_fail(f"ไม่สามารถเชื่อมต่อ Backend API: {e}")
        print_info("รัน: ./start_web.sh")
        return False


# ==================================================
# 2. Check if ROS 2 is available
# ==================================================
def check_ros2():
    print_header("2. ตรวจสอบ ROS 2 Environment")
    result = subprocess.run(["which", "ros2"], capture_output=True, text=True)
    if result.returncode == 0:
        print_ok(f"พบ ROS 2: {result.stdout.strip()}")
        return True
    else:
        print_fail("ไม่พบ ROS 2 ในระบบ")
        print_info("รัน: source /opt/ros/humble/setup.bash")
        return False


# ==================================================
# 3. Check ROS 2 Nodes
# ==================================================
def check_ros2_nodes():
    print_header("3. ตรวจสอบ ROS 2 Nodes")
    
    # Source ROS 2 first
    source_cmd = "source /opt/ros/humble/setup.bash && source /home/robot22/microros_ws/install/setup.bash && ros2 node list"
    result = subprocess.run(["bash", "-c", source_cmd], capture_output=True, text=True, timeout=10)
    
    if result.returncode == 0:
        nodes = result.stdout.strip().split('\n')
        print_ok(f"พบ {len(nodes)} nodes:")
        for node in nodes:
            if node.strip():
                print(f"      - {node.strip()}")
        
        # Check for specific nodes
        if "/ros2_bridge" in result.stdout or "/kinematics_bridge" in result.stdout:
            print_ok("พบ Bridge Nodes")
        else:
            print_warn("ไม่พบ ros2_bridge หรือ kinematics_bridge node")
            print_info("ต้องรัน ros2_bridge.sh ก่อน")
        
        return True
    else:
        print_fail(f"ไม่สามารถ list nodes: {result.stderr}")
        return False


# ==================================================
# 4. Check cmd_vel topic
# ==================================================
def check_cmd_vel_topic():
    print_header("4. ตรวจสอบ cmd_vel Topic")
    
    source_cmd = "source /opt/ros/humble/setup.bash && source /home/robot22/microros_ws/install/setup.bash && ros2 topic list"
    result = subprocess.run(["bash", "-c", source_cmd], capture_output=True, text=True, timeout=10)
    
    if "/cmd_vel" in result.stdout:
        print_ok("พบ /cmd_vel topic")
        
        # Echo one message
        echo_cmd = "source /opt/ros/humble/setup.bash && timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null || echo 'No message'"
        result = subprocess.run(["bash", "-c", echo_cmd], capture_output=True, text=True, timeout=5)
        print_info(f"Last message: {result.stdout.strip()[:100] if result.stdout else 'No data'}")
        return True
    else:
        print_warn("ไม่พบ /cmd_vel topic")
        return False


# ==================================================
# 5. Check wheel command topics
# ==================================================
def check_wheel_topics():
    print_header("5. ตรวจสอบ wheel*_command Topics")
    
    source_cmd = "source /opt/ros/humble/setup.bash && source /home/robot22/microros_ws/install/setup.bash && ros2 topic list | grep wheel"
    result = subprocess.run(["bash", "-c", source_cmd], capture_output=True, text=True, timeout=10)
    
    wheel_topics = result.stdout.strip().split('\n')
    if wheel_topics and wheel_topics[0]:
        print_ok(f"พบ {len(wheel_topics)} wheel topics:")
        for topic in wheel_topics:
            if topic.strip():
                print(f"      - {topic.strip()}")
        return True
    else:
        print_warn("ไม่พบ wheel*_command topics")
        print_info("kinematics_bridge.py ต้องรันอยู่")
        return False


# ==================================================
# 6. Check Micro-ROS Agent
# ==================================================
def check_microros_agent():
    print_header("6. ตรวจสอบ Micro-ROS Agent")
    
    result = subprocess.run(["pgrep", "-f", "micro_ros_agent"], capture_output=True, text=True)
    if result.returncode == 0:
        pids = result.stdout.strip().split('\n')
        print_ok(f"พบ Micro-ROS Agent (PID: {', '.join(pids)})")
        return True
    else:
        print_warn("ไม่พบ Micro-ROS Agent")
        print_info("ESP32 จะเชื่อมต่อไม่ได้")
        return False


# ==================================================
# 7. Check ESP32 Node
# ==================================================
def check_esp32_node():
    print_header("7. ตรวจสอบ ESP32 Node")
    
    source_cmd = "source /opt/ros/humble/setup.bash && source /home/robot22/microros_ws/install/setup.bash && ros2 node list | grep base_control"
    result = subprocess.run(["bash", "-c", source_cmd], capture_output=True, text=True, timeout=10)
    
    if "base_control" in result.stdout:
        print_ok("พบ base_control_node จาก ESP32!")
        return True
    else:
        print_fail("ไม่พบ base_control_node")
        print_info("ตรวจสอบ:")
        print_info("  1. ESP32 ได้รับไฟเลี้ยง?")
        print_info("  2. LED กะพริบ (รอ Agent) หรือติดค้าง (Connected)?")
        print_info("  3. Micro-ROS Agent รันอยู่?")
        return False


# ==================================================
# 8. Send Test Command via UDP
# ==================================================
def send_test_udp():
    print_header("8. ส่งคำสั่งทดสอบผ่าน UDP")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    test_cmd = {"vx": 1.0, "vy": 0.0, "wz": 0.0}
    payload = json.dumps(test_cmd).encode('utf-8')
    
    try:
        sock.sendto(payload, ("127.0.0.1", 9999))
        print_ok(f"ส่งคำสั่ง: {test_cmd}")
        print_info("ถ้า ROS 2 Bridge ทำงาน จะเห็น log ที่ terminal")
        
        time.sleep(0.5)
        
        # Stop command
        stop_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        sock.sendto(json.dumps(stop_cmd).encode('utf-8'), ("127.0.0.1", 9999))
        print_ok("ส่งคำสั่งหยุด")
        return True
    except Exception as e:
        print_fail(f"ส่ง UDP ไม่ได้: {e}")
        return False
    finally:
        sock.close()


# ==================================================
# 9. Summary
# ==================================================
def main():
    print(f"\n{CYAN}🔍 Movement Debug Tool - ตรวจสอบระบบควบคุมหุ่นยนต์{RESET}")
    print(f"{CYAN}{'='*60}{RESET}")
    
    results = {}
    
    results['backend'] = check_backend_api()
    results['ros2'] = check_ros2()
    
    if results['ros2']:
        results['nodes'] = check_ros2_nodes()
        results['cmd_vel'] = check_cmd_vel_topic()
        results['wheel'] = check_wheel_topics()
        results['agent'] = check_microros_agent()
        results['esp32'] = check_esp32_node()
    
    results['udp'] = send_test_udp()
    
    # Summary
    print_header("📊 สรุปผล")
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    print(f"  ผ่าน: {passed}/{total} รายการ\n")
    
    # Recommendation
    if not results.get('backend'):
        print_fail("แนะนำ: รัน ./start_web.sh ก่อน")
    elif not results.get('agent'):
        print_fail("แนะนำ: กด 'Start Robot' ที่หน้า Settings ก่อน")
    elif not results.get('esp32'):
        print_fail("แนะนำ: ตรวจสอบ ESP32 - ได้รับไฟ? LED กะพริบ?")
    elif not results.get('wheel'):
        print_fail("แนะนำ: kinematics_bridge อาจยังไม่รัน")
    else:
        print_ok("ระบบดูปกติ ลองเทสอีกรอบ")


if __name__ == "__main__":
    main()
