#!/usr/bin/env python3
"""
🔄 ESP32/Teensy Auto-Reset Script
==================================
ใช้ pyserial toggle DTR line เพื่อบังคับให้ ESP32/Teensy reset
ก่อนที่ Micro-ROS Agent จะเริ่มทำงาน

วิธีการทำงาน:
1. เปิด Serial Port
2. Toggle DTR line (OFF -> ON) เพื่อ trigger reset circuit
3. ปิด Port และรอให้บอร์ด boot

การใช้งาน:
    python3 esp32_reset.py /dev/ttyACM0
"""

import sys
import time
import serial

def reset_esp32(port: str = "/dev/ttyACM0", baudrate: int = 115200):
    """Reset ESP32/Teensy by toggling DTR line."""
    print(f"🔄 Resetting device on {port}...")
    
    try:
        # Open serial port
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.dtr = False  # Start with DTR off
        ser.rts = False
        
        # Open port (this will set DTR based on default)
        ser.open()
        
        # Toggle DTR to trigger reset
        ser.dtr = False
        ser.rts = True  # Some boards use RTS for reset
        time.sleep(0.1)
        
        ser.dtr = True  # Set DTR high (triggers reset on many ESP32 boards)
        ser.rts = False
        time.sleep(0.1)
        
        ser.dtr = False  # Release DTR
        time.sleep(0.1)
        
        # Close port
        ser.close()
        
        print(f"✅ Reset complete! Waiting for device to boot...")
        time.sleep(2)  # Wait for ESP32 to boot
        
        return True
        
    except serial.SerialException as e:
        print(f"⚠️ Serial error: {e}")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    success = reset_esp32(port)
    sys.exit(0 if success else 1)
