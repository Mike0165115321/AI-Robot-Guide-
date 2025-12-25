### 3.4. การติดตั้ง Go (Golang)

#### 3.4.1. สำหรับ Windows
  - (1) ดาวน์โหลดตัวติดตั้งจาก https://go.dev/dl/
  - (2) ติดตั้งและทำตามขั้นตอน
  - (3) เปิด Command Prompt แล้วพิมพ์ `go version` เพื่อตรวจสอบ

#### 3.4.2. สำหรับ Linux
  - (1) ดาวน์โหลดและติดตั้ง:
    ```bash
    wget https://go.dev/dl/go1.22.0.linux-amd64.tar.gz
    sudo rm -rf /usr/local/go && sudo tar -C /usr/local -xzf go1.22.0.linux-amd64.tar.gz
    ```
  - (2) เพิ่ม Path ใน `~/.bashrc`:
    ```bash
    export PATH=$PATH:/usr/local/go/bin
    ```
  - (3) รัน `source ~/.bashrc` และตรวจสอบด้วย `go version`

### 3.5. การติดตั้ง C++ Build Tools

#### 3.5.1. สำหรับ Windows
  - (1) ติดตั้ง Visual Studio Community
  - (2) เลือก "Desktop development with C++"
  - (3) ตรวจสอบว่ามี CMake ถูกเลือกอยู่

#### 3.5.2. สำหรับ Linux
  - (1) รันคำสั่ง:
    ```bash
    sudo apt install build-essential cmake -y
    ```
