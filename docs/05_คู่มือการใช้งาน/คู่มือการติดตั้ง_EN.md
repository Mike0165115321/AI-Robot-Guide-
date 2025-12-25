# 📖 AI Robot Guide Installation Guide - Nan Province

## Table of Contents
- [1. System Overview](#1-system-overview)
- [2. System Requirements](#2-system-requirements)
- [3. Prerequisite Software Installation](#3-prerequisite-software-installation)
- [4. Project Download](#4-project-download)
- [5. Database Setup](#5-database-setup)
- [6. Python Environment Setup](#6-python-environment-setup)
- [7. API Keys Configuration](#7-api-keys-configuration)
- [8. Vector Database Creation](#8-vector-database-creation)
- [9. Running the System](#9-running-the-system)
- [10. System Testing](#10-system-testing)
- [11. Basic Troubleshooting](#11-basic-troubleshooting)

---

## 1. System Overview

### 1.1. Project Description
- AI Robot Guide is an AI system designed to recommend tourist attractions and important information for Nan province.

### 1.2. System Architecture

#### 1.2.1. Main Components
  - (1) **Frontend** - Web interface for users (HTML5, CSS3, JavaScript)
  - (2) **Backend** - API Server (FastAPI/Python)
  - (3) **MongoDB** - Main database for storing location data
  - (4) **Qdrant** - Vector database for intelligent search

#### 1.2.2. Technologies Used
  - (1) **AI/LLM**: Gemini, Groq (Llama 3.x)
  - (2) **Speech-to-Text (STT)**: Whisper
  - (3) **Text-to-Speech (TTS)**: EdgeTTS, gTTS
  - (4) **RAG Pipeline**: Information retrieval system using Vector DB

---

## 2. System Requirements

### 2.1. Hardware Requirements

#### 2.1.1. Minimum Requirements
  - (1) CPU: Intel Core i5 or equivalent
  - (2) RAM: 8 GB or higher
  - (3) Storage: 20 GB or more
  - (4) Internet connection

#### 2.1.2. Recommended Requirements (For Maximum Performance)
  - (1) CPU: Intel Core i7 or equivalent
  - (2) RAM: 16 GB or higher
  - (3) GPU: NVIDIA GPU (for local Whisper usage)
  - (4) Storage: 50 GB or more

### 2.2. Software Requirements

#### 2.2.1. Supported Operating Systems
  - (1) Windows 10/11
  - (2) Ubuntu 20.04 LTS or higher
  - (3) macOS 11 or higher

#### 2.2.2. Required Software
  - (1) Python 3.12 or higher
  - (2) Docker Desktop
  - (3) Git (Optional - for cloning the project)

---

## 3. Prerequisite Software Installation

### 3.1. Python Installation

#### 3.1.1. For Windows

##### 3.1.1.1. Download Python
  - (1) Open a web browser
  - (2) Go to https://www.python.org/downloads/
  - (3) Click the "Download Python 3.12.x" button (or the latest version)
  - (4) Wait for the download to complete

##### 3.1.1.2. Install Python
  - (1) Double-click the downloaded file
  - (2) **Important**: Check the box "Add python.exe to PATH"
  - (3) Click "Install Now"
  - (4) Wait for the installation to complete
  - (5) Click "Close" when finished

##### 3.1.1.3. Verify Installation
  - (1) Open Command Prompt (Press Windows + R, type cmd, and press Enter)
  - (2) Type the command:
    ```bash
    python --version
    ```
  - (3) Output should be: `Python 3.12.x`

#### 3.1.2. For Linux (Ubuntu/Debian)

##### 3.1.2.1. Update System
  - (1) Open Terminal
  - (2) Run command:
    ```bash
    sudo apt update && sudo apt upgrade -y
    ```

##### 3.1.2.2. Install Python
  - (1) Run command:
    ```bash
    sudo apt install python3.12 python3.12-venv python3-pip -y
    ```

##### 3.1.2.3. Verify Installation
  - (1) Run command:
    ```bash
    python3 --version
    ```
  - (2) Output should be: `Python 3.12.x`

### 3.2. Docker Desktop Installation

#### 3.2.1. For Windows

##### 3.2.1.1. Download Docker Desktop
  - (1) Go to https://www.docker.com/products/docker-desktop/
  - (2) Click "Download for Windows"
  - (3) Wait for the download to complete

##### 3.2.1.2. Install Docker Desktop
  - (1) Double-click the `Docker Desktop Installer.exe` file
  - (2) Check "Use WSL 2 instead of Hyper-V"
  - (3) Click "OK" and wait for installation
  - (4) Restart your computer when prompted

##### 3.2.1.3. Start Docker Desktop
  - (1) Open Docker Desktop from the Start Menu
  - (2) Accept the terms of use
  - (3) Wait until the Docker Engine status is "Running"

##### 3.2.1.4. Verify Installation
  - (1) Open Command Prompt
  - (2) Type command:
    ```bash
    docker --version
    ```
  - (3) Should display the Docker version

#### 3.2.2. For Linux (Ubuntu)

##### 3.2.2.1. Install Docker Engine
  - (1) Run command:
    ```bash
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    ```

##### 3.2.2.2. Add User to Docker Group
  - (1) Run command:
    ```bash
    sudo usermod -aG docker $USER
    ```
  - (2) Log out and log back in

##### 3.2.2.3. Install Docker Compose
  - (1) Run command:
    ```bash
    sudo apt install docker-compose-plugin -y
    ```

##### 3.2.2.4. Verify Installation
  - (1) Run command:
    ```bash
    docker --version
    docker compose version
    ```

### 3.3. Git Installation (Optional)

#### 3.3.1. For Windows
  - (1) Go to https://git-scm.com/downloads
  - (2) Click "Download for Windows"
  - (3) Install using all default settings

#### 3.3.2. For Linux
  - (1) Run command:
    ```bash
    sudo apt install git -y
    ```

---

## 4. Project Download

### 4.1. Method 1: Download ZIP File (Recommended for Beginners)

#### 4.1.1. Download from GitHub
  - (1) Go to https://github.com/Mike0165115321/AI-Robot-Guide-
  - (2) Click the green "Code" button
  - (3) Click "Download ZIP"
  - (4) Wait for the download to complete

#### 4.1.2. Extract Files
  - (1) Open File Explorer to the Downloads folder
  - (2) Right-click the `AI-Robot-Guide--main.zip` file
  - (3) Select "Extract All..."
  - (4) Choose the destination for the project
  - (5) Click "Extract"

### 4.2. Method 2: Clone with Git

#### 4.2.1. Open Terminal/Command Prompt
  - (1) Navigate to the directory where you want to keep the project:
    ```bash
    cd ~/Documents
    ```
    Or on Windows:
    ```bash
    cd C:\Users\YourName\Documents
    ```

#### 4.2.2. Clone Project
  - (1) Run command:
    ```bash
    git clone https://github.com/Mike0165115321/AI-Robot-Guide-.git
    ```

#### 4.2.3. Enter Project Directory
  - (1) Run command:
    ```bash
    cd AI-Robot-Guide-
    ```

---

## 5. Database Setup

### 5.1. Start Docker

#### 5.1.1. Verify Docker is Running

##### 5.1.1.1. For Windows
  - (1) Open Docker Desktop application
  - (2) Wait until status shows "Docker Desktop is running"
  - (3) Check that the Docker icon in System Tray is green

##### 5.1.1.2. For Linux
  - (1) Run command:
    ```bash
    sudo systemctl start docker
    sudo systemctl status docker
    ```
  - (2) Status should be "active (running)"

### 5.2. Run Database with Docker Compose

#### 5.2.1. Open Terminal in Project Folder

##### 5.2.1.1. For Windows
  - (1) Open File Explorer to the project folder
  - (2) Right-click in empty space
  - (3) Select "Open in Terminal" or "Open PowerShell window here"

##### 5.2.1.2. For Linux
  - (1) Right-click in the project folder
  - (2) Select "Open Terminal Here"
  - (3) Or use `cd` command to navigate

#### 5.2.2. Run Docker Compose
  - (1) Run command:
    ```bash
    docker compose up -d mongodb qdrant
    ```
    Or (for older Docker Compose versions):
    ```bash
    docker-compose up -d mongodb qdrant
    ```
  - (2) Wait until images download and containers are created

#### 5.2.3. Check Status

##### 5.2.3.1. Verify Containers are Running
  - (1) Run command:
    ```bash
    docker ps
    ```
  - (2) You should see both containers:
    - `mongodb_db` - running on port 27017
    - `qdrant_db` - running on port 6333, 6334

##### 5.2.3.2. Test Qdrant Connection
  - (1) Open web browser
  - (2) Go to http://localhost:6333/dashboard
  - (3) You should see the Qdrant Dashboard

### 5.3. Stop and Restart Database

#### 5.3.1. Stop Database
  - (1) Run command:
    ```bash
    docker compose down
    ```
  - **Note**: Data will persist in Docker volumes

#### 5.3.2. Restart Database
  - (1) Run command:
    ```bash
    docker compose up -d mongodb qdrant
    ```

---

## 6. Python Environment Setup

### 6.1. Create Virtual Environment

#### 6.1.1. Open Terminal in Project Folder
  - (1) Verify you are in the main project folder (containing `Back-end` and `frontend` folders)

#### 6.1.2. Create Virtual Environment

##### 6.1.2.1. For Windows
  - (1) Run command:
    ```bash
    python -m venv .venv-robot
    ```

##### 6.1.2.2. For Linux/macOS
  - (1) Run command:
    ```bash
    python3 -m venv .venv-robot
    ```

### 6.2. Activate Virtual Environment

#### 6.2.1. For Windows (Command Prompt)
  - (1) Run command:
    ```bash
    .venv-robot\Scripts\activate
    ```

#### 6.2.2. For Windows (PowerShell)
  - (1) Run command:
    ```bash
    .venv-robot\Scripts\Activate.ps1
    ```
  - **Note**: If there is an execution policy error, run:
    ```bash
    Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
    ```

#### 6.2.3. For Linux/macOS
  - (1) Run command:
    ```bash
    source .venv-robot/bin/activate
    ```

#### 6.2.4. Verify Virtual Environment is Active
  - (1) Check the terminal prompt
  - (2) You should see `(.venv-robot)` preceding the prompt
  - (3) Example:
    ```
    (.venv-robot) C:\Users\YourName\AI-Robot-Guide->
    ```
    Or
    ```
    (.venv-robot) user@computer:~/AI-Robot-Guide-$
    ```

### 6.3. Install Dependencies

#### 6.3.1. Update pip
  - (1) Run command:
    ```bash
    pip install --upgrade pip
    ```

#### 6.3.2. Install All Packages
  - (1) Run command:
    ```bash
    pip install -r Back-end/requirements.txt
    ```
  - (2) Wait for installation (may take 5-15 minutes depending on internet speed)

#### 6.3.3. Verify Installation
  - (1) Run command:
    ```bash
    pip list
    ```
  - (2) Should see list of installed packages

---

## 7. API Keys Configuration

### 7.1. Create .env File

#### 7.1.1. Create New .env File

##### 7.1.1.1. Method 1: Copy from Example File
  - (1) Go to `Back-end` folder
  - (2) Copy `.env.example` to `.env`:
    - **Windows**:
      ```bash
      copy Back-end\.env.example Back-end\.env
      ```
    - **Linux/macOS**:
      ```bash
      cp Back-end/.env.example Back-end/.env
      ```

##### 7.1.1.2. Method 2: Create New File
  - (1) Open Text Editor (Notepad, VS Code, etc.)
  - (2) Create a new file named `.env` in the `Back-end` folder

### 7.2. Request API Keys

#### 7.2.1. GEMINI_API_KEYS (Gemini AI)

##### 7.2.1.1. Create API Key
  - (1) Go to https://aistudio.google.com/app/apikey
  - (2) Sign in with Google Account
  - (3) Click "Create API Key"
  - (4) Select Project or create new Project
  - (5) Copy the obtained API Key

##### 7.2.1.2. Note
  - You can create multiple keys and separate them with commas `,`
  - Example: `AIzaSy...key1,AIzaSy...key2`

#### 7.2.2. GROQ_API_KEYS (Groq AI)

##### 7.2.2.1. Create API Key
  - (1) Go to https://console.groq.com/keys
  - (2) Register/Sign in
  - (3) Click "Create API Key"
  - (4) Name the Key
  - (5) Copy the obtained API Key

##### 7.2.2.2. Note
  - You can create multiple keys and separate them with commas `,`
  - Example: `gsk_...key1,gsk_...key2`

#### 7.2.3. YOUTUBE_API_KEY and GOOGLE_API_KEY

##### 7.2.3.1. Enable API
  - (1) Go to https://console.cloud.google.com/
  - (2) Create new Project or select existing Project
  - (3) Go to "APIs & Services" > "Library"
  - (4) Search and enable:
    - "YouTube Data API v3"
    - "Custom Search API"

##### 7.2.3.2. Create API Key
  - (1) Go to https://console.cloud.google.com/apis/credentials
  - (2) Click "+ CREATE CREDENTIALS"
  - (3) Select "API key"
  - (4) Copy the obtained API Key

#### 7.2.4. GOOGLE_CSE_ID (Custom Search Engine ID)

##### 7.2.4.1. Create Custom Search Engine
  - (1) Go to https://programmablesearchengine.google.com/controlpanel/all
  - (2) Click "Add" or "New search engine"
  - (3) Enter required information
  - (4) Click "Create"

##### 7.2.4.2. Get Search Engine ID
  - (1) Go to the created Search Engine
  - (2) Click "Setup"
  - (3) Look at "Basic" section > "Search engine ID"
  - (4) Copy the ID

### 7.3. Fill Information in .env File

#### 7.3.1. Complete .env File Structure
  - (1) Open `Back-end/.env` file with Text Editor
  - (2) Fill in information following this format:

```env
# Database Configuration
MONGO_URI=mongodb://localhost:27017/
MONGO_DATABASE_NAME=nanaiguide

# AI Model Configuration
EMBEDDING_MODEL_NAME=intfloat/multilingual-e5-large
RERANKER_MODEL_NAME=BAAI/bge-reranker-base

# API Keys
GEMINI_API_KEYS=AIzaSy...your_key_1,AIzaSy...your_key_2
GROQ_API_KEYS=gsk_...your_key_1,gsk_...your_key_2
GOOGLE_API_KEY=your_google_api_key
GOOGLE_CSE_ID=your_google_cse_id
YOUTUBE_API_KEY=your_youtube_api_key

# Vector DB
QDRANT_HOST=localhost
QDRANT_PORT=6333

# Server Config
API_HOST=0.0.0.0
API_PORT=9090
```

#### 7.3.2. Precautions
  - (1) No spaces before or after `=` sign
  - (2) Do not enclose values in quotes (unless special characters are present)
  - (3) Must use localhost or 127.0.0.1 for MONGO_URI and QDRANT_HOST

---

## 8. Vector Database Creation

### 8.1. Preparation

#### 8.1.1. Check Prerequisites

##### 8.1.1.1. Check Docker Containers are Running
  - (1) Run command:
    ```bash
    docker ps
    ```
  - (2) `mongodb_db` and `qdrant_db` must be visible

##### 8.1.1.2. Check Virtual Environment is Active
  - (1) Look for `(.venv-robot)` preceding the prompt

##### 8.1.1.3. Check .env File is Correct
  - (1) Verify API Keys are filled in completely

### 8.2. Run Preparation Script

#### 8.2.1. Add Image Links

##### 8.2.1.1. Run Command
  - (1) Run command:
    - **Windows**:
      ```bash
      python Back-end/utils/add_image_links.py
      ```
    - **Linux/macOS**:
      ```bash
      python3 Back-end/utils/add_image_links.py
      ```
  - (2) Wait for script to finish

##### 8.2.1.2. Verify Results
  - (1) Should see success message in Terminal

### 8.3. Build Vector Database

#### 8.3.1. Run Vector Build Script

##### 8.3.1.1. Run Command
  - (1) Run command:
    - **Windows**:
      ```bash
      python Back-end/scripts/build_vectors.py
      ```
    - **Linux/macOS**:
      ```bash
      python3 Back-end/scripts/build_vectors.py
      ```
  - (2) Wait for script to finish (may take 5-30 minutes for the first time)

##### 8.3.1.2. What will Happen
  - (1) Download Embedding Model (First time ~2GB)
  - (2) Read data from JSONL files
  - (3) Create Vector Embeddings
  - (4) Save to MongoDB and Qdrant

##### 8.3.1.3. Verify Results
  - (1) Should see "Vector database built successfully" or similar message
  - (2) Can check in Qdrant Dashboard at http://localhost:6333/dashboard

### 8.4. Clear and Rebuild Database (If Necessary)

#### 8.4.1. Clear Database

> ⚠️ **Warning**: Running this command will delete ALL data in the database!

##### 8.4.1.1. Run Clear Database Command
  - (1) Run command:
    - **Windows**:
      ```bash
      python Back-end/scripts/clear_database.py
      ```
    - **Linux/macOS**:
      ```bash
      python3 Back-end/scripts/clear_database.py
      ```

##### 8.4.1.2. Rebuild Database
  - (1) Go back and repeat steps 8.2 and 8.3

---

## 9. Running the System

### 9.1. Recommended Method (Automatic)

#### 9.1.1. Using start_all.sh
  - (1) **Linux/macOS**:
    ```bash
    chmod +x start_all.sh
    ./start_all.sh
    ```
  - (2) **Windows**: Recommended to run via WSL 2 or Git Bash

### 9.2. Manual Method (Step-by-Step)

#### 9.2.1. Step 1: Run Databases
  - (1) Run command:
    ```bash
    docker compose up -d mongodb qdrant
    ```

#### 9.2.2. Step 2: Run Python Backend
  - (1) Open new Terminal
  - (2) Navigate to folder: `cd Back-end`
  - (3) Activate venv: `source ../.venv-robot/bin/activate`
  - (4) Run: `uvicorn api.main:app --host 127.0.0.1 --port 9090 --reload`

### 9.3. Accessing the Application
  - (1) Open browser to: `http://localhost:9090`
  - (2) You can now use the Chat and Avatar interfaces.

### 9.4. Stopping the System
  - (1) Press `Ctrl + C` in all Terminals
  - (2) Run `docker compose down` to stop databases

---

## 10. System Testing

### 10.1. Backend API Testing

#### 10.1.1. Test Health Check
  - (1) Open web browser
  - (2) Go to http://127.0.0.1:9090/health
  - (3) Should get response: `{"status": "ok"}` or similar

#### 10.1.2. Test API Documentation
  - (1) Go to http://127.0.0.1:9090/docs
  - (2) Try using various endpoints via Swagger UI

### 10.2. Frontend Testing

#### 10.2.1. Test Chat Page
  - (1) Open `frontend/chat.html`
  - (2) Try typing questions about Nan tourist attractions
  - (3) Should receive answers from AI

#### 10.2.2. Test Avatar Page
  - (1) Open `frontend/robot_avatar.html`
  - (2) Try using Wake Word "Nong Nan" (or configured word)
  - (3) Test speaking through microphone

### 10.3. Database Testing

#### 10.3.1. Check MongoDB
  - (1) Run command:
    ```bash
    docker exec -it mongodb_db mongosh
    ```
  - (2) Type command:
    ```
    show dbs
    use nanaiguide
    show collections
    ```
  - (3) Type `exit` to quit

#### 10.3.2. Check Qdrant
  - (1) Open http://localhost:6333/dashboard
  - (2) Verify Collections and data exist

---

## 11. Basic Troubleshooting

### 11.1. Docker Issues

#### 11.1.1. Docker Not Running

##### 11.1.1.1. Possible Causes
  - (1) Docker Desktop is not open
  - (2) WSL 2 is not installed (Windows)
  - (3) Docker service is not started

##### 11.1.1.2. Solution
  - (1) Open Docker Desktop and wait for Running status
  - (2) Restart Docker Desktop
  - (3) For Linux run: `sudo systemctl restart docker`

#### 11.1.2. Port Already in Use

##### 11.1.2.1. Error Message
  - (1) `Error: Port 27017 (or 6333) is already in use`

##### 11.1.2.2. Solution
  - (1) Stop the program using that port
  - (2) Or modify port in `docker-compose.yml`

### 11.2. Python Issues

#### 11.2.1. ModuleNotFoundError

##### 11.2.1.1. Cause
  - (1) Virtual Environment is not activated
  - (2) Packages are not installed

##### 11.2.1.2. Solution
  - (1) Check if `(.venv-robot)` preceeds the prompt
  - (2) Run: `pip install -r Back-end/requirements.txt`

#### 11.2.2. Permission Error

##### 11.2.2.1. Solution (Linux)
  - (1) Use `sudo` before command (not recommended)
  - (2) Or use `--user` flag:
    ```bash
    pip install --user -r Back-end/requirements.txt
    ```

### 11.3. API Keys Issues

#### 11.3.1. Invalid API Key

##### 11.3.1.1. Symptoms
  - (1) Error: "Invalid API key"
  - (2) Response: 401 Unauthorized

##### 11.3.1.2. Solution
  - (1) Verify API Key is copied correctly
  - (2) Check for no spaces before/after Key
  - (3) Generate new Key if necessary

### 11.4. Database Connection Issues

#### 11.4.1. Cannot Connect to MongoDB/Qdrant

##### 11.4.1.1. Possible Causes
  - (1) Docker containers are not running
  - (2) Incorrect MONGO_URI or QDRANT_HOST

##### 11.4.1.2. Solution
  - (1) Run `docker ps` to check containers
  - (2) Check values in `.env` file:
    - `MONGO_URI=mongodb://localhost:27017/`
    - `QDRANT_HOST=localhost`

### 11.5. Getting Help

#### 11.5.1. Additional Resources
  - (1) Read README.md in project
  - (2) Check Issues on GitHub
  - (3) Contact development team

---

## 📞 Contact

If you encounter installation problems or need assistance:
- GitHub Issues: https://github.com/Mike0165115321/AI-Robot-Guide-/issues

---

**Version**: 1.0
**Last Updated**: December 2024
