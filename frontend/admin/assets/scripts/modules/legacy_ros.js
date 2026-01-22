const LegacyROS = {
    apiBase: '/api/ros',

    async launchScript(scriptName) {
        try {
            const response = await fetch(`${this.apiBase}/launch/${scriptName}`, {
                method: 'POST'
            });
            if (!response.ok) throw new Error(await response.text());
            return await response.json();
        } catch (error) {
            console.error(`Failed to launch ${scriptName}:`, error);
            throw error;
        }
    },

    async stopScript(scriptName) {
        try {
            const response = await fetch(`${this.apiBase}/stop/${scriptName}`, {
                method: 'POST'
            });
            if (!response.ok) throw new Error(await response.text());
            return await response.json();
        } catch (error) {
            console.error(`Failed to stop ${scriptName}:`, error);
            throw error;
        }
    },

    async getStatus() {
        try {
            const response = await fetch(`${this.apiBase}/status`);
            if (!response.ok) throw new Error(await response.text());
            return await response.json();
        } catch (error) {
            console.error("Failed to get status:", error);
            throw error;
        }
    }
};

// UI Handler
document.addEventListener('DOMContentLoaded', () => {
    const scripts = [
        { id: 'btn-start_all', script: 'start_all', name: 'เริ่มระบบทั้งหมด', combo: ['bringup', 'nav2'] },
        { id: 'btn-setstart', script: 'setstart', name: 'จุดเริ่มต้น' },
        { id: 'btn-twisz', script: 'twisz', name: 'หมุนตาม' },
        { id: 'btn-m_viz2', script: 'm_viz2', name: 'เปิดแผนที่' },
        { id: 'btn-follow_lidar', script: 'follow_lidar', name: 'เดินตามด้วย Lidar' },
        { id: 'btn-key_control', script: 'key_control', name: 'ควบคุมด้วย Keyboard' },
        { id: 'btn-chaba', script: 'chaba', name: 'เปิดกล้อง' }
    ];

    const statusDiv = document.getElementById('status-display');

    async function updateStatus() {
        try {
            const data = await LegacyROS.getStatus();
            const running = data.running_scripts || {};

            scripts.forEach(item => {
                const btn = document.getElementById(item.id);
                if (!btn) return;

                // SPECIAL CASE: skip status check for key_control (client-side only)
                if (item.script === 'key_control') return;

                const isRunning = running[item.script] === 'running';

                if (isRunning) {
                    btn.classList.add('running');
                    btn.classList.remove('stopped');
                    btn.innerText = `หยุด ${item.name}`;
                } else {
                    btn.classList.add('stopped');
                    btn.classList.remove('running');
                    btn.innerText = item.name;
                }
            });
        } catch (e) {
            console.error("Status update failed", e);
        }
    }

    // ==========================================
    // ⌨️ Keyboard Control Logic
    // ==========================================
    const KeyboardController = {
        active: false,
        interval: null,
        keys: { w: false, a: false, s: false, d: false, ArrowUp: false, ArrowDown: false, ArrowLeft: false, ArrowRight: false },
        btnId: 'btn-key_control',
        debugPanelId: 'keyboard-debug-panel',

        toggle() {
            this.active = !this.active;
            const btn = document.getElementById(this.btnId);
            const debugPanel = document.getElementById(this.debugPanelId);

            if (this.active) {
                this.start();
                if (btn) {
                    btn.classList.add('running');
                    btn.classList.remove('stopped');
                    btn.innerText = 'หยุด Keyboard Control';
                }
                if (debugPanel) debugPanel.style.display = 'block';
                alert('Keyboard Control Activated: Use W/A/S/D to move.');
            } else {
                this.stop();
                if (btn) {
                    btn.classList.remove('running');
                    btn.classList.add('stopped');
                    btn.innerText = 'ควบคุมด้วย Keyboard';
                }
                if (debugPanel) debugPanel.style.display = 'none';
            }
        },

        start() {
            window.addEventListener('keydown', this.handleKeyDown);
            window.addEventListener('keyup', this.handleKeyUp);
            this.interval = setInterval(() => this.loop(), 100); // Send command every 100ms
        },

        stop() {
            window.removeEventListener('keydown', this.handleKeyDown);
            window.removeEventListener('keyup', this.handleKeyUp);
            if (this.interval) clearInterval(this.interval);
            LegacyROS.moveRobot(0, 0); // Stop robot
            this.resetDebugUI();
        },

        handleKeyDown: (e) => {
            if (KeyboardController.keys.hasOwnProperty(e.key)) {
                KeyboardController.keys[e.key] = true;
            }
        },

        handleKeyUp: (e) => {
            if (KeyboardController.keys.hasOwnProperty(e.key)) {
                KeyboardController.keys[e.key] = false;
            }
        },

        loop() {
            let vx = 0.0;
            let wz = 0.0;

            if (this.keys.w || this.keys.ArrowUp) vx += 1.0;
            if (this.keys.s || this.keys.ArrowDown) vx -= 1.0;
            if (this.keys.a || this.keys.ArrowLeft) wz += 1.0;
            if (this.keys.d || this.keys.ArrowRight) wz -= 1.0;

            // Send command
            LegacyROS.moveRobot(vx, wz);
            this.updateDebugUI(vx, wz);
        },

        updateDebugUI(vx, wz) {
            // Update Key Classes
            const setKey = (id, isActive) => {
                const el = document.getElementById(id);
                if (el) isActive ? el.classList.add('active') : el.classList.remove('active');
            };

            setKey('key-w', this.keys.w || this.keys.ArrowUp);
            setKey('key-s', this.keys.s || this.keys.ArrowDown);
            setKey('key-a', this.keys.a || this.keys.ArrowLeft);
            setKey('key-d', this.keys.d || this.keys.ArrowRight);

            // Update Values
            const vxEl = document.getElementById('debug-vx');
            const wzEl = document.getElementById('debug-wz');
            if (vxEl) vxEl.innerText = vx.toFixed(1);
            if (wzEl) wzEl.innerText = wz.toFixed(1);
        },

        resetDebugUI() {
            const keys = ['key-w', 'key-a', 'key-s', 'key-d'];
            keys.forEach(k => {
                const el = document.getElementById(k);
                if (el) el.classList.remove('active');
            });
            const vxEl = document.getElementById('debug-vx');
            const wzEl = document.getElementById('debug-wz');
            if (vxEl) vxEl.innerText = "0.0";
            if (wzEl) wzEl.innerText = "0.0";
        }
    };

    // Bind 'this' for event handlers
    KeyboardController.handleKeyDown = KeyboardController.handleKeyDown.bind(KeyboardController);
    KeyboardController.handleKeyUp = KeyboardController.handleKeyUp.bind(KeyboardController);


    scripts.forEach(item => {
        const btn = document.getElementById(item.id);
        if (btn) {
            btn.addEventListener('click', async () => {
                // SPECIAL CASE: Keyboard Control
                if (item.script === 'key_control') {
                    KeyboardController.toggle();
                    return;
                }

                // SPECIAL CASE: Combo script (e.g., start_all = bringup + nav2)
                if (item.combo && Array.isArray(item.combo)) {
                    const isRunning = btn.classList.contains('running');
                    btn.disabled = true;
                    try {
                        if (isRunning) {
                            // Stop all scripts in combo (reverse order)
                            for (let i = item.combo.length - 1; i >= 0; i--) {
                                await LegacyROS.stopScript(item.combo[i]);
                            }
                            alert(`หยุด ${item.name} แล้ว`);
                        } else {
                            // Launch all scripts in combo (in order)
                            for (const scriptName of item.combo) {
                                await LegacyROS.launchScript(scriptName);
                            }
                            alert(`เริ่ม ${item.name} แล้ว (Bringup + Nav2)`);
                        }
                        await updateStatus();
                    } catch (err) {
                        alert(`Error: ${err.message}`);
                    } finally {
                        btn.disabled = false;
                    }
                    return;
                }

                // Normal single script handling
                const isRunning = btn.classList.contains('running');
                btn.disabled = true;
                try {
                    if (isRunning) {
                        await LegacyROS.stopScript(item.script);
                        alert(`หยุด ${item.name} แล้ว`);
                    } else {
                        await LegacyROS.launchScript(item.script);
                        alert(`เริ่ม ${item.name} แล้ว`);
                    }
                    await updateStatus();
                } catch (err) {
                    alert(`Error: ${err.message}`);
                } finally {
                    btn.disabled = false;
                }
            });
        }
    });

    // Poll status every 2 seconds
    setInterval(updateStatus, 2000);
    updateStatus();

    // ==========================================
    // 🕹️ Joystick Logic
    // ==========================================
    const zone = document.getElementById('joystick-zone');
    if (zone && window.nipplejs) {
        const manager = nipplejs.create({
            zone: zone,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#3b82f6',
            size: 250  // Larger joystick for better control
        });

        let lastCall = 0;
        const THROTTLE_MS = 50; // Faster response: ~20 calls/sec
        const SENSITIVITY = 1.5; // Boost joystick sensitivity

        manager.on('move', (evt, data) => {
            const now = Date.now();
            if (now - lastCall < THROTTLE_MS) return;
            lastCall = now;

            // Get force (0-1) and apply sensitivity
            const force = Math.min(data.force, 1.0);
            const multiplier = force * SENSITIVITY;

            // Map joystick to robot control
            // Forward (Screen Up) -> ROS +vx
            // Right (Screen Right) -> ROS -wz (turn right)
            let vx = data.vector.y * multiplier;
            let wz = -data.vector.x * multiplier;

            // Clamp values to [-1, 1]
            vx = Math.max(-1, Math.min(1, vx));
            wz = Math.max(-1, Math.min(1, wz));

            LegacyROS.moveRobot(vx, wz);
        });

        manager.on('end', () => {
            // Stop immediately on release
            LegacyROS.moveRobot(0.0, 0.0);
        });
    }
});

// Add Move function to LegacyROS object (Patching it in)
LegacyROS.moveRobot = async function (vx, wz) {
    try {
        await fetch(`${this.apiBase}/move`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ vx: vx, wz: wz })
        });
    } catch (error) {
        console.error("Joystick Move Error:", error);
    }
};
