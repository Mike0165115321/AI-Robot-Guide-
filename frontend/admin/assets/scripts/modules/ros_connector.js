const ROSConnector = {
    apiBase: '/api/hardware',

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
        { id: 'btn-start_all', script: 'start_all', name: 'เริ่มระบบทั้งหมด', combo: ['bringup', 'ros2_bridge', 'nav2'] },
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
            const data = await ROSConnector.getStatus();
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
        keys: { w: false, a: false, s: false, d: false, q: false, e: false, ArrowUp: false, ArrowDown: false, ArrowLeft: false, ArrowRight: false },
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
            ROSConnector.moveRobot(0, 0, 0); // Stop robot (fixed: 3 params)
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
            let vy = 0.0;
            let wz = 0.0;

            if (this.keys.w === true || this.keys.ArrowUp === true) vx += 1.0;
            if (this.keys.s === true || this.keys.ArrowDown === true) vx -= 1.0;
            if (this.keys.a === true || this.keys.ArrowLeft === true) wz -= 1.0; // Inverted
            if (this.keys.d === true || this.keys.ArrowRight === true) wz += 1.0; // Inverted
            if (this.keys.q === true) vy += 1.0; // Strafe Left
            if (this.keys.e === true) vy -= 1.0; // Strafe Right

            // Debug log every 10 ticks or if keys pressed
            if (vx !== 0 || vy !== 0 || wz !== 0) {
                console.log(`[Keyboard] Sending: vx=${vx}, vy=${vy}, wz=${wz}`);
            }

            // Throttling Logic
            this.sendThrottledCommand(vx, vy, wz);
            this.updateDebugUI(vx, vy, wz);
        },

        lastSentStart: { vx: 0, vy: 0, wz: 0 },
        lastSentTime: 0,

        sendThrottledCommand(vx, vy, wz) {
            const now = Date.now();
            const KEEP_ALIVE_MS = 400; // Send at least every 400ms to prevent safety timeout (0.5s)

            // Check if value changed
            const isChanged = (
                vx !== this.lastSentStart.vx ||
                vy !== this.lastSentStart.vy ||
                wz !== this.lastSentStart.wz
            );

            // Send conditions:
            // 1. Value Changed (Immediate)
            // 2. Keep Alive timeout passed (Safety)
            if (isChanged || (now - this.lastSentTime > KEEP_ALIVE_MS)) {
                ROSConnector.moveRobot(vx, vy, wz);
                this.lastSentStart = { vx, vy, wz };
                this.lastSentTime = now;

                if (isChanged) {
                    console.log(`[Keyboard] Sent Changed: ${vx}, ${vy}, ${wz}`);
                }
            }
        },

        updateDebugUI(vx, vy, wz) {
            // Update Key Classes
            const setKey = (id, isActive) => {
                const el = document.getElementById(id);
                if (el) isActive ? el.classList.add('active') : el.classList.remove('active');
            };

            setKey('key-w', this.keys.w || this.keys.ArrowUp);
            setKey('key-s', this.keys.s || this.keys.ArrowDown);
            setKey('key-a', this.keys.a || this.keys.ArrowLeft);
            setKey('key-d', this.keys.d || this.keys.ArrowRight);
            setKey('key-q', this.keys.q);
            setKey('key-e', this.keys.e);

            setKey('key-e', this.keys.e);

            // Update Values
            const vxEl = document.getElementById('debug-vx');
            const vyEl = document.getElementById('debug-vy');
            const wzEl = document.getElementById('debug-wz');
            if (vxEl) vxEl.innerText = vx.toFixed(1);
            if (vyEl) vyEl.innerText = vy.toFixed(1);
            if (wzEl) wzEl.innerText = wz.toFixed(1);

            // Speed Control Keys (Z = Decrease, X = Increase)
            if (this.keys.z) this.adjustSpeed(-10);
            if (this.keys.x) this.adjustSpeed(10);
        },

        lastSpeedUpdate: 0,
        adjustSpeed(deltaPercent) {
            const now = Date.now();
            if (now - this.lastSpeedUpdate < 200) return; // Debounce 200ms
            this.lastSpeedUpdate = now;

            const slider = document.getElementById('speed-slider');
            if (!slider) return;

            let current = parseInt(slider.value, 10);
            let newVal = current + deltaPercent;
            newVal = Math.max(10, Math.min(100, newVal)); // Clamp 10-100%

            if (newVal !== current) {
                slider.value = newVal;
                // Trigger change event manually to update backend
                slider.dispatchEvent(new Event('change'));

                // Update UI text immediately
                const speedVal = document.getElementById('speed-value');
                if (speedVal) speedVal.innerText = newVal;

                console.log(`[Keyboard] Speed adjusted to ${newVal}%`);
            }
        },

        resetDebugUI() {
            const keys = ['key-w', 'key-a', 'key-s', 'key-d', 'key-q', 'key-e'];
            keys.forEach(k => {
                const el = document.getElementById(k);
                if (el) el.classList.remove('active');
            });
            const vxEl = document.getElementById('debug-vx');
            const vyEl = document.getElementById('debug-vy');
            const wzEl = document.getElementById('debug-wz');
            if (vxEl) vxEl.innerText = "0.0";
            if (vyEl) vyEl.innerText = "0.0";
            if (wzEl) wzEl.innerText = "0.0";
        }
    };

    // Bind new keys
    KeyboardController.keys.z = false;
    KeyboardController.keys.x = false;

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
                                await ROSConnector.stopScript(item.combo[i]);
                            }
                            alert(`หยุด ${item.name} แล้ว`);
                        } else {
                            // Launch all scripts in combo (in order)
                            for (const scriptName of item.combo) {
                                await ROSConnector.launchScript(scriptName);
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
                        await ROSConnector.stopScript(item.script);
                        alert(`หยุด ${item.name} แล้ว`);
                    } else {
                        await ROSConnector.launchScript(item.script);
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
    // 🕹️ Dual Joystick Logic
    // ==========================================
    const leftZone = document.getElementById('joystick-left-zone');
    const rightZone = document.getElementById('joystick-right-zone');
    const legacyZone = document.getElementById('joystick-zone'); // Fallback

    console.log('[Joystick Debug] leftZone:', leftZone, 'rightZone:', rightZone, 'legacyZone:', legacyZone, 'nipplejs:', window.nipplejs);

    if (leftZone && rightZone && window.nipplejs) {
        console.log('[Joystick] Initializing Dual Joystick Mode');
        // Shared State
        let stickState = {
            vx: 0.0,
            vy: 0.0, // Strafing from Left Stick
            wz: 0.0  // Rotation from Right Stick
        };

        const THROTTLE_MS = 50;
        let lastCall = 0;

        // Helper to send command
        const sendStickCommand = () => {
            const now = Date.now();
            if (now - lastCall < THROTTLE_MS) return;
            lastCall = now;

            ROSConnector.moveRobot(stickState.vx, stickState.vy, stickState.wz);
        };

        // --- Left Stick (Move: Vx / Vy) ---
        const leftManager = nipplejs.create({
            zone: leftZone,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#3b82f6', // Blue
            size: 150
        });

        leftManager.on('move', (evt, data) => {
            const force = Math.min(data.force, 1.0);
            stickState.vx = data.vector.y * force;
            stickState.vy = -data.vector.x * force;
            sendStickCommand();
        });

        leftManager.on('end', () => {
            stickState.vx = 0.0;
            stickState.vy = 0.0;
            sendStickCommand();
        });

        // --- Right Stick (Rotate: Wz) ---
        const rightManager = nipplejs.create({
            zone: rightZone,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#22c55e', // Green
            size: 150
        });

        rightManager.on('move', (evt, data) => {
            const force = Math.min(data.force, 1.0);
            stickState.wz = -data.vector.x * force;
            sendStickCommand();
        });

        rightManager.on('end', () => {
            stickState.wz = 0.0;
            sendStickCommand();
        });
    } else if (legacyZone && window.nipplejs) {
        // ========== FALLBACK: Single Joystick Mode ==========
        console.log('[Joystick] Fallback to Single Joystick Mode');

        // IMPORTANT: Show the fallback container!
        const legacyContainer = document.getElementById('joystick-legacy-container');
        if (legacyContainer) {
            legacyContainer.style.display = 'flex';
            console.log('[Joystick] Showing legacy container');
        }

        const manager = nipplejs.create({
            zone: legacyZone,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#3b82f6',
            size: 250
        });

        let lastCall = 0;
        const THROTTLE_MS = 50;
        const SENSITIVITY = 1.5;

        manager.on('move', (evt, data) => {
            const now = Date.now();
            if (now - lastCall < THROTTLE_MS) return;
            lastCall = now;

            const force = Math.min(data.force, 1.0);
            const multiplier = force * SENSITIVITY;

            let vx = data.vector.y * multiplier;
            let wz = -data.vector.x * multiplier;
            let vy = 0.0;

            vx = Math.max(-1, Math.min(1, vx));
            wz = Math.max(-1, Math.min(1, wz));

            console.log(`[Joystick] Move: vx=${vx.toFixed(2)}, wz=${wz.toFixed(2)}`);
            ROSConnector.moveRobot(vx, vy, wz);
        });

        manager.on('end', () => {
            console.log('[Joystick] Released');
            ROSConnector.moveRobot(0.0, 0.0, 0.0);
        });
    } else {
        console.error('[Joystick] No joystick zones found or nipplejs not loaded!');
        console.error('[Joystick] Debug: leftZone=', leftZone, 'rightZone=', rightZone, 'legacyZone=', legacyZone);
    }

    // ==========================================
    // 🚀 Speed Control Logic
    // ==========================================
    const speedSlider = document.getElementById('speed-slider');
    const speedValue = document.getElementById('speed-value');

    if (speedSlider && speedValue) {
        speedSlider.addEventListener('input', (e) => {
            const percent = e.target.value;
            speedValue.innerText = percent;
        });

        speedSlider.addEventListener('change', async (e) => {
            const percent = parseInt(e.target.value, 10);
            // Linear: 10% -> 0.1 m/s, 100% -> 1.0 m/s (Example scale)
            // Let's say Base Max Linear = 1.0, Base Max Angular = 2.0
            const maxScalableLinear = 1.0;
            const maxScalableAngular = 2.0;

            const newLinear = (percent / 100.0) * maxScalableLinear;
            const newAngular = (percent / 100.0) * maxScalableAngular;

            try {
                await fetch(`${ROSConnector.apiBase}/config/speed`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ max_linear: newLinear, max_angular: newAngular })
                });
                console.log(`Speed updated: Linear=${newLinear.toFixed(2)}, Angular=${newAngular.toFixed(2)}`);
            } catch (err) {
                console.error("Failed to update speed:", err);
            }
        });
    }

    // Auto-Start Keyboard Control
    console.log('[Keyboard] Auto-starting keyboard control...');
    KeyboardController.toggle(); // Turn it on by default
});

// Add Move function to LegacyROS object (Patching it in)
ROSConnector.moveRobot = async function (vx, vy, wz) {
    try {
        await fetch(`${this.apiBase}/move`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ vx: vx, vy: vy, wz: wz })
        });
    } catch (error) {
        console.error("Joystick Move Error:", error);
    }
};
