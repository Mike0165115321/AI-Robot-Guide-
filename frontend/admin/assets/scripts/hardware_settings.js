// Hardware Manager Class to handle System Toggle
// Independent script to avoid module dependencies

class HardwareManager {
    constructor() {
        console.log("HardwareManager Init (New IDs)");
        this.apiBase = '/api/ros';

        // New IDs
        this.statusLabel = document.getElementById('sys-status-label');
        this.toggleBtn = document.getElementById('sys-toggle-btn');
        this.shortcutBox = document.getElementById('sys-shortcut-box');

        if (!this.toggleBtn) console.error("Sys Toggle Btn Missing!");

        this.init();
    }

    init() {
        if (this.toggleBtn) {
            // Clone to clear listeners
            const newBtn = this.toggleBtn.cloneNode(true);
            this.toggleBtn.parentNode.replaceChild(newBtn, this.toggleBtn);
            this.toggleBtn = newBtn;

            this.toggleBtn.addEventListener('click', (e) => {
                console.log("System Toggle Clicked");
                e.preventDefault();
                this.toggleSystem();
            });
        }

        // Initial check and polling
        this.checkStatus();
        setInterval(() => this.checkStatus(), 2000);
    }

    async checkStatus() {
        try {
            const response = await fetch(`${this.apiBase}/status`);
            if (!response.ok) return;
            const data = await response.json();
            const running = data.running_scripts || {};

            const isBringupRunning = running['bringup'] === 'running';
            this.updateUI(isBringupRunning);

        } catch (e) {
            console.error("Hardware status check failed", e);
            if (this.statusLabel) {
                this.statusLabel.innerText = "Error";
                this.statusLabel.className = "badge bg-danger";
            }
        }
    }

    updateUI(isRunning) {
        if (isRunning) {
            // System ON
            if (this.statusLabel) {
                this.statusLabel.innerText = "Online (ทำงานอยู่)";
                this.statusLabel.className = "badge bg-success";
                this.statusLabel.style.backgroundColor = "#22c55e";
                this.statusLabel.style.color = "white";
            }

            if (this.toggleBtn) {
                this.toggleBtn.innerHTML = '<i class="fa-solid fa-power-off"></i> ปิดระบบ (Stop)';
                this.toggleBtn.style.background = "rgba(239, 68, 68, 0.2)";
                this.toggleBtn.style.color = "#ef4444";
                this.toggleBtn.style.border = "1px solid rgba(239, 68, 68, 0.5)";
            }

            // Show Shortcut
            if (this.shortcutBox) this.shortcutBox.style.display = 'block';
        } else {
            // System OFF
            if (this.statusLabel) {
                this.statusLabel.innerText = "Offline (ปิดอยู่)";
                this.statusLabel.className = "badge bg-secondary";
                this.statusLabel.style.backgroundColor = "#666";
                this.statusLabel.style.color = "white";
            }

            if (this.toggleBtn) {
                this.toggleBtn.innerHTML = '<i class="fa-solid fa-power-off"></i> เปิดระบบ (Start)';
                this.toggleBtn.style.background = "var(--color-primary)";
                this.toggleBtn.style.color = "white";
                this.toggleBtn.style.border = "none";
            }

            // Hide Shortcut
            if (this.shortcutBox) this.shortcutBox.style.display = 'none';
        }
    }

    async toggleSystem() {
        if (!this.toggleBtn) return;

        this.toggleBtn.disabled = true;
        const isRunning = this.toggleBtn.innerText.includes("ปิดระบบ") || this.toggleBtn.innerText.includes("Stop");

        try {
            if (isRunning) {
                // Stop
                await fetch(`${this.apiBase}/stop/bringup`, { method: 'POST' });
                alert("กำลังปิดระบบ...");
            } else {
                // Start
                await fetch(`${this.apiBase}/launch/bringup`, { method: 'POST' });
                alert("กำลังเปิดระบบ hardware... (รอสักครู่)");
            }
            // Immediate check
            setTimeout(() => this.checkStatus(), 1000);
        } catch (e) {
            alert(`Error: ${e.message}`);
        } finally {
            this.toggleBtn.disabled = false;
        }
    }
}

// Initialize immediately
document.addEventListener('DOMContentLoaded', () => {
    new HardwareManager();
});
