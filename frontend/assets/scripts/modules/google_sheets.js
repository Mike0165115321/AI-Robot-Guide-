const SHEETS_STORAGE_KEY = 'nongnan_sheets_config';

// Load saved config from localStorage
function loadSheetsConfig() {
    try {
        const saved = localStorage.getItem(SHEETS_STORAGE_KEY);
        return saved ? JSON.parse(saved) : null;
    } catch (e) {
        console.error('Failed to load sheets config:', e);
        return null;
    }
}

// Save config to localStorage
function saveSheetsConfig(config) {
    try {
        localStorage.setItem(SHEETS_STORAGE_KEY, JSON.stringify(config));
    } catch (e) {
        console.error('Failed to save sheets config:', e);
    }
}

// Clear saved config
function clearSheetsConfig() {
    localStorage.removeItem(SHEETS_STORAGE_KEY);
}

async function initGoogleSheetsModule() {
    console.log('📊 Initializing Google Sheets Module...');

    try {
        // [V5.5] Also check server capabilities (do we have credentials?)
        checkServerCapabilities();

        // ============================================================
        // [FIX] STEP 1: Always restore URL from localStorage FIRST!
        // This ensures URL is never lost, even if API fails
        // ============================================================
        const savedConfig = loadSheetsConfig();
        if (savedConfig && savedConfig.sheet_url) {
            const urlInput = document.getElementById('sheets-url-input');
            if (urlInput) {
                urlInput.value = savedConfig.sheet_url;
                console.log('📋 Restored URL from localStorage:', savedConfig.sheet_url.substring(0, 50) + '...');
            }
        }

        // ============================================================
        // STEP 2: Check if backend has active connection
        // ============================================================
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/status`);

        if (response.ok) {
            const data = await response.json();

            if (data.connection && data.connection.connected) {
                // Backend says connected - great!
                console.log('✅ Already connected to backend');
                updateSheetsUI(data.connection);

                // Restore auto-polling if it was running
                const wasPollingEnabled = localStorage.getItem('sheets_auto_polling_enabled') === 'true';
                if (wasPollingEnabled) {
                    console.log('🔄 Auto-restoring polling state...');
                    setTimeout(() => startAutoPolling(), 1000);
                }
                return; // Done!
            }
        }

        // ============================================================
        // STEP 3: Not connected to backend - try auto-reconnect
        // ============================================================
        console.log('⚠️ Not connected to backend, attempting auto-reconnect...');
        updateSheetsUI({ connected: false });

        if (savedConfig && savedConfig.sheet_url) {
            // Set default mode
            localStorage.setItem('sheets_sync_mode', 'polling');

            // Try to reconnect
            await autoReconnectSheet(savedConfig.sheet_url);

            // Restore auto-polling if it was running
            const wasPollingEnabled = localStorage.getItem('sheets_auto_polling_enabled') === 'true';
            if (wasPollingEnabled) {
                console.log('🔄 Auto-restoring polling after reconnect...');
                setTimeout(() => startAutoPolling(), 2000);
            }
        } else {
            console.log('💡 No saved URL - waiting for user to enter URL');
        }

    } catch (error) {
        console.error('Init error:', error);
        updateSheetsUI({ connected: false });

        // Still restore URL even if there's an error!
        const savedConfig = loadSheetsConfig();
        if (savedConfig && savedConfig.sheet_url) {
            const urlInput = document.getElementById('sheets-url-input');
            if (urlInput) urlInput.value = savedConfig.sheet_url;
        }
    }
}

async function checkServerCapabilities() {
    // [REMOVED] Developer warning removed - not needed for regular users
    // ผู้ใช้ทั่วไปไม่จำเป็นต้องเห็น warning เรื่อง Public Mode
    console.log('🔍 ตรวจสอบ: ระบบทำงานในโหมด Public (ไม่ต้องใช้ credentials)');
}

// Auto-reconnect without alert
async function autoReconnectSheet(sheetUrl) {
    // [FIX] Always restore URL to input field first - don't lose it!
    const urlInput = document.getElementById('sheets-url-input');
    if (urlInput) urlInput.value = sheetUrl;

    try {
        console.log('🔄 Attempting auto-reconnect to:', sheetUrl);

        // [FIX] Use connect-public endpoint for Public Mode
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/connect-public`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sheet_url: sheetUrl })
        });

        const data = await response.json();

        if (response.ok && data.success) {
            console.log('✅ Auto-reconnected to:', data.status.sheet_title);
            updateSheetsUI(data.status);
        } else {
            // [FIX] Don't clear config on failure - keep URL so user can retry
            console.warn('⚠️ Auto-reconnect failed:', data.detail || 'Unknown error');
            console.log('💡 URL preserved in input field - user can click "เชื่อมต่อ" to retry');
            // Just update UI to show disconnected, but keep URL in input
            updateSheetsUI({ connected: false });
        }
    } catch (error) {
        console.error('Auto-reconnect error:', error.message);
        // [FIX] Still show URL so user knows what we tried
        updateSheetsUI({ connected: false });
    }
}

function updateSheetsUI(status) {
    const statusDot = document.getElementById('sheets-status-dot');
    const statusText = document.getElementById('sheets-status-text');
    const sheetsInfo = document.getElementById('sheets-info');
    const sheetsTitle = document.getElementById('sheets-title');
    const sheetsLastSync = document.getElementById('sheets-last-sync');
    const syncBtn = document.getElementById('sheets-sync-btn');
    const disconnectBtn = document.getElementById('sheets-disconnect-btn');
    const urlInput = document.getElementById('sheets-url-input');
    const connectBtn = document.getElementById('sheets-connect-btn');
    const modeSelection = document.getElementById('sheets-mode-selection');

    if (status && status.connected) {
        // Connected state
        statusDot.style.background = '#22c55e';

        let modeText = '';
        if (status.mode === 'public') {
            modeText = ' <span style="font-size:0.8em; opacity:0.8; background:rgba(255,255,255,0.2); padding:2px 6px; border-radius:4px;">(Public Mode)</span>';
        } else if (status.mode === 'service_account') {
            modeText = ' <span style="font-size:0.8em; opacity:0.8;">(Service Account)</span>';
        }

        statusText.innerHTML = '✅ เชื่อมต่อแล้ว' + modeText;
        sheetsTitle.textContent = status.sheet_title || status.sheet_id;
        sheetsLastSync.textContent = status.last_sync || 'ยังไม่ได้ sync';
        sheetsInfo.style.display = 'block';

        // Show sync/disconnect buttons
        syncBtn.style.display = 'inline-block';
        syncBtn.disabled = false;
        disconnectBtn.style.display = 'inline-block';
        disconnectBtn.disabled = false;

        // Show delete button when connected
        const deleteBtn = document.getElementById('sheets-disconnect-delete-btn');
        if (deleteBtn) {
            deleteBtn.style.display = 'inline-block';
            deleteBtn.disabled = false;
        }

        // Disable URL input
        urlInput.disabled = true;
        connectBtn.disabled = true;
        connectBtn.style.display = 'none'; // Hide connect button

        // We want to KEEP mode selection visible so user can choose/see Polling vs Webhook
        if (modeSelection) modeSelection.style.display = 'block';
    } else {
        // Disconnected state
        statusDot.style.background = '#ef4444';
        statusText.textContent = 'ยังไม่ได้เชื่อมต่อ';
        sheetsInfo.style.display = 'none';

        // Hide sync/disconnect/delete buttons when not connected
        syncBtn.style.display = 'none';
        disconnectBtn.style.display = 'none';

        // Hide delete button when disconnected
        const deleteBtn = document.getElementById('sheets-disconnect-delete-btn');
        if (deleteBtn) {
            deleteBtn.style.display = 'none';
        }

        // Enable URL input and show connect button
        if (urlInput) urlInput.disabled = false;
        if (connectBtn) {
            connectBtn.disabled = false;
            connectBtn.style.display = 'inline-block'; // Show connect button
            connectBtn.textContent = '🔗 เชื่อมต่อ';
        }

        // Mode selection is no longer shown - we only use polling mode
        // Keep modeSelection for JS compatibility but it's hidden in HTML

        // URL section should always be visible (no mode selection step needed)
    }
}

async function connectGoogleSheet() {
    const urlInput = document.getElementById('sheets-url-input');
    const url = urlInput.value.trim();

    if (!url) {
        alert('กรุณาวาง Google Sheets URL');
        return;
    }

    const connectBtn = document.getElementById('sheets-connect-btn');
    connectBtn.disabled = true;
    connectBtn.textContent = '⏳ กำลังเชื่อมต่อ...';

    try {
        // [FIX] Use connect-public endpoint directly for Public Mode Polling
        // This avoids Service Account fallback errors and works without credentials
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/connect-public`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sheet_url: url })
        });

        const data = await response.json();

        if (response.ok && data.success) {
            // Save config to localStorage for persistence
            saveSheetsConfig({ sheet_url: url, sheet_id: data.status.sheet_id });

            // [SIMPLIFIED] Auto-set polling mode (no more mode selection UI)
            selectedSyncMode = 'polling';
            localStorage.setItem('sheets_sync_mode', 'polling');

            alert('✅ เชื่อมต่อสำเร็จ! ระบบจะซิงค์ข้อมูลทุก 5 นาทีโดยอัตโนมัติ');
            updateSheetsUI(data.status);

            // Auto-start polling after connect
            setTimeout(() => {
                startAutoPolling();
            }, 500);
        } else {
            alert('❌ เชื่อมต่อไม่สำเร็จ!\n\n' + (data.detail || 'ตรวจสอบว่า:\n1. Sheet ถูกแชร์เป็น "ทุกคนที่มีลิงก์" แล้ว\n2. URL ถูกคัดลอกมาครบถ้วน'));
        }
    } catch (error) {
        alert('❌ เกิดข้อผิดพลาด: ' + error.message);
    } finally {
        connectBtn.disabled = false;
        connectBtn.textContent = '🔗 เชื่อมต่อ';
    }
}

async function syncGoogleSheet() {
    const syncBtn = document.getElementById('sheets-sync-btn');
    const resultDiv = document.getElementById('sheets-sync-result');

    syncBtn.disabled = true;
    syncBtn.textContent = '⏳ กำลัง Sync...';
    resultDiv.style.display = 'none';

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/sync-now`, {
            method: 'POST'
        });

        const data = await response.json();

        resultDiv.style.display = 'block';

        if (data.success) {
            const r = data.result;
            resultDiv.innerHTML = `
                <div style="color: #22c55e; font-weight: bold; margin-bottom: 0.5rem;">✅ Sync สำเร็จ!</div>
                <div>➕ สร้างใหม่: ${r.created} รายการ</div>
                <div>📝 อัพเดท: ${r.updated} รายการ</div>
                <div>🗑️ ลบ: ${r.deleted} รายการ</div>
                <div style="font-size: 0.8rem; color: #888; margin-top: 0.5rem;">เวลา: ${r.timestamp}</div>
            `;
            resultDiv.style.borderColor = 'rgba(34,197,94,0.5)';

            // Refresh table
            fetchAndDisplayLocations();

            // Update last sync time
            document.getElementById('sheets-last-sync').textContent = r.timestamp;
        } else {
            resultDiv.innerHTML = `
                <div style="color: #ef4444; font-weight: bold;">❌ Sync ไม่สำเร็จ</div>
                <div>${data.result?.errors?.join(', ') || 'Unknown error'}</div>
            `;
            resultDiv.style.borderColor = 'rgba(239,68,68,0.5)';
        }
    } catch (error) {
        resultDiv.style.display = 'block';
        resultDiv.innerHTML = `<div style="color: #ef4444;">❌ Error: ${error.message}</div>`;
    } finally {
        syncBtn.disabled = false;
        syncBtn.textContent = '🔄 Sync ตอนนี้';
    }
}

async function disconnectGoogleSheet() {
    if (!confirm('ยืนยันยกเลิกการเชื่อมต่อ Google Sheet?\n\n(ข้อมูลที่ซิงค์มาจะยังคงอยู่ในฐานข้อมูล)')) return;

    stopAutoPolling();

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/disconnect`, {
            method: 'DELETE'
        });

        if (response.ok) {
            // Clear saved config from localStorage
            clearSheetsConfig();

            alert('✅ ยกเลิกการเชื่อมต่อแล้ว (ข้อมูลยังคงอยู่)');
            updateSheetsUI({ connected: false });
            document.getElementById('sheets-url-input').value = '';
            document.getElementById('sheets-sync-result').style.display = 'none';
        }
    } catch (error) {
        alert('❌ เกิดข้อผิดพลาด: ' + error.message);
    }
}

async function disconnectAndDeleteGoogleSheet() {
    // Double confirmation for destructive action
    if (!confirm('⚠️ คำเตือน: การดำเนินการนี้จะลบข้อมูลทั้งหมดที่ซิงค์มาจาก Sheet นี้!\n\nคุณแน่ใจหรือไม่?')) return;
    if (!confirm('⚠️ ยืนยันอีกครั้ง: ข้อมูลที่ลบจะไม่สามารถกู้คืนได้!\n\nกด OK เพื่อยืนยันการลบ')) return;

    stopAutoPolling();

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/sheets/disconnect-and-delete`, {
            method: 'DELETE'
        });

        const data = await response.json();

        if (response.ok && data.success) {
            // Clear saved config from localStorage
            clearSheetsConfig();

            alert(`✅ ${data.message}`);
            updateSheetsUI({ connected: false });
            document.getElementById('sheets-url-input').value = '';
            document.getElementById('sheets-sync-result').style.display = 'none';

            // Refresh the location list
            if (typeof fetchAndDisplayLocations === 'function') {
                fetchAndDisplayLocations();
            }
        } else {
            alert('❌ ' + (data.detail || 'เกิดข้อผิดพลาด'));
        }
    } catch (error) {
        alert('❌ เกิดข้อผิดพลาด: ' + error.message);
    }
}

// ==========================================================
//  SYNC MODE SELECTION & AUTO POLLING
// ==========================================================

let selectedSyncMode = null;
let pollingInterval = null;
const POLLING_INTERVAL_MS = 5 * 60 * 1000; // 5 minutes

function showModeSelection() {
    document.getElementById('sheets-mode-selection').style.display = 'block';
}

function selectSyncMode(mode) {
    selectedSyncMode = mode;


    // Persist mode to localStorage
    if (mode) {
        localStorage.setItem('sheets_sync_mode', mode);
    }

    // Update card styles
    const pollingCard = document.getElementById('mode-polling-card');
    const webhookCard = document.getElementById('mode-webhook-card');
    const instructionsDiv = document.getElementById('mode-instructions');
    const startPollingBtn = document.getElementById('sheets-start-polling-btn');
    const stopPollingBtn = document.getElementById('sheets-stop-polling-btn');
    const sheetsMode = document.getElementById('sheets-mode');
    const urlSection = document.getElementById('sheets-url-section');
    const connectBtn = document.getElementById('sheets-connect-btn');

    // Reset styles
    pollingCard.style.borderWidth = '2px';
    webhookCard.style.borderWidth = '2px';
    pollingCard.style.transform = 'scale(1)';
    webhookCard.style.transform = 'scale(1)';

    // Show URL section after selecting mode
    if (urlSection) {
        urlSection.style.display = 'block';
    }

    // Reset connect button text
    if (connectBtn) {
        connectBtn.textContent = '🔗 เชื่อมต่อ';
    }

    if (mode === 'polling') {
        pollingCard.style.borderWidth = '3px';
        pollingCard.style.transform = 'scale(1.02)';
        sheetsMode.textContent = '🔄 Auto Polling (ทุก 5 นาที)';

        startPollingBtn.style.display = 'inline-block';
        startPollingBtn.disabled = false;
        stopPollingBtn.style.display = 'none';

        instructionsDiv.style.display = 'block';
        instructionsDiv.innerHTML = `
            <div style="color: #22c55e; font-weight: bold; margin-bottom: 0.5rem;">✅ พร้อมใช้งาน Auto Polling แล้ว!</div>
            <div style="font-size: 0.85rem; color: var(--text-light); line-height: 1.6;">
                <p style="margin-bottom: 5px;"><strong>📋 เช็คก่อนเริ่ม:</strong></p>
                <ul style="margin-top: 0; padding-left: 20px; color: #ccc;">
                    <li>✅ ตรวจสอบว่า Sheet ถูกแชร์เป็น <strong>"ทุกคนที่มีลิงก์"</strong> แล้ว</li>
                    <li>✅ คัดลอก URL <strong>ทั้งหมด</strong> จาก Address Bar ของเบราว์เซอร์</li>
                    <li>✅ URL ต้องมีลักษณะแบบนี้: <code>https://docs.google.com/spreadsheets/d/...</code></li>
                </ul>

                <p style="margin-bottom: 5px; margin-top: 10px;"><strong>▶️ วิธีเริ่มใช้งาน:</strong></p>
                <ul style="margin-top: 0; padding-left: 20px; color: #ccc;">
                     <li>กดปุ่ม <strong>"▶️ เริ่ม Auto Sync"</strong> ด้านล่าง</li>
                     <li>ระบบจะดึงข้อมูลใหม่จาก Sheet <strong>ทุกๆ 5 นาที</strong> โดยอัตโนมัติ</li>
                </ul>
                
                <div style="background: rgba(251,191,36,0.1); border: 1px solid rgba(251,191,36,0.3); padding: 8px; border-radius: 4px; margin-top: 10px; color: #fbbf24; font-size: 0.8rem;">
                    ⚠️ <strong>ถ้าเชื่อมต่อไม่ได้:</strong> ลองคัดลอก URL ใหม่อีกครั้ง และตรวจสอบว่า URL ไม่ถูกตัดทอน
                </div>
            </div>
        `;
    } else if (mode === 'webhook') {
        webhookCard.style.borderWidth = '3px';
        webhookCard.style.transform = 'scale(1.02)';
        sheetsMode.textContent = '⚡ Webhook (Real-time)';

        startPollingBtn.style.display = 'none';
        stopPollingBtn.style.display = 'none';

        instructionsDiv.style.display = 'block';

        // Load saved state (Default: hidden)
        const isGuideVisible = localStorage.getItem('webhook_guide_visible') === 'true';
        const initialMaxHeight = isGuideVisible ? '2000px' : '0px';
        const initialOpacity = isGuideVisible ? '1' : '0';
        const initialBtnText = isGuideVisible ? '🔽 ซ่อนวิธีทำ' : '📖 แสดงวิธีทำ';

        instructionsDiv.innerHTML = `
            <div style="color: #fbbf24; font-weight: bold; margin-bottom: 0.5rem; display: flex; justify-content: space-between; align-items: center;">
                <span>⚡ วิธี Setup Webhook (Real-time)</span>
                <button onclick="toggleWebhookGuide(this)" 
                        style="background: rgba(251,191,36,0.2); border: 1px solid rgba(251,191,36,0.4); color: #fbbf24; padding: 4px 10px; border-radius: 4px; cursor: pointer; font-size: 0.8rem; transition: all 0.2s;">
                    ${initialBtnText}
                </button>
            </div>
            
            <div id="webhook-guide-content" style="overflow: hidden; transition: all 0.5s ease-in-out; max-height: ${initialMaxHeight}; opacity: ${initialOpacity};">
                <div style="font-size: 0.85rem; color: var(--text-light); line-height: 1.6; padding-top: 10px;">
                    <p style="margin-bottom: 5px;"><strong>ขั้นตอนที่ 1: เตรียม Script</strong></p>
                    <ul style="margin-top: 0; padding-left: 20px; color: #ccc;">
                        <li>เปิด Google Sheet ของคุณ</li>
                        <li>ไปที่เมนู <strong>Extensions (ส่วนขยาย)</strong> > <strong>Apps Script</strong></li>
                        <li>ลบ Code เดิมออกทั้งหมด แล้ววาง Code ด้านล่างนี้:</li>
                    </ul>
                    
                    <pre style="background: rgba(0,0,0,0.3); padding: 0.75rem; border-radius: 4px; overflow-x: auto; font-size: 0.75rem; border: 1px solid rgba(251,191,36,0.3); color: #e2e8f0; margin: 10px 0;">
function onEdit(e) {
  const sheet = e.source.getActiveSheet();
  const row = e.range.getRow();
  if (row === 1) return; // ข้าม Header

  // 1. อ่านข้อมูลจากแถวที่แก้ไข
  const headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
  const values = sheet.getRange(row, 1, 1, sheet.getLastColumn()).getValues()[0];
  
  const data = {};
  headers.forEach((h, i) => { data[h] = values[i]; });
  
  // 2. ส่งข้อมูลไปยัง Server
  // ⚠️ อย่าลืมแก้ URL นี้ให้เป็น URL ของ Server คุณ
  const webhookUrl = "YOUR_SERVER_URL/api/admin/sheets/webhook";
  
  try {
    UrlFetchApp.fetch(webhookUrl, {
      method: "POST",
      contentType: "application/json",
      payload: JSON.stringify({ 
        event: "edit", 
        row_data: data,
        sheet_url: e.source.getUrl()
      })
    });
  } catch (error) {
    Logger.log("Webhook Error: " + error);
  }
}</pre>

                    <p style="margin-bottom: 5px; margin-top: 15px;"><strong>ขั้นตอนที่ 2: ตั้งค่า Server URL</strong></p>
                    <ul style="margin-top: 0; padding-left: 20px; color: #ccc;">
                        <li>ใน Code บรรทัดที่ 15: เปลี่ยน <code>YOUR_SERVER_URL</code> เป็น URL ของเว็บนี้</li>
                        <li><span style="color: #fbbf24;">⚠️ สำคัญ:</span> ถ้าเปิดบนเครื่องตัวเอง (localhost) Google จะมองไม่เห็น <br>ต้องใช้ <strong>ngrok</strong> หรือ <strong>Cloud Server</strong> เท่านั้น</li>
                        <li>กดไอคอน 💾 <strong>Save Project</strong></li>
                    </ul>

                    <p style="margin-bottom: 5px; margin-top: 15px;"><strong>ขั้นตอนที่ 3: เปิดใช้งาน (Trigger)</strong></p>
                    <ul style="margin-top: 0; padding-left: 20px; color: #ccc;">
                        <li>กดไอคอน ⏰ <strong>Triggers</strong> (รูปนาฬิกา) แถบซ้ายมือ</li>
                        <li>กดปุ่มสีฟ้า <strong>+ Add Trigger</strong> (มุมขวาล่าง)</li>
                        <li>ตั้งค่าตามนี้:
                            <ul style="margin-top: 5px;">
                                <li>Select event type: <strong>On edit</strong></li>
                            </ul>
                        </li>
                        <li>กด <strong>Save</strong> (อาจจต้องกด Allow/Advanced > Go to... เพื่ออนุญาตสิทธิ์)</li>
                    </ul>
                    <div style="background: rgba(34,197,94,0.1); padding: 8px; border-radius: 4px; margin-top: 10px; text-align: center; color: #4ade80;">
                        ✅ เสร็จสิ้น! ลองแก้ไขข้อมูลใน Sheet แล้วดูผลลัพธ์ในเว็บได้เลย
                    </div>
                    
                    <div style="text-align: center; margin-top: 15px;">
                        <button onclick="toggleWebhookGuide(this)"
                                style="background: rgba(100,116,139,0.2); border: 1px solid rgba(148,163,184,0.4); color: #cbd5e1; padding: 6px 12px; border-radius: 4px; cursor: pointer; font-size: 0.85rem; transition: all 0.2s;">
                            ⬆️ ซ่อน/เก็บคำแนะนำ
                        </button>
                    </div>
                </div>
            </div>

        `;
    }
}

function startAutoPolling() {
    if (pollingInterval) {
        console.log('Polling already running');
        return;
    }

    const startBtn = document.getElementById('sheets-start-polling-btn');
    const stopBtn = document.getElementById('sheets-stop-polling-btn');
    const sheetsMode = document.getElementById('sheets-mode');

    startBtn.style.display = 'none';
    stopBtn.style.display = 'inline-block';
    stopBtn.disabled = false;
    sheetsMode.textContent = '🔄 Auto Polling (กำลังทำงาน...)';

    // [NEW] Save polling state to localStorage for persistence across refresh
    localStorage.setItem('sheets_auto_polling_enabled', 'true');

    // Sync immediately
    syncGoogleSheet();

    // Start polling
    pollingInterval = setInterval(() => {
        console.log('🔄 Auto polling: syncing Google Sheet...');
        syncGoogleSheet();
    }, POLLING_INTERVAL_MS);

    console.log(`✅ Auto polling started (every ${POLLING_INTERVAL_MS / 1000 / 60} minutes)`);
}

function stopAutoPolling() {
    if (pollingInterval) {
        clearInterval(pollingInterval);
        pollingInterval = null;

        const startBtn = document.getElementById('sheets-start-polling-btn');
        const stopBtn = document.getElementById('sheets-stop-polling-btn');
        const sheetsMode = document.getElementById('sheets-mode');

        if (startBtn) startBtn.style.display = 'inline-block';
        if (stopBtn) stopBtn.style.display = 'none';
        if (sheetsMode) sheetsMode.textContent = '🔄 Auto Polling (หยุดแล้ว)';

        // [NEW] Clear polling state from localStorage
        localStorage.removeItem('sheets_auto_polling_enabled');

        console.log('⏹️ Auto polling stopped');
    }
}

// Helper for Webhook guide toggle
window.toggleWebhookGuide = function (btn) {
    const content = document.getElementById('webhook-guide-content');
    const container = document.getElementById('mode-instructions');
    const buttons = container.querySelectorAll('button');

    // Check current state
    // Note: if style is not set inline initially, it might be empty.
    // We assume default is open (maxHeight 2000px) from the HTML injection
    const currentMaxHeight = content.style.maxHeight;
    const isClosed = currentMaxHeight === '0px';

    if (isClosed) {
        // OPEN
        content.style.maxHeight = '2000px';
        content.style.opacity = '1';
        localStorage.setItem('webhook_guide_visible', 'true'); // Save state

        buttons.forEach(b => {
            // Top button
            if (b.parentElement.style.justifyContent === 'space-between') {
                b.innerHTML = '🔽 ซ่อนวิธีทำ';
            } else {
                // Bottom button
                b.innerHTML = '⬆️ ซ่อน/เก็บคำแนะนำ';
            }
        });
    } else {
        // CLOSE
        content.style.maxHeight = '0px';
        content.style.opacity = '0';
        localStorage.setItem('webhook_guide_visible', 'false'); // Save state

        buttons.forEach(b => {
            // Top button
            if (b.parentElement.style.justifyContent === 'space-between') {
                b.innerHTML = '📖 แสดงวิธีทำ';
            } else {
                // Bottom button
                b.innerHTML = '⬆️ ซ่อน/เก็บคำแนะนำ';
            }
        });
    }
};
