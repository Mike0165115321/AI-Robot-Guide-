// map_manager.js - Real-time Update

let ros = null;
let viewer = null;
let gridClient = null;

let robotMarker = null;
let useHTTP = false;
let pollingInterval = null;
let currentWaypoints = [];
let currentMapMeta = null;

document.addEventListener('DOMContentLoaded', () => {
    loadMaps();
    initROS(); // Initialize ROS connection
});

// --- ROS Connection ---
function initROS() {
    ros = new ROSLIB.Ros({
        url: 'ws://' + window.location.hostname + ':9090'
    });

    ros.on('connection', () => {
        document.getElementById('ros-status').innerText = 'Connected to Robot';
        document.getElementById('ros-status').className = 'status-badge status-active';
    });

    ros.on('error', (error) => {
        document.getElementById('ros-status').innerText = 'Connection Error';
        document.getElementById('ros-status').className = 'status-badge status-inactive';
        enableHTTPMode();
    });

    ros.on('close', () => {
        document.getElementById('ros-status').innerText = 'Disconnected';
        document.getElementById('ros-status').className = 'status-badge status-inactive';
        enableHTTPMode();
    });

    setTimeout(() => {
        if (!ros.isConnected) enableHTTPMode();
    }, 3000);
}

function enableHTTPMode() {
    if (useHTTP) return;
    useHTTP = true;
    const statusEl = document.getElementById('ros-status');
    if (statusEl) {
        statusEl.innerText = 'HTTP Mode (Polling)';
        statusEl.className = 'status-badge';
        statusEl.style.backgroundColor = 'rgba(234, 179, 8, 0.2)';
        statusEl.style.color = '#eab308';
    }
    console.log("Switched to HTTP Polling Mode");
}

// --- API --- (Unchanged fetchMaps, deleteMapAPI)
async function fetchMaps() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/navigation/maps`);
        if (!response.ok) throw new Error('Failed to fetch maps');
        const data = await response.json();
        return data.maps || [];
    } catch (error) {
        console.error('Error fetching maps:', error);
        return [];
    }
}

async function deleteMapAPI(mapName) {
    if (!confirm(`ต้องการลบแผนที่ "${mapName}" ใช่หรือไม่?`)) return;
    try {
        const response = await fetch(`${API_BASE_URL}/api/navigation/maps/${mapName}`, { method: 'DELETE' });
        if (response.ok) {
            alert('ลบแผนที่เรียบร้อย');
            loadMaps();
        } else alert('เกิดข้อผิดพลาดในการลบแผนที่');
    } catch (error) {
        alert('API Connection Failed');
    }
}

// --- UI Logic (Unchanged loadMaps, createMapCard) ---
async function loadMaps() {
    const grid = document.getElementById('maps-grid');
    grid.innerHTML = '<div style="color:white;">Loading...</div>';
    const maps = await fetchMaps();
    grid.innerHTML = '';
    if (maps.length === 0) {
        grid.innerHTML = '<div style="color:#888; grid-column:1/-1; text-align:center;">ยังไม่มีแผนที่ กด "สร้างแผนที่ใหม่" เพื่อเริ่มสร้าง</div>';
        return;
    }
    maps.forEach(map => grid.appendChild(createMapCard(map)));
}

function createMapCard(map) {
    const div = document.createElement('div');
    div.className = 'map-card';
    const date = new Date(map.created_at * 1000).toLocaleString('th-TH');
    div.innerHTML = `
        <div class="map-preview"><span style="color:white; font-size:2rem;">🗺️</span></div>
        <div class="map-info">
            <div class="map-title">${map.name}</div>
            <div class="map-meta">📅 ${date}</div>
            <div class="map-actions">
                <button class="btn btn-sm btn-submit" onclick="loadMap('${map.name}')" style="flex:1;"><i class="fa-solid fa-play"></i> เลือกใช้</button>
                <button class="btn btn-sm btn-cta" onclick="deleteMapAPI('${map.name}')" style="background:rgba(239,68,68,0.2); color:#ef4444;"><i class="fa-solid fa-trash"></i></button>
            </div>
        </div>
    `;
    return div;
}

// --- Mapping Logic (REPLACED WITH ROS2D) ---
window.openMappingWizard = function () {
    const name = prompt("ตั้งชื่อแผนที่ใหม่ (ภาษาอังกฤษ):");
    if (!name || !/^[a-zA-Z0-9_-]+$/.test(name)) {
        alert("ชื่อไม่ถูกต้อง"); return;
    }
    document.getElementById('mapping-name-display').innerText = name;
    document.getElementById('mapping-wizard').style.display = 'flex';

    // Start Mapping Backend
    fetch(`${API_BASE_URL}/api/navigation/mapping/start`, { method: 'POST' })
        .then(() => {
            console.log("Mapping Started");
            startRealTimeMapping(); // START ROS2D
        })
        .catch(err => alert("Failed to start mapping: " + err));

    initJoystick();
}

function startRealTimeMapping() {
    const container = document.getElementById('canvas-container');
    // Clear previous canvas if any (ROS2D creates its own canvas)
    container.innerHTML = '';

    if (useHTTP) {
        startHTTPMapping(container);
        return;
    }

    // Initialize Viewer
    try {
        viewer = new ROS2D.Viewer({
            divID: 'canvas-container',
            width: container.clientWidth,
            height: container.clientHeight,
            background: '#000000'
        });

        // Add Grid Client (Map)
        // continuous: true is important for SLAM updates
        gridClient = new ROS2D.OccupancyGridClient({
            ros: ros,
            rootObject: viewer.scene,
            topic: '/map',
            continuous: true
        });

        // Scale to fit when map updates
        gridClient.on('change', () => {
            viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
            viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        });

        // Add Robot Marker
        // Tip: For 2D map, a simple triangle or arrow shape
        robotMarker = new ROS2D.NavigationArrow({
            size: 0.5,
            strokeSize: 0.05,
            fillColor: createjs.Graphics.getRGB(0, 255, 0, 0.8),
            pulse: false
        });
        viewer.scene.addChild(robotMarker);

        // Subscribe to TF / Robot Pose
        // Since we don't have setup TFClient comfortably, let's subscribe to /odom for simplified visualization
        // Or if SLAM is running, map->odom->base_link should be available. 
        // Let's use a simpler subscriber to /odom for now as it refreshes fast.
        // Ideally we use tf2_web_republisher but let's stick to simple topics if possible.
        // Actually, without tf2_web_republisher, visualizing robot on map frame is hard if map correction happens.
        // Let's rely on `gridClient` for the map, but for the robot, we might need to subscribe to /tf or /pose

        subscribeToRobotPose();

    } catch (e) {
        console.error("ROS2D Error: ", e);
        // Fallback if ROS2D fails even if connected
        enableHTTPMode();
        startHTTPMapping(container);
    }
}

function startHTTPMapping(container) {
    if (pollingInterval) clearInterval(pollingInterval);

    // Create Canvas
    const canvas = document.createElement('canvas');
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    canvas.style.objectFit = 'contain';
    container.appendChild(canvas);

    // Initial Size (will be updated)
    canvas.width = 800;
    canvas.height = 600;

    const ctx = canvas.getContext('2d');

    pollingInterval = setInterval(async () => {
        try {
            // 1. Get Metadata
            const metaRes = await fetch(`${API_BASE_URL}/api/navigation/mapping/live/metadata`);
            if (!metaRes.ok) return;
            const meta = await metaRes.json();
            if (!meta || meta.width === 0) return;

            // 2. Get Image
            const imgRes = await fetch(`${API_BASE_URL}/api/navigation/mapping/preview`);
            if (!imgRes.ok) return;
            const blob = await imgRes.blob();
            const imgBitmap = await createImageBitmap(blob);

            // 3. Get Robot Pose
            const poseRes = await fetch(`${API_BASE_URL}/api/navigation/pose`);
            const pose = await poseRes.json();

            // Update Canvas Size to match Map Resolution
            if (canvas.width !== meta.width || canvas.height !== meta.height) {
                canvas.width = meta.width;
                canvas.height = meta.height;
            }

            // Draw Map
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(imgBitmap, 0, 0);

            // Draw Robot
            if (pose && !pose.error && pose.x !== undefined) {
                // Coordinate Transform
                // Map Origin (meters) -> Pixel (0, Height)
                // px = (x - origin_x) / res
                // py = height - (y - origin_y) / res

                const px = (pose.x - meta.origin[0]) / meta.resolution;
                const py = meta.height - ((pose.y - meta.origin[1]) / meta.resolution);

                ctx.save();
                ctx.translate(px, py);
                // Rotate: ROS (CCW+) -> Canvas (CW+) with Y flip
                // Theta in ROS is standard trig (CCW).
                // In our canvas coords, Y is down.
                // A logic check: 
                // Robot at 0, facing X+ (0 rad). 
                // Robot facing Y+ (PI/2 rad).
                // In image, facing Y+ (North) should be Up.
                // Since Y moves Down in canvas, Up is Negative Y.
                // So we need to rotate such that PI/2 points to -Y.
                // standard canvas rotate(0) points +X. 
                // rotate(-PI/2) points -Y (Up).
                // So rotation = -theta.
                ctx.rotate(-pose.theta);

                // Draw Arrow
                ctx.beginPath();
                ctx.moveTo(10, 0); // Tip
                ctx.lineTo(-5, 5);
                ctx.lineTo(-5, -5);
                ctx.closePath();
                ctx.fillStyle = '#00FF00';
                ctx.fill();
                ctx.restore();
            }

        } catch (e) { console.error(e); }
    }, 500); // 2Hz
}

function subscribeToRobotPose() {
    // Listener for TF logic or Pose
    // For simplicity, let's subscribe to a topic that gives map->base_link if available?
    // Usually AMCL publishes /amcl_pose. In SLAM mode, we might need to trust /odom for raw movement 
    // or rely on the fact that map frame is static.

    // Let's try listening to /tf for odom->base_link and map->odom? 
    // That's complex in JS without tf library.

    // Fallback: Subscribe to /odom and assume map frame ~ odom frame (start at 0,0) for crude visualization
    // OR if we assume SLAM is working, the map expands around the robot.

    const odomListener = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });

    odomListener.subscribe((msg) => {
        if (robotMarker) {
            robotMarker.x = msg.pose.pose.position.x;
            robotMarker.y = -msg.pose.pose.position.y; // Canvas Y flip? ROS2D Viewer handles coordinates usually?
            // ROS2D viewer uses ROS coordinates! so +y is up.
            robotMarker.y = msg.pose.pose.position.y;

            // Quaternion to Yaw
            const q = msg.pose.pose.orientation;
            const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            const yaw_rad = Math.atan2(siny_cosp, cosy_cosp);

            robotMarker.rotation = -yaw_rad * 180.0 / Math.PI; // EaselJS rotation is degrees, CW?
            // ROS is CCW positive. EaselJS is CW positive. So negate.
        }
    });
}


window.cancelMapping = function () {
    if (!confirm("ยกเลิก?")) return;
    document.getElementById('mapping-wizard').style.display = 'none';
    if (pollingInterval) clearInterval(pollingInterval);
    fetch(`${API_BASE_URL}/api/navigation/mapping/stop`, { method: 'POST' });
}

window.saveAndFinishMapping = async function () {
    const name = document.getElementById('mapping-name-display').innerText;
    const res = await fetch(`${API_BASE_URL}/api/navigation/mapping/save/${name}`, { method: 'POST' });
    if (res.ok) {
        alert("Saved!");
        document.getElementById('mapping-wizard').style.display = 'none';
        if (pollingInterval) clearInterval(pollingInterval);
        loadMaps();
        fetch(`${API_BASE_URL}/api/navigation/mapping/stop`, { method: 'POST' });
    } else alert("Save Failed");
}


// --- Joystick / Keyboard (Unchanged) ---
let joystickInterval = null;
function initJoystick() {
    document.addEventListener('keydown', handleKeyboardControl);
    document.addEventListener('keyup', () => sendVelocity(0, 0, 0));
}
function handleKeyboardControl(e) {
    let vx = 0, vy = 0, wz = 0;
    const speed = document.getElementById('speed-slider') ? parseFloat(document.getElementById('speed-slider').value) : 0.3;
    const key = e.key.toLowerCase();

    if (e.key === 'ArrowUp' || key === 'w') vx = speed;
    else if (e.key === 'ArrowDown' || key === 's') vx = -speed;
    else if (e.key === 'ArrowLeft' || key === 'a') vy = speed; // Strafe Left
    else if (e.key === 'ArrowRight' || key === 'd') vy = -speed; // Strafe Right
    else if (key === 'q') wz = speed; // Turn Left
    else if (key === 'e') wz = -speed; // Turn Right
    else return;
    sendVelocity(vx, vy, wz);
}
async function sendVelocity(vx, vy, wz) {
    try {
        await fetch(`${API_BASE_URL}/api/hardware/move`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ vx, vy, wz })
        });
    } catch (e) { }
}

// Navigation Visualization (Simplified for now - can use same ROS2D logic later)
// Navigation Visualization
async function loadMap(mapName) {
    // 1. Tell Backend to Load this Map into Nav2
    try {
        await fetch(`${API_BASE_URL}/api/navigation/nav/load_map/${mapName}`, { method: 'POST' });
        console.log("Map Load Requested: " + mapName);
    } catch (e) {
        console.error("Failed to load map backend:", e);
        // Continue anyway to show static map? Or stop?
        // Better to warn user but let them see it.
    }

    document.getElementById('nav-interface').style.display = 'flex';
    document.getElementById('nav-map-name').innerText = mapName;

    const container = document.getElementById('nav-canvas-container');
    container.innerHTML = ''; // Clear

    // Create Canvas
    const canvas = document.createElement('canvas');
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    canvas.style.objectFit = 'contain';
    container.appendChild(canvas);

    // Default size
    canvas.width = 800;
    canvas.height = 600;
    const ctx = canvas.getContext('2d');

    // Load Static Map Image & Metadata
    try {
        const [imgRes, metaRes] = await Promise.all([
            fetch(`${API_BASE_URL}/api/navigation/maps/${mapName}/image`),
            fetch(`${API_BASE_URL}/api/navigation/maps/${mapName}/metadata`)
        ]);

        if (imgRes.ok && metaRes.ok) {
            const blob = await imgRes.blob();
            const imgBitmap = await createImageBitmap(blob);
            const meta = await metaRes.json();
            currentMapMeta = meta; // Store for global access

            // Load Waypoints
            await loadWaypoints(mapName);

            // Set Canvas Size to Match Map
            canvas.width = meta.img_width || imgBitmap.width; // Fallback to image size
            canvas.height = meta.img_height || imgBitmap.height;

            // Start Animation Loop for Robot Position
            startNavLoop(ctx, canvas, imgBitmap, meta);
        } else {
            alert("Failed to load map data");
            document.getElementById('nav-interface').style.display = 'none';
        }
    } catch (e) {
        console.error("Error loading map:", e);
        alert("Error loading map");
    }

    // Initialize Joystick for Nav Mode
    initJoystick(); // Enable Keyboard
    initNavJoystickUI(); // Enable UI Joystick
    initMapInteraction(canvas); // Enable Zoom/Pan
}

window.openRViz = async function () {
    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/nav/open_rviz`, { method: 'POST' });
        console.log("RViz Launch requested");
    } catch (e) { console.error(e); }
}

// --- Map Rendering & Loop ---
let navLoopInterval = null;
let mapScale = 1.0;
let mapOffsetX = 0.0;
let mapOffsetY = 0.0;
let initialBaseScale = 1.0;
let initialOffsetX = 0.0;
let initialOffsetY = 0.0;

function startNavLoop(ctx, canvas, mapImg, meta) {
    if (navLoopInterval) clearInterval(navLoopInterval);

    // Fix: Set Canvas resolution to Screen resolution
    const container = document.getElementById('nav-canvas-container');
    const cw = container.clientWidth || 800; // Fallback
    const ch = container.clientHeight || 600;
    canvas.width = cw;
    canvas.height = ch;

    // Calc Base Scale to Fit Map into Canvas
    const scaleX = cw / mapImg.width;
    const scaleY = ch / mapImg.height;

    // Global State Init
    initialBaseScale = Math.min(scaleX, scaleY) * 0.9; // 90% Fit
    initialOffsetX = (cw - (mapImg.width * initialBaseScale)) / 2;
    initialOffsetY = (ch - (mapImg.height * initialBaseScale)) / 2;

    // RViz Metadata (For Inverse Transform)
    window.activeMapMeta = meta;
    window.activeMapHeight = mapImg.height;

    // Reset interaction state

    // Reset interaction state
    resetMapView();

    navLoopInterval = setInterval(async () => {
        try {
            // Reset Transform Matrix to Identity (Safety)
            ctx.setTransform(1, 0, 0, 1, 0, 0);

            // Draw Map Background
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = "#1e1b4b"; // Dark BG
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            ctx.save();
            // Apply Map Interaction Transform
            // Note: mapOffsetX in this logic is Screen Pixels. mapScale is Total Scale.
            ctx.translate(mapOffsetX, mapOffsetY);
            ctx.scale(mapScale, mapScale);

            // Draw Image at (0,0) -> It will be scaled and translated
            ctx.drawImage(mapImg, 0, 0);

            // Invert Transform for drawing Robot/Waypoints on top of Map? 
            // NO, we want them attached to map frame. So keep transform active.
            // BUT: ctx.scale applies to their coordinates too.
            // We need to calculate their position relative to Map Image Origin (0,0 in context).

            // Get Robot Pose
            const res = await fetch(`${API_BASE_URL}/api/navigation/pose`);
            const pose = await res.json();

            // Common Metadata
            const resolution = meta.resolution || 0.05;
            const originX = meta.origin ? meta.origin[0] : 0;
            const originY = meta.origin ? meta.origin[1] : 0;

            if (pose && !pose.error && pose.x !== undefined) {

                const px = (pose.x - originX) / resolution;
                const py = mapImg.height - ((pose.y - originY) / resolution); // Use mapImg.height!

                // Visualize Robot
                ctx.save();
                ctx.translate(px, py);
                ctx.rotate(-pose.theta);

                // Draw Robot Arrow
                ctx.beginPath();
                ctx.moveTo(15, 0);
                ctx.lineTo(-10, 10);
                ctx.lineTo(-10, -10);
                ctx.closePath();
                ctx.fillStyle = '#00CCFF'; // Blue for Nav Mode
                ctx.fill();
                ctx.restore();
            }

            // Visualize Waypoints
            if (currentWaypoints.length > 0) {

                currentWaypoints.forEach(wp => {
                    const wx = (wp.x - originX) / resolution;
                    const wy = mapImg.height - ((wp.y - originY) / resolution);

                    // Draw Pin
                    // Scale Pin size to remain visible? 
                    // If we draw in scaled context, pin size also scales.
                    // To keep constant pin size on screen, divide radius by mapScale?
                    // Let's just draw normally for now.
                    ctx.beginPath();
                    ctx.arc(wx, wy, 8 / mapScale, 0, 2 * Math.PI); // Inverse Scale for consistent size
                    ctx.fillStyle = '#FFD700'; // Gold
                    ctx.fill();
                    ctx.strokeStyle = '#FFFFFF';
                    ctx.stroke();

                    // Draw Label
                    ctx.fillStyle = '#FFFFFF';
                    ctx.font = `${14 / mapScale}px Arial`; // Scale font
                    ctx.fillText(wp.name, wx + (10 / mapScale), wy + (5 / mapScale));
                });
            }



            // Draw RViz Overlays (Lidar, Interaction Arrows)
            drawRVizOverlays(ctx, mapImg, meta, pose);

            ctx.restore(); // Restore from Zoom/Pan
        } catch (e) { console.error(e); }
    }, 100); // 10Hz Smooth
}

window.closeNavInterface = function () {
    document.getElementById('nav-interface').style.display = 'none';
    if (navLoopInterval) clearInterval(navLoopInterval);
}

// --- Virtual Joystick UI ---
function initNavJoystickUI() {
    const joystick = document.getElementById('nav-joystick');
    const handle = joystick.querySelector('.joystick-handle');
    let isDragging = false;
    let startX, startY;

    const maxDist = 35; // px

    function startDrag(e) {
        isDragging = true;
        const clientX = e.touches ? e.touches[0].clientX : e.clientX;
        const clientY = e.touches ? e.touches[0].clientY : e.clientY;
        const rect = handle.getBoundingClientRect();
        // Center of handle relative to screen? 
        // Better: Center of joystick base.
        const baseRect = joystick.getBoundingClientRect();
        const centerX = baseRect.left + baseRect.width / 2;
        const centerY = baseRect.top + baseRect.height / 2;

        startX = centerX;
        startY = centerY;
    }

    function moveDrag(e) {
        if (!isDragging) return;
        e.preventDefault(); // Prevent scroll on touch

        const clientX = e.touches ? e.touches[0].clientX : e.clientX;
        const clientY = e.touches ? e.touches[0].clientY : e.clientY;

        let dx = clientX - startX;
        let dy = clientY - startY;

        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > maxDist) {
            const ratio = maxDist / dist;
            dx *= ratio;
            dy *= ratio;
        }

        handle.style.transform = `translate(${dx}px, ${dy}px)`;

        // Map to Velocity
        // dy < 0 is Forward (Vx > 0)
        // dx > 0 is Right (Vy < 0) or Turn? 
        // Standard Arcade: Y=Speed, X=Turn (Wz)
        // Or Holonomic: Y=Vx, X=Vy?
        // Let's assume Holonomic for Mecanum? Or Arcade?
        // Let's stick to Arcade for safety on phone? 
        // Actually Mecanum usually expects strafe.
        // Let's do: Y -> Vx, X -> Wz (Turn) for mobile ease.
        // Or if user wants Strafe, maybe X -> Vy?
        // "AI Robot Guide" usually differential/mecanum. Let's do Vx/Wz.

        const maxSpeed = 0.3;
        const vx = -(dy / maxDist) * maxSpeed;
        const wz = -(dx / maxDist) * maxSpeed * 2.0; // Turn faster

        sendVelocity(vx, 0, wz);
    }

    function endDrag() {
        if (!isDragging) return;
        isDragging = false;
        handle.style.transform = `translate(0px, 0px)`;
        sendVelocity(0, 0, 0);
    }

    // Mouse
    joystick.addEventListener('mousedown', startDrag);
    document.addEventListener('mousemove', moveDrag);
    document.addEventListener('mouseup', endDrag);

    // Touch
    joystick.addEventListener('touchstart', startDrag);
    document.addEventListener('touchmove', moveDrag, { passive: false });
    document.addEventListener('touchend', endDrag);
}

// --- RViz Tools State ---
let currentNavTool = 'none'; // 'none', 'estimate', 'goal'
let showLidar = false;
let lidarData = null;
let interactionStart = null; // {x, y} screen pixels
let interactionEnd = null;   // {x, y} screen pixels

window.setNavTool = function (tool) {
    // Toggle check
    if (currentNavTool === tool) tool = 'none';
    currentNavTool = tool;

    // UI Update
    document.getElementById('btn-tool-estimate').style.background = (tool === 'estimate') ? '#4f46e5' : 'rgba(255,255,255,0.1)';
    document.getElementById('btn-tool-estimate').style.color = (tool === 'estimate') ? '#fff' : '#ccc';

    document.getElementById('btn-tool-goal').style.background = (tool === 'goal') ? '#4f46e5' : 'rgba(255,255,255,0.1)';
    document.getElementById('btn-tool-goal').style.color = (tool === 'goal') ? '#fff' : '#ccc';

    document.getElementById('btn-tool-waypoint').style.background = (tool === 'waypoint') ? '#4f46e5' : 'rgba(255,255,255,0.1)';
    document.getElementById('btn-tool-waypoint').style.color = (tool === 'waypoint') ? '#fff' : '#ccc';
}

window.toggleLidar = function () {
    showLidar = !showLidar;
    document.getElementById('btn-toggle-lidar').style.background = showLidar ? '#4f46e5' : 'rgba(255,255,255,0.1)';
    document.getElementById('btn-toggle-lidar').style.color = showLidar ? '#fff' : '#ccc';
    if (showLidar) {
        // Start fetching lidar data periodically
        fetchLidarInterval = setInterval(fetchLidar, 200); // Fetch every 200ms
    } else {
        clearInterval(fetchLidarInterval);
        lidarData = null; // Clear data when hidden
    }
}
let fetchLidarInterval = null;

// Fetch Lidar Data
async function fetchLidar() {
    if (!showLidar) return;
    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/nav/scan`);
        if (res.ok) lidarData = await res.json();
    } catch (e) { }
}

// Draw RViz Overlays (Called inside startNavLoop)
function drawRVizOverlays(ctx, mapImg, meta, pose) {
    if (showLidar && lidarData && pose && !pose.error) {
        drawLidar(ctx, lidarData, pose, meta, mapImg);
    }

    if (interactionStart && interactionEnd) {
        drawInteractionArrow(ctx, interactionStart, interactionEnd);
    }
}

function drawLidar(ctx, scan, pose, meta, mapImg) {
    // 1. Transform Robot Pose to Pixel Coords
    const resolution = meta.resolution || 0.05;
    const originX = meta.origin ? meta.origin[0] : 0;
    const originY = meta.origin ? meta.origin[1] : 0;

    // Robot Pos in Pixels/Map Frame
    // BUT we are already in Transformed Context (translated/scaled) IF called inside ctx.save()
    // Wait, startNavLoop calls ctx.save() -> transform -> drawMap.
    // If we draw here, we assume we are in Map Coordinate Space (Pixels).

    // Robot Pose (World) -> Map Pixels
    const rx_px = (pose.x - originX) / resolution;
    const ry_px = mapImg.height - ((pose.y - originY) / resolution);
    const th = -pose.theta; // Canvas Y is inverted relative to ROS Y?
    // ROS: X right, Y up. Canvas: X right, Y down.
    // So +Y ROS = -Y Canvas.
    // Rotation: ROS CCW is positive. Canvas CW is positive.
    // So theta should be negated.

    ctx.save();
    ctx.translate(rx_px, ry_px);
    ctx.rotate(th);

    ctx.fillStyle = 'rgba(255, 0, 0, 0.5)';

    // Scan Points are in Robot Frame (X forward)
    // Canvas Robot Frame: X right (if rotated by theta).
    // Loop ranges
    const angle_min = scan.angle_min;
    const angle_inc = scan.angle_increment;

    // Optimization: Draw simply
    for (let i = 0; i < scan.ranges.length; i++) {
        const r = scan.ranges[i];
        if (r < 0.1 || r > scan.max_range) continue;

        const a = angle_min + (i * angle_inc);
        // Polar to Cartesian (Robot Frame)
        // ROS: x = r*cos(a), y = r*sin(a)
        // Canvas: x = x, y = -y (because Y axis is down)
        const lx = r * Math.cos(a) / resolution;
        const ly = -(r * Math.sin(a)) / resolution;

        ctx.fillRect(lx, ly, 2, 2);
    }
    ctx.restore();
}

function drawInteractionArrow(ctx, start, end) {
    // Draw arrow from start to end (Screen Coords transformed to Map Coords? NO)
    // Interaction is in SCREEN PIXELS. 
    // We should draw this AFTER ctx.restore() in main loop, OR Transform back.
    // EASIER: Draw in screen space.
    // BUT startNavLoop applies transform.
    // Let's modify startNavLoop to call this AFTER restore.
    // This function is called *before* ctx.restore(), so it's in map-transformed space.
    // We need to convert screen coords (start, end) to map-transformed coords.

    // Convert screen coordinates to map-transformed coordinates
    const mapStart = {
        x: (start.x - mapOffsetX) / mapScale,
        y: (start.y - mapOffsetY) / mapScale
    };
    const mapEnd = {
        x: (end.x - mapOffsetX) / mapScale,
        y: (end.y - mapOffsetY) / mapScale
    };

    ctx.save();
    ctx.beginPath();
    ctx.moveTo(mapStart.x, mapStart.y);
    ctx.lineTo(mapEnd.x, mapEnd.y);
    ctx.strokeStyle = '#FFFF00'; // Yellow arrow
    ctx.lineWidth = 3 / mapScale; // Consistent line width
    ctx.stroke();

    // Draw arrowhead
    const headlen = 15 / mapScale; // length of head in pixels
    const angle = Math.atan2(mapEnd.y - mapStart.y, mapEnd.x - mapStart.x);
    ctx.lineTo(mapEnd.x - headlen * Math.cos(angle - Math.PI / 6), mapEnd.y - headlen * Math.sin(angle - Math.PI / 6));
    ctx.moveTo(mapEnd.x, mapEnd.y);
    ctx.lineTo(mapEnd.x - headlen * Math.cos(angle + Math.PI / 6), mapEnd.y - headlen * Math.sin(angle + Math.PI / 6));
    ctx.stroke();
    ctx.restore();
}

// --- Modified Interaction Logic ---
// We need to inject logic into canvas events.
// This replaces old initMapInteraction partially or wraps it.

function handleToolInteract(action, e, canvas) {
    if (currentNavTool === 'none') {
        // Fallback to Pan/Zoom
        return false; // Not handled
    }

    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    if (action === 'down') {
        interactionStart = { x, y };
        interactionEnd = { x, y };
        // Disable Pan
        return true;
    }
    else if (action === 'move') {
        if (interactionStart) {
            interactionEnd = { x, y };
            return true;
        }
    }
    else if (action === 'up') {
        if (interactionStart) {
            // FINALIZE ACTION
            const dx = interactionEnd.x - interactionStart.x;
            const dy = interactionEnd.y - interactionStart.y; // Screen Y

            // Calculate Theta (Screen Space)
            // Start -> End vector.
            // Canvas: 0 is Right. 90 is Down.
            // We want ROS Theta.
            // ROS: 0 is Right. 90 is Up.
            // So angle = -atan2(dy, dx)
            const angle = -Math.atan2(dy, dx);

            // Calculate World Position of START point
            // We need to inverse transform (Screen -> Map Pixels -> World)
            // Screen (sx, sy) -> Map Pixels (mx, my)
            // mx = (sx - mapOffsetX) / mapScale
            // my = (sy - mapOffsetY) / mapScale
            const mx = (interactionStart.x - mapOffsetX) / mapScale;
            const my = (interactionStart.y - mapOffsetY) / mapScale;

            // Map Pixels -> World (wx, wy)
            // wx = (mx * res) + originX
            // wy = originY + (height - my) * res  <-- Inverse of: my = height - (wy-oy)/res
            // Wait: my = H - (dy/res) => dy/res = H - my => dy = (H-my)*res => wy = oy + dy

            // Get meta from global or pass it? Global `meta` from startNavLoop?
            // Let's assume meta is stored in window or accessible.
            if (!window.activeMapMeta) {
                alert("Error: Map Metadata not loaded. Please reloading the map.");
                return false;
            }

            if (activeMapMeta) {
                const res = activeMapMeta.resolution;
                const ox = activeMapMeta.origin[0];
                const oy = activeMapMeta.origin[1];
                const h = activeMapHeight; // We need Map Height in PIXELS

                // Oops, meta.height might be missing or meters.
                // Use mapImg.height if available context?
                // Let's attach meta to window.activeMapMeta in startNavLoop.

                const wx = (mx * res) + ox;
                const wy = oy + (h - my) * res;

                // EXECUTE COMMAND
                if (currentNavTool === 'estimate') {
                    sendInitialPose(wx, wy, angle);
                } else if (currentNavTool === 'goal') {
                    sendNavGoal(wx, wy, angle);
                } else if (currentNavTool === 'waypoint') {
                    saveClickedWaypoint(wx, wy, angle);
                }
            }

            interactionStart = null;
            interactionEnd = null;
            // Reset tool? Maybe keep it active for multiple clicks.
            setNavTool('none'); // Auto-deselect for safety
            return true;
        }
    }
    return false;
}

async function sendInitialPose(x, y, theta) {
    try {
        await fetch(`${API_BASE_URL}/api/navigation/nav/initial_pose`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, theta })
        });
        console.log("Sent Initial Pose:", x, y, theta);
    } catch (e) { console.error(e); }
}

async function sendNavGoal(x, y, theta) {
    try {
        await fetch(`${API_BASE_URL}/api/navigation/goto`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, theta })
        });
        console.log("Sent Nav Goal:", x, y, theta);
    } catch (e) { console.error(e); }
}



async function saveClickedWaypoint(x, y, theta) {
    const mapName = document.getElementById('nav-map-name').innerText;
    const name = prompt("ตั้งชื่อจุด (WayPoint Name):");
    if (!name) return;

    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/waypoints`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                map_name: mapName,
                name: name,
                x: x,
                y: y,
                theta: theta
            })
        });

        if (res.ok) {
            loadWaypoints(mapName);
            // alert("Saved Waypoint: " + name); // Optional: less intrusive
        } else {
            alert("Failed to save waypoint");
        }
    } catch (e) { alert("Error: " + e); }
}

// --- Map Interaction (Zoom/Pan) ---
function initMapInteraction(canvas) {
    let isDragging = false;
    let startX, startY;

    canvas.onmousedown = (e) => {
        if (handleToolInteract('down', e, canvas)) {
            return; // Tool handled the event
        }
        isDragging = true;
        startX = e.clientX - mapOffsetX;
        startY = e.clientY - mapOffsetY;
        canvas.style.cursor = 'grabbing';
    };

    canvas.onmousemove = (e) => {
        if (handleToolInteract('move', e, canvas)) return;
        if (!isDragging) return;
        mapOffsetX = e.clientX - startX;
        mapOffsetY = e.clientY - startY;
        clampMap(canvas);
    };

    canvas.onmouseup = () => {
        if (handleToolInteract('up', e, canvas)) return;
        isDragging = false;
        canvas.style.cursor = 'grab';
    };

    canvas.onmouseleave = () => { isDragging = false; };

    canvas.onwheel = (e) => {
        e.preventDefault();
        const zoomIntensity = 0.001; // Less sensitive
        const delta = -e.deltaY * zoomIntensity;
        const newScale = mapScale * (1 + delta); // Multiplicative Zoom for smoothness

        // Limit Zoom
        if (newScale < 0.1 || newScale > 20.0) return;

        // Zoom towards mouse pointer
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        // OffsetNew = Mouse - (Mouse - OffsetOld) * (NewScale / OldScale)
        mapOffsetX = mouseX - (mouseX - mapOffsetX) * (newScale / mapScale);
        mapOffsetY = mouseY - (mouseY - mapOffsetY) * (newScale / mapScale);

        mapScale = newScale;
        clampMap(canvas);
    };

    canvas.style.cursor = 'grab';
}

function clampMap(canvas) {
    // Prevent map from flying off screen
    // Simplified constraint: Center of map must be within canvas bounds?
    // Or at least one corner visible?

    // Let's ensure at least 50px of map is visible
    const margin = 50;

    // Bounds check not strictly necessary if reset exists, but good for UX.
    // Implementing loose bounds:
    // We allow panning freely but maybe stop if ridiculous.
    if (Math.abs(mapOffsetX) > 5000) mapOffsetX = 0;
    if (Math.abs(mapOffsetY) > 5000) mapOffsetY = 0;
}

window.resetMapView = function () {
    mapScale = initialBaseScale;
    mapOffsetX = initialOffsetX;
    mapOffsetY = initialOffsetY;
}

// --- Waypoint Management ---

async function loadWaypoints(mapName) {
    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/waypoints/${mapName}`);
        const data = await res.json();
        currentWaypoints = data.waypoints || [];
        renderWaypointList(mapName);
    } catch (e) { console.error("Load WP Error:", e); }
}

function renderWaypointList(mapName) {
    const list = document.getElementById('waypoint-list');
    list.innerHTML = '';

    if (currentWaypoints.length === 0) {
        list.innerHTML = '<div style="color:#aaa; text-align:center; padding:10px;">No waypoints saved.</div>';
        return;
    }

    currentWaypoints.forEach(wp => {
        const div = document.createElement('div');
        div.style.cssText = 'background:rgba(255,255,255,0.1); margin-bottom:5px; padding:8px; border-radius:4px; display:flex; justify-content:space-between; align-items:center;';
        div.innerHTML = `
            <span style="color:white; cursor:pointer;" onclick="gotoWaypoint('${wp.name}', ${wp.x}, ${wp.y}, ${wp.theta})">
                <i class="fa-solid fa-location-dot" style="color:#FFD700; margin-right:5px;"></i> ${wp.name}
            </span>
            <button onclick="deleteWaypoint('${mapName}', '${wp.name}')" style="background:none; border:none; color:#ef4444; cursor:pointer;">
                <i class="fa-solid fa-trash"></i>
            </button>
        `;
        list.appendChild(div);
    });
}

window.saveRobotPose = async function () {
    const mapName = document.getElementById('nav-map-name').innerText;
    const name = prompt("ตั้งชื่อจุด (เช่น Kitchen, Dock):");
    if (!name) return;

    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/waypoints/save_pose`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map_name: mapName, point_name: name })
        });

        if (res.ok) {
            loadWaypoints(mapName); // Refresh list
        } else {
            alert("บันทึกไม่สำเร็จ (ตรวจสอบว่า Robot Pose ใช้ได้หรือไม่)");
        }
    } catch (e) { alert("Error saving waypoint: " + e); }
}


window.importRVizGoal = async function () {
    const mapName = document.getElementById('nav-map-name').innerText;
    const name = prompt("ตั้งชื่อจุดจาก RViz (WayPoint Name):");
    if (!name) return;

    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/waypoints/save_rviz`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map_name: mapName, point_name: name })
        });

        if (res.ok) {
            loadWaypoints(mapName);
            alert("✓ Imported Waypoint: " + name);
        } else {
            const err = await res.json();
            alert("Import Failed: " + (err.detail || "Unknown Error"));
        }
    } catch (e) { alert("Error: " + e); }
}

window.deleteWaypoint = async function (mapName, pointName) {
    if (!confirm(`ลบจุด "${pointName}" ?`)) return;
    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/waypoints/${mapName}/${pointName}`, { method: 'DELETE' });
        if (res.ok) loadWaypoints(mapName);
    } catch (e) { alert("Error deleting: " + e); }
}

window.gotoWaypoint = async function (name, x, y, theta) {
    if (!confirm(`เดินไปที่ "${name}" ?`)) return;
    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/goto`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, theta })
        });
        if (res.ok) {
            // Optional: Show status
        }
    } catch (e) { alert("Navigation Error: " + e); }
}



async function startAutoWalk() {
    const mapName = document.getElementById('nav-map-name').innerText;
    if (!confirm(`เริ่มเดินอัตโนมัติสำหรับแผนที่ "${mapName}"?`)) return;

    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/autowalk/${mapName}`, { method: 'POST' });
        if (res.ok) {
            document.getElementById('btn-autowalk').style.display = 'none';
            document.getElementById('btn-stopwalk').style.display = 'block';
            document.getElementById('autowalk-status-bar').style.display = 'block';
            console.log("Auto Walk started");
        } else {
            const err = await res.json();
            alert("เริ่มเดินอัตโนมัติไม่สำเร็จ: " + (err.detail || "Unknown Error"));
        }
    } catch (e) { alert("Error: " + e); }
}
window.startAutoWalk = startAutoWalk;

async function stopAutoWalk() {
    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/stop`, { method: 'POST' });
        if (res.ok) {
            document.getElementById('btn-autowalk').style.display = 'block';
            document.getElementById('btn-stopwalk').style.display = 'none';
            document.getElementById('autowalk-status-bar').style.display = 'none';
            console.log("Auto Walk stopped");
        }
    } catch (e) { alert("Error stopping: " + e); }
}
window.stopAutoWalk = stopAutoWalk;

// Poll for Auto Walk Status (Optional but good for feedback)
setInterval(async () => {
    const statusDiv = document.getElementById('autowalk-status-bar');
    if (!statusDiv || statusDiv.offsetParent === null) return; // Only if visible

    try {
        const res = await fetch(`${API_BASE_URL}/api/navigation/pose`); // We check pose to see if it's alive, but better if we had an is_walking endpoint
        // For now, reliance on start/stop buttons is enough.
        // If we want real sequencing feedback, we need to add a specific endpoint to backend.
    } catch (e) { }
}, 2000);

// Close Button (Helper)
window.closeMapping = function () {
    document.getElementById('mapping-wizard').style.display = 'none';
}
