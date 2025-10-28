// // REAL LUNABOT DASHBOARD WITH SLAM - NO FAKE DATA
// console.log('🌐 REAL LunaBot SLAM Dashboard loading...');

// const socket = io();
// let isConnected = false;
// let cameraFrameCount = 0;
// let chart = null;

// // ═══════════════════════════════════════════════════════════════════
// // NEW: SLAM DATA STORAGE
// // ═══════════════════════════════════════════════════════════════════
// let slamData = {
//     mapImage: null,
//     semanticMap: {rocks: [], base: [], flags: [], antennas: []},
//     missionStatus: {
//         battery: 100.0,
//         mode: 'EXPLORATION',
//         base_x: 0.0,
//         base_y: 0.0,
//         distance_to_base: 0.0
//     }
// };

// document.addEventListener('DOMContentLoaded', function() {
//     console.log('📱 REAL SLAM Dashboard initializing...');
    
//     setupSocketEvents();
//     setupSliders();
//     initializeCamera();
//     initializeSLAMMap();  // NEW
//     initializeChart();
    
//     console.log('✅ REAL SLAM Dashboard ready');
// });

// // ═══════════════════════════════════════════════════════════════════
// // NEW: SLAM MAP INITIALIZATION
// // ═══════════════════════════════════════════════════════════════════
// function initializeSLAMMap() {
//     const canvas = document.getElementById('slamMapCanvas');
//     if (canvas) {
//         const ctx = canvas.getContext('2d');
//         drawSLAMPlaceholder(ctx, canvas.width, canvas.height);
//         console.log('✅ SLAM map canvas initialized');
//     }
// }

// function drawSLAMPlaceholder(ctx, width, height) {
//     ctx.fillStyle = '#1a1a1a';
//     ctx.fillRect(0, 0, width, height);
    
//     ctx.fillStyle = '#ffffff';
//     ctx.font = '16px Arial';
//     ctx.textAlign = 'center';
//     ctx.fillText('🗺️ Waiting for SLAM map...', width/2, height/2 - 10);
//     ctx.font = '12px Arial';
//     ctx.fillText('Building occupancy grid', width/2, height/2 + 15);
// }

// // REAL CHART INITIALIZATION
// function initializeChart() {
//     console.log('📈 Initializing REAL path chart...');
    
//     const mapDiv = document.getElementById('pathMap');
//     if (mapDiv && typeof Plotly !== 'undefined') {
//         try {
//             const data = [{
//                 x: [0],
//                 y: [0],
//                 mode: 'lines+markers',
//                 type: 'scatter',
//                 name: 'Rover Path',
//                 line: {color: '#00ff88', width: 3},
//                 marker: {color: '#00ff88', size: 6}
//             }];
            
//             const layout = {
//                 title: 'Real Rover Path',
//                 xaxis: {title: 'X Position (m)', color: 'white'},
//                 yaxis: {title: 'Y Position (m)', color: 'white'},
//                 paper_bgcolor: 'rgba(0,0,0,0)',
//                 plot_bgcolor: 'rgba(0,0,0,0.2)',
//                 font: {color: 'white'},
//                 showlegend: false
//             };
            
//             Plotly.newPlot(mapDiv, data, layout);
//             chart = mapDiv;
//             console.log('✅ REAL chart initialized');
//         } catch (error) {
//             console.error('❌ Chart initialization error:', error);
//         }
//     }
// }

// function setupSocketEvents() {
//     console.log('📡 Setting up REAL socket events with SLAM...');
    
//     socket.on('connect', function() {
//         console.log('✅ Connected to REAL server');
//         isConnected = true;
//         updateConnectionDisplay('🟢 Connected (REAL)');
//         addLog('🌐 Connected to REAL server with SLAM');
//     });
    
//     socket.on('disconnect', function() {
//         console.log('❌ Disconnected from REAL server');
//         isConnected = false;
//         updateConnectionDisplay('🔴 Disconnected');
//         addLog('🌐 Disconnected from REAL server');
//     });
    
//     socket.on('initial_state', function(data) {
//         console.log('📊 REAL initial state received:', data);
//         if (data.rover_data) {
//             updateRoverData(data.rover_data);
//         }
//         if (data.slam_data) {
//             updateSLAMData(data.slam_data);
//         }
//     });
    
//     socket.on('camera_update', function(data) {
//         console.log('📷 REAL camera update received');
//         updateCameraFrame(data);
//         cameraFrameCount++;
        
//         const cameraStatusEl = document.getElementById('cameraStatusDisplay');
//         if (cameraStatusEl) {
//             cameraStatusEl.textContent = '🟢 Online (REAL)';
//             cameraStatusEl.style.color = '#00ff88';
//         }
//     });
    
//     socket.on('rover_status', function(data) {
//         console.log('📊 REAL rover status received:', data);
//         updateRoverData(data);
//         updateChart(data);
//     });
    
//     // ═══════════════════════════════════════════════════════════════
//     // NEW: SLAM SOCKET EVENTS
//     // ═══════════════════════════════════════════════════════════════
//     socket.on('map_update', function(data) {
//         console.log('🗺️ SLAM map update received');
//         updateSLAMMapImage(data);
//     });
    
//     socket.on('semantic_update', function(data) {
//         console.log('🔍 Semantic map update received');
//         updateSemanticMap(data);
//     });
    
//     socket.on('mission_update', function(data) {
//         console.log('🎯 Mission status update received');
//         updateMissionStatus(data);
//     });
    
//     socket.on('path_update', function(data) {
//         console.log('🛤️ Path history update received');
//         // Path already updated via rover_status
//     });
//     // ═══════════════════════════════════════════════════════════════
    
//     socket.on('activity_update', function(data) {
//         console.log('📝 REAL activity update:', data);
//         if (data.message) {
//             addLog(data.message);
//         }
//     });
    
//     socket.on('command_success', function(data) {
//         console.log('✅ REAL command success:', data);
//         addLog('✅ REAL: ' + data.message);
//     });
    
//     socket.on('command_error', function(data) {
//         console.log('❌ REAL command error:', data);
//         addLog('❌ REAL: ' + data.error);
//     });
    
//     console.log('✅ REAL socket events configured with SLAM');
// }

// // ═══════════════════════════════════════════════════════════════════
// // NEW: SLAM DATA UPDATE FUNCTIONS
// // ═══════════════════════════════════════════════════════════════════
// function updateSLAMData(data) {
//     console.log('🗺️ Updating SLAM data:', data);
    
//     if (data.map_available && slamData.mapImage === null) {
//         // Request initial map
//         fetch('/api/map_image')
//             .then(response => response.json())
//             .then(result => {
//                 if (result.status === 'success') {
//                     updateSLAMMapImage({map: result.map});
//                 }
//             })
//             .catch(err => console.error('Map fetch error:', err));
//     }
    
//     if (data.semantic_map) {
//         slamData.semanticMap = data.semantic_map;
//         updateSemanticDisplay();
//     }
    
//     if (data.mission_status) {
//         slamData.missionStatus = data.mission_status;
//         updateMissionDisplay();
//     }
// }

// function updateSLAMMapImage(data) {
//     console.log('🗺️ Updating SLAM map image');
    
//     try {
//         const canvas = document.getElementById('slamMapCanvas');
//         if (!canvas || !data.map) return;
        
//         const ctx = canvas.getContext('2d');
//         const img = new Image();
        
//         img.onload = function() {
//             ctx.clearRect(0, 0, canvas.width, canvas.height);
//             ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            
//             // Add timestamp overlay
//             ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
//             ctx.fillRect(5, 5, 200, 20);
//             ctx.fillStyle = '#00ff88';
//             ctx.font = '12px monospace';
//             ctx.fillText(`SLAM MAP ${new Date().toLocaleTimeString()}`, 10, 18);
            
//             console.log('✅ SLAM map updated');
            
//             // Update status
//             const statusEl = document.getElementById('mapStatus');
//             if (statusEl) {
//                 statusEl.textContent = '🟢 Map Active';
//                 statusEl.style.color = '#00ff88';
//             }
//         };
        
//         img.onerror = function() {
//             console.error('❌ SLAM map image load error');
//             drawSLAMPlaceholder(ctx, canvas.width, canvas.height);
//         };
        
//         // Handle base64 data
//         if (data.map.startsWith('data:image')) {
//             img.src = data.map;
//         } else {
//             img.src = 'data:image/jpeg;base64,' + data.map;
//         }
        
//         slamData.mapImage = data.map;
        
//     } catch (error) {
//         console.error('❌ SLAM map update error:', error);
//     }
// }

// function updateSemanticMap(data) {
//     console.log('🔍 Updating semantic map');
    
//     if (data.semantic_map) {
//         slamData.semanticMap = data.semantic_map;
//     }
    
//     if (data.counts) {
//         // Update detection counts display
//         const rocksEl = document.getElementById('rocksCount');
//         const baseEl = document.getElementById('baseCount');
//         const flagsEl = document.getElementById('flagsCount');
        
//         if (rocksEl) rocksEl.textContent = data.counts.rocks;
//         if (baseEl) baseEl.textContent = data.counts.base;
//         if (flagsEl) flagsEl.textContent = data.counts.flags;
        
//         console.log(`🔍 Detections: ${data.counts.rocks} rocks, ${data.counts.base} base, ${data.counts.flags} flags`);
//     }
// }

// function updateSemanticDisplay() {
//     const counts = {
//         rocks: slamData.semanticMap.rocks?.length || 0,
//         base: slamData.semanticMap.base?.length || 0,
//         flags: slamData.semanticMap.flags?.length || 0
//     };
    
//     const rocksEl = document.getElementById('rocksCount');
//     const baseEl = document.getElementById('baseCount');
//     const flagsEl = document.getElementById('flagsCount');
    
//     if (rocksEl) rocksEl.textContent = counts.rocks;
//     if (baseEl) baseEl.textContent = counts.base;
//     if (flagsEl) flagsEl.textContent = counts.flags;
// }

// function updateMissionStatus(data) {
//     console.log('🎯 Updating mission status');
    
//     if (data.mission_status) {
//         slamData.missionStatus = data.mission_status;
//         updateMissionDisplay();
//     }
// }

// function updateMissionDisplay() {
//     const status = slamData.missionStatus;
    
//     // Update mode
//     const modeEl = document.getElementById('missionMode');
//     if (modeEl) {
//         modeEl.textContent = status.mode || 'EXPLORATION';
        
//         // Color coding
//         if (status.mode === 'EMERGENCY_RETURN') {
//             modeEl.style.color = '#ff4757';
//         } else if (status.mode === 'BACKTRACKING') {
//             modeEl.style.color = '#ffdd44';
//         } else {
//             modeEl.style.color = '#00ff88';
//         }
//     }
    
//     // Update battery
//     const batteryEl = document.getElementById('missionBattery');
//     const batteryIndicator = document.getElementById('batteryIndicator');
//     const battery = status.battery || 100;
    
//     if (batteryEl) {
//         batteryEl.textContent = `${battery.toFixed(0)}%`;
        
//         if (battery < 15) {
//             batteryEl.style.color = '#ff4757';
//         } else if (battery < 30) {
//             batteryEl.style.color = '#ffdd44';
//         } else {
//             batteryEl.style.color = '#00ff88';
//         }
//     }
    
//     if (batteryIndicator) {
//         batteryIndicator.textContent = `${battery.toFixed(0)}%`;
        
//         if (battery < 15) {
//             batteryIndicator.classList.add('low');
//             batteryIndicator.style.color = '#ff4757';
//         } else {
//             batteryIndicator.classList.remove('low');
//             batteryIndicator.style.color = '#00ff88';
//         }
//     }
    
//     // Update distance to base
//     const distEl = document.getElementById('distanceToBase');
//     if (distEl) {
//         const dist = status.distance_to_base || 0;
//         distEl.textContent = `${dist.toFixed(1)}m`;
//     }
// }
// // ═══════════════════════════════════════════════════════════════════

// // UPDATE CHART WITH REAL PATH DATA
// function updateChart(data) {
//     if (chart && data.path && data.path.length > 0) {
//         try {
//             const x = data.path.map(p => p.x);
//             const y = data.path.map(p => p.y);
            
//             Plotly.restyle(chart, {
//                 x: [x],
//                 y: [y]
//             }, 0);
            
//             console.log(`📈 Chart updated with ${data.path.length} real points`);
//         } catch (error) {
//             console.error('❌ Chart update error:', error);
//         }
//     }
// }

// // NAVIGATION FUNCTIONS (YOUR EXISTING CODE - UNCHANGED)
// function moveForward() {
//     console.log('🎮 REAL moveForward() called');
//     sendRoverCommand(0.5, 0, 2, 'Forward');
// }

// function moveBackward() {
//     console.log('🎮 REAL moveBackward() called');
//     sendRoverCommand(-0.5, 0, 2, 'Backward');
// }

// function moveLeft() {
//     console.log('🎮 REAL moveLeft() - IN-PLACE TURN');
//     sendInPlaceTurn('left', 10);
// }

// function moveRight() {
//     console.log('🎮 REAL moveRight() - IN-PLACE TURN');
//     sendInPlaceTurn('right', 10);
// }

// function sendInPlaceTurn(direction, degrees) {
//     const degreesPerSecond = 100;
//     const duration = Math.max(0.15, degrees / degreesPerSecond);
//     const angularVelocity = direction === 'left' ? 0.8 : -0.8;
    
//     console.log(`🔄 ${direction} in-place turn: ${degrees}° in ${duration.toFixed(2)}s`);
//     sendRoverCommand(0, angularVelocity, duration, `In-Place ${degrees}° ${direction}`);
// }

// function emergencyStop() {
//     console.log('🛑 REAL emergencyStop() called');
    
//     const stopCommand = {
//         cmd_vel: {
//             linear: { x: 0, y: 0, z: 0 },
//             angular: { x: 0, y: 0, z: 0 }
//         },
//         duration: 0.1
//     };
    
//     for (let i = 0; i < 3; i++) {
//         setTimeout(() => {
//             if (socket && socket.connected) {
//                 socket.emit('rover_command', stopCommand);
//             }
//         }, i * 100);
//     }
    
//     addLog('🛑 REAL EMERGENCY STOP');
// }

// function sendCustomCommand() {
//     console.log('🚀 REAL sendCustomCommand() called');
    
//     try {
//         const linearSlider = document.getElementById('customLinear');
//         const angularSlider = document.getElementById('customAngular');
//         const durationInput = document.getElementById('customDuration');
        
//         const linear = parseFloat(linearSlider?.value || 0);
//         const angular = parseFloat(angularSlider?.value || 0);
//         const duration = parseFloat(durationInput?.value || 1);
        
//         sendRoverCommand(linear, angular, duration, 'Custom');
        
//     } catch (error) {
//         console.error('❌ REAL custom command error:', error);
//         addLog(`❌ REAL custom error: ${error.message}`);
//     }
// }

// function sendRoverCommand(linear, angular, duration, name) {
//     console.log(`🎮 REAL ${name}: L=${linear}, A=${angular}, D=${duration}`);
    
//     try {
//         const command = {
//             cmd_vel: {
//                 linear: { x: linear, y: 0, z: 0 },
//                 angular: { x: 0, y: 0, z: angular }
//             },
//             duration: duration
//         };
        
//         if (socket && socket.connected) {
//             socket.emit('rover_command', command);
//             console.log('✅ REAL command sent via socket');
//             addLog(`🎮 REAL ${name} sent to ROS`);
//         } else {
//             console.warn('⚠️ Socket not connected to REAL server');
//             addLog('❌ Not connected - REAL command failed');
//         }
        
//     } catch (error) {
//         console.error('❌ REAL command error:', error);
//         addLog(`❌ REAL command error: ${error.message}`);
//     }
// }

// // Goal functions
// function sendGoal(x, y) {
//     console.log(`🎯 REAL sendGoal(${x}, ${y}) called`);
    
//     try {
//         const goalCommand = {
//             goal: {
//                 position: { x: parseFloat(x), y: parseFloat(y), z: 0 },
//                 orientation: { x: 0, y: 0, z: 0, w: 1 }
//             }
//         };
        
//         if (socket && socket.connected) {
//             socket.emit('navigation_goal', goalCommand);
//             addLog(`🎯 REAL goal: (${x}, ${y})`);
//         } else {
//             addLog('❌ Not connected - cannot send REAL goal');
//         }
        
//     } catch (error) {
//         console.error('❌ REAL goal error:', error);
//         addLog(`❌ REAL goal error: ${error.message}`);
//     }
// }

// function sendCustomGoal() {
//     console.log('🎯 REAL sendCustomGoal() called');
    
//     const xInput = document.getElementById('goalX');
//     const yInput = document.getElementById('goalY');
    
//     if (xInput && yInput) {
//         const x = parseFloat(xInput.value || 0);
//         const y = parseFloat(yInput.value || 0);
//         sendGoal(x, y);
//     }
// }

// // Test functions
// function testCamera() {
//     console.log('📷 REAL testCamera() called');
//     addLog('📷 Testing REAL camera...');
    
//     if (socket && socket.connected) {
//         socket.emit('test_connection');
//     } else {
//         addLog('❌ Not connected - cannot test REAL camera');
//     }
// }

// function testConnection() {
//     console.log('🔗 REAL testConnection() called');
//     addLog('🔗 Testing REAL ROS connection...');
    
//     if (socket && socket.connected) {
//         socket.emit('test_connection');
//     } else {
//         addLog('❌ Socket not connected to REAL server');
//     }
// }

// // Data update functions
// function updateRoverData(data) {
//     console.log('📊 Updating REAL rover data:', data);
    
//     try {
//         if (data.position) {
//             const posEl = document.getElementById('positionDisplay');
//             if (posEl) {
//                 posEl.textContent = `X: ${data.position.x.toFixed(2)}, Y: ${data.position.y.toFixed(2)}`;
//             }
//         }
        
//         if (data.velocity) {
//             const velEl = document.getElementById('velocityDisplay');
//             if (velEl) {
//                 const linear = data.velocity.linear_x || data.velocity.linear || 0;
//                 const angular = data.velocity.angular_z || data.velocity.angular || 0;
//                 velEl.textContent = `L: ${linear.toFixed(2)}, A: ${angular.toFixed(2)}`;
//             }
//         }
        
//         if (data.mode) {
//             const modeEl = document.getElementById('modeDisplay');
//             if (modeEl) {
//                 modeEl.textContent = data.mode + ' (REAL)';
                
//                 if (data.mode.includes('MOVING') || data.mode.includes('MANUAL')) {
//                     modeEl.style.color = '#00ff88';
//                 } else if (data.mode.includes('STOPPED') || data.mode.includes('EMERGENCY')) {
//                     modeEl.style.color = '#ff4757';
//                 } else {
//                     modeEl.style.color = '#ffffff';
//                 }
//             }
//         }
        
//         if (data.connected !== undefined) {
//             updateConnectionDisplay(data.connected ? '🟢 Connected (REAL ROS)' : '🔴 ROS Disconnected');
//         }
        
//         console.log('✅ REAL rover data updated');
        
//     } catch (error) {
//         console.error('❌ REAL data update error:', error);
//     }
// }

// function updateCameraFrame(data) {
//     console.log('📷 Updating REAL camera frame');
    
//     try {
//         const canvas = document.getElementById('cameraCanvas');
//         if (!canvas || !data.data) return;
        
//         const ctx = canvas.getContext('2d');
//         const img = new Image();
        
//         img.onload = function() {
//             ctx.clearRect(0, 0, canvas.width, canvas.height);
//             ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            
//             ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
//             ctx.fillRect(5, 5, 200, 20);
//             ctx.fillStyle = '#00ff00';
//             ctx.font = '12px monospace';
//             ctx.fillText(`REAL LIVE ${new Date().toLocaleTimeString()}`, 10, 18);
            
//             console.log('✅ REAL camera frame updated');
//         };
        
//         img.onerror = function() {
//             console.error('❌ REAL camera image load error');
//             drawCameraPlaceholder(canvas);
//         };
        
//         if (data.data.startsWith('data:image')) {
//             img.src = data.data;
//         } else {
//             img.src = 'data:image/jpeg;base64,' + data.data;
//         }
        
//     } catch (error) {
//         console.error('❌ REAL camera update error:', error);
//     }
// }

// function initializeCamera() {
//     const canvas = document.getElementById('cameraCanvas');
//     if (canvas) {
//         drawCameraPlaceholder(canvas);
//         console.log('✅ REAL camera canvas initialized');
//     }
// }

// function drawCameraPlaceholder(canvas) {
//     const ctx = canvas.getContext('2d');
//     ctx.fillStyle = '#1a1a1a';
//     ctx.fillRect(0, 0, canvas.width, canvas.height);
    
//     ctx.fillStyle = '#ffffff';
//     ctx.font = '16px Arial';
//     ctx.textAlign = 'center';
//     ctx.fillText('📷 Waiting for REAL camera...', canvas.width/2, canvas.height/2 - 10);
//     ctx.font = '12px Arial';
//     ctx.fillText('(No fake data)', canvas.width/2, canvas.height/2 + 15);
// }

// function updateConnectionDisplay(status) {
//     const connectionEl = document.getElementById('connectionDisplay');
//     if (connectionEl) {
//         connectionEl.textContent = status;
        
//         if (status.includes('Connected')) {
//             connectionEl.style.color = '#00ff88';
//         } else {
//             connectionEl.style.color = '#ff4757';
//         }
//     }
// }

// function addLog(message) {
//     const timestamp = new Date().toLocaleTimeString();
//     const logEntry = `[${timestamp}] ${message}`;
    
//     const logContainer = document.getElementById('activityLog');
//     if (logContainer) {
//         logContainer.innerHTML = logEntry + '<br>' + logContainer.innerHTML;
        
//         const lines = logContainer.innerHTML.split('<br>');
//         if (lines.length > 25) {
//             logContainer.innerHTML = lines.slice(0, 25).join('<br>');
//         }
//     }
    
//     console.log(`📝 ${logEntry}`);
// }

// function setupSliders() {
//     const linearSlider = document.getElementById('customLinear');
//     const angularSlider = document.getElementById('customAngular');
//     const linearValue = document.getElementById('linearValue');
//     const angularValue = document.getElementById('angularValue');
    
//     if (linearSlider && linearValue) {
//         linearSlider.oninput = function() {
//             linearValue.textContent = parseFloat(this.value).toFixed(1);
//         };
//         linearSlider.oninput();
//     }
    
//     if (angularSlider && angularValue) {
//         angularSlider.oninput = function() {
//             angularValue.textContent = parseFloat(this.value).toFixed(1);
//         };
//         angularSlider.oninput();
//     }
    
//     console.log('✅ REAL sliders configured');
// }

// // Performance counters
// setInterval(() => {
//     const fpsEl = document.getElementById('fpsCounter');
//     const msgEl = document.getElementById('msgCounter');
    
//     if (fpsEl) fpsEl.textContent = cameraFrameCount;
//     if (msgEl) msgEl.textContent = isConnected ? 'REAL' : 'OFF';
    
//     cameraFrameCount = 0;
// }, 1000);

// console.log('✅ REAL SLAM Dashboard JavaScript loaded successfully');
// console.log('🎮 All buttons connect to REAL ROS');
// console.log('📷 Camera shows REAL feed only');
// console.log('🗺️ Position data is REAL odometry');
// console.log('📈 Chart shows REAL path');
// console.log('🗺️ SLAM map shows REAL occupancy grid');
// console.log('🔍 YOLO detections show REAL objects');
// console.log('🎯 Mission status shows REAL battery & mode');

// LUNABOT DASHBOARD WITH SLAM
// LUNABOT DASHBOARD WITH SLAM
console.log('🌐 LunaBot SLAM Dashboard loading...');

const socket = io();
let isConnected = false;
let cameraFrameCount = 0;
let chart = null;

let chartLayout = {}; // NEW: A global variable to store our chart's layout

// SLAM DATA STORAGE
let slamData = {
    mapImage: null,
    semanticMap: {rocks: [], base: [], flags: [], antennas: []},
    missionStatus: {
        battery: 100.0,
        mode: 'EXPLORATION',
        base_x: 0.0,
        base_y: 0.0,
        distance_to_base: 0.0
    }
};

document.addEventListener('DOMContentLoaded', function() {
    console.log('📱 SLAM Dashboard initializing...');
    
    setupSocketEvents();
    setupSliders();
    initializeCamera();
    initializeSLAMMap();
    initializeChart();
    
    console.log('✅ SLAM Dashboard ready');
});

// SLAM MAP INITIALIZATION
function initializeSLAMMap() {
    const canvas = document.getElementById('slamMapCanvas');
    if (canvas) {
        const ctx = canvas.getContext('2d');
        drawSLAMPlaceholder(ctx, canvas.width, canvas.height);
        console.log('✅ SLAM map canvas initialized');
    }
}

function drawSLAMPlaceholder(ctx, width, height) {
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);
    
    ctx.fillStyle = '#ffffff';
    ctx.font = '16px Arial';
    ctx.textAlign = 'center';
    ctx.fillText('🗺️ Waiting for SLAM map...', width/2, height/2 - 10);
    ctx.font = '12px Arial';
    ctx.fillText('Building occupancy grid', width/2, height/2 + 15);
}

// CHART INITIALIZATION
function initializeChart() {
    console.log('📈 Initializing advanced path chart...');

    const mapDiv = document.getElementById('pathMap');
    if (mapDiv && typeof Plotly !== 'undefined') {
        try {
            const pathTrace = {
                x: [0], y: [0], mode: 'lines', type: 'scatter', name: 'Path History',
                line: { color: '#848d97', width: 2 }
            };

            const roverMarkerTrace = {
                x: [0], y: [0], mode: 'markers', type: 'scatter', name: 'Current Position',
                marker: {
                    symbol: 'triangle-up', color: '#c5a347', size: 16,
                    line: { color: '#e6edf3', width: 2 }
                }
            };

            // **KEY CHANGE #1**: We now define the layout in our global variable.
            chartLayout = {
                xaxis: {
                    title: 'X Position (m)', color: '#848d97', gridcolor: 'rgba(197, 163, 71, 0.2)',
                    autorange: true, zeroline: false
                },
                yaxis: {
                    title: 'Y Position (m)', color: '#848d97', gridcolor: 'rgba(197, 163, 71, 0.2)',
                    scaleanchor: "x", scaleratio: 1, autorange: true, zeroline: false
                },
                paper_bgcolor: 'transparent',
                plot_bgcolor: 'transparent',
                font: { color: '#bdc6d3' },
                showlegend: false,
                margin: { l: 50, r: 20, b: 40, t: 20 }
            };

            const config = { responsive: true, displayModeBar: false };
            
            Plotly.newPlot(mapDiv, [pathTrace, roverMarkerTrace], chartLayout, config);
            
            chart = mapDiv;
            console.log('✅ Advanced chart initialized and layout saved.');

        } catch (error) {
            console.error('❌ Chart initialization error:', error);
        }
    }
}

// UPDATE CHART WITH PATH DATA
function updateChart(data) {
    if (!chart || !data.path || data.path.length === 0) {
        return;
    }

    try {
        const pathX = data.path.map(p => p.x);
        const pathY = data.path.map(p => p.y);
        const lastPoint = data.path[data.path.length - 1];
        
        let rotationAngle = 0;

        if (data.path.length > 1) {
            const prevPoint = data.path[data.path.length - 2];
            const dx = lastPoint.x - prevPoint.x;
            const dy = lastPoint.y - prevPoint.y;
            if (dx !== 0 || dy !== 0) {
                 const angleInRadians = Math.atan2(dy, dx);
                 rotationAngle = angleInRadians * (180 / Math.PI) - 90;
            }
        }

        // **KEY CHANGE #2**: We pass the full, saved 'chartLayout' object on every update.
        // This forces Plotly to re-apply the correct scaling and colors every time.
        Plotly.react(chart, [{
            x: pathX,
            y: pathY
        }, {
            x: [lastPoint.x],
            y: [lastPoint.y],
            marker: {
                symbol: 'triangle-up', color: '#c5a347', size: 16,
                line: { color: '#e6edf3', width: 2 },
                angle: rotationAngle
            }
        }], chartLayout); // <-- THIS IS THE FIX

    } catch (error) {
        console.error('❌ Chart update error:', error);
    }
}

// --- ALL OTHER FUNCTIONS BELOW THIS LINE ARE UNCHANGED ---

function setupSocketEvents() {
    console.log('📡 Setting up socket events...');
    
    socket.on('connect', function() {
        isConnected = true;
        updateConnectionDisplay('🟢 Connected');
        addLog('🌐 Connected to server with SLAM');
    });
    
    socket.on('disconnect', function() {
        isConnected = false;
        updateConnectionDisplay('🔴 Disconnected');
        addLog('🌐 Disconnected from server');
    });
    
    socket.on('initial_state', function(data) {
        if (data.rover_data) updateRoverData(data.rover_data);
        if (data.slam_data) updateSLAMData(data.slam_data);
    });
    
    socket.on('camera_update', function(data) {
        updateCameraFrame(data);
        cameraFrameCount++;
    });
    
    socket.on('rover_status', function(data) {
        updateRoverData(data);
        updateChart(data);
    });
    
    socket.on('map_update', function(data) {
        updateSLAMMapImage(data);
    });
    
    socket.on('semantic_update', function(data) {
        updateSemanticMap(data);
    });
    
    socket.on('mission_update', function(data) {
        updateMissionStatus(data);
    });
    
    socket.on('activity_update', function(data) {
        if (data.message) addLog(data.message);
    });
    
    socket.on('command_success', function(data) {
        addLog('✅ ' + data.message);
    });
    
    socket.on('command_error', function(data) {
        addLog('❌ ' + data.error);
    });
    
    console.log('✅ Socket events configured');
}

function updateSLAMData(data) {
    if (data.map_available && slamData.mapImage === null) {
        fetch('/api/map_image')
            .then(response => response.json())
            .then(result => {
                if (result.status === 'success') {
                    updateSLAMMapImage({map: result.map});
                }
            })
            .catch(err => console.error('Map fetch error:', err));
    }
    if (data.semantic_map) {
        slamData.semanticMap = data.semantic_map;
        updateSemanticDisplay();
    }
    if (data.mission_status) {
        slamData.missionStatus = data.mission_status;
        updateMissionDisplay();
    }
}

function updateSLAMMapImage(data) {
    try {
        const canvas = document.getElementById('slamMapCanvas');
        if (!canvas || !data.map) return;
        const ctx = canvas.getContext('2d');
        const img = new Image();
        img.onload = function() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            const statusEl = document.getElementById('mapStatus');
            if (statusEl) {
                statusEl.textContent = '🟢 Map Active';
                statusEl.style.color = '#238636';
            }
        };
        img.onerror = () => drawSLAMPlaceholder(ctx, canvas.width, canvas.height);
        img.src = data.map.startsWith('data:image') ? data.map : 'data:image/jpeg;base64,' + data.map;
    } catch (error) {
        console.error('❌ SLAM map update error:', error);
    }
}

function updateSemanticMap(data) {
    if (data.counts) {
        document.getElementById('rocksCount').textContent = data.counts.rocks || 0;
        document.getElementById('baseCount').textContent = data.counts.base || 0;
        document.getElementById('flagsCount').textContent = data.counts.flags || 0;
    }
}

function updateMissionStatus(data) {
    if (data.mission_status) {
        slamData.missionStatus = data.mission_status;
        updateMissionDisplay();
    }
}

function updateMissionDisplay() {
    const status = slamData.missionStatus;
    const modeEl = document.getElementById('missionMode');
    if (modeEl) modeEl.textContent = status.mode || 'IDLE';
    const batteryEl = document.getElementById('missionBattery');
    if (batteryEl) batteryEl.textContent = `${(status.battery || 100).toFixed(0)}%`;
    const distEl = document.getElementById('distanceToBase');
    if (distEl) distEl.textContent = `${(status.distance_to_base || 0).toFixed(1)}m`;
}

function moveForward() { sendRoverCommand(0.5, 0, 1, 'Forward'); }
function moveBackward() { sendRoverCommand(-0.5, 0, 1, 'Backward'); }
function moveLeft() { sendInPlaceTurn('left', 10); }
function moveRight() { sendInPlaceTurn('right', 10); }

function sendInPlaceTurn(direction, degrees) {
    const duration = Math.max(0.15, degrees / 100);
    const angularVelocity = direction === 'left' ? 0.8 : -0.8;
    sendRoverCommand(0, angularVelocity, duration, `Turn ${direction}`);
}

function emergencyStop() {
    const stopCommand = { cmd_vel: { linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0} }, duration: 0.1 };
    socket.emit('rover_command', stopCommand);
    addLog('🛑 EMERGENCY STOP');
}

function sendCustomCommand() {
    const linear = parseFloat(document.getElementById('customLinear')?.value || 0);
    const angular = parseFloat(document.getElementById('customAngular')?.value || 0);
    const duration = parseFloat(document.getElementById('customDuration')?.value || 1);
    sendRoverCommand(linear, angular, duration, 'Custom');
}

function sendRoverCommand(linear, angular, duration, name) {
    const command = {
        cmd_vel: { linear: { x: linear, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angular } },
        duration: duration
    };
    if (socket && socket.connected) {
        socket.emit('rover_command', command);
        addLog(`🎮 ${name} command sent`);
    } else {
        addLog('❌ Not connected - command failed');
    }
}

function sendGoal(x, y) {
    const goalCommand = {
        goal: {
            position: { x: parseFloat(x), y: parseFloat(y), z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
    };
    if (socket && socket.connected) {
        socket.emit('navigation_goal', goalCommand);
        addLog(`🎯 New goal set: (${x}, ${y})`);
    } else {
        addLog('❌ Not connected - cannot send goal');
    }
}

function sendCustomGoal() {
    const x = parseFloat(document.getElementById('goalX').value || 0);
    const y = parseFloat(document.getElementById('goalY').value || 0);
    sendGoal(x, y);
}

function updateRoverData(data) {
    try {
        if (data.position) {
            document.getElementById('positionDisplay').textContent = `(${data.position.x.toFixed(2)}, ${data.position.y.toFixed(2)})`;
        }
        if (data.velocity) {
            const linear = data.velocity.linear_x || 0;
            const angular = data.velocity.angular_z || 0;
            document.getElementById('velocityDisplay').textContent = `(${linear.toFixed(2)}, ${angular.toFixed(2)})`;
        }
        if (data.mode) {
            document.getElementById('modeDisplay').textContent = data.mode;
        }
    } catch (error) {
        console.error('❌ Data update error:', error);
    }
}

function updateCameraFrame(data) {
    try {
        const canvas = document.getElementById('cameraCanvas');
        if (!canvas || !data.data) return;
        const ctx = canvas.getContext('2d');
        const img = new Image();
        img.onload = () => ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        img.onerror = () => drawCameraPlaceholder(canvas);
        img.src = data.data.startsWith('data:image') ? data.data : 'data:image/jpeg;base64,' + data.data;
    } catch (error) {
        console.error('❌ Camera update error:', error);
    }
}

function initializeCamera() {
    const canvas = document.getElementById('cameraCanvas');
    if (canvas) {
        drawCameraPlaceholder(canvas);
        console.log('✅ Camera canvas initialized');
    }
}

function drawCameraPlaceholder(canvas) {
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#ffffff';
    ctx.font = '16px Arial';
    ctx.textAlign = 'center';
    ctx.fillText('📷 Waiting for camera...', canvas.width/2, canvas.height/2);
}

function updateConnectionDisplay(status) {
    const headerText = document.getElementById('connection-text-header');
    const headerIndicator = document.getElementById('connection-indicator');
    
    if (status.includes('Connected')) {
        headerText.textContent = 'Connected';
        headerText.className = 'connected';
        headerIndicator.className = 'connected';
    } else {
        headerText.textContent = 'Disconnected';
        headerText.className = 'disconnected';
        headerIndicator.className = 'disconnected';
    }
}

function addLog(message) {
    const logContainer = document.getElementById('activityLog');
    if (logContainer) {
        const p = document.createElement('p');
        const timestamp = new Date().toLocaleTimeString();
        p.textContent = `[${timestamp}] ${message}`;

        if (message.startsWith('✅')) p.className = 'log-success';
        else if (message.startsWith('❌') || message.startsWith('🛑')) p.className = 'log-error';
        else p.className = 'log-info';

        logContainer.prepend(p);
        while (logContainer.children.length > 50) {
            logContainer.removeChild(logContainer.lastChild);
        }
    }
}

function setupSliders() {
    const linearSlider = document.getElementById('customLinear');
    const angularSlider = document.getElementById('customAngular');
    const linearValue = document.getElementById('linearValue');
    const angularValue = document.getElementById('angularValue');

    if (linearSlider && linearValue) {
        linearSlider.oninput = () => linearValue.textContent = parseFloat(linearSlider.value).toFixed(1);
        linearSlider.oninput();
    }
    if (angularSlider && angularValue) {
        angularSlider.oninput = () => angularValue.textContent = parseFloat(angularSlider.value).toFixed(1);
        angularSlider.oninput();
    }
    console.log('✅ Sliders configured');
}

setInterval(() => {
    cameraFrameCount = 0;
}, 1000);

console.log('✅ SLAM Dashboard JavaScript loaded successfully');