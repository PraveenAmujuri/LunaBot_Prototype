console.log('LunaBot SLAM Dashboard starting...');

const socket = io();
let isConnected = false;
let cameraFrameCount = 0;
let chart = null;
let chartLayout = {};

// SLAM data
let slamData = {
  mapImage: null,
  semanticMap: { rocks: [], base: [], flags: [], antennas: [] },
  missionStatus: {
    battery: 100.0,
    mode: 'EXPLORATION',
    base_x: 0.0,
    base_y: 0.0,
    distance_to_base: 0.0
  }
};

let cameraActive = false;

const NAVIGATION_GOALS = {
  north: { x: 0, y: 10 },
  south: { x: 0, y: -10 },
  east:  { x: 10, y: 0 },
  west:  { x: -10, y: 0 },
  home:  { x: 0, y: 0 }
};

document.addEventListener('DOMContentLoaded', () => {
  setupSocketEvents();
  setupSliders();
  initializeCameraStream();
  initializeSLAMMap();
  initializeChart();
  console.log('Dashboard ready');
});

function initializeSLAMMap() {
  const canvas = document.getElementById('slamMapCanvas');
  if (!canvas) return;
  canvas.width = canvas.clientWidth || 640;
  canvas.height = canvas.clientHeight || 360;
  const ctx = canvas.getContext('2d');
  drawSLAMPlaceholder(ctx, canvas.width, canvas.height);
}

function drawSLAMPlaceholder(ctx, width, height) {
  ctx.fillStyle = '#1a1a1a';
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = '#ffffff';
  ctx.font = '16px Arial';
  ctx.textAlign = 'center';
  ctx.fillText('🗺️ Waiting for SLAM map...', width / 2, height / 2 - 10);
  ctx.font = '12px Arial';
  ctx.fillText('Building occupancy grid', width / 2, height / 2 + 15);
}

function initializeChart() {
  const mapDiv = document.getElementById('pathMap');
  if (!mapDiv || typeof Plotly === 'undefined') return;

  const pathTrace = { 
    x: [0], 
    y: [0], 
    mode: 'lines', 
    type: 'scatter', 
    line: { width: 2, color: '#c5a347' }  
  };
  const roverMarkerTrace = {
    x: [0],
    y: [0],
    mode: 'markers',
    type: 'scatter',
    marker: { 
      symbol: 'triangle-up', 
      size: 16, 
      color: '#c5a347',  
      line: { width: 2, color: '#ffffff' }  
    }
  };


  chartLayout = {
    xaxis: { title: 'X Position (m)', autorange: true, zeroline: false },
    yaxis: { title: 'Y Position (m)', autorange: true, zeroline: false, scaleanchor: 'x', scaleratio: 1 },
    paper_bgcolor: 'transparent',
    plot_bgcolor: 'transparent',
    font: { color: '#bdc6d3' },
    showlegend: false,
    margin: { l: 50, r: 20, b: 40, t: 20 }
  };

  Plotly.newPlot(mapDiv, [pathTrace, roverMarkerTrace], chartLayout, { responsive: true, displayModeBar: false });
  chart = mapDiv;
}

function updateChart(data) {
  if (!chart || !data.path || data.path.length === 0) return;
  try {
    const pathX = data.path.map(p => p.x);
    const pathY = data.path.map(p => p.y);
    const lastPoint = data.path[data.path.length - 1];

    let rotationAngle = 0;
    if (data.path.length > 1) {
      const prev = data.path[data.path.length - 2];
      const dx = lastPoint.x - prev.x;
      const dy = lastPoint.y - prev.y;
      if (dx !== 0 || dy !== 0) {
        rotationAngle = Math.atan2(dy, dx) * (180 / Math.PI) - 90;
      }
    }

    Plotly.react(chart,
  [
    { 
      x: pathX, 
      y: pathY,
      line: { color: '#c5a347', width: 2 }  // Gold line
    },
    {
      x: [lastPoint.x],
      y: [lastPoint.y],
      marker: { 
        symbol: 'triangle-up', 
        size: 16, 
        angle: rotationAngle,
        color: '#c5a347',  // Gold rover marker
        line: { width: 2, color: '#ffffff' }  // White border
      }
    }
  ],
  chartLayout
);

  } catch (e) {
    console.error('Chart update error:', e);
  }
}

function setupSocketEvents() {
  socket.on('connect', () => {
    isConnected = true;
    updateConnectionDisplay('Connected');
    addLog('Connected to server');
  });

  socket.on('disconnect', () => {
    isConnected = false;
    updateConnectionDisplay('Disconnected');
    addLog('Disconnected from server');
  });

  socket.on('initial_state', data => {
    if (data.rover_data) updateRoverData(data.rover_data);
    if (data.slam_data) updateSLAMData(data.slam_data);
  });

  socket.on('rover_status', data => {
    updateRoverData(data);
    updateChart(data);
  });

  socket.on('map_update', updateSLAMMapImage);
  socket.on('semantic_update', updateSemanticMap);
  socket.on('mission_update', updateMissionStatus);
  socket.on('activity_update', d => { if (d.message) addLog(d.message); });
  socket.on('command_success', d => addLog('✅ ' + d.message));
  socket.on('command_error', d => addLog('❌ ' + d.error));
  socket.on('unity_restart_detected', d => addLog(`Unity restart detected: ${d.distance_jump.toFixed(1)}m`));
}

function initializeCameraStream() {
  const cameraStream = document.getElementById('cameraStream');
  const cameraPlaceholder = document.getElementById('cameraPlaceholder');
  const cameraStatus = document.getElementById('cameraStatus');
  if (!cameraStream || !cameraPlaceholder || !cameraStatus) return;

  cameraStream.onload = () => {
    if (!cameraActive) {
      cameraActive = true;
      cameraStream.style.display = 'block';
      cameraPlaceholder.style.display = 'none';
      cameraStatus.textContent = 'ACTIVE';
      cameraStatus.className = 'status-indicator online';
      addLog('Camera stream active');
    }
  };

  cameraStream.onerror = () => {
    cameraActive = false;
    cameraStream.style.display = 'none';
    cameraPlaceholder.style.display = 'flex';
    cameraStatus.textContent = 'OFFLINE';
    cameraStatus.className = 'status-indicator offline';
    addLog('Camera stream offline');
  };

  setInterval(() => {
    if (!cameraActive && document.visibilityState === 'visible') {
      const ts = Date.now();
      cameraStream.src = `http://localhost:8080/stream?topic=/camera/color/image_raw&type=mjpeg&quality=50&t=${ts}`;
    }
  }, 5000);
}

function updateSLAMData(data) {
  if (data.map_available && slamData.mapImage === null) {
    fetch('/api/map_image').then(r => r.json()).then(result => {
      if (result.status === 'success') updateSLAMMapImage({ map: result.map });
    }).catch(e => console.error('Map fetch error:', e));
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
    canvas.width = canvas.clientWidth || 640;
    canvas.height = canvas.clientHeight || 360;
    const ctx = canvas.getContext('2d');
    const img = new Image();
    img.onload = function () {
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
  } catch (e) {
    console.error('SLAM map update error:', e);
  }
}

function updateSemanticMap(data) {
  if (!data.counts) return;
  document.getElementById('rocksCount').textContent = data.counts.rocks || 0;
  document.getElementById('baseCount').textContent = data.counts.base || 0;
  document.getElementById('flagsCount').textContent = data.counts.flags || 0;
}

function updateMissionStatus(data) {
  if (!data.mission_status) return;
  slamData.missionStatus = data.mission_status;
  updateMissionDisplay();
}

function updateMissionDisplay() {
  const s = slamData.missionStatus;
  const modeEl = document.getElementById('missionMode');
  if (modeEl) modeEl.textContent = s.mode || 'IDLE';
  const batteryEl = document.getElementById('missionBattery');
  if (batteryEl) batteryEl.textContent = `${(s.battery || 100).toFixed(0)}%`;
  const distEl = document.getElementById('distanceToBase');
  if (distEl) distEl.textContent = `${(s.distance_to_base || 0).toFixed(1)}m`;
}

function updateSemanticDisplay() {
  const semantic = slamData.semanticMap;
  document.getElementById('rocksCount').textContent = semantic.rocks?.length || 0;
  document.getElementById('baseCount').textContent = semantic.base?.length || 0;
  document.getElementById('flagsCount').textContent = semantic.flags?.length || 0;
}

/* Navigation */
function goNorth() { sendGoal(NAVIGATION_GOALS.north.x, NAVIGATION_GOALS.north.y); addLog('Heading North'); }
function goSouth() { sendGoal(NAVIGATION_GOALS.south.x, NAVIGATION_GOALS.south.y); addLog('Heading South'); }
function goEast()  { sendGoal(NAVIGATION_GOALS.east.x, NAVIGATION_GOALS.east.y); addLog('Heading East'); }
function goWest()  { sendGoal(NAVIGATION_GOALS.west.x, NAVIGATION_GOALS.west.y); addLog('Heading West'); }
function goHome()  { sendGoal(NAVIGATION_GOALS.home.x, NAVIGATION_GOALS.home.y); addLog('Returning Home'); }

function sendCustomGoal() {
  const x = parseFloat(document.getElementById('goalX').value || 0);
  const y = parseFloat(document.getElementById('goalY').value || 0);
  sendGoal(x, y);
}

function sendGoal(x, y) {
  const goalCommand = { goal: { position: { x: parseFloat(x), y: parseFloat(y), z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } };
  if (socket && socket.connected) {
    socket.emit('navigation_goal', goalCommand);
    addLog(`Goal set: (${x}, ${y})`);
  } else addLog('Not connected - cannot send goal');
}

/* Movement */
function moveForward()  { sendRoverCommand(0.5, 0, 1, 'Forward'); }
function moveBackward() { sendRoverCommand(-0.5, 0, 1, 'Backward'); }
function moveLeft()     { sendInPlaceTurn('left', 10); }
function moveRight()    { sendInPlaceTurn('right', 10); }

function sendInPlaceTurn(direction, degrees) {
  const duration = Math.max(0.15, degrees / 100);
  const angularVelocity = direction === 'left' ? 0.8 : -0.8;
  sendRoverCommand(0, angularVelocity, duration, `Turn ${direction}`);
}

function emergencyStop() {
  const stopCommand = { cmd_vel: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }, duration: 0.1 };
  socket.emit('rover_command', stopCommand);
  addLog('EMERGENCY STOP');
}

function sendCustomCommand() {
  const linear = parseFloat(document.getElementById('customLinear')?.value || 0);
  const angular = parseFloat(document.getElementById('customAngular')?.value || 0);
  const duration = parseFloat(document.getElementById('customDuration')?.value || 1);
  sendRoverCommand(linear, angular, duration, 'Custom');
}

function sendRoverCommand(linear, angular, duration, name) {
  const command = { cmd_vel: { linear: { x: linear, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angular } }, duration };
  if (socket && socket.connected) {
    socket.emit('rover_command', command);
    addLog(`${name} command sent`);
  } else addLog('Not connected - command failed');
}

function updateRoverData(data) {
  try {
    if (data.position) {
      const px = data.position.x !== undefined ? data.position.x.toFixed(2) : '0.00';
      const py = data.position.y !== undefined ? data.position.y.toFixed(2) : '0.00';
      document.getElementById('positionDisplay').textContent = `(${px}, ${py})`;
    }
    if (data.velocity) {
      const linear = data.velocity.linear_x || 0;
      const angular = data.velocity.angular_z || 0;
      document.getElementById('velocityDisplay').textContent = `(${linear.toFixed(2)}, ${angular.toFixed(2)})`;
    }
    if (data.mode) document.getElementById('modeDisplay').textContent = data.mode;
  } catch (e) {
    console.error('Data update error:', e);
  }
}

function updateConnectionDisplay(status) {
  const headerText = document.getElementById('connection-text-header');
  const headerIndicator = document.getElementById('connection-indicator');
  if (!headerText || !headerIndicator) return;
  if (typeof status === 'string' && status.toLowerCase().includes('connected')) {
    headerText.textContent = 'Connected';
    headerText.className = 'connected';
    headerIndicator.className = 'connected';
  } else {
    headerText.textContent = 'Disconnected';
    headerText.className = 'disconnected';
    headerIndicator.className = 'disconnected';
  }
}
function toggleFullscreenCamera() {
    const cameraCard = document.getElementById('cameraCard');
    if (cameraCard) {
        cameraCard.classList.toggle('fullscreen');
    }
}


function addLog(message) {
  const logContainer = document.getElementById('activityLog');
  if (!logContainer) return;
  const p = document.createElement('p');
  const timestamp = new Date().toLocaleTimeString();
  p.textContent = `[${timestamp}] ${message}`;

  if (message.startsWith('✅')) p.className = 'log-success';
  else if (message.startsWith('❌') || message.startsWith('🛑')) p.className = 'log-error';
  else p.className = 'log-info';

  logContainer.prepend(p);
  while (logContainer.children.length > 50) logContainer.removeChild(logContainer.lastChild);
}

function setupSliders() {
  const linearSlider = document.getElementById('customLinear');
  const angularSlider = document.getElementById('customAngular');
  const durationSlider = document.getElementById('customDuration');
  const linearValue = document.getElementById('linearValue');
  const angularValue = document.getElementById('angularValue');
  const durationValue = document.getElementById('durationValue');

  if (linearSlider && linearValue) {
    linearSlider.oninput = () => linearValue.textContent = parseFloat(linearSlider.value).toFixed(1);
    linearSlider.oninput();
  }
  if (angularSlider && angularValue) {
    angularSlider.oninput = () => angularValue.textContent = parseFloat(angularSlider.value).toFixed(1);
    angularSlider.oninput();
  }
  if (durationSlider && durationValue) {
    durationSlider.oninput = () => durationValue.textContent = parseFloat(durationSlider.value).toFixed(1);
    durationSlider.oninput();
  }
}

setInterval(() => { cameraFrameCount = 0; }, 1000);

console.log('Dashboard script loaded');
