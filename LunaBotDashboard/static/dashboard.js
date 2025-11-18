console.log('🌐 LunaBot SLAM Dashboard loading...');

const socket = io();
let isConnected = false;
let cameraFrameCount = 0;
let chart = null;
let chartLayout = {};

// SLAM data storage
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

document.addEventListener('DOMContentLoaded', function () {
  console.log('📱 SLAM Dashboard initializing...');

  setupSocketEvents();
  setupSliders();
  initializeCamera();
  initializeSLAMMap();
  initializeChart();

  console.log('✅ SLAM Dashboard ready');
});

function initializeSLAMMap() {
  const canvas = document.getElementById('slamMapCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  drawSLAMPlaceholder(ctx, canvas.width, canvas.height);
  console.log('🗺️ SLAM map canvas initialized');
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
  console.log('📈 Initializing path chart...');

  const mapDiv = document.getElementById('pathMap');
  if (!mapDiv || typeof Plotly === 'undefined') return;

  try {
    const pathTrace = {
      x: [0],
      y: [0],
      mode: 'lines',
      type: 'scatter',
      name: 'Path History',
      line: { color: '#848d97', width: 2 }
    };

    const roverMarkerTrace = {
      x: [0],
      y: [0],
      mode: 'markers',
      type: 'scatter',
      name: 'Current Position',
      marker: {
        symbol: 'triangle-up',
        color: '#c5a347',
        size: 16,
        line: { color: '#e6edf3', width: 2 }
      }
    };

    chartLayout = {
      xaxis: {
        title: 'X Position (m)',
        color: '#848d97',
        gridcolor: 'rgba(197, 163, 71, 0.2)',
        autorange: true,
        zeroline: false
      },
      yaxis: {
        title: 'Y Position (m)',
        color: '#848d97',
        gridcolor: 'rgba(197, 163, 71, 0.2)',
        scaleanchor: 'x',
        scaleratio: 1,
        autorange: true,
        zeroline: false
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
    console.log('✅ Advanced chart initialized.');
  } catch (error) {
    console.error('❌ Chart initialization error:', error);
  }
}

function updateChart(data) {
  if (!chart || !data.path || data.path.length === 0) return;

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

    Plotly.react(
      chart,
      [
        { x: pathX, y: pathY },
        {
          x: [lastPoint.x],
          y: [lastPoint.y],
          marker: {
            symbol: 'triangle-up',
            color: '#c5a347',
            size: 16,
            line: { color: '#e6edf3', width: 2 },
            angle: rotationAngle
          }
        }
      ],
      chartLayout
    );
  } catch (error) {
    console.error('❌ Chart update error:', error);
  }
}

function setupSocketEvents() {
  console.log('📡 Setting up socket events...');

  socket.on('connect', function () {
    isConnected = true;
    updateConnectionDisplay('Connected');
    addLog('🌐 Connected to server with SLAM');
  });

  socket.on('disconnect', function () {
    isConnected = false;
    updateConnectionDisplay('Disconnected');
    addLog('🌐 Disconnected from server');
  });

  socket.on('initial_state', function (data) {
    if (data.rover_data) updateRoverData(data.rover_data);
    if (data.slam_data) updateSLAMData(data.slam_data);
  });

  socket.on('camera_update', function (data) {
    updateCameraFrame(data);
    cameraFrameCount++;
  });

  socket.on('rover_status', function (data) {
    updateRoverData(data);
    updateChart(data);
  });

  socket.on('map_update', function (data) {
    updateSLAMMapImage(data);
  });

  socket.on('semantic_update', function (data) {
    updateSemanticMap(data);
  });

  socket.on('mission_update', function (data) {
    updateMissionStatus(data);
  });

  socket.on('activity_update', function (data) {
    if (data.message) addLog(data.message);
  });

  socket.on('command_success', function (data) {
    addLog('✅ ' + data.message);
  });

  socket.on('command_error', function (data) {
    addLog('❌ ' + data.error);
  });

  console.log('✅ Socket events configured');
}

function updateSLAMData(data) {
  if (data.map_available && slamData.mapImage === null) {
    fetch('/api/map_image')
      .then(response => response.json())
      .then(result => {
        if (result.status === 'success') updateSLAMMapImage({ map: result.map });
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
  } catch (error) {
    console.error('❌ SLAM map update error:', error);
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
  const stopCommand = { cmd_vel: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }, duration: 0.1 };
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
      const px = data.position.x !== undefined ? data.position.x.toFixed(2) : '0.00';
      const py = data.position.y !== undefined ? data.position.y.toFixed(2) : '0.00';
      document.getElementById('positionDisplay').textContent = `(${px}, ${py})`;
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
  if (!canvas) return;
  drawCameraPlaceholder(canvas);
  console.log('📷 Camera canvas initialized');
}

function drawCameraPlaceholder(canvas) {
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#1a1a1a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#ffffff';
  ctx.font = '16px Arial';
  ctx.textAlign = 'center';
  ctx.fillText('📷 Waiting for camera...', canvas.width / 2, canvas.height / 2);
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
  while (logContainer.children.length > 50) {
    logContainer.removeChild(logContainer.lastChild);
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

setInterval(() => { cameraFrameCount = 0; }, 1000);

console.log('✅ SLAM Dashboard JavaScript loaded successfully');
