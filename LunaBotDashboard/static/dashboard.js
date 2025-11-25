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

// Traffic monitoring system with rolling average
let trafficStats = {
  bytesReceived: 0,
  messagesReceived: 0,
  lastCheck: Date.now()
};
let trafficHistory = {
  samples: [],
  maxSamples: 5
};

const originalSocketOn = socket.on.bind(socket);
socket.on = function(event, callback) {
  return originalSocketOn(event, function(data) {
    // Traffic counting
    try {
      const dataSize = JSON.stringify(data).length;
      trafficStats.bytesReceived += dataSize;
      trafficStats.messagesReceived++;
    } catch (e) {}
    return callback(data);
  });
};

setInterval(() => {
  const now = Date.now();
  trafficHistory.samples.push({
    bytes: trafficStats.bytesReceived,
    messages: trafficStats.messagesReceived,
    timestamp: now
  });
  if (trafficHistory.samples.length > trafficHistory.maxSamples) {
    trafficHistory.samples.shift();
  }
  // Calculate averages
  const totalBytes = trafficHistory.samples.reduce((sum, s) => sum + s.bytes, 0);
  const totalMessages = trafficHistory.samples.reduce((sum, s) => sum + s.messages, 0);
  const timeSpan = Math.max(1, trafficHistory.samples.length);
  const bytesPerSec = totalBytes / timeSpan;
  const messagesPerSec = totalMessages / timeSpan;
  
  // Update traffic UI
  const trafficEl = document.getElementById('networkTraffic');
  const iconEl = document.getElementById('trafficIcon');
  const msgRateEl = document.getElementById('messageRate');
  
  if (trafficEl) {
    let displayText = '', color = '', icon = '';
    if (bytesPerSec > 1024 * 1024) {
      displayText = `${(bytesPerSec / (1024 * 1024)).toFixed(2)} MB/s`; color = '#ff4444'; icon = '🔥';
    } else if (bytesPerSec > 100 * 1024) {
      displayText = `${(bytesPerSec / 1024).toFixed(1)} KB/s`; color = '#ff8844'; icon = '📡';
    } else if (bytesPerSec > 10 * 1024) {
      displayText = `${(bytesPerSec / 1024).toFixed(1)} KB/s`; color = '#00ff88'; icon = '📊';
    } else if (bytesPerSec > 1024) {
      displayText = `${(bytesPerSec / (1024)).toFixed(1)} KB/s`; color = '#00d4ff'; icon = '📈';
    } else if (bytesPerSec > 100) {
      displayText = `${bytesPerSec.toFixed(0)} B/s`; color = '#00d4ff'; icon = '📉';
    } else if (bytesPerSec > 0) {
      displayText = `${bytesPerSec.toFixed(0)} B/s`; color = '#888888'; icon = '📉';
    } else {
      displayText = '0 B/s'; color = '#666666'; icon = '💤';
    }
    trafficEl.textContent = displayText;
    trafficEl.style.color = color;
    if (iconEl) iconEl.textContent = icon;
  }
  
  if (msgRateEl) {
    const msgRate = messagesPerSec.toFixed(1);
    msgRateEl.textContent = msgRate;
    if (messagesPerSec > 15) {
      msgRateEl.style.color = '#00ff88'; 
    } else if (messagesPerSec > 8) {
      msgRateEl.style.color = '#00d4ff'; 
    } else if (messagesPerSec > 3) {
      msgRateEl.style.color = '#ff8844'; 
    } else if (messagesPerSec > 0.5) {
      msgRateEl.style.color = '#ffaa44'; 
    } else {
      msgRateEl.style.color = '#666666'; 
    }
  }
  trafficStats.bytesReceived = 0;
  trafficStats.messagesReceived = 0;
  trafficStats.lastCheck = now;
}, 1000);

function addLog(message, level = 'info') {
  const logContainer = document.getElementById('activityLog');
  if (!logContainer) return;

  const p = document.createElement('p');
  const timestamp = new Date().toLocaleTimeString();
  p.textContent = `[${timestamp}] ${message}`;

  // Normalize level
  const lvl = (level || 'info').toString().toLowerCase();
  if (lvl === 'success' || lvl === 'ok' || lvl === 'done') {
    p.className = 'log-success';
  } else if (lvl === 'error' || lvl === 'err' || lvl === 'danger' || lvl === 'critical') {
    p.className = 'log-error';
  } else if (lvl === 'warning' || lvl === 'warn') {
    p.className = 'log-warning';
  } else {
    p.className = 'log-info';
  }

  logContainer.prepend(p);
  while (logContainer.children.length > 50) logContainer.removeChild(logContainer.lastChild);
}
function logSuccess(message) { addLog(message, 'success'); }
function logError(message)   { addLog(message, 'error'); }
function logWarn(message)    { addLog(message, 'warning'); }
function logInfo(message)    { addLog(message, 'info'); }


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
          line: { color: '#c5a347', width: 2 }  
        },
        {
          x: [lastPoint.x],
          y: [lastPoint.y],
          marker: { 
            symbol: 'triangle-up', 
            size: 16, 
            angle: rotationAngle,
            color: '#c5a347',  
            line: { width: 2, color: '#ffffff' } 
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
    logSuccess('✅ Connected to server');
  });

  socket.on('disconnect', () => {
    isConnected = false;
    updateConnectionDisplay('Disconnected');
    logError('❌ Disconnected from server');
  });

  socket.on('initial_state', data => {
    if (data.rover_data) updateRoverData(data.rover_data);
    if (data.slam_data) updateSLAMData(data.slam_data);
  });

  socket.on('rover_status', (data) => {
    updateRoverData(data);
    updateChart(data);
    if (data.server_time) {
        const now = Date.now();
        const latency = Math.max(0, now - data.server_time); 
        
        const latencyText = document.getElementById('latency-text');
        const latencyDot = document.getElementById('latency-dot');
        
        if (latencyText) latencyText.textContent = `Latency: ${latency.toFixed(0)} ms`;
        if (latencyDot) {
            latencyDot.style.background = latency < 200 ? "#00ff88" : (latency < 500 ? "#ffaa44" : "#ff4444");
        }
    }
  });

  socket.on('map_update', updateSLAMMapImage);
  socket.on('semantic_update', updateSemanticMap);
  socket.on('mission_update', updateMissionStatus);
  socket.on('activity_update', d => { if (d.message) logInfo(d.message); });
  socket.on('command_success', d => logSuccess('✅ ' + d.message));
  socket.on('command_error', d => logError('❌ ' + d.error));
  socket.on('unity_restart_detected', d => logWarn(`Unity restart detected: ${d.distance_jump.toFixed(1)}m`));
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
      logSuccess('📷 Camera stream active');
    }
  };

  cameraStream.onerror = () => {
    cameraActive = false;
    cameraStream.style.display = 'none';
    cameraPlaceholder.style.display = 'flex';
    cameraStatus.textContent = 'OFFLINE';
    cameraStatus.className = 'status-indicator offline';
    logError('📷 Camera stream offline');
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
  
  const battery = s.battery || 100;
  const batteryEl = document.getElementById('missionBattery');
  const batteryBar = document.getElementById('batteryBar');
  
  if (batteryEl) batteryEl.textContent = `${battery.toFixed(0)}%`;
  if (batteryBar) {
    batteryBar.style.width = `${battery}%`;
    if (battery < 15) {
      batteryBar.setAttribute('data-level', 'critical');
    } else if (battery < 30) {
      batteryBar.setAttribute('data-level', 'low');
    } else {
      batteryBar.removeAttribute('data-level');
    }
  }
  
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
function goNorth() { sendGoal(NAVIGATION_GOALS.north.x, NAVIGATION_GOALS.north.y); logInfo('Heading North'); }
function goSouth() { sendGoal(NAVIGATION_GOALS.south.x, NAVIGATION_GOALS.south.y); logInfo('Heading South'); }
function goEast()  { sendGoal(NAVIGATION_GOALS.east.x, NAVIGATION_GOALS.east.y); logInfo('Heading East'); }
function goWest()  { sendGoal(NAVIGATION_GOALS.west.x, NAVIGATION_GOALS.west.y); logInfo('Heading West'); }
function goHome()  { sendGoal(NAVIGATION_GOALS.home.x, NAVIGATION_GOALS.home.y); logInfo('Returning Home'); }

function sendCustomGoal() {
  const x = parseFloat(document.getElementById('goalX').value || 0);
  const y = parseFloat(document.getElementById('goalY').value || 0);
  sendGoal(x, y);
}

function sendGoal(x, y) {
  const goalCommand = { goal: { position: { x: parseFloat(x), y: parseFloat(y), z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } };
  if (socket && socket.connected) {
    socket.emit('navigation_goal', goalCommand);
    logSuccess(`✅ Goal set: (${x}, ${y})`);
  } else logError('❌ Not connected - cannot send goal');
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
  logError('🛑 EMERGENCY STOP');
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
    logSuccess(`${name} command sent`);
  } else logError('❌ Not connected - command failed');
}

function resetNavigation() {
    if (socket && socket.connected) {
        socket.emit('reset_navigation');
        logInfo('🔄 Navigation reset requested');
    } else {
        logError('❌ Not connected - cannot reset');
    }
}

// Listen for response
socket.on('reset_success', (data) => {
    logSuccess('✅ ' + data.message);
});

socket.on('reset_error', (data) => {
    logError('❌ Reset error: ' + data.error);
});

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

    if (data.speed !== undefined) {
      const speedEl = document.getElementById('speedDisplay');
      if (speedEl) speedEl.textContent = `${data.speed.toFixed(2)} m/s`;
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




socket.on('system_metrics', (data) => {
    updateSystemMetrics(data);
});

function updateSystemMetrics(data) {
    const cpu = data.cpu || 0;
    const gpu = data.gpu || 0;
    const ram = data.ram || 0;
    const temp = data.temp || 0;
    
    // 1. CPU Update
    const cpuVal = document.getElementById('cpu-value');
    if (cpuVal) cpuVal.textContent = cpu.toFixed(0) + "%";

    const sparkPath = document.getElementById('cpu-sparkline-path');
    const sparkFill = document.getElementById('cpu-sparkline-fill');
    
    if (sparkPath) {
        const height = 50;
        // Map CPU 0-100% to Y 50-0
        const y = 50 - (cpu / 100 * 50); 
        const dLine = `M0,50 L20,50 L40,${y} L80,${y} L90,50 L100,50`;
        
        sparkPath.setAttribute('d', dLine);
        if (sparkFill) {
             const dFill = `${dLine} L100,50 L0,50 Z`;
             sparkFill.setAttribute('d', dFill);
        }
    }

    // 2. GPU Update
    updateRadial('gpu', gpu);
    const gpuBig = document.getElementById('gpu-big-text');
    if(gpuBig) gpuBig.textContent = gpu.toFixed(0) + "%";
const gpuDetailsEl = document.getElementById('gpu-details');
if (gpuDetailsEl) {
  const memUsed = data.gpu_mem_used || 0;
  const memTotal = data.gpu_mem_total || 0;
  const memPct  = (data.gpu_mem_percent !== undefined) ? data.gpu_mem_percent : (memTotal ? (memUsed / memTotal * 100) : 0);

  const usedGB  = (memUsed / (1024 ** 3));
  const totalGB = (memTotal / (1024 ** 3));

  if (memTotal > 0) {
    gpuDetailsEl.textContent = `${usedGB.toFixed(1)} / ${totalGB.toFixed(1)} GB`;
  } else {
    gpuDetailsEl.textContent = `Load`;
  }
}



    // 3. RAM Update
    updateRadial('ram', ram);
    const ramBig = document.getElementById('ram-big-text');
    if(ramBig) ramBig.textContent = ram.toFixed(0) + "%";
    
    const ramDet = document.getElementById('ram-details');
    if (ramDet && data.ram_used !== undefined) {
        ramDet.textContent = `${data.ram_used} / ${data.ram_total} GB`;
    } else if (ramDet) {
        ramDet.textContent = "-- / -- GB";
    }

    const tempBar = document.getElementById('temp-bar');
    const tempVal = document.getElementById('temp-value');
    const moonTemp = document.getElementById('moon-temp'); 
    const gpuTemp = (data.gpu_temp !== undefined && data.gpu_temp !== null) ? data.gpu_temp : (data.temp !== undefined ? data.temp : 0);
    const safeTemp = Math.min(Math.max(Number(gpuTemp) || 0, 0), 200);

    if (tempBar && tempVal) {
        const fillPct = Math.min(100, Math.round((safeTemp / 100) * 100));
        tempBar.style.width = fillPct + "%";
        tempVal.textContent = safeTemp.toFixed(0) + "°C";

        if (safeTemp > 85) {
            tempBar.style.background = 'linear-gradient(90deg, #ff4444, #ff0000)';
        } else {
            tempBar.style.background = ''; 
        }
    }
    if (moonTemp) {
        moonTemp.textContent = `${safeTemp.toFixed(0)}°C`;
    }
}

function updateRadial(type, percentage) {
    const ring = document.getElementById(`${type}-ring`);
    const innerText = document.getElementById(`${type}-value`);
    
    const validPercentage = Math.min(Math.max(percentage, 0), 100);

    if (ring) {
        const circumference = 226; // 2 * PI * 36
        const offset = circumference - (validPercentage / 100 * circumference);
        
        ring.style.strokeDasharray = `${circumference}`;
        ring.style.strokeDashoffset = offset;
    }
    
    if (innerText) innerText.textContent = validPercentage.toFixed(0) + "%";
}




function downloadDashboardData() {
  // Grab the SLAM canvas as base64 image
  const canvas = document.getElementById('slamMapCanvas');
  let slamMapImage = null;
  if (canvas) {
    try {
      slamMapImage = canvas.toDataURL('image/png');
    } catch (e) {
      console.warn('Could not capture SLAM map image:', e);
    }
  }
  const positionText = document.getElementById('positionDisplay')?.textContent || '(0, 0)';
  const velocityText = document.getElementById('velocityDisplay')?.textContent || '(0, 0)';
  const speedText = document.getElementById('speedDisplay')?.textContent || '0';

  const data = {
    timestamp: new Date().toISOString(),
    
    // Robot position and motion
    position: {
      x: parseFloat(positionText.match(/\(([-\d.]+)/)?.[1] || 0),
      y: parseFloat(positionText.match(/,\s*([\-\d.]+)/)?.[1] || 0)
    },
    velocity: {
      linear: parseFloat(velocityText.match(/\(([-\d.]+)/)?.[1] || 0),
      angular: parseFloat(velocityText.match(/,\s*([\-\d.]+)/)?.[1] || 0)
    },
    speed: parseFloat(speedText.match(/([\-\d.]+)/)?.[1] || 0),
    mode: document.getElementById('modeDisplay')?.textContent || 'Unknown',
    
    missionStatus: {
      battery: slamData?.missionStatus?.battery || 100,
      mode: slamData?.missionStatus?.mode || 'IDLE',
      distance_to_base: slamData?.missionStatus?.distance_to_base || 0,
    },
    
    camera: {
      active: typeof cameraActive !== 'undefined' ? cameraActive : false,
      status: document.getElementById('cameraStatus')?.textContent || 'OFFLINE'
    },
    
    detections: {
      rocks: parseInt(document.getElementById('rocksCount')?.textContent || '0'),
      base: parseInt(document.getElementById('baseCount')?.textContent || '0'),
      flags: parseInt(document.getElementById('flagsCount')?.textContent || '0')
    },
    
    semanticMap: slamData?.semanticMap || {},
    
    path: (chart && chart.data && chart.data[0]) ? {
      x: chart.data[0].x || [],
      y: chart.data[0].y || []
    } : { x: [], y: [] },
    
    connection: {
      status: document.getElementById('connection-text-header')?.textContent || 'Unknown',
      isConnected: isConnected || false
    },
    
    network: {
      traffic: document.getElementById('networkTraffic')?.textContent || '0 KB/s',
      messageRate: document.getElementById('messageRate')?.textContent || '0'
    },
    
    activityLog: Array.from(document.querySelectorAll('#activityLog p')).map(p => p.textContent),
    
    slamMapImage: slamMapImage
  };

  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `lunabot_data_${new Date().toISOString().replace(/[:.]/g,'-')}.json`;
  link.click();
  
  addLog('📥 Dashboard data downloaded', 'success');
}

socket.on('goal_success', d => {
  showToast('success', 'Goal Set', d.message);
  logSuccess('✅ Goal: ' + (d.message || 'OK'));
});

socket.on('goal_error', d => {
  showToast('error', 'Goal Failed', d.error || 'Unknown error');
  logError('❌ Goal failed: ' + (d.error || 'Unknown'));
});

socket.on('unity_restart_detected', d => {
  showToast('warning', 'Unity Restart Detected', `Position jumped ${d.distance_jump.toFixed(1)}m`);
  logWarn(`Unity restart: ${d.distance_jump.toFixed(1)}m`);
});

socket.on('reset_success', d => {
  showToast('success', 'Navigation Reset', d.message);
  logSuccess('✅ ' + (d.message || 'Reset OK'));
});

socket.on('connect', () => {
  showToast('success', 'Connected', 'Dashboard connected to server');
});

socket.on('disconnect', () => {
  showToast('error', 'Disconnected', 'Lost connection to server');
});
socket.on('system_metrics', (data) => {
    // console.log("Metrics:", data); // Uncomment to debug data flow if needed
    updateSystemMetrics(data);
});

setInterval(() => { cameraFrameCount = 0; }, 1000);

console.log('Dashboard script loaded');

// Toast notification system 
function showToast(type, title, message, duration = 4000) {
  const container = document.getElementById('toast-container');
  if (!container) return;
  
  const toast = document.createElement('div');
  toast.className = `toast ${type}`;
  
  const icons = {
    success: '✅',
    error: '❌',
    warning: '⚠️',
    info: 'ℹ️'
  };
  
  toast.innerHTML = `
    <div class="toast-icon">${icons[type] || 'ℹ️'}</div>
    <div class="toast-content">
      <div class="toast-title">${title}</div>
      <div class="toast-message">${message}</div>
    </div>
  `;
  
  container.appendChild(toast);

  setTimeout(() => {
    toast.style.animation = 'slideOut 0.3s ease';
    setTimeout(() => {
      if (toast.parentNode) {
        toast.parentNode.removeChild(toast);
      }
    }, 300);
  }, duration);
}



// Slider setup helpers
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
