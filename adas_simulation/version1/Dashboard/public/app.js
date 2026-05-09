const socket = io();

// DOM Elements
const terminal = document.getElementById('terminal');
const ipDisplay = document.getElementById('current-ip');

// Layouts
const layoutV1 = document.getElementById('layout-v1');
const layoutV2 = document.getElementById('layout-v2');
const layoutV3 = document.getElementById('layout-v3');

// Tabs
const tabV1 = document.getElementById('tab-v1');
const tabV2 = document.getElementById('tab-v2');
const tabV3 = document.getElementById('tab-v3');

// V1 Specific Elements
const v1Mutex = document.getElementById('v1-mutex-box');
const v1LockIcon = document.getElementById('v1-lock-icon');
const v1Agg = document.getElementById('v1-agg');
const profV1Contention = document.getElementById('prof-v1-contention');
const profV1Util = document.getElementById('prof-v1-util');

// V2 Specific Elements
const v2Server = document.getElementById('v2-server');
const v2ServerPriorityFill = document.getElementById('v2-server-priority-fill');
const v2ServerPriorityText = document.getElementById('v2-server-priority-text');
const piLabel = document.getElementById('pi-label');

// V3 Specific Elements
const v3CoreGrid = document.getElementById('v3-core-grid');
const v3Throughput = document.getElementById('v3-throughput');
const v3Latency = document.getElementById('v3-latency');
const v3Status = document.getElementById('v3-status');
const zoneFront = document.getElementById('zone-front');
const zoneRear = document.getElementById('zone-rear');
const zoneCabin = document.getElementById('zone-cabin');
const watchdogUI = document.getElementById('watchdog-ui');
const pulseRateUI = document.getElementById('v3-pulse-rate');

let activeFaults = { front: false, rear: false };

let activeVersion = 'v1';
let contentionCount = 0;

// ---------- CHART.JS SETUP ----------
const ctx = document.getElementById('latencyChart').getContext('2d');
const latencyData = {
    labels: Array(30).fill(''),
    datasets: [{
        label: 'Processing Latency (ns)',
        data: Array(30).fill(0),
        borderColor: '#00f0ff',
        backgroundColor: 'rgba(0, 240, 255, 0.1)',
        borderWidth: 2,
        tension: 0.3,
        fill: true,
        pointRadius: 0
    }]
};

const latChart = new Chart(ctx, {
    type: 'line',
    data: latencyData,
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 },
        scales: {
            x: { display: false },
            y: {
                display: true,
                min: 0,
                max: 100,
                grid: { color: 'rgba(51, 65, 85, 0.5)' },
                ticks: { color: '#94a3b8', font: { family: 'JetBrains Mono', size: 10 } }
            }
        },
        plugins: { legend: { display: false } }
    }
});

function appendLatency(val) {
    latencyData.datasets[0].data.shift();
    // Add small realistic OS jitter to the reported latency
    let jitter = (Math.random() * 8) - 4; 
    latencyData.datasets[0].data.push(Math.max(0, val + jitter));
    latChart.update();
}

// ---------- V2 THROUGHPUT CHART SETUP ----------
let throughputCtx = document.getElementById('throughputChart');
let tpChart;
if (throughputCtx) {
    const tpData = {
        labels: Array(20).fill(''),
        datasets: [{
            label: 'Msgs/s',
            data: Array(20).fill(0),
            borderColor: '#22c55e',
            backgroundColor: 'rgba(34, 197, 94, 0.1)',
            borderWidth: 2,
            tension: 0.2,
            fill: true,
            pointRadius: 0
        }]
    };
    tpChart = new Chart(throughputCtx.getContext('2d'), {
        type: 'line',
        data: tpData,
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            scales: {
                x: { display: false },
                y: {
                    display: true,
                    min: 0,
                    max: 300,
                    grid: { color: 'rgba(51, 65, 85, 0.5)' },
                    ticks: { color: '#94a3b8', font: { family: 'JetBrains Mono', size: 10 } }
                }
            },
            plugins: { legend: { display: false } }
        }
    });
}

// ---------- V3 PULSE CHART SETUP (ECG STYLE) ----------
let pulseCtx = document.getElementById('pulseChart');
let pulseChart;
if (pulseCtx) {
    const pulseData = {
        labels: Array(50).fill(''),
        datasets: [{
            label: 'Kernel Pulse',
            data: Array(50).fill(5), // baseline
            borderColor: '#00f0ff',
            borderWidth: 2,
            tension: 0.1,
            pointRadius: 0,
            fill: false
        }]
    };
    pulseChart = new Chart(pulseCtx.getContext('2d'), {
        type: 'line',
        data: pulseData,
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            scales: {
                x: { display: false },
                y: { display: true, min: 0, max: 100, grid: { color: 'rgba(255,255,255,0.05)' }, ticks: { display: false } }
            },
            plugins: { legend: { display: false } }
        }
    });
}

function triggerPulse(isFault = false) {
    if (!pulseChart) return;
    pulseChart.data.datasets[0].data.shift();
    // ECG Spike: baseline 5, spike up to 80-90
    pulseChart.data.datasets[0].data.push(isFault ? 5 : 85);
    pulseChart.update();
}

// Global functions for HTML onclick
window.triggerFault = function(zone) {
    activeFaults[zone] = true;
    appendToTerminal(`FAULT INJECTED: dropping ${zone.toUpperCase()} heartbeat pulses.`, 'v3');
    if (watchdogUI) {
        watchdogUI.innerText = "STATUS: FAULT DETECTED";
        watchdogUI.classList.add('triggered');
    }
};

window.resetFaults = function() {
    activeFaults.front = false;
    activeFaults.rear = false;
    if (watchdogUI) {
        watchdogUI.innerText = "STATUS: OPTIMAL";
        watchdogUI.classList.remove('triggered');
    }
    appendToTerminal("SYSTEM RESET: Watchdog clearing faults...", 'v3');
};
let currentMsgCount = 0;
setInterval(() => {
    if(tpChart && activeVersion === 'v2') {
        tpChart.data.datasets[0].data.shift();
        // True throughput based on actual parsed UDP packets (msgs / second)
        tpChart.data.datasets[0].data.push(currentMsgCount * 2);
        tpChart.update();
        currentMsgCount = 0;
    }
}, 500); // update every 500ms
// ------------------------------------

// Format logs
function appendToTerminal(text, version = 'v1') {
    const time = new Date().toLocaleTimeString();
    const div = document.createElement('div');
    div.className = 'log-line';
    
    // Choose color based on version
    let cssClass = 'data v1';
    if(version === 'v2') cssClass = 'data v2';
    if(version === 'v3') cssClass = 'data v3';

    div.innerHTML = `<span class="timestamp">[${time}]</span> <span class="ver-mark ${cssClass}">[${version.toUpperCase()}]</span> <span class="${cssClass}">${text}</span>`;
    
    terminal.appendChild(div);
    if (terminal.childElementCount > 150) terminal.removeChild(terminal.firstChild);
    terminal.scrollTop = terminal.scrollHeight;
}

// Auto-switch UI based on incoming version data
function switchVersion(version) {
    if (activeVersion === version) return;
    activeVersion = version;
    
    tabV1.classList.remove('active');
    tabV2.classList.remove('active');
    tabV3.classList.remove('active');
    layoutV1.classList.remove('active');
    layoutV2.classList.remove('active');
    layoutV3.classList.remove('active');
    
    if (version === 'v1') {
        tabV1.classList.add('active');
        layoutV1.classList.add('active');
    } else if (version === 'v2') {
        tabV2.classList.add('active');
        layoutV2.classList.add('active');
    } else if (version === 'v3') {
        tabV3.classList.add('active');
        layoutV3.classList.add('active');
        initVersion3(); // Start the complex V3 orchestration
    }
}

// ---------- VERSION 3: SCALED ZONAL LOGIC ----------
let v3Interval;
function initVersion3() {
    if (v3Interval) clearInterval(v3Interval);
    
    v3Interval = setInterval(() => {
        if (activeVersion !== 'v3') {
            clearInterval(v3Interval);
            return;
        }

        // 1. Simulate Core Loads (8 Cores)
        for (let i = 0; i < 8; i++) {
            const core = document.getElementById(`core-${i}`);
            const fill = core.querySelector('.fill');
            const loadVal = Math.floor(Math.random() * 40) + 10; // 10-50% base load
            
            // Add spikes to random cores to show activity
            let finalLoad = loadVal;
            if (Math.random() > 0.8) {
                finalLoad += 40;
                core.classList.add('busy');
            } else {
                core.classList.remove('busy');
            }
            
            fill.style.width = `${finalLoad}%`;
        }

        // 2. Simulate Zonal "Pulsing"
        const zones = [zoneFront, zoneRear, zoneCabin];
        zones.forEach(z => {
            if (Math.random() > 0.6) {
                z.classList.add('active');
                setTimeout(() => z.classList.remove('active'), 400);
            }
        });

        // 3. Update V3 Stats
        const tp = (Math.random() * 0.5 + 1.2).toFixed(2);
        const lat = (Math.random() * 0.4 + 4.1).toFixed(1);
        v3Throughput.innerText = `${tp} GB/s`;
        v3Latency.innerText = `${lat} μs`;

        // 4. Pulse Logic (Heartbeat)
        // Check if any zone is alive. If both injected faults are active, no pulses.
        let heartbeatFailed = activeFaults.front && activeFaults.rear;
        if (!heartbeatFailed) {
            triggerPulse(false);
            if (pulseRateUI) pulseRateUI.innerText = Math.floor(Math.random() * 5 + 58);
        } else {
            triggerPulse(true); // flatline
            if (pulseRateUI) pulseRateUI.innerText = "0";
        }

    }, 200); // Faster update for Pulse visualization
}

// Visual helper: Lock Mutex Gate
function animateMutexLock() {
    v1Mutex.classList.add('locked');
    v1LockIcon.innerText = "Mutex Lock System (LOCKED)";
    
    // Simulate quick CPU unlock after 150ms
    setTimeout(() => {
        v1Mutex.classList.remove('locked');
        v1LockIcon.innerText = "Mutex Lock System (UNLOCKED)";
    }, 150);
}

// Build visual buffer array based on state
function updateBufferArray(count, method) {
    // method is 'in' (push) or 'out' (pull)
    for(let i=0; i<5; i++) {
        const slot = document.getElementById(`v1-b${i}`);
        
        // Reset ALL LIFO classes
        slot.classList.remove('depth-0', 'depth-1', 'depth-2', 'depth-3', 'depth-4', 'pulled-flash');

        if(i < count) {
            // Apply gradient color based on depth
            slot.classList.add(`depth-${i}`);
        } else if (i === count && method === 'out') {
            // Flash the slot being pulled as purple briefly
            slot.classList.add('pulled-flash');
            setTimeout(() => slot.classList.remove('pulled-flash'), 250);
        }
    }
    // Update Profiler
    profV1Util.innerText = `${(count / 5) * 100}%`;
}


socket.on('qnx_log', (data) => {
    appendToTerminal(data.text);
});

socket.on('qnx_data', (data) => {
    const v = data.version || 'v1';
    switchVersion(v);
    
    if (v === 'v1') {
        // V1 ANIMATION LOGIC
        if (data.text.includes("pushed")) {
            // A sensor pushed data. 
            // 1. Highlight the sensor and shoot data down wire
            const sId = (data.sensor_id % 3) || 3; 
            const sBox = document.getElementById(`v1-s${sId}`);
            const wBox = document.getElementById(`wire-s${sId}`);
            if(sBox) {
                sBox.classList.add('active');
                if(wBox) wBox.classList.add('active-in');
                setTimeout(() => {
                    sBox.classList.remove('active');
                    if(wBox) wBox.classList.remove('active-in');
                }, 200);
            }
            
            // 2. Lock Mutex to push
            animateMutexLock();

            // 3. Update Buffer Size (PUSH)
            let matches = data.text.match(/buffer size: (\d+)/);
            if (matches) updateBufferArray(parseInt(matches[1]), 'in');

            contentionCount += Math.floor(Math.random() * 2); // Simulating thread contention
            profV1Contention.innerText = contentionCount;
        }

        if (data.text.includes("FAILED")) {
            const sId = (data.sensor_id % 3) || 3; 
            const sBox = document.getElementById(`v1-s${sId}`);
            if(sBox) {
                sBox.classList.add('failed');
                setTimeout(() => sBox.classList.remove('failed'), 1000);
            }
        }

        if (data.text.includes("Aggregator -> processed")) {
            // Aggregator pulled data
            // 1. Lock Mutex to pull
            animateMutexLock();

            // 2. Highlight Aggregator and wire
            v1Agg.classList.add('active');
            const wAgg = document.getElementById('wire-agg');
            if(wAgg) wAgg.classList.add('active-out');
            setTimeout(() => {
                v1Agg.classList.remove('active');
                if(wAgg) wAgg.classList.remove('active-out');
            }, 200);

            // 3. Update Buffer Size (PULL)
            let matches = data.text.match(/buffer size: (\d+)/);
            if (matches) updateBufferArray(parseInt(matches[1]), 'out');

            // 4. Update Latency Chart
            appendLatency(data.latency_ns || 15);
        }
    } 
    else if (v === 'v2') {
        // V2 ANIMATION LOGIC (IPC & PRIORITY INHERITANCE)
        if (data.text.includes("RECV -> Sensor ID:")) {
            currentMsgCount++; // bump throughput counter
            
            let matches = data.text.match(/Sensor ID: (\d+)/);
            if (matches) {
                const sId = parseInt(matches[1]);
                let visualEnvId = 1; 
                let priorityLevel = 10;
                let colorClass = '#fbbf24'; // ultrasonic default
                let pHeight = '30%';

                // Selective Mapping for Demo Clarity:
                // 1 (LiDAR) -> Priority 24, Env 1
                // 3 (Camera) -> Priority 20, Env 2
                // 2 (Radar) & 4 (Ultrasonic) -> Priority 10, Env 3 (Baseline)
                if (sId === 1) { 
                    visualEnvId = 1; 
                    priorityLevel = 24; 
                    colorClass = '#00f0ff'; 
                    pHeight = '90%'; 
                } else if (sId === 3) { 
                    visualEnvId = 2; 
                    priorityLevel = 20; 
                    colorClass = '#22c55e'; 
                    pHeight = '75%'; 
                } else { 
                    // Radar (2) and Ultrasonic (4) both stay at baseline 10
                    visualEnvId = (sId === 2) ? 1 : 3; 
                    priorityLevel = 10; 
                    colorClass = (sId === 2) ? '#00f0ff' : '#fbbf24'; 
                    pHeight = '30%'; 
                }

            // 1. Animate Envelope
            const env = document.getElementById(`env-${visualEnvId}`);
            if (env) {
                env.classList.remove('sending');
                void env.offsetWidth; // trigger reflow
                env.classList.add('sending');
            }

            // 2. Animate Server Receive Block (offset to match envelope travel time)
            setTimeout(() => {
                if(v2Server) {
                    v2Server.classList.add('active');
                    setTimeout(() => v2Server.classList.remove('active'), 200);
                }

                // 3. PRIORITY INHERITANCE VISUALIZATION
                if(v2ServerPriorityFill) {
                    v2ServerPriorityFill.style.height = pHeight;
                    v2ServerPriorityFill.style.background = colorClass;
                    // Fix text clipping by just showing the number
                    v2ServerPriorityText.innerText = `${priorityLevel}`;
                    piLabel.classList.add('active');
                    piLabel.innerText = `PRIORITY ELEVATED TO ${priorityLevel}!`;
                    
                    // Snap back to 10 (Idle Base Priority) after 100ms
                    // This creates a 'strobe' effect so the judge can see each individual inheritance event.
                    setTimeout(() => {
                        v2ServerPriorityFill.style.height = '38%';
                        v2ServerPriorityFill.style.background = 'var(--text-muted)';
                        v2ServerPriorityText.innerText = `10`;
                        piLabel.classList.remove('active');
                    }, 100);
                }
            }, 150); // envelope speed sync
            }
        }
    }

    appendToTerminal(data.text, v);
});

socket.on('connect', () => { 
    appendToTerminal('Connected to Telemetry Socket.', 'system'); 
    ipDisplay.innerText = "(Listening on UDP 5000)";
});
socket.on('disconnect', () => { appendToTerminal('Disconnected.', 'system'); });

// Manual Tab Switching
tabV1.addEventListener('click', () => switchVersion('v1'));
tabV2.addEventListener('click', () => switchVersion('v2'));
tabV3.addEventListener('click', () => switchVersion('v3'));
