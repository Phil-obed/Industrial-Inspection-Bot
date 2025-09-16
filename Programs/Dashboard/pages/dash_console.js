document.addEventListener("DOMContentLoaded", function () {
    const input = document.getElementById("command-input");
    const output = document.getElementById("output");

    // connection buttons
    const connectBtn = document.getElementById("connect-btn");
    const modeBtn = document.getElementById("mode-btn");
    const modeStatus = document.getElementById("mode-status");
    let mode = "manual"; // default

    // ===== GAS GRAPH SETUP =====
const ctx = document.getElementById('gasChart').getContext('2d');
const gasChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [], // timestamps
        datasets: [
            {
                label: 'CO',
                borderColor: 'rgba(80, 80, 80, 1)',         // dark gray
                backgroundColor: 'rgba(80, 80, 80, 0.3)',   // transparent gray fill
                data: [],
                fill: true,
                tension: 0.4   // <--- smooth curve
            },
            {
                label: 'CH4',
                borderColor: 'rgba(220, 220, 220, 1)',      // dim white / off-white
                backgroundColor: 'rgba(220, 220, 220, 0.3)',
                data: [],
                fill: true,
                tension: 0.4
            },
            {
                label: 'LPG',
                borderColor: 'rgba(100, 130, 180, 1)',      // soft steel blue
                backgroundColor: 'rgba(100, 130, 180, 0.3)',
                data: [],
                fill: true,
                tension: 0.4
            },
            {
                label: 'Air Quality',
                borderColor: 'rgba(143, 184, 113, 1)',      // greenish gray
                backgroundColor: 'rgba(180, 180, 180, 0.3)',
                data: [],
                fill: true,
                tension: 0.4
            }
        ]
    },
    options: {
        responsive: true,
        animation: false,
        plugins: {
            legend: {
                labels: { color: 'white' }
            }
        },
        scales: {
            x: {
                ticks: { color: 'white' }
            },
            y: {
                ticks: { color: 'white' },
                beginAtZero: true,
                suggestedMax: 100
            }
        }
    }
});

// Simulate sensor updates every 2s
setInterval(() => {
    const now = new Date().toLocaleTimeString();

    gasChart.data.labels.push(now);
    if (gasChart.data.labels.length > 10) gasChart.data.labels.shift();

    gasChart.data.datasets[0].data.push(Math.floor(Math.random() * 100)); // CO
    gasChart.data.datasets[1].data.push(Math.floor(Math.random() * 100)); // Methane
    gasChart.data.datasets[2].data.push(Math.floor(Math.random() * 100)); // LPG
    gasChart.data.datasets[3].data.push(Math.floor(Math.random() * 100)); // Air Quality

    gasChart.data.datasets.forEach(ds => {
        if (ds.data.length > 10) ds.data.shift();
    });

    gasChart.update();
}, 2000);


    // ====== MAP SETUP ======
    const map = L.map("map").setView([7.351136, -2.341782], 17);

    // Dark theme (Carto basemap)
    L.tileLayer(
        "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png",
        {
            attribution:
                '&copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/">CARTO</a>',
            subdomains: "abcd",
            maxZoom: 20,
        }
    ).addTo(map);

    // Default bot position (from GPS)
    let botPosition = L.circleMarker([7.351136, -2.341782], {
        radius: 7,
        color: "white",
        fillColor: "white",
        fillOpacity: 1,
    }).addTo(map).bindPopup("Bot Position");

    let currentRoute = null;
    let destinationMarker = null;

    function generateCrookedRoute(from, to, actionType = "Go To") {
    if (currentRoute) map.removeLayer(currentRoute);
    if (destinationMarker) map.removeLayer(destinationMarker);

    // drop destination pin
    destinationMarker = L.marker(to)
        .addTo(map)
        .bindPopup(`Destination (${actionType})`)
        .openPopup();

    // crooked route points
    let latlngs = [from];
    let numPoints = 3;

    // scale zigzag "noise" by distance
    let latDiff = Math.abs(to.lat - from.lat);
    let lngDiff = Math.abs(to.lng - from.lng);
    let scale = Math.max(latDiff, lngDiff) * 0.5; // half of the span
    let maxJitter = Math.min(scale, 0.0003); // cap so it doesn’t get crazy

    for (let i = 0; i < numPoints; i++) {
        let lat =
            from.lat +
            (to.lat - from.lat) * (i + 1) / (numPoints + 1) +
            (Math.random() - 0.5) * maxJitter;
        let lng =
            from.lng +
            (to.lng - from.lng) * (i + 1) / (numPoints + 1) +
            (Math.random() - 0.5) * maxJitter;
        latlngs.push([lat, lng]);
    }
    latlngs.push(to);

    // dark golden glowing route
    currentRoute = L.polyline(latlngs, {
        color: "#b8860b", // dark golden
        weight: 6,
        opacity: 0.9,
        className: "glow-route",
    }).addTo(map);

    map.fitBounds(currentRoute.getBounds());

    // log to console
    output.innerHTML += `<div>${actionType} (${to.lat.toFixed(
        6
    )}, ${to.lng.toFixed(6)})...</div>`;
}


    // Custom CSS for glowing effect
    const style = document.createElement("style");
    style.innerHTML = `
        .glow-route {
            stroke: #b8860b;
            stroke-width: 6px;
            filter: drop-shadow(0px 0px 6px #ffd700);
        }
    `;
    document.head.appendChild(style);

    // Map click menu
    map.on("click", function (e) {
        let container = L.DomUtil.create("div");
        let goBtn = L.DomUtil.create("button", "", container);
        goBtn.innerHTML = "Go To";
        goBtn.style.margin = "2px";
        goBtn.style.padding = "4px 8px";
        goBtn.style.cursor = "pointer";

        let inspectBtn = L.DomUtil.create("button", "", container);
        inspectBtn.innerHTML = "Inspect";
        inspectBtn.style.margin = "2px";
        inspectBtn.style.padding = "4px 8px";
        inspectBtn.style.cursor = "pointer";

        let popup = L.popup().setLatLng(e.latlng).setContent(container).openOn(map);

        // Go To action
        L.DomEvent.on(goBtn, "click", function () {
            generateCrookedRoute(botPosition.getLatLng(), e.latlng, "Go To");
            popup.remove();
        });

        // Inspect action
        L.DomEvent.on(inspectBtn, "click", function () {
            generateCrookedRoute(botPosition.getLatLng(), e.latlng, "Inspect");
            popup.remove();
        });
    });

    // ====== TERMINAL LOGIC ======
    input.addEventListener("keydown", function (event) {
        if (event.key === "Enter") {
            const command = input.value.trim();
            output.innerHTML +=
                "<div><span class='prompt'>argus-bot: $</span> " + command + "</div>";

            if (command === "help") {
                output.innerHTML += `<div>
<pre>
System / Info
-------------
help         -> Show this help menu
about        -> Show bot details and version
status       -> Health check(battery, sensor states, connectivity)
uptime       -> How long the bot has been running
date         -> Show system date/time
clear        -> Clear terminal output

Navigation / Movement
---------------------
move forward -> Move bot forward
move back    -> Move bot backward
turn left    -> Rotate bot left
turn right   -> Rotate bot right
stop         -> Emergency stop
goto <x,y>   -> Move to specific coordinate (plot on map)

Sensors
-------
ultrasonic   -> Print distance readings
thermal      -> Show thermal camera status or snapshot
gas          -> Show gas levels (CO, methane, LPG, air quality)
camera on    -> Enable front camera feed
camera off   -> Disable front camera feed
sensors      -> List all active sensors with status

Data / Logs
-----------
log list     -> Show available log files
log view <f> -> Display a specific log
log clear    -> Clear stored logs

Connections
-----------
wifi status  -> Check WiFi status
bt status    -> Check Bluetooth status
ping         -> Test connectivity

Control
-------
mode         -> Show current control mode
manual       -> Switch to manual control mode
auto         -> Switch to autonomous mode
reboot       -> Restart bot system
shutdown     -> Safely power off
</pre>
</div>`;
            } else if (command === "about") {
                output.innerHTML +=
                    "<div><br><br><br>Industrial Inspection Bot v1.0<br>Developed by ePIKK Robotics <br><br>Credits:<br>&nbsp;&nbsp;&nbsp;Philemon Obed Obeng<br>&nbsp;&nbsp;&nbsp;Benjamine Asare<br>&nbsp;&nbsp;&nbsp;Evans Tetteh<br>&nbsp;&nbsp;&nbsp;Akwesi Frimpong<br><br>Special Thanks:<br> &nbsp;&nbsp;&nbsp;Mr. Bright Ayasu (Project Supervisor)<br> &nbsp;&nbsp;&nbsp;Mr. William Asamoah (Technical Guidance)<br><br>This is a terminal to send commands to the Argus bot...<br>Use 'help' to list available commands.</div>";
            } else if (command === "clear") {
                output.innerHTML = "";
            } else if (command === "mode") {
                output.innerHTML += `<div>Current control mode: <b>${mode.toUpperCase()}</b></div>`;
            } else if (command.startsWith("goto")) {
                let coords = command.replace("goto", "").trim().split(",");
                if (coords.length === 2) {
                    let lat = parseFloat(coords[0]);
                    let lng = parseFloat(coords[1]);
                    if (!isNaN(lat) && !isNaN(lng)) {
                        let to = L.latLng(lat, lng);
                        generateCrookedRoute(botPosition.getLatLng(), to, "Go To");
                    }
                } else {
                    output.innerHTML += `<div>Invalid format. Use: goto <lat,lng></div>`;
                }
            } else if (command !== "") {
                output.innerHTML += "<div>Command not found: " + command + "</div>";
            }

            input.value = "";
            output.scrollTop = output.scrollHeight;
        }
    });

    function updateModeStatus() {
        modeStatus.textContent = `MODE: ${mode.toUpperCase()}`;
    }

    function printToConsole(message) {
        output.innerHTML += `<div><span class="prompt">$argus-bot:</span> ${message}</div>`;
        output.scrollTop = output.scrollHeight;
    }

    connectBtn.addEventListener("click", function () {
        printToConsole("Attempting connection...");
        setTimeout(() => printToConsole("Connection established successfully."), 1000);
    });

    modeBtn.addEventListener("click", function () {
        mode = mode === "manual" ? "auto" : "manual";
        printToConsole(`Switched mode to: ${mode.toUpperCase()}`);
        updateModeStatus();
    });

    const sensors = [
  { x: 0.00,  y: 2.5,  angle:  90, max_r: 4.0, fov: 30 },
  { x: -1.75, y: 2.5,  angle: 110, max_r: 4.0, fov: 30 },
  { x:  1.75, y: 2.5,  angle:  70, max_r: 4.0, fov: 30 },
  { x: -1.75, y: 0.0,  angle: 180, max_r: 3.0, fov: 30 },
  { x:  1.75, y: 0.0,  angle:   0, max_r: 3.0, fov: 30 },
];

const svg = document.getElementById('svg');

function polarToSvg(cx, cy, radius, angleDeg) {
  const cx_svg = cx;
  const cy_svg = -cy;
  const rad = angleDeg * Math.PI / 180;
  const x = cx_svg + radius * Math.cos(rad);
  const y = cy_svg - radius * Math.sin(rad);
  return { x, y };
}

function makeWedgePath(cx, cy, radius, startAngle, endAngle) {
  const start = polarToSvg(cx, cy, radius, startAngle);
  const end   = polarToSvg(cx, cy, radius, endAngle);

  let fov = endAngle - startAngle;
  if (fov < 0) fov += 360;
  const largeArcFlag = (fov > 180) ? 1 : 0;
  const sweepFlag = 0;

  return `M ${cx} ${-cy} L ${start.x} ${start.y} A ${radius} ${radius} 0 ${largeArcFlag} ${sweepFlag} ${end.x} ${end.y} Z`;
}

// Draw arcs (no dots)
sensors.forEach(s => {
  const cx = s.x, cy = s.y, angle = s.angle, fov = s.fov, maxR = s.max_r;
  const startAngle = angle - fov/2;
  const endAngle = angle + fov/2;

  const path = makeWedgePath(cx, cy, maxR, startAngle, endAngle);
  const wedge = document.createElementNS('http://www.w3.org/2000/svg','path');
  wedge.setAttribute('d', path);
  wedge.setAttribute('class', 'wedge');
  svg.appendChild(wedge);
});


    updateModeStatus();


    // ====== ESP32 WEBSOCKET HOOK ======
    let espSocket = null;

    function connectToESP(ip = "ws://192.168.4.1:81/") {
        printToConsole("Connecting to ESP32...");
        espSocket = new WebSocket(ip);

        espSocket.onopen = () => {
            printToConsole("Connected to ESP32 WebSocket ✅");
        };

        espSocket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);

                // --- Example JSON from ESP32 ---
                // {
                //   "gas": { "co": 32, "ch4": 50, "lpg": 18, "airq": 72 },
                //   "gps": { "lat": 7.351136, "lng": -2.341782 },
                //   "ultrasonic": [120, 85, 200, 95, 60],
                //   "battery": 87
                // }

                // ---- Update gas chart ----
                if (data.gas) {
                    const now = new Date().toLocaleTimeString();
                    gasChart.data.labels.push(now);
                    if (gasChart.data.labels.length > 10) gasChart.data.labels.shift();

                    gasChart.data.datasets[0].data.push(data.gas.co);
                    gasChart.data.datasets[1].data.push(data.gas.ch4);
                    gasChart.data.datasets[2].data.push(data.gas.lpg);
                    gasChart.data.datasets[3].data.push(data.gas.airq);

                    gasChart.data.datasets.forEach(ds => {
                        if (ds.data.length > 10) ds.data.shift();
                    });
                    gasChart.update();
                }

                // ---- Update GPS/map ----
                if (data.gps) {
                    botPosition.setLatLng([data.gps.lat, data.gps.lng]);
                    map.panTo([data.gps.lat, data.gps.lng]);
                }

                // ---- Update ultrasonic wedges ----
                if (data.ultrasonic) {
                    const wedges = svg.querySelectorAll(".wedge");
                    data.ultrasonic.forEach((d, i) => {
                        const wedge = wedges[i];
                        if (wedge) {
                            wedge.setAttribute("fill", d < 30 ? "red" : "green");
                            wedge.setAttribute("fill-opacity", 0.3);
                        }
                    });
                }

                // ---- Battery status to console ----
                if (data.battery !== undefined) {
                    printToConsole(`Battery: ${data.battery}%`);
                }

            } catch (err) {
                console.error("Bad ESP32 data:", event.data);
            }
        };

        espSocket.onclose = () => {
            printToConsole("ESP32 WebSocket disconnected ❌");
        };

        espSocket.onerror = (err) => {
            console.error("ESP32 WebSocket Error:", err);
        };
    }

    // Hook the existing connect button to ESP WebSocket
    connectBtn.addEventListener("click", function () {
        connectToESP("ws://192.168.4.1:81/"); // Change IP if ESP is STA on your router
    });
});
