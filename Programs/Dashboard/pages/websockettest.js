// Connect to ESP32 WebSocket AP
const ws = new WebSocket("ws://192.168.4.1/ws"); // ESP32 AP default IP

ws.onopen = () => {
    console.log("Connected to ESP32 WebSocket");
};

ws.onmessage = (event) => {
    // Incoming sensor data from ESP32 (JSON format)
    try {
        const data = JSON.parse(event.data);
        if(data.distance !== undefined) document.getElementById("distance").innerText = data.distance;
        if(data.temperature !== undefined) document.getElementById("temperature").innerText = data.temperature;
    } catch(e) {
        console.error("Invalid JSON:", event.data);
    }
};

ws.onclose = () => console.log("Disconnected from ESP32 WebSocket");

ws.onerror = (err) => console.error("WebSocket error:", err);

// Send command to ESP32
function sendCommand(action) {
    if(ws.readyState === WebSocket.OPEN) {
        const cmd = { action: action };
        ws.send(JSON.stringify(cmd));
        console.log("Sent command:", cmd);
    } else {
        console.warn("WebSocket not connected");
    }
}
