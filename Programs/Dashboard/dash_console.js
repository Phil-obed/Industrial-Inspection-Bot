document.addEventListener("DOMContentLoaded", function() {
    const input = document.getElementById("command-input");
    const output = document.getElementById("output");

    // connection buttons
    const connectBtn = document.getElementById("connect-btn");
    const modeBtn = document.getElementById("mode-btn");
    const modeStatus = document.getElementById("mode-status");
    let mode = "manual"; // default

    input.addEventListener("keydown", function(event) {
        if (event.key === "Enter") {
            const command = input.value.trim();
            output.innerHTML += "<div><span class='prompt'>argus-bot: $</span> " + command + "</div>";

            // Fake command responses
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
goto &lt;x,y&gt;   -> Move to specific coordinate

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
log view &lt;f&gt; -> Display a specific log
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
                output.innerHTML += "<div>Industrial Inspection Bot v1.0<br>Developed by ePIKK Robotics <br><br>Credits:<br>&nbsp;&nbsp;&nbsp;Philemon Obed Obeng<br>&nbsp;&nbsp;&nbsp;Benjamine Asare<br>&nbsp;&nbsp;&nbsp;Evans Tetteh<br>&nbsp;&nbsp;&nbsp;Akwesi Frimpong<br><br>This is a terminal to send commands to the Argus bot...<br>Use 'help' to list available commands.</div>";
            } else if (command === "clear") {
                output.innerHTML = "";
            } else if (command === "mode") {
                output.innerHTML += `<div>Current control mode: <b>${mode.toUpperCase()}</b></div>`;
            } else if (command !== "") {
                output.innerHTML += "<div>Command not found: " + command + "</div>";
            }

            input.value = "";
            // Scroll to bottom
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

    connectBtn.addEventListener("click", function() {
        printToConsole("Attempting connection...");
        setTimeout(() => printToConsole("Connection established successfully."), 1000);
    });

    modeBtn.addEventListener("click", function() {
        mode = mode === "manual" ? "auto" : "manual";
        printToConsole(`Switched mode to: ${mode.toUpperCase()}`);
        updateModeStatus();
    });

    updateModeStatus();
});
