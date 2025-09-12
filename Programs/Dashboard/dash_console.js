document.addEventListener("DOMContentLoaded", function() {
    const input = document.getElementById("command-input");
    const output = document.getElementById("output");

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
reboot       -> Restart bot system
shutdown     -> Safely power off
</pre>
</div>`;
            } else if (command === "about") {
                output.innerHTML += "<div>Industrial Inspection Bot v1.0<br>Developed by ePIKK Robotics <br><br>Credits:<br>&nbsp;&nbsp;&nbsp;Philemon Obed Obeng<br>&nbsp;&nbsp;&nbsp;Benjamine Asare<br>&nbsp;&nbsp;&nbsp;Evans Tetteh<br>&nbsp;&nbsp;&nbsp;Akwesi Frimpong<br><br>This is a terminal to send commands to the Argus bot...<br>Use 'help' to list available commands.</div>";
            } else if (command === "clear") {
                output.innerHTML = "";
            } else if (command !== "") {
                output.innerHTML += "<div>Command not found: " + command + "</div>";
            }

            input.value = "";
            // Scroll to bottom
            output.scrollTop = output.scrollHeight;
        }
    });
});
