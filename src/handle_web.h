// Root page: displays current CAN and battery data, plus a form to update charging parameters.
void handleRoot()
{
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>CAN Monitor</title>";

    // Sisipkan variabel JavaScript untuk batas maksimum agar dapat diakses client-side
    html += "<script>";
    html += "var maxAllowedVoltage = " + String(MAX_ALLOWED_VOLTAGE, 1) + ";";
    html += "var maxAllowedCurrent = " + String(MAX_ALLOWED_CURRENT, 1) + ";";
    html += "</script>";

    // JavaScript to poll the /data endpoint every second and update the displayed values
    html += "<script>";
    html += "function updateData() {";
    html += "  var xhttp = new XMLHttpRequest();";
    html += "  xhttp.onreadystatechange = function() {";
    html += "    if (this.readyState == 4 && this.status == 200) {";
    html += "      var data = JSON.parse(this.responseText);";
    html += "      document.getElementById('chargingStatus').innerHTML = data.chargingStatus;";
    html += "      document.getElementById('canId').innerHTML = data.canId;";
    html += "      document.getElementById('canData').innerHTML = data.canData;";
    html += "      document.getElementById('voltage').innerHTML = data.voltage + ' V';";
    html += "      document.getElementById('current').innerHTML = data.current + ' A';";
    html += "      document.getElementById('targetVoltageLabel').innerHTML = data.targetVoltage + ' V';";
    html += "      document.getElementById('targetCurrentLabel').innerHTML = data.targetCurrent + ' A';";
    // Safety Alerts: tampilkan notifikasi jika kondisi tidak aman
    html += "      var alertDiv = document.getElementById('safetyAlert');";
    html += "      if(data.chargingStatus.indexOf('Cutoff') != -1) {";
    html += "        alertDiv.style.display = 'block';";
    html += "        alertDiv.innerHTML = 'Safety Alert: Unsafe condition detected!';";
    html += "      } else {";
    html += "        alertDiv.style.display = 'none';";
    html += "      }";
    html += "    }";
    html += "  };";
    html += "  xhttp.open('GET', '/data', true);";
    html += "  xhttp.send();";
    html += "}";
    html += "setInterval(updateData, 1000);";
    html += "</script>";

    html += "</head><body>";
    // Safety Alert area
    html += "<div id='safetyAlert' style='display:none; padding:10px; background-color: #ffcccc; color: #cc0000; margin-bottom:10px;'></div>";
    html += "<h1>CAN Monitor</h1>";
    html += "<p><strong>uptime:</strong> <span id='uptime'>" + String(uptime) + " sec</span></p>";
    html += "<p><strong>Charging Status:</strong> <span id='chargingStatus'>" + chargingStatusText + "</span></p>";
    html += "<p><strong>Maximum Voltage Rating:</strong> <span id='maxAlowedVoltage'>" + String(MAX_ALLOWED_VOLTAGE, 1) + " V</span></p>";
    html += "<p><strong>Maximum Current Rating:</strong> <span id='maxAlowedCurrent'>" + String(MAX_ALLOWED_CURRENT, 1) + " A</span></p>";
    html += "<p><strong>Received CAN ID:</strong> <span id='canId'>" + webCANId + "</span></p>";
    html += "<p><strong>Received CAN Data:</strong> <span id='canData'>" + webCANData + "</span></p>";
    html += "<p><strong>Charging Voltage:</strong> <span id='voltage'>" + String(batteryVoltage, 1) + " V</span></p>";
    html += "<p><strong>Charging Current:</strong> <span id='current'>" + String(batteryCurrent, 1) + " A</span></p>";

    // Form untuk mengupdate parameter charger
    html += "<h2>Update Charger Parameters</h2>";
    html += "<form action='/set' method='GET'>";
    html += "Set Cutoff Current (A): <input type='number' step='0.1' name='cutoff' id='cutoffCurrent' min='1' max='15'> ";
    html += "<span id='cutoffCurrentLabel'>" + String(cutoffCurrent, 1) + " A</span><br>";
    html += "Set Charging Voltage (V): <input type='number' step='0.1' name='v' id='targetVoltage' min='30' max='" + String(MAX_ALLOWED_VOLTAGE, 1) + "'> ";
    html += "<span id='targetVoltageLabel'>" + String(targetVoltage, 1) + " V</span><br>";
    html += "Set Charging Current (A): <input type='number' step='0.1' name='c' id='targetCurrent' min='1' max='" + String(MAX_ALLOWED_CURRENT, 1) + "'> ";
    html += "<span id='targetCurrentLabel'>" + String(targetCurrent, 1) + " A</span><br>";
    html += "<input type='submit' value='Update Parameters'>";
    html += "</form>";

    // Control Charger
    html += "<h2>Control Charger</h2>";
    html += "<button onclick=\"location.href='/control?cmd=start'\">Start Charging</button> ";
    html += "<button onclick=\"location.href='/control?cmd=stop'\">Stop Charging</button>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

void handleSet()
{
    bool valid = true;
    String errorMsg = "";

    if (server.hasArg("v"))
    {
        float newVoltage = server.arg("v").toFloat();
        if (newVoltage > 0 && newVoltage <= MAX_ALLOWED_VOLTAGE)
        {
            targetVoltage = newVoltage;
            preferences.putFloat("targetVoltage", targetVoltage);
            Serial.print("Updated target voltage: ");
            Serial.println(targetVoltage);
        }
        else
        {
            valid = false;
            errorMsg += "Voltage out of range. ";
        }
    }

    if (server.hasArg("c"))
    {
        float newCurrent = server.arg("c").toFloat();
        if (newCurrent > 0 && newCurrent <= MAX_ALLOWED_CURRENT)
        {
            targetCurrent = newCurrent;
            preferences.putFloat("targetCurrent", targetCurrent);
            Serial.print("Updated target current: ");
            Serial.println(targetCurrent);
        }
        else
        {
            valid = false;
            errorMsg += "Current out of range. ";
        }
    }

    if (server.hasArg("cutoff"))
    {
        float newCutoff = server.arg("cutoff").toFloat();
        if (newCutoff > 0 && newCutoff <= 15)
        {
            cutoffCurrent = newCutoff;
            preferences.putFloat("cutoffCurrent", cutoffCurrent);
            Serial.print("Updated cutoff current: ");
            Serial.println(cutoffCurrent);
        }
        else
        {
            valid = false;
            errorMsg += "Cutoff current out of range. ";
        }
    }

    if (!valid)
    {
        server.send(400, "text/plain", errorMsg);
        return;
    }

    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

void handleData()
{
    String json = "{";
    json += "\"voltage\":" + String(batteryVoltage, 1) + ",";
    json += "\"current\":" + String(batteryCurrent, 1) + ",";
    json += "\"canId\":\"" + webCANId + "\",";
    json += "\"cutoffCurrent\":" + String(cutoffCurrent, 1) + ",";
    json += "\"canData\":\"" + webCANData + "\",";
    json += "\"chargingStatus\":\"" + chargingStatusText + "\",";
    json += "\"targetVoltage\":" + String(targetVoltage, 1) + ",";
    json += "\"targetCurrent\":" + String(targetCurrent, 1);
    json += "}";
    server.send(200, "application/json", json);
}

void handleControl()
{
    if (server.hasArg("cmd"))
    {
        String cmd = server.arg("cmd");
        if (cmd == "start")
        {
            startCharger();
        }
        else if (cmd == "stop")
        {
            stopCharger();
        }
    }
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}
