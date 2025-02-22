void handleRoot()
{
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>CAN Monitor</title>";

  // CSS for minimalist design
  html += "<style>";
  html += "* { margin: 0; padding: 0; box-sizing: border-box; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; }";
  html += "body { max-width: 800px; margin: 0 auto; padding: 20px; background: #f5f5f5; }";
  html += ".card { background: white; border-radius: 8px; padding: 20px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += ".grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; }";
  html += ".info-list { list-style: none; }";
  html += ".info-list li { padding: 8px 0; border-bottom: 1px solid #eee; display: flex; justify-content: space-between; }";
  html += ".info-list li:last-child { border-bottom: none; }";
  html += ".info-label { color: #666; font-weight: 500; }";
  html += ".info-value { color: #333; }";
  html += ".metric { margin-bottom: 15px; }";
  html += ".metric-label { color: #666; font-size: 0.9em; }";
  html += ".metric-value { font-size: 1.2em; font-weight: 500; }";
  html += ".alert { background: #ffebee; color: #c62828; padding: 12px; border-radius: 6px; margin-bottom: 20px; display: none; }";
  html += "h1, h2 { color: #333; margin-bottom: 20px; }";
  html += "form { display: grid; gap: 15px; }";
  html += "input[type='number'] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }";
  html += "button, input[type='submit'] { background: #2196F3; color: white; border: none; padding: 10px 20px; border-radius: 4px; cursor: pointer; }";
  html += "button:hover, input[type='submit']:hover { background: #1976D2; }";
  html += ".button-group { display: flex; gap: 10px; }";

  // css
  html += ".info-value.error {";
  html += "  background: #ffebee;";
  html += "  color: #c62828;";
  html += "  padding: 2px 4px;";
  html += "  border-radius: 4px;";
  html += "}";

  // Example of other CSS rules in the same format
  html += "button:hover, input[type='submit']:hover {";
  html += "  background: #1976D2;";
  html += "}";

  html += ".button-group {";
  html += "  display: flex;";
  html += "  gap: 10px;";
  html += "}";

  // Switch styling
  html += ".switch { position: relative; display: inline-block; width: 60px; height: 34px; }";
  html += ".switch input { opacity: 0; width: 0; height: 0; }";
  html += ".slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }";
  html += ".slider:before { position: absolute; content: ''; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }";
  html += "input:checked + .slider { background-color: #2196F3; }";
  html += "input:checked + .slider:before { transform: translateX(26px); }";
  html += "</style>";

  // JavaScript variables and functions
  html += "<script>";
  html += "var maxAllowedVoltage = " + String(MAX_ALLOWED_VOLTAGE, 1) + ";";
  html += "var maxAllowedCurrent = " + String(MAX_ALLOWED_CURRENT, 1) + ";";

  // Improved updateData function
  html += "function updateData() {";
  html += "  fetch('/data')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      Object.keys(data).forEach(key => {";
  html += "        const element = document.getElementById(key);";
  html += "        if (element) {";
  html += "          if (key === 'isActiveOnStartup') {";
  html += "            element.checked = data[key];";
  html += "          } else if (key === 'voltage' || key === 'current') {";
  html += "            element.textContent = data[key] + (key === 'voltage' ? ' V' : ' A');";
  html += "          } else {";
  html += "            element.textContent = data[key];";
  html += "          }";
  html += "        }";
  html += "      });";

  // Safety alert handling
  html += "      const alertDiv = document.getElementById('safetyAlert');";
  html += "      if(data.chargingStatus.indexOf('Cutoff') !== -1) {";
  html += "        alertDiv.style.display = 'block';";
  html += "        alertDiv.textContent = 'Safety Alert: Unsafe condition detected!';";
  html += "      } else {";
  html += "        alertDiv.style.display = 'none';";
  html += "      }";
  html += "    })";
  html += "    .catch(error => console.error('Error fetching data:', error));";
  html += "}";

  // Switch toggle handler
  html += "function toggleSwitch(checkbox) {";
  html += "  fetch('/set?active=' + checkbox.checked)";
  html += "    .then(response => {";
  html += "      if (!response.ok) {";
  html += "        checkbox.checked = !checkbox.checked;";
  html += "        alert('Failed to update switch state');";
  html += "      }";
  html += "    })";
  html += "    .catch(error => {";
  html += "      checkbox.checked = !checkbox.checked;";
  html += "      console.error('Error:', error);";
  html += "    });";
  html += "}";

  // Tambahkan fungsi JavaScript untuk toggle CP Mode (dalam bagian <script>)
  html += "function toggleCPMode(checkbox) {";
  html += "  fetch('/set?modeCp=' + checkbox.checked)";
  html += "    .then(response => {";
  html += "      if (!response.ok) {";
  html += "        checkbox.checked = !checkbox.checked;";
  html += "        alert('Failed to update CP Mode');";
  html += "      }";
  html += "    })";
  html += "    .catch(error => {";
  html += "      checkbox.checked = !checkbox.checked;";
  html += "      console.error('Error:', error);";
  html += "    });";
  html += "}";

  html += "setInterval(updateData, 1000);";
  html += "</script>";
  html += "</head><body>";

  // Main content
  html += "<div id='safetyAlert' class='alert'></div>";

  // Status Card with detailed information
  html += "<div class='card'>";
  html += "<h1>CAN Monitor</h1>";
  html += "<ul class='info-list'>";
  html += "<li><span class='info-label'>Uptime</span><span id='uptime' class='info-value'>" + String(uptime) + " sec</span></li>";
  html += "<li><span class='info-label'>Charging Status</span><span id='chargingStatus' class='info-value'>" + chargingStatusText + "</span></li>";
  html += "<li><span class='info-label'>Maximum Voltage Rating</span><span id='maxAllowedVoltage' class='info-value'>" + String(MAX_ALLOWED_VOLTAGE, 1) + " V</span></li>";
  html += "<li><span class='info-label'>Maximum Current Rating</span><span id='maxAllowedCurrent' class='info-value'>" + String(MAX_ALLOWED_CURRENT, 1) + " A</span></li>";
  html += "<li><span class='info-label'>Received CAN ID</span><span id='canId' class='info-value'>" + webCANId + "</span></li>";
  html += "<li><span class='info-label'>Received CAN Data</span><span id='canData' class='info-value'>" + webCANData + "</span></li>";
  html += "<li><span class='info-label'>Charging Voltage</span><span id='voltage' class='info-value'>" + String(batteryVoltage, 1) + " V</span></li>";
  html += "<li><span class='info-label'>Charging Current</span><span id='current' class='info-value'>" + String(batteryCurrent, 1) + " A</span></li>";
  html += "</ul>";
  html += "</div>";

  // Error Status Card
  html += "<div class='card'>";
  html += "<h2>Error Status</h2>";
  html += "<ul class='info-list'>";

  // Hardware Failure
  html += "<li><span class='info-label'>Hardware Failure</span>";
  html += "<span class='info-value' id='hardwareFailure'";
  html += hardwareFailure ? " class='error'>" : ">";
  html += hardwareFailure ? "Hardware Failure" : "OK";
  html += "</span></li>";

  // Charger Temperature
  html += "<li><span class='info-label'>Charger Temperature</span>";
  html += "<span class='info-value' id='chargerTemp'";
  html += chargerTemp ? " class='error'>" : ">";
  html += chargerTemp ? "Overheated" : "Normal";
  html += "</span></li>";

  // Input Voltage
  html += "<li><span class='info-label'>Input Voltage</span>";
  html += "<span class='info-value' id='inputVoltage'";
  html += inputVoltage ? " class='error'>" : ">";
  html += inputVoltage ? "Low Voltage" : "Normal";
  html += "</span></li>";

  // Start State
  html += "<li><span class='info-label'>Start State</span>";
  html += "<span class='info-value' id='startState'";
  html += startState ? " class='error'>" : ">";
  html += startState ? "Initializing" : "Ready";
  html += "</span></li>";

  // Communication Timeout
  html += "<li><span class='info-label'>Communication</span>";
  html += "<span class='info-value' id='communicationTimeout'";
  html += communicationTimeout ? " class='error'>" : ">";
  html += communicationTimeout ? "Timeout" : "Connected";
  html += "</span></li>";

  html += "</ul>";
  html += "</div>";

  // CP signal information
  html += "<div class='card'>";
  html += "<h2>CP Signal Information</h2>";
  html += "<ul class='info-list'>";
  html += "<li><strong>Peak Voltage:</strong> <span id='cp-peak-voltage'> " + String(peakVoltage, 1) + "</span> V</li>";
  html += "<li><strong>Frequency:</strong> <span id='cp-frequency'>" + String(frequency, 1) + " </span> Hz</li>";
  html += "<li><strong>Duty Cycle:</strong> <span id='cp-duty-cycle'>" + String(dutyCycle, 1) + "</span> %</li>";
  html += "<li><strong>CP volt:</strong> <span id='cp-duty-cycle'>" + String(convertAdcToCpVoltage(peakVoltage)) + "</span> V</li>";
  html += "<li><strong>EVSE State:</strong> <span id='cp-duty-cycle'>" + getCPStatus(convertAdcToCpVoltage(peakVoltage)) + "</span> V</li>";
  html += "<li><strong>max evse 220 current:</strong> <span id='cp-duty-cycle'>" + String(getMaxCurrent(dutyCycle)) + "</span> A</li>";
  html += "<li><strong>max current battery obc:</strong> <span id='cp-duty-cycle'>" + String(getMaxCurrentForObc()) + "</span> A</li>";
  html += "<li><strong>watt obc continue:</strong> <span id='cp-duty-cycle'>" + String(getBatteryVoltage() * batteryCurrent) + "</span> W</li>";
  html += "<li><strong>max watt evse:</strong> <span id='cp-duty-cycle'>" + String(getMaxCurrent(dutyCycle) * 220) + "</span> W</li>";

  // html += "<li><strong>max current evse:</strong> <span id='cp-duty-cycle'>" + String(getMaxWattEvse(dutyCycle)) + "</span> A</li>";
  html += "</ul>";
  html += "</div>";

  // Parameters Card
  html += "<div class='card'>";
  html += "<h2>Charger Parameters</h2>";
  html += "<form action='/set' method='GET' onsubmit='return validateForm()'>";
  html += "<div class='grid'>";
  html += "<div><label>Cutoff Current (A)</label><input type='number' step='0.1' name='cutoff' id='cutoffCurrent' min='1' max='15' value='" + String(cutoffCurrent, 1) + "'></div>";
  html += "<div><label>Target Voltage (V)</label><input type='number' step='0.1' name='v' id='targetVoltage' min='30' max='" + String(MAX_ALLOWED_VOLTAGE, 1) + "' value='" + String(targetVoltage, 1) + "'></div>";
  html += "<div><label>Target Current (A)</label><input type='number' step='0.1' name='c' id='targetCurrent' min='1' max='" + String(MAX_ALLOWED_CURRENT, 1) + "' value='" + String(targetCurrent, 1) + "'></div>";
  html += "</div>";
  html += "<input type='submit' value='Update Parameters' style='margin-top: 15px'>";
  html += "</form>";
  html += "</div>";

  // Control Card - Modified to include both switches
  html += "<div class='card'>";
  html += "<h2>Control</h2>";

  // Switch container with improved layout
  html += "<div style='margin-bottom: 20px;'>";

  // First switch with improved layout
  html += "<div style='display: flex; align-items: center; margin-bottom: 12px;'>";
  html += "<span style='flex: 1; font-size: 0.9em;'>charging on startup (only active if cp mode disabled):</span>";
  html += "<label class='switch' style='margin-left: 10px;'>";
  html += "<input type='checkbox' id='isActiveOnStartup' " + String(isActiveOnStartup ? "checked" : "") + " onchange='toggleSwitch(this)'>";
  html += "<span class='slider'></span>";
  html += "</label>";
  html += "</div>";

  // Second switch with matching layout
  html += "<div style='display: flex; align-items: center; margin-bottom: 12px;'>";
  html += "<span style='flex: 1; font-size: 0.9em;'>CP Mode:</span>";
  html += "<label class='switch' style='margin-left: 10px;'>";
  html += "<input type='checkbox' id='modeCp' " + String(cpModeEnabled ? "checked" : "") + " onchange='toggleCPMode(this)'>";
  html += "<span class='slider'></span>";
  html += "</label>";
  html += "</div>";

  html += "</div>";

  // Keep the existing buttons
  html += "<div class='button-group'>";
  html += "<button onclick=\"location.href='/control?cmd=start'\">Start Charging</button>";
  html += "<button onclick=\"location.href='/control?cmd=stop'\">Stop Charging</button>";
  html += "<button onclick=\"location.href='/control?cmd=restart'\">Restart</button>";
  html += "</div>";

  html += "</div>";

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
      preferences.putFloat("targetVoltage", targetVoltage); // Save to preferences
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

  // Handle the startup toggle switch.
  // If the checkbox "active" is present in the GET request, it is checked (true); if not, it is false.
  // Fixed auto-startup switch handling
  if (server.hasArg("active"))
  {
    String activeValue = server.arg("active");
    if (activeValue == "true" || activeValue == "false")
    {
      isActiveOnStartup = (activeValue == "true");
      preferences.putBool("onStartup", isActiveOnStartup);
      Serial.print("Updated isActiveOnStartup: ");
      Serial.println(isActiveOnStartup ? "true" : "false");
    }
    else
    {
      valid = false;
      errorMsg += "Invalid active value. ";
    }
  }

  // Tambahkan penanganan untuk modeCp
  if (server.hasArg("modeCp"))
  {
    String modeCpValue = server.arg("modeCp");
    if (modeCpValue == "true" || modeCpValue == "false")
    {
      cpModeEnabled = (modeCpValue == "true");
      preferences.putBool("cpMode", cpModeEnabled);
      Serial.println("Updated modeCp: " + modeCpValue);
      if (cpModeEnabled)
      {
        // Jika CP Mode diaktifkan, nonaktifkan charging on startup
        isActiveOnStartup = false;
        preferences.putBool("onStartup", false);

        // Stop charging jika sedang berjalan
        if (chargerActive)
        {
          stopCharger();
        }
      }

      Serial.print("Updated modeCp: ");
      Serial.println(cpModeEnabled ? "true" : "false");
    }
    else
    {
      valid = false;
      errorMsg += "Invalid modeCp value. ";
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
  json += "\"targetCurrent\":" + String(targetCurrent, 1) + ",";
  json += "\"isActiveOnStartup\":" + String(isActiveOnStartup ? "true" : "false");
  json += "\"modeCp\":" + String(cpModeEnabled ? "true" : "false");
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
    else if (cmd == "restart")
    {
      Serial.println("Restarting ESP32...");
      server.send(200, "text/plain", "Restarting...");
      delay(100); // Ensure response is sent
      ESP.restart();
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
bool keyExists(const char *input, const char *key)
{
  size_t keyLength = strlen(key) + 1;
  char keyWithColon[keyLength + 1];
  strcpy(keyWithColon, key);
  strcat(keyWithColon, ":");
  return strncmp(input, keyWithColon, keyLength) == 0;
}

const char *getValueKey(const char *input, const char *key)
{
  size_t keyLength = strlen(key) + 1;
  return input + keyLength;
}
void serialLoop()
{
  static char dat[2548];   // Buffer untuk menampung data serial
  static int datIndex = 0; // Indeks untuk buffer, statis agar tersimpan antar panggilan

  while (Serial.available() > 0)
  {
    char incomingChar = Serial.read();

    // Pastikan indeks tidak melebihi ukuran buffer
    if (datIndex < sizeof(dat) - 1)
    {
      dat[datIndex++] = incomingChar;
    }
    else
    {
      Serial.println("Warning: Buffer overflow, reset buffer");
      datIndex = 0; // Reset jika terjadi overflow
    }

    // Jika ditemukan karakter newline, proses pesan yang telah diterima
    if (incomingChar == '\n')
    {
      // Jika ada double newline, lewati pesan kosong
      if (datIndex > 1 && dat[datIndex - 2] == '\n')
      {
        datIndex = 0; // Reset buffer agar tidak terus menumpuk
        continue;
      }
      dat[datIndex - 1] = '\0'; // Null-terminate string

      // Cek apakah pesan mengandung key "WRITE_SIMULATE"
      if (keyExists(dat, "WRITE_SIMULATE"))
      {
        const char *value = getValueKey(dat, "WRITE_SIMULATE");
        String fullValue = String(value);

        // Mencari posisi koma untuk mem-parsing parameter
        int firstComma = fullValue.indexOf(',');
        int secondComma = fullValue.indexOf(',', firstComma + 1);

        // Validasi posisi koma
        if (firstComma == -1 || secondComma == -1)
        {
          Serial.println("Error: Format tidak valid. Format yang diharapkan: 'status,voltage,current'");
          datIndex = 0; // Reset buffer setelah error
          continue;     // Hentikan pemrosesan jika format salah
        }

        // Mengkonversi nilai parameter
        bool _status = fullValue.substring(0, firstComma).toInt();
        float _voltage = fullValue.substring(firstComma + 1, secondComma).toFloat();
        float _current = fullValue.substring(secondComma + 1).toFloat();

        // Tampilkan data pada Serial Monitor
        Serial.print("status:");
        Serial.print(String(_status));
        Serial.print(", volt:");
        Serial.print(String(_voltage));
        Serial.print(", curr:");
        Serial.println(String(_current));

        // Panggil fungsi simulasi CAN bus dengan nilai yang telah diparsing
        simulateCanbus(_voltage, _current, _status);
      }

      // Reset buffer agar siap untuk membaca pesan berikutnya
      datIndex = 0;
    }
  }
}