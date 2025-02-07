#include "WebServerHandler.h"
#include "CANHandler.h"
#include "ChargerCommands.h"

const char *ssid = "bms Charger";
const char *password = "bms12345";

void setupWebServer()
{
  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/control", handleControl);
  server.on("/data", handleData);
  server.begin();
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());
}
// Tampilan antarmuka web
void handleRoot()
{
  String html = R"=====(
  <!DOCTYPE html><html><head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Battery Charger</title>
  <style>
    body { 
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 10px;
      background: #f5f5f5;
    }
    .container {
      max-width: 400px;
      margin: 0 auto;
    }
    .card {
      background: white;
      border-radius: 8px;
      padding: 15px;
      margin: 10px 0;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    button {
      padding: 10px;
      margin: 5px;
      background: #4CAF50;
      color: white;
      border: none;
      border-radius: 4px;
    }
  </style>
  <script>
  setInterval(function() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if(this.readyState == 4 && this.status == 200) {
        var data = JSON.parse(this.responseText);
        document.getElementById("chargingStatus").textContent = data.status;
        document.getElementById("canId").textContent = data.canId;
        document.getElementById("canData").textContent = data.canData;
        document.getElementById("battVolt").textContent = data.voltage.toFixed(1) + 'V';
        document.getElementById("battCurrent").textContent = data.current.toFixed(1) + 'A';
      }
    };
    xhttp.open("GET", "/data", true);
    xhttp.send();
  }, 1000);
  </script></head>
  <body>
    <div class="container">
      <div class="card">
        <div>Status Charging: <span id="chargingStatus">)=====" +
                chargingStatus + R"=====(</span></div>
        <div>ID CAN Terakhir: <span id="canId">)=====" +
                receivedCANId + R"=====(</span></div>
        <div>Data CAN: <span id="canData">)=====" +
                receivedCANData + R"=====(</span></div>
        <div>Tegangan Baterai: <span id="battVolt">0.0V</span></div>
        <div>Arus Baterai: <span id="battCurrent">0.0A</span></div>
        <button onclick="fetch('/control?cmd=start')">Start</button>
        <button onclick="fetch('/control?cmd=stop')">Stop</button>
      </div>
    </div>
  </body></html>)=====";
  server.send(200, "text/html", html);
}
// Fungsi untuk menangani perubahan parameter
void handleSet()
{
  String response;
  if (server.hasArg("v"))
  {
    float newVoltage = server.arg("v").toFloat();
    Serial.print("Voltage changed to: ");
    Serial.println(newVoltage);

    if (newVoltage > MAX_VOLTAGE)
    {
      response = "Voltage exceeds 115.2V limit!";
      server.send(400, "text/plain", response);
      return;
    }
    setVoltage = newVoltage;
  }

  if (server.hasArg("c"))
  {
    float newCurrent = server.arg("c").toFloat();
    Serial.print("Current changed to: ");
    Serial.println(newCurrent);

    if (newCurrent > MAX_CURRENT)
    {
      response = "Current exceeds 35A limit!";
      server.send(400, "text/plain", response);
      return;
    }
    setCurrent = newCurrent;
  }

  Serial.println("New parameters saved");
  server.send(200, "text/plain", "Parameters updated");
}
// Fungsi kontrol start/stop charger
void handleControl()
{
  if (server.hasArg("cmd"))
  {
    String cmd = server.arg("cmd");
    Serial.print("Received command: ");
    Serial.println(cmd);

    if (cmd == "start")
    {
      Serial.println("Starting charger...");
      chargerActive = true;
    }
    else if (cmd == "stop")
    {
      Serial.println("Stopping charger...");
      stopChargerCommand();
    }
  }
  server.send(200, "text/plain", "Command received");
}
// API untuk mendapatkan data
void handleData()
{
  JsonDocument doc;
  doc["status"] = chargingStatus;
  doc["canId"] = receivedCANId;
  doc["canData"] = receivedCANData;
  doc["voltage"] = batteryVoltage;
  doc["current"] = batteryCurrent;

  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// Implementasi fungsi handleRoot(), handleSet(), handleControl(), dan handleData()
// ... (sama seperti kode sebelumnya)
