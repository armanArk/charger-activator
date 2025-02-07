#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#define CAN_CS 5
#define CAN_INT 4

void handleRoot();
void handleSet();
void handleControl();
void handleData();
void sendChargerCommand();
void handleCANMessages();
void printBatteryStatus();

MCP_CAN CAN(CAN_CS);

// Konfigurasi Web Server
const char *ssid = "bms Charger";
const char *password = "bms12345";
WebServer server(80);

// Parameter Charger
const float MAX_VOLTAGE = 115.2f;
const float MAX_CURRENT = 35.0f;
float setVoltage = 108.8f;
float setCurrent = 5.0f;
bool chargerActive = false;
unsigned long lastUpdate = 0;

// Konfigurasi CAN
#define CAN_ID_RECEIVE 0x98FF50E5
#define CAN_ID_RESPONSE 0x1806E5F4

// Status Baterai
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
String chargingStatus = "Disconnected";
String receivedCANId = "";
String receivedCANData = "";

void setup()
{
  Serial.begin(115200);

  // Inisialisasi CAN Bus
  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    Serial.println("Gagal inisialisasi CAN");
    while (1)
      ;
  }
  CAN.setMode(MCP_NORMAL);

  // Membuat WiFi Access Point
  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/control", handleControl);
  server.on("/data", handleData);
  server.begin();
  Serial.println(WiFi.softAPIP());
}

void loop()
{
  server.handleClient();

  if (chargerActive && millis() - lastUpdate > 1000)
  {
    sendChargerCommand();
    lastUpdate = millis();
  }

  if (digitalRead(CAN_INT) == LOW)
  {
    handleCANMessages();
  }
}

// Fungsi untuk memproses pesan CAN
void handleCANMessages()
{
  unsigned long id;
  byte len, data[8];
  int result = CAN.readMsgBuf(&id, &len, data);
  if (result == CAN_OK)
  {
    receivedCANId = "0x" + String(id, HEX);
    receivedCANData = "";
    for (int i = 0; i < len; i++)
    {
      receivedCANData += String(data[i], HEX) + " ";
    }

    if (id == CAN_ID_RECEIVE && len == 8)
    {
      batteryVoltage = ((data[0] << 8) | data[1]) * 0.1f;
      batteryCurrent = ((data[2] << 8) | data[3]) * 0.1f;
      chargingStatus = (data[4] == 0) ? "Charging" : "Stopped";
      printBatteryStatus();
    }
  }
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

// Fungsi pengiriman perintah ke charger
void sendChargerCommand()
{
  uint16_t voltage = setVoltage * 10;
  uint16_t current = setCurrent * 10;
  byte data[8] = {
      highByte(voltage), lowByte(voltage),
      highByte(current), lowByte(current),
      0x00, 0x00, 0x00, 0x00};
  Serial.print("sending: ");
  Serial.print(voltage);
  Serial.println("v,");
  Serial.print(current);
  Serial.println("a");
  // if (CAN.sendMsgBuf(CAN_ID_RESPONSE, 0, 8, data) == CAN_OK)
  // {
  //   Serial.print("Terkirim: ");
  //   Serial.print(setVoltage, 1);
  //   Serial.print("V, ");
  //   Serial.print(setCurrent, 1);
  //   Serial.println("A");
  // }
}

// Fungsi untuk menghentikan charger
void stopChargerCommand()
{
  byte data[8] = {0, 0, 0, 0, 0x01, 0, 0, 0};
  CAN.sendMsgBuf(CAN_ID_RESPONSE, 0, 8, data);
  chargerActive = false;
  Serial.println("Charging dihentikan");
}

// Fungsi bantuan
void printBatteryStatus()

{
  Serial.println("-------------------------------------");
  Serial.print("Tegangan: ");
  Serial.println(batteryVoltage);
  Serial.print("Arus: ");
  Serial.println(batteryCurrent);
  Serial.print("Status: ");
  Serial.println(chargingStatus);
  Serial.println("-------------------------------------");
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
