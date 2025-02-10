/*
NOTES:
SESUAIKAN DAHULU DENGAN SPESIFIKASI RATING MAKSIMUM DARI CHARGER/BATTERY UNTUK SAFETY KESALAHAN INPUT VOLTAGE/CURRENT
VARIABLE DARI MAX_ALLOWED_VOLTAGE MAX_ALLOWED_CURRENT
*/

#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h> // Library for internal (non-volatile) storage

#define CAN_CS 5  // Chip Select pin for CAN
#define CAN_INT 4 // Interrupt pin for CAN

MCP_CAN CAN(CAN_CS); // Create CAN instance

// ----- Shared Preferences Setup -----
Preferences preferences;

// ----- WiFi & Web Server Setup -----
const char *ssid = "enpowerCharger2";
// const char *ssid = "ra6pCharger";
const char *password = "password123";
WebServer server(80);

const float CELL_VOLTAGE = 3.5; // Maximum per-cell voltage
const int NUM_CELLS = 120;      // 32S configuration

// HARUS DISESUAIKAN DENGAN SPESIFIKASI MAKSIMUM DARI CHARGER
const float MAX_ALLOWED_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // Maximum allowed voltage (115.2V)
const float MAX_ALLOWED_CURRENT = 5;                        // Maximum allowed current

// Charging parameters (default values)
float targetVoltage = MAX_ALLOWED_VOLTAGE; // Default target voltage
float targetCurrent = 6;                   // Default target current

// ----- Other Global Variables -----
unsigned long pmillis = 0;             // For timing the CAN command transmissions
bool chargerActive = false;            // Charging flag (true = charging, false = stopped)
String chargingStatusText = "Stopped"; // Charging status text for display

// Global variables for web display
String webCANId = "";
String webCANData = "";
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;

// Charging current cutoff
float cutoffCurrent = 4.0;               // Default cutoff current
unsigned long cutoffStartTime = 0;       // Timer untuk cutoff
bool checkingCutoff = false;             // Flag pengecekan cutoff
bool cutoffTriggered = false;            // Flag untuk menandai bahwa cutoff telah terjadi
const unsigned long delayCutoff = 30000; // Delay cutoff dalam milidetik
unsigned long uptime;

#define CHARGER_CONTROL_ID 0x1806E5F4
byte sendCanCounter = 0;
// ----- Function Prototypes -----
void handleRoot();
void handleSet();
void handleControl();
void handleData();
void sendChargerCommand();
void stopChargerCommand();
void stopCharger();
void startCharger();
void handleReceivingCanbus();
void simulateCanbus(float _batteryVoltage, float _batteryCurrent, bool _state);
void serialLoop();
void decodeChargerBroadcast(byte msgData[], byte len);

#include <handle_web.h>
#include <handle_canbus.h>

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial to be ready

  // Initialize CAN bus
  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    Serial.println("CAN init failed");
    while (1)
      ;
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  Serial.println("CAN Ready");

  // Initialize shared preferences and load stored settings.
  preferences.begin("settings", false);
  targetVoltage = preferences.getFloat("targetVoltage", targetVoltage);
  targetCurrent = preferences.getFloat("targetCurrent", targetCurrent);
  cutoffCurrent = preferences.getFloat("cutoffCurrent", cutoffCurrent);

  Serial.print("Loaded target voltage: ");
  Serial.println(targetVoltage);
  Serial.print("Loaded target current: ");
  Serial.println(targetCurrent);
  Serial.print("Loaded target cutoff current: ");
  Serial.println(cutoffCurrent);

  // Initialize WiFi in Access Point mode
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/control", handleControl);
  server.on("/data", handleData);
  server.begin();
}

// Di loop utama
void loop()
{
  server.handleClient();
  serialLoop();
  // Periodic CAN command sending
  if (millis() - pmillis > 1000)
  {
    pmillis = millis();
    uptime = millis() / 1000;
    sendCanCounter += 1;
    if (chargerActive)
    {
      sendChargerCommand();
    }

    // if (sendCanCounter >= 3)
    // {
    //   sendCanCounter = 0;
    //   if (chargerActive)
    //   {
    //     sendChargerCommand();
    //   }
    // }
  }

  handleReceivingCanbus();

  // Logika Cutoff yang diperbaiki
  if (chargerActive && !cutoffTriggered)
  {
    // if (batteryCurrent < cutoffCurrent)
    // {
    //   // Mulai timer jika belum dimulai
    //   if (!checkingCutoff)
    //   {
    //     cutoffStartTime = millis();
    //     checkingCutoff = true;
    //     Serial.println("Start monitoring low current...");
    //   }
    //   // Cek apakah sudah 10 detik kontinyu
    //   else if ((millis() - cutoffStartTime) >= delayCutoff)
    //   {
    //     // Trigger cutoff setelah 10 detik
    //     stopCharger();
    //     cutoffTriggered = true;
    //     chargingStatusText = "cutoff current";
    //     Serial.println("=====================================");
    //     Serial.println("Cutoff Triggered after continuous!");
    //     Serial.print("Battery Voltage: ");
    //     Serial.print(batteryVoltage);
    //     Serial.println(" V");
    //     Serial.print("Battery Current: ");
    //     Serial.print(batteryCurrent);
    //     Serial.println(" A");
    //     Serial.print("Cutoff Current Threshold: ");
    //     Serial.print(cutoffCurrent);
    //     Serial.println(" A");
    //     Serial.println("=====================================");
    //   }
    // }
    // else
    // {
    //   // Reset timer jika arus naik di atas cutoff
    //   if (checkingCutoff)
    //   {
    //     checkingCutoff = false;
    //     Serial.println("Current recovered, reset monitoring");
    //   }
    // }
  }
  else
  {
    checkingCutoff = false;
  }

  delay(100);
}

void startCharger()
{
  chargerActive = true;
  checkingCutoff = false;
  cutoffTriggered = false; // Reset the cutoff flag when charging starts
  chargingStatusText = "Charging";
  Serial.println("Charger started");
}

void stopCharger()
{
  chargerActive = false;
  stopChargerCommand();
  checkingCutoff = false;
  cutoffTriggered = false; // Clear any previous cutoff trigger
  chargingStatusText = "Stopped";
  Serial.println("Charger stopped");
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
