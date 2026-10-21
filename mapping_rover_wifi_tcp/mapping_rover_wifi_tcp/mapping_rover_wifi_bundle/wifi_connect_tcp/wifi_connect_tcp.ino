#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

// Wi-Fi credentials
const char* WIFI_SSID = "HONOR";
const char* WIFI_PASSWORD = "aaaaaaaaa";

// Laptop IP and UDP port
const char* LAPTOP_IP = "10.249.219.100";
const uint16_t LAPTOP_PORT = 5005;

// ESP32 UDP receive port for laptop ACKs
const uint16_t ESP32_UDP_PORT = 5006;

// STM32 UART connection
static const uint32_t STM32_BAUD = 115200;
static const int STM32_RX_PIN = 16;
static const int STM32_TX_PIN = 17;

// UDP
WiFiUDP udp;
char packetBuffer[256];

// STM32 line buffer
char lineBuffer[160];
size_t lineLength = 0;
bool droppingLongLine = false;

// Wait for an ACK from the laptop over UDP
bool waitForAck(uint32_t timeout_ms) {
  uint32_t start = millis();

  while (millis() - start < timeout_ms) {
    int packetSize = udp.parsePacket();

    if (packetSize > 0) {
      
      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);

      if (len >= 0) {
        packetBuffer[len] = '\0';

        Serial.print("UDP response from ");
        Serial.print(udp.remoteIP());
        Serial.print(":");
        Serial.println(udp.remotePort());

        Serial.print("Response: ");
        Serial.println(packetBuffer);

        // Accept only the exact ACK response
        if (strcmp(packetBuffer, "ACK") == 0) {
          return true;
        }
      }
    }

    delay(1);
  }

  Serial.println("Laptop ACK timeout");
  return false;
}

// Send ACK to STM32 through UART2
void sendAckToSTM32() {
  Serial2.print("A\r\n");
  Serial2.flush();
  Serial.println("ACK transmitted over UART2");
}

void setup() {
  Serial.begin(115200);

  // STM32 PA2 TX -> ESP32 GPIO16 RX
  // STM32 PA3 RX <- ESP32 GPIO17 TX
  Serial2.begin(
    STM32_BAUD,
    SERIAL_8N1,
    STM32_RX_PIN,
    STM32_TX_PIN
  );

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  if (udp.begin(ESP32_UDP_PORT)) {
    Serial.print("UDP listening on port ");
    Serial.println(ESP32_UDP_PORT);
  } else {
    Serial.println("UDP initialization failed");
  }
}

void loop() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();

    if (c == '\n') {
      if (!droppingLongLine && lineLength > 0) {
        // Terminate the STM32 message
        lineBuffer[lineLength] = '\0';

        Serial.print("STM32: ");
        Serial.println(lineBuffer);

        bool laptopAckReceived = false;

        if (WiFi.status() == WL_CONNECTED) {
          // Forward the STM32 record to the laptop
          if (udp.beginPacket(LAPTOP_IP, LAPTOP_PORT)) {
            udp.write(
              (const uint8_t*)lineBuffer,
              lineLength
            );

            if (udp.endPacket()) {
              Serial.println("UDP packet sent; waiting for laptop ACK");

              laptopAckReceived = waitForAck(500);
            } else {
              Serial.println("UDP packet send failed");
            }
          } else {
            Serial.println("Could not start UDP packet");
          }
        } else {
          Serial.println("Wi-Fi disconnected; no ACK sent");
        }

        // Acknowledge STM32 only if the laptop ACKed
        if (laptopAckReceived) {
          sendAckToSTM32();
        } else {
          Serial.println("No laptop ACK; STM32 will time out");
        }
      }

      lineLength = 0;
      droppingLongLine = false;
    }
    else if (c != '\r' && !droppingLongLine) {
      if (lineLength < sizeof(lineBuffer) - 1) {
        lineBuffer[lineLength++] = c;
      } else {
        lineLength = 0;
        droppingLongLine = true;
        Serial.println("Warning: STM32 line too long");
      }
    }
  }
}