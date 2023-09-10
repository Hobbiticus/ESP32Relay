#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include "../../Relay-WiFi/include/SerialMessages.h"
#include "../include/NowMessages.h"

//TODO: have a way to resend incomplete/corrupt serial data?
//TODO: use a UW for the start of messages?

//pins 16/17
HardwareSerial SerialLink(2);

EventGroupHandle_t SendEvent;
unsigned char SendSuccess;

void xxd(const unsigned char* data, unsigned int dataLen)
{
  for (int i = 0; i < dataLen; i++)
  {
    Serial.printf("%02hhx ", data[i]);
  }
  Serial.println("");
}


static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  SendSuccess = status == ESP_OK;
  xEventGroupSetBits(SendEvent, 1);
}

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

  //this is just a pass-through for the wifi module
  NowMsgHeader* nowHeader = (NowMsgHeader*)incomingData;
  Serial.printf("Received %d bytes from %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx, type = %hhu\n", len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], nowHeader->m_Type);
  xxd(incomingData, len);
  unsigned char buffer[250 + sizeof(SMsgHeader)];
  SMsgHeader* header = (SMsgHeader*)buffer;
  header->m_UW = UNIQUE_WORD;
  header->m_Length = len + sizeof(SMsgHeader);
  memcpy(header->m_MAC, mac, 6);
  memcpy(header + 1, incomingData, len);

  WriteToSerial(buffer, sizeof(SMsgHeader) + len, SerialLink);
}

void setup()
{
  Serial.begin(115200);
  Serial.println(WiFi.macAddress());
  SerialLink.begin(115200); //TODO: see if this can go higher
  WiFi.mode(WIFI_MODE_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  SendEvent = xEventGroupCreate();
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW started");
}

bool SendNow(unsigned char mac[6], const unsigned char* data, unsigned int dataLength)
{
  for (int tries = 0; tries < 3; tries++)
  {
    if (esp_now_send(mac, data, dataLength) == ESP_NOW_SEND_SUCCESS)
    {
      Serial.println("Sent successfully!");
      return true;
    }
  }
  Serial.println("Failed to send");
  return false;
}

void loop()
{
  //this reads from the serial port and forwards the stuff out to ESP-NOW
  unsigned char inBuff[1024];
  while (SerialLink.available())
  {
    size_t numBytes = ReadFromSerial(inBuff, sizeof(inBuff), SerialLink);
    if (numBytes <= 0)
      break;

    SMsgHeader* header = (SMsgHeader*)inBuff;
    NowMsgHeader* nowHeader = (NowMsgHeader*)(header + 1);
    Serial.printf("Got a SERIAL message of type %hhu\n", nowHeader->m_Type);
    xxd(inBuff, header->m_Length);
    Serial.printf("Sending to %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx --> ", header->m_MAC[0], header->m_MAC[1], header->m_MAC[2], header->m_MAC[3], header->m_MAC[4], header->m_MAC[5]);
    xxd((const unsigned char*)(nowHeader), header->m_Length - sizeof(SMsgHeader));
    SendNow(header->m_MAC, (const unsigned char*)(nowHeader), header->m_Length - sizeof(SMsgHeader));
  }
  delay(10);
}
//64:B7:08:CA:5E:58 <-- tester
