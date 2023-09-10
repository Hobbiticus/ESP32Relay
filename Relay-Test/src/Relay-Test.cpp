#include <Arduino.h>
#include "EspNowRelay.h"
#include <WiFi.h>
//C8:C9:A3:D2:9D:C8
unsigned char RelayMAC[6] = {0xC8, 0xC9, 0xA3, 0xD2, 0x9D, 0xC8};

EspNowRelay relay;

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  Serial.begin(115200);
  Serial.println(WiFi.macAddress());

  relay.Init(RelayMAC);

  unsigned int addr;
  unsigned char* pAddr = (unsigned char*)&addr;
  pAddr[0] = 192; pAddr[1] = 168; pAddr[2] = 1; pAddr[3] = 55;
  Serial.println("Connecting...");
  int sock = relay.Connect(addr, 4565);
  Serial.printf("Connect returned %d\n", sock);

  if (sock >= 0)
  {
    digitalWrite(2, HIGH);
    unsigned char asdf[1024];
    for (int i = 0; i < sizeof(asdf); i++)
    {
      asdf[i] = (unsigned char)((i % 26) + 'a');
    }
    relay.Send(sock, asdf, sizeof(asdf));
    int bytesRecv = relay.Recv(sock, asdf, sizeof(asdf));
    Serial.printf("Received %d bytes\n", bytesRecv);
    delay(1000);
    //relay.Close(sock);
  }
}

void loop()
{
  delay(1000);
}
