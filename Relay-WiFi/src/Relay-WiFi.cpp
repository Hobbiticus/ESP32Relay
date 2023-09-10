#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include "../include/SerialMessages.h"
#include <map>
#include <vector>

#define MY_SSID "cjweiland"
#define MY_WIFI_PASSWORD "areallygoodkey"

//pins 16/17
HardwareSerial SerialLink(2);

void xxd(const unsigned char* data, unsigned int dataLen)
{
  for (int i = 0; i < dataLen; i++)
  {
    Serial.printf("%02hhx ", data[i]);
  }
  Serial.println("");
}

struct Socket
{
  WiFiClient* m_Socket;
  unsigned long m_LastSeenTime;
  unsigned long m_LastKeepaliveTime;
  unsigned char m_MAC[6];
};
std::map<int, Socket> gSockets;

void setup()
{
  Serial.begin(115200);
  SerialLink.begin(115200);

  //connect to WiFi
  WiFi.begin(MY_SSID, MY_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.println("Waiting for wifi to connect...");
  }
  Serial.println("Connected to WiFi!");

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}


void SendCloseSocket(unsigned char mac[6], int sock)
{
  SMsgClose out;
  out.m_Header.m_UW = UNIQUE_WORD;
  out.m_Header.m_Length = sizeof(out);
  memcpy(out.m_Header.m_MAC, mac, 6);
  out.m_Msg.m_Header.m_Type = NOWMSG_CLOSE;
  out.m_Msg.m_Socket = sock;
  WriteToSerial((uint8_t*)&out, sizeof(out), SerialLink);  
}

void CheckSerial()
{
  unsigned char inBuff[1024];
  while (SerialLink.available())
  {
    size_t numBytes = ReadFromSerial(inBuff, sizeof(inBuff), SerialLink);
    if (numBytes <= 0)
      break;

    SMsgHeader* header = (SMsgHeader*)inBuff;
    NowMsgHeader* nowHeader = (NowMsgHeader*)(header + 1);

    Serial.printf("Got message len = %hu, type = %hhu\n", header->m_Length, nowHeader->m_Type);
    xxd(inBuff, header->m_Length);

    //handle this message!
    switch (nowHeader->m_Type)
    {
      case NOWMSG_CONNECT:
      {
        NowMsgConnect* msg = (NowMsgConnect*)nowHeader;
        char ip[16] = {0};
        unsigned char* pIP = (unsigned char*)(&msg->m_Address);
        sprintf(ip, "%hhu.%hhu.%hhu.%hhu", pIP[0], pIP[1], pIP[2], pIP[3]);
        Serial.printf("Connecting to %s:%hu...\n", ip, msg->m_Port);
        WiFiClient* sock = new WiFiClient();

        SMsgConnectResult out;
        out.m_Msg.m_Address = msg->m_Address;
        out.m_Msg.m_Port = msg->m_Port;
        out.m_Header.m_UW = UNIQUE_WORD;
        out.m_Header.m_Length = sizeof(out);
        memcpy(out.m_Header.m_MAC, header->m_MAC, 6);
        out.m_Msg.m_Header.m_Type = NOWMSG_CONNECT_RESULT;

        if (!sock->connect(ip, msg->m_Port))
        {
          //TODO: send response that went badly
          out.m_Msg.m_Socket = -1;
          WriteToSerial((uint8_t*)&out, sizeof(out), SerialLink);
          delete sock;
          return;
        }

        //yay! connected!  inform the other side
        Socket s;
        s.m_Socket = sock;
        s.m_LastSeenTime = millis();
        s.m_LastKeepaliveTime = s.m_LastSeenTime;
        memcpy(s.m_MAC, header->m_MAC, 6);
        gSockets[sock->fd()] = s;

        out.m_Msg.m_Socket = sock->fd();
        WriteToSerial((uint8_t*)&out, sizeof(out), SerialLink);
      }
      break;
      case NOWMSG_CLOSE:
      {
        SMsgClose* msg = (SMsgClose*)header;
        std::map<int, Socket>::iterator itr = gSockets.find(msg->m_Msg.m_Socket);
        if (itr == gSockets.end())
          return; //nothing to do
        //dispose of the socket
        Socket sock = (*itr).second;
        sock.m_Socket->stop();
        delete sock.m_Socket;
        gSockets.erase(itr);
      }
      break;
      case NOWMSG_DATA:
      {
        SMsgData* msg = (SMsgData*)header;
        std::map<int, Socket>::iterator itr = gSockets.find(msg->m_Msg.m_Socket);
        if (itr == gSockets.end())
        {
          //send a Close message to indicate that the connection doesn't exist
          SendCloseSocket(header->m_MAC, msg->m_Msg.m_Socket);
          break;
        }

        //send an ACK? or maybe not? idk...
        Socket& sock = (*itr).second;
        sock.m_LastSeenTime = millis();
        unsigned char* outData = (unsigned char*)(msg + 1);
        unsigned int outLength = header->m_Length - sizeof(SMsgData);
        if (sock.m_Socket->write(outData, outLength) != outLength)
        {
          //close this socket i guess
          SendCloseSocket(header->m_MAC, msg->m_Msg.m_Socket);
          //dispose of the socket
          sock.m_Socket->stop();
          delete sock.m_Socket;
          gSockets.erase(itr);
          break;
        }
        Serial.println("Sent data to wifi successfully!");
      }
      break;
      case NOWMSG_KEEPALIVE_RESP:
      {
        SMsgSocketKeepaliveResponse* msg = (SMsgSocketKeepaliveResponse*)header;
        Serial.printf("Got keepalive response for %d, in use = %c\n", msg->m_Msg.m_Socket, msg->m_Msg.m_InUse ? 'Y' : 'N');
        std::map<int, Socket>::iterator itr = gSockets.find(msg->m_Msg.m_Socket);
        if (itr == gSockets.end())
          break; //nothing to do!
        Socket& sock = (*itr).second;
        if (msg->m_Msg.m_InUse)
        {
          sock.m_LastSeenTime = sock.m_LastKeepaliveTime = millis();
          break;
        }
        //not in use! destroy it!
        sock.m_Socket->stop();
        delete sock.m_Socket;
        gSockets.erase(itr);
      }
      break;
    }
    //process the next message if it exists
  }
}

void CheckSockets()
{
  unsigned char inBuff[250 - sizeof(NowMsgData) + sizeof(SMsgData)] = {0};
  SMsgData* outMsg = (SMsgData*)inBuff;
  outMsg->m_Header.m_UW = UNIQUE_WORD;
  std::vector<int> socketsToDispose;
  for (std::map<int, Socket>::iterator itr = gSockets.begin(); itr != gSockets.end(); ++itr)
  {
    Socket& sock = (*itr).second;
    while (sock.m_Socket->available())
    {
      sock.m_LastSeenTime = millis();
      size_t numBytes = sock.m_Socket->readBytes(inBuff + sizeof(SMsgData), sizeof(inBuff) - sizeof(SMsgData));
      Serial.printf("Received %d bytes from socket %d\n", numBytes, sock.m_Socket->fd());
      xxd(inBuff + sizeof(NowMsgData), numBytes);
      outMsg->m_Header.m_Length = sizeof(SMsgData) + numBytes;
      memcpy(outMsg->m_Header.m_MAC, sock.m_MAC, 6);
      outMsg->m_Msg.m_Header.m_Type = NOWMSG_DATA;
      outMsg->m_Msg.m_Socket = (*itr).first;
      WriteToSerial(inBuff, sizeof(SMsgData) + numBytes, SerialLink);
    }

    unsigned long nowMS = millis();
    if (nowMS - sock.m_LastSeenTime > 5000)
    {
      Serial.printf("Timing out socket %d!\n", sock.m_Socket->fd());
      SendCloseSocket(sock.m_MAC, sock.m_Socket->fd());
      socketsToDispose.push_back(sock.m_Socket->fd());
      continue;
    }

    //send keepalive if necessary
    if (nowMS - sock.m_LastSeenTime > 2000 && nowMS - sock.m_LastKeepaliveTime > 2000)
    {
      //send keepalive
      Serial.printf("Sending keepalive req for socket %d at %lu\n", sock.m_Socket->fd(), nowMS);
      SMsgSocketKeepaliveRequest ka;
      ka.m_Header.m_UW = UNIQUE_WORD;
      ka.m_Header.m_Length = sizeof(ka);
      memcpy(ka.m_Header.m_MAC, sock.m_MAC, 6);
      ka.m_Msg.m_Header.m_Type = NOWMSG_KEEPALIVE_REQ;
      ka.m_Msg.m_Socket = sock.m_Socket->fd();
      WriteToSerial((unsigned char*)&ka, sizeof(ka), SerialLink);
      sock.m_LastKeepaliveTime = nowMS; //don't send these too frequently
    }
  }

  for (size_t i = 0; i < socketsToDispose.size(); i++)
  {
    int sfd = socketsToDispose[i];
    Socket& sock = gSockets[sfd];
    sock.m_Socket->stop();
    delete sock.m_Socket;
    gSockets.erase(sfd);
  }
}

void loop()
{
  CheckSerial();
  CheckSockets();
  delay(10);
}
