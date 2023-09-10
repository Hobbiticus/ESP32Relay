#pragma once

#include "../../Relay-EspNow/include/NowMessages.h"

#pragma pack(push, 1)

#define UNIQUE_WORD 0x6F7DEAF8

struct SMsgHeader
{
    unsigned int m_UW;
    unsigned short m_Length; //length of entire packet
    unsigned short m_CRC;    //CRC of entire message
    unsigned char m_MAC[6];  //who this message is going to/coming from
};

// //connect request
struct SMsgConnect
{
     SMsgHeader m_Header;
     NowMsgConnect m_Msg;
};

//connect response (positive socket value = success)
struct SMsgConnectResult
{
    SMsgHeader m_Header;
    NowMsgConnectResult m_Msg;
};

//close socket
struct SMsgClose
{
    SMsgHeader m_Header;
    NowMsgClose m_Msg;
};

//not sure if we want to support accepting sockets...

struct SMsgData
{
    SMsgHeader m_Header;
    NowMsgData m_Msg;
    //data follows
};

struct SMsgSocketKeepaliveRequest
{
    SMsgHeader m_Header;
    NowMsgSocketKeepaliveRequest m_Msg;
};

struct SMsgSocketKeepaliveResponse
{
    SMsgHeader m_Header;
    NowMsgSocketKeepaliveResponse m_Msg;
};

#pragma pack(pop)

size_t ReadFromSerial(unsigned char* buff, unsigned int buffSize, HardwareSerial& hwSerial)
{
start_again:
  if (!hwSerial.available())
    return 0;
  
  //look for the unique word
  size_t bytesRead = hwSerial.readBytes(buff, 4);
  if (bytesRead != 4)
    return 0;
  const static unsigned int UW = UNIQUE_WORD;
  unsigned int* uw = (unsigned int*)buff;
  while (*uw != UW)
  {
    //try the next byte
    memmove(buff, buff + 1, 3);
    if (!hwSerial.available())
      return 0; //not found
    bytesRead = hwSerial.readBytes(buff + 3, 1);
    if (bytesRead != 1)
      return 0; //not found
  }

  SMsgHeader* header = (SMsgHeader*)buff;

  //found unique word, read the message length now
  bytesRead = hwSerial.readBytes((unsigned char*)&header->m_Length, 2);
  if (bytesRead != 2)
    goto start_again; //blast, still isn't an actual message

  //make sure the length isn't too big
  if (header->m_Length > buffSize)
    goto start_again;

  //found the length, read the rest
  bytesRead = hwSerial.readBytes((unsigned char*)&header->m_CRC, header->m_Length - 6);
  if (bytesRead != header->m_Length - 6)
    goto start_again;
  
  //check the CRC
  //TODO: check the CRC
  if (header->m_CRC != 0)
    goto start_again;
  
  return bytesRead + 6;
}

bool WriteToSerial(unsigned char* buff, unsigned int buffSize, HardwareSerial& hwSerial)
{
    while (!hwSerial.availableForWrite())
    {
        delay(10);
    }

    SMsgHeader* header = (SMsgHeader*)buff;
    header->m_CRC = 0; //TODO: actually write this
    
    size_t bytesWritten = 0;
    while (bytesWritten < buffSize)
    {
        size_t numBytes = hwSerial.write(buff + bytesWritten, buffSize - bytesWritten);
        if (numBytes <= 0)
            return false; //failed to write
        bytesWritten += numBytes;
    }

    //TODO: wait for ACK
    return true;
}