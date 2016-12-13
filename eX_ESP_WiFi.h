#ifndef EX_ESP_WIFI_H
#define EX_ESP_WIFI_H

#include <WiFiUdp.h>

WiFiUDP UdP;

const char *ssid = "= eX-Robot =";
const char *password = "1231231231";
uint16_t   RxPort = 8000;
uint16_t   TxPort = 9000;

char PacketBuffer[128];                 //буфер для хранения принимаемых и отправляемых пакетов
int  PacketSize = 0;

union{
  unsigned char Buff[4];
  float d;
}u;

void WiFi_Start()
{
//  WiFi.disconnect();
  WiFi.softAP(ssid, password);
  UdP.begin(RxPort);
}

boolean WiFi_MSG_Read()
{
  PacketSize = UdP.parsePacket();
  if (!PacketSize) return false;
  UdP.read(PacketBuffer,PacketSize); // read the packet into the buffer
  return true;
}

void WiFi_MSG_Send_Float(char *c, int msgSize, float p)
{
  u.d = p;
  c[msgSize-4] = u.Buff[3];
  c[msgSize-3] = u.Buff[2];
  c[msgSize-2] = u.Buff[1];
  c[msgSize-1] = u.Buff[0];
  UdP.beginPacket(UdP.remoteIP(), TxPort);
  UdP.write(c, msgSize);
  UdP.endPacket();
}

#endif //EX_ESP_WIFI_H
