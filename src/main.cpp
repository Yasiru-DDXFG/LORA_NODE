//    LoRa node for a machine monitoring system
//    Author : Moraketiya D.G.Y.L

#include <LoRa.h>
#include "NODE.h"
#include "Adafruit_SleepyDog.h"
#include "Adafruit_NeoPixel.h"

#ifdef SAMD
//samd cs,rst,irq
NODE mynode(28, 26, 27);
#elif MEGA
//mega cs,rst,irq
NODE mynode(43, 42, 2);
#endif

void setup()
{
  SerialUSB.begin(9600);
  delay(3000);
  //while (!SerialUSB)
  //  ;
  debug_println("SerialUSB init success!");
  pixels.begin();
  pixels.setBrightness(50);
  debug_println("Pixels init success!");

  // Read pins and set device ID.
  mynode.setID();

  //Set tx and rx frequency channel according to the higher byte of Device ID
  uint8_t h_byte = ((mynode.getID() >> 8) & 0x00FF);
  if (h_byte >= 1 && h_byte <= 6)
  {
    mynode.setFrequencies(gaterxFreq1, gatetxFreq1);
  }
  else if (h_byte >= 5 && h_byte <= 9)
  {
    mynode.setFrequencies(gaterxFreq1, gatetxFreq1);
  }
  else if (h_byte >= 10 && h_byte <= 12)
  {
    mynode.setFrequencies(gaterxFreq1, gatetxFreq1);
  }
  else if (h_byte >= 13 && h_byte <= 15)
  {
    mynode.setFrequencies(gaterxFreq1, gatetxFreq1);
  }

  mynode.Init();
  debug_println("LoRa NODE");
  debug_print("Device ID > PREFIX:");
  debug_print((mynode.getID() >> 8) & 0xFF);
  debug_print(" NUMBER:");
  debug_println(mynode.getID() & 0x00FF);

  LoRa.onReceive(callbackfunc);
  LoRa.dumpRegisters(Serial);
  Watchdog.enable(8000);
  debug_println("Watchdog init success!");
}

void loop()
{
  mynode.printStatus();
  mynode.checkStatus();
  Watchdog.reset();
  mynode.checkIO();
  runPixel();
}

/*
#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
  SerialUSB.begin(9600);
  LoRa.setPins(28,26,27);
  while (!SerialUSB);

  SerialUSB.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    SerialUSB.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  SerialUSB.print("Sending packet: ");
  SerialUSB.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(1000);
}
*/