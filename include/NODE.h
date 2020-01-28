#include <LoRa.h>
#include "Adafruit_SleepyDog.h"
#include "Adafruit_NeoPixel.h"

#define DEBUG_PRINT

#define gatetxFreq1 433E6
#define gaterxFreq1 433.5E6
#define gatetxFreq2 433E6
#define gaterxFreq2 433.5E6
#define gatetxFreq3 433E6
#define gaterxFreq3 433.5E6
#define gatetxFreq4 433E6
#define gaterxFreq4 433.5E6

#define PRIVATEKEY 0xB5
#define OFFSET_ID 0x12;


#define LINK_CHECK_INTERVAL 120000
#define JOIN_TX_INTERVAL 10000
#define TX_TIMEOUT 30000
#define NOT_JOINED 0
#define JOINED 1
#define JOIN_COMPLETE 2
#define LIVE_PIN 17
#define COUNT_PIN 18
#define STOP_RELAY_PIN 21
#define SOLONOID_RELAY_PIN 20
#define STOP_RELAY_ON 49
#define SOLONOID_RELAY_ON 50
#define DEBUG_SERIAL SerialUSB
#define PIXEL_PIN 16
#define NUMPIXELS 1
#define RED 16711680
#define BLUE 255
#define GREEN 65280
#define YELLOW 8355584
#define VIOLET 8323199
#define WHITE 16777215

#define OFF_PATTERN 0
#define OFF_NOTCONNECTED_PATTERN 1
#define OFF_CONNECTED_PATTERN 2
#define ON_NOTCONNECTED_PATTERN 3
#define ON_CONNECTED_PATTERN 4
#define COUNT_PATTERN 5
#define SOLONOID_RELAY_PATTERN 6
#define STOP_RELAY_PATTERN 7
#define POWERUP_PATTERN 8

#ifdef SAMD
#define Serial SerialUSB
#elif MEGA
#define Serial Serial
#endif

#ifdef DEBUG_PRINT
#define debug_print(str) Serial.print(str)
#define debug_println(str) Serial.println(str)
#else
#define debug_print(str)
#define debug_println(str)
#endif

enum States
{
  POWER_ON,
  IDLE,
  WAITFORACK,
  SOMETHING_HAPPENED
};

enum Commands
{
  AREYOUREADY = 69,
  STATUS_PUSH,
  ACK,
  REGISTER_ME,
  REGISTER_ACK,
  GATEWAY_PUSH
};

typedef struct
{
  byte iovar; // rx stop relay // tx live availability
  byte iovar2; // rx solenoid relay // tx countval
} IO_Variable;

typedef struct
{
  uint16_t sourceID;
  uint16_t destinationtID;
  uint8_t command;
  IO_Variable data;
} Packet;

class NODE
{
private:
  int csPin;         // LoRa radio chip select
  int resetPin;      // LoRa radio reset
  int irqPin;        // change for your board; must be a hardware interrupt pin
  Packet datapacket; // Container to hold tx and rx payload
  IO_Variable lastIOstate;
  IO_Variable currentIOstate;
  Packet *packet_ptr;
  volatile uint8_t STATE = POWER_ON;
  uint8_t *data;
  uint16_t MY_ID = 0xFFF4; // Device ID
  unsigned long txFrequency, rxFrequency;
  void pushPacket();
  bool randomElapsedWithin(unsigned long from_ms, unsigned long to_ms);

public:
  bool stateChanged = false;
  NODE(int csPin, int resetpin, int irqpin);
  void Init();
  uint16_t getID();
  void callBack(int packetSize);
  void checkStatus();
  bool isConnected();
  void setFrequencies(unsigned long tx, unsigned long rx);
  void setID();
  void checkIO();
  void printStatus();
  ~NODE();
};

extern Adafruit_NeoPixel pixels;
extern volatile bool liveFlag, countFlag, prevLiveFlag;
extern unsigned long liveTimestamp, stopRelayTimestamp, solonidRelayTimestamp, pixelTimestamp;
extern unsigned long loopCoun;
extern uint8_t patternIndex, currentPattern, countValue;
extern NODE mynode;
extern bool lastStopRelayState;
extern bool lastSolenoidRelayState;

void readIO(IO_Variable *IO);
void callbackfunc(int packetsize);
byte readPrefix();
byte readNumber();
void runPixel();
void getStatus1();
void getCount1();
