#include "LoRa.h"
#include "NODE.h"

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
volatile bool liveFlag = false, countFlag = false, prevLiveFlag = false;
unsigned long liveTimestamp, stopRelayTimestamp, solonidRelayTimestamp, pixelTimestamp;
uint8_t patternIndex, currentPattern, countValue;
bool lastStopRelayState = false;
bool lastSolenoidRelayState = false;

NODE::NODE(int cs, int rst, int irq)
{
    this->csPin = cs;
    this->irqPin = irq;
    this->resetPin = rst;
}

void NODE::Init()
{
    packet_ptr = &datapacket;
    data = (uint8_t *)packet_ptr;
    datapacket.command = STATUS_PUSH;
    LoRa.setPins(csPin, resetPin, irqPin);
    debug_print("LoRa Pins > Cs:");
    debug_print(csPin);
    debug_print(", Rst:");
    debug_print(resetPin);
    debug_print(", IRQ:");
    debug_print(irqPin);
    debug_print(", tx Frq:");
    debug_print(txFrequency);
    debug_print(", rx Frq:");
    debug_println(rxFrequency);

    while (!LoRa.begin(rxFrequency))
    {
        debug_println("LoRa init failed. Check your connections.");
        delay(1000); // if failed, keep trying every second
    }
    debug_println("Lora init success!");
    LoRa.receive();
    debug_println("Lora mode : recieve");
    readIO(&lastIOstate);
    readIO(&currentIOstate);
    currentPattern = POWERUP_PATTERN;
    pinMode(LIVE_PIN, INPUT_PULLUP);
    pinMode(COUNT_PIN, INPUT_PULLUP);
    pinMode(STOP_RELAY_PIN, OUTPUT);
    pinMode(SOLONOID_RELAY_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(LIVE_PIN), getStatus1, FALLING);
    attachInterrupt(digitalPinToInterrupt(COUNT_PIN), getCount1, FALLING);
}

void NODE::setFrequencies(unsigned long tx, unsigned long rx)
{
    this->rxFrequency = rx;
    this->txFrequency = tx;
}

void NODE::setID()
{
    for (int i = 2; i <= 13; i++) // Device address pins init
    {
        pinMode(i, INPUT_PULLUP);
    }
    uint16_t ID_container = 0;
    ID_container |= (readPrefix() << 8);
    ID_container |= readNumber();
    MY_ID = ID_container;
    debug_print("ID:");
    debug_println(MY_ID);
    datapacket.sourceID = MY_ID;
    datapacket.destinationtID = 0;
}

uint16_t NODE::getID()
{
    return MY_ID;
}

void NODE::pushPacket()
{
    //setPacketData(&lastIOstate, ptr);
    LoRa.setFrequency(txFrequency);
    LoRa.beginPacket();
    //uint8_t *pt = (uint8_t *)packet_ptr;
    for (unsigned long long i = 0; i < sizeof(Packet); i++)
    {
        LoRa.write(data[i] ^ PRIVATEKEY);
    }
    //LoRa.write(pt, sizeof(Packet));
    LoRa.endPacket();
    LoRa.setFrequency(rxFrequency);
    LoRa.receive();
}

void NODE::callBack(int packetSize)
{

    if (packetSize == 0)
        return; // if there's no packet, return
    if (LoRa.packetRssi() > 100)
    {
        return; // if rssi greater than 100 reject the packet
    }
    LoRa.readBytes(data, sizeof(Packet));
    for (unsigned long long i = 0; i < sizeof(Packet); i++)
    {
        data[i] ^= PRIVATEKEY;
    }
    debug_print("Recv! ----> Command:");
    if (datapacket.command == AREYOUREADY)
    {
        debug_print("Who is:   ");
    }
    if (datapacket.command == ACK)
    {
        debug_print("Ack from gateway!   ");
    }
    if (datapacket.command == STATUS_PUSH)
    {
        debug_print("Status push!   ");
    }
    if (datapacket.command == GATEWAY_PUSH)
    {
        debug_print("Data from gateway!");
    }
    
    debug_print("source ID:");
    debug_print(datapacket.sourceID);
    debug_print("   destination ID:");
    debug_print(datapacket.destinationtID);
    debug_print("    with RSSI: ");
    debug_println(LoRa.packetRssi());

    if (STATE == POWER_ON)
    { // ON POWER ON CHECK FOR REGISTER PACKET FROM GATEWAY THEN PUSH A FIRST PACKET
        if (datapacket.command == AREYOUREADY && datapacket.destinationtID == MY_ID)
        {
            debug_println("Register request from gateway!");
            STATE = SOMETHING_HAPPENED;
        }
    }
    else if (STATE == SOMETHING_HAPPENED)
    { // WAIT FOR ACK FROM GATEWAY DURING IO PUSH AND GO BACK TO IDLE
        if (datapacket.command == ACK && datapacket.destinationtID == MY_ID)
        {
            debug_println("GATEWAY acknowledged data");
            STATE = IDLE;
        }
    }
    else if (STATE == IDLE)
    { // WAIT FOR ACK FROM GATEWAY DURING IO PUSH AND GO BACK TO IDLE
        if (datapacket.command == AREYOUREADY && datapacket.destinationtID == MY_ID)
        {
            debug_println("Register request from gateway!");
            STATE = SOMETHING_HAPPENED;
        }
        else if (datapacket.command == GATEWAY_PUSH && datapacket.destinationtID == MY_ID)
        {
            debug_println("IO data from gateway!");
            if (datapacket.data.iovar == 1)
            {
                digitalWrite(STOP_RELAY_PIN, 1);
                stopRelayTimestamp = millis();
                debug_println("STOP RELAY ON!");
                lastStopRelayState = true;
            }
            if (datapacket.data.iovar2 == 1)
            {
                digitalWrite(SOLONOID_RELAY_PIN, 1);
                solonidRelayTimestamp = millis();
                debug_println("SOLENOID RELAY ON!");
                lastSolenoidRelayState = true;
            }
        }
    }
}

void NODE::checkStatus()
{
    // check for IO pin changes
    if (STATE == POWER_ON)
    {
    }
    else if (STATE == IDLE)
    { // AFTER REGISTRATION GO TO IDLE MODE... POLL FOR IO CHANGES

        readIO(&currentIOstate); // read IO pins into  current ioSTATE
        //debug_print("Current state:");
        //debug_print(currentIOstate.iovar);
        //debug_print("   last state:");
        //debug_print(lastIOstate.iovar);
        if (currentIOstate.iovar != lastIOstate.iovar)
        {
            STATE = SOMETHING_HAPPENED;
            debug_println("Live signal unavailable! Sending data..");
            lastIOstate.iovar = currentIOstate.iovar;
        }
    }
    else if (STATE == SOMETHING_HAPPENED)
    { // IO CHANGED!  TRY TO PUSH IO DATA TO GATEWAY AT RANDOM INTERVALS
        if (randomElapsedWithin(1000, 5000))
        {
            datapacket.command = STATUS_PUSH;
            datapacket.sourceID = MY_ID;
            datapacket.data = currentIOstate;
            pushPacket();
            debug_println("Sending data...");
        }
    }
}

bool NODE::randomElapsedWithin(unsigned long from_ms, unsigned long to_ms)
{
    static unsigned long last = millis();
    static unsigned long target = rand() % ((to_ms - from_ms) + 1) + from_ms;
    bool elapsed = false;
    if ((millis() - last) > target)
    {
        elapsed = true;
        target = rand() % ((to_ms - from_ms) + 1) + from_ms;
        last = millis();
    }
    return elapsed;
}

bool NODE::isConnected()
{
    bool connection = false;
    if (STATE == IDLE || STATE == SOMETHING_HAPPENED)
    {
        connection = true;
    }
    return connection;
}

void NODE::printStatus()
{
    static unsigned long last;
    if ((millis() - last) > 1000)
    {
        debug_print("Live state:");
        debug_print(liveFlag);
        debug_print("   Count:");
        debug_print(countValue);
        debug_print("   ");
        if (STATE == POWER_ON)
        {
            debug_println("POWER ON STATE");
        }
        else if (STATE == IDLE)
        {
            debug_println("IDLE STATE");
        }
        else if (STATE == SOMETHING_HAPPENED)
        {
            debug_println("SOMETHING HAPPENED STATE");
        }
        last = millis();
    }
}

NODE::~NODE()
{
}

byte readPrefix()
{
    byte lt = (!digitalRead(2)) * 16 + (!digitalRead(3)) * 8 + (!digitalRead(4)) * 4 + (!digitalRead(5)) * 2 + (!digitalRead(6));
    debug_println(lt);
    return lt;
}

byte readNumber()
{
    byte nm = 0;
    for (int i = 1; i <= 7; i++)
    {
        nm += (!digitalRead(14 - i)) * pow(2, (i - 1));
    }
    debug_println(nm);
    return nm;
}

void runPixel()
{
    static unsigned long pixelTimestamp = 0;
    static unsigned long pixelPatterns[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},                          // power on not connected led pattern
                                                {RED, 0, 0, 0, 0, 0, 0, 0, 0},                        // power on not connected led pattern
                                                {RED, 0, BLUE, 0, 0, 0, 0, 0, 0},                     // power on not connected led pattern
                                                {GREEN, 0, 0, 0, 0, 0, 0, 0, 0},                      // power on not connected led pattern
                                                {GREEN, 0, BLUE, 0, 0, 0, 0, 0, 0},                   // power on not connected led pattern
                                                {YELLOW, 0, YELLOW, 0, YELLOW, 0, YELLOW, 0, YELLOW}, // power on not connected led pattern
                                                {VIOLET, 0, VIOLET, 0, VIOLET, 0, VIOLET, 0, VIOLET}, // power on not connected led pattern
                                                {WHITE, 0, WHITE, 0, WHITE, 0, WHITE, 0, WHITE},      // power on not connected led pattern
                                                {RED, 0, 0, RED, 0, 0, RED, 0, 0}};                   // POWERUP_PATTERN

    if (millis() - pixelTimestamp > 200)
    {
        pixels.setPixelColor(0, pixelPatterns[currentPattern][patternIndex]);
        pixels.show();

        patternIndex++;
        if (patternIndex > 8)
        {
            patternIndex = 0;
            currentPattern = OFF_PATTERN;
        }
        pixelTimestamp = millis();
    }
}

// Implement IO read mechansim
//example
//IO->b |= (1<<<digitalRead(some_pin))  /////////////  implement this
void readIO(IO_Variable *IO)
{
    if (liveFlag)
    {
        IO->iovar = 1;
    }
    else
    {
        IO->iovar = 0;
    }
    IO->iovar2 = countValue;
}

void getCount1()
{
    //static unsigned long count_t = 0;
    debug_println("COUNT PULSE");
    ///if ((millis() - count_t > 3000) || (millis() - count_t < 0))
    //{
    countFlag = true;
    //count_t = millis();
    countValue++;
    //}
}

void getStatus1()
{
    debug_println("LIVE PULSE");
    liveTimestamp = millis();
    liveFlag = true;
}

void NODE::checkIO()
{
    //This function is called from the main loop. Do I/O checks here and call the txData function as below.
    if ((millis() - liveTimestamp) > 5000)
    {
        liveFlag = false;
        liveTimestamp = millis();
    }
    if (((millis() - stopRelayTimestamp) > 5000) && lastStopRelayState == true)
    {
        stopRelayTimestamp = millis();
        digitalWrite(STOP_RELAY_PIN, LOW);
        debug_println("STOP RELAY OFF");
        lastStopRelayState =false;
    }

    if (((millis() - solonidRelayTimestamp) > 5000) && lastSolenoidRelayState == true)
    {
        solonidRelayTimestamp = millis();
        digitalWrite(SOLONOID_RELAY_PIN, LOW);
        debug_println("SOLONOID RELAY OFF");
        lastSolenoidRelayState = false;
    }

    prevLiveFlag = liveFlag;
    //txData(uint8_t* <ARRAY OF THE DATA TO BE SENT>, int <LENGTH OF DATA>, bool <true=CONFIRMED (with ack and retransmission, false=UNCONFIRMED (no ack)>)
    // txData(payload,2,true);

    if (liveFlag && mynode.isConnected() && currentPattern <= ON_CONNECTED_PATTERN)
        currentPattern = ON_CONNECTED_PATTERN;
    if (!liveFlag && mynode.isConnected() && currentPattern <= OFF_CONNECTED_PATTERN)
        currentPattern = OFF_CONNECTED_PATTERN;
    if (liveFlag && !mynode.isConnected() && currentPattern <= ON_NOTCONNECTED_PATTERN)
        currentPattern = ON_NOTCONNECTED_PATTERN;
    if (!liveFlag && !mynode.isConnected() && currentPattern <= ON_NOTCONNECTED_PATTERN)
        currentPattern = OFF_NOTCONNECTED_PATTERN;
    if (digitalRead(STOP_RELAY_PIN))
        currentPattern = STOP_RELAY_PATTERN;
    if (digitalRead(SOLONOID_RELAY_PIN))
        currentPattern = SOLONOID_RELAY_PATTERN;

    if (countFlag && STATE == IDLE)
    {
        STATE = SOMETHING_HAPPENED;
        if (currentPattern <= COUNT_PATTERN)
            currentPattern = COUNT_PATTERN;
        countFlag = false;
    }
}

void callbackfunc(int packetsize)
{
    mynode.callBack(packetsize);
}
