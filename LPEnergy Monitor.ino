/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EnableInterrupt.h>
#include <LowPower.h>


#define TRIGGER 8
#define DONEPIN A0
#define PULSESWITCH 7
#define DEBUG
#define DEBUGS


volatile uint16_t triggerCount = 0;
int InterruptCount = 0;


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x32, 0x51, 0x8B, 0xDA, 0xD7, 0x0B, 0x35, 0x3A, 0x48, 0x55, 0xAD, 0xD2, 0xC6, 0xB6, 0xCE, 0x93 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x77, 0x4F, 0x88, 0xCE, 0x6C, 0x22, 0x0B, 0xC3, 0x68, 0x59, 0xF1, 0xFC, 0x19, 0xC4, 0xF9, 0x22 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x0015db51; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

struct data {
  int batteryVoltage;
  int energy;
 // char err;
}mydata;

static osjob_t sendjob;
bool next = false;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};


void onEvent (ev_t ev) {
    #ifdef DEBUGS
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            #ifdef DEBUGS
            Serial.println(F("EV_SCAN_TIMEOUT"));
            #endif
            break;
        case EV_BEACON_FOUND:
            #ifdef DEBUGS
            Serial.println(F("EV_BEACON_FOUND"));
            #endif
            break;
        case EV_BEACON_MISSED:
            #ifdef DEBUGS
            Serial.println(F("EV_BEACON_MISSED"));
            #endif
            break;
        case EV_BEACON_TRACKED:
            #ifdef DEBUGS
            Serial.println(F("EV_BEACON_TRACKED"));
            #endif
            break;
        case EV_JOINING:
            #ifdef DEBUGS
            Serial.println(F("EV_JOINING"));
            #endif
            break;
        case EV_JOINED:
            #ifdef DEBUGS
            Serial.println(F("EV_JOINED"));
            #endif
            break;
        case EV_RFU1:
            #ifdef DEBUGS
            Serial.println(F("EV_RFU1"));
            #endif
            break;
        case EV_JOIN_FAILED:
            #ifdef DEBUGS
            Serial.println(F("EV_JOIN_FAILED"));
            #endif
            break;
        case EV_REJOIN_FAILED:
            #ifdef DEBUGS
            Serial.println(F("EV_REJOIN_FAILED"));
            #endif
            break;
        case EV_TXCOMPLETE:
            #ifdef DEBUGS
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            #endif
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            next = true;
            InterruptCount=0;
            break;
        case EV_LOST_TSYNC:
            #ifdef DEBUGS
            Serial.println(F("EV_LOST_TSYNC"));
            #endif
            break;
        case EV_RESET:
            #ifdef DEBUGS
            Serial.println(F("EV_RESET"));
            #endif
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            #ifdef DEBUGS
            Serial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
            #ifdef DEBUGS
            Serial.println(F("EV_LINK_DEAD"));
            #endif
            break;
        case EV_LINK_ALIVE:
            #ifdef DEBUGS
            Serial.println(F("EV_LINK_ALIVE"));
            #endif
            break;
         default:
            #ifdef DEBUGS
            Serial.println(F("Unknown event"));
            #endif
            break;
    }
}

void do_send(osjob_t* j){
   
    
    #ifdef DEBUG
    Serial.println("Energy");
    Serial.println(mydata.energy);
    Serial.println("Intcount");
    Serial.println(InterruptCount);
    #endif
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        #ifdef DEBUG
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata) - 1, 0);
        #ifdef DEBUG
        Serial.println(F("Packet queued"));
        #endif
        digitalWrite(DONEPIN, LOW);
    }
    // Next TX is scheduled after TX_COMPLETE event

}

void setup() {
    #ifdef DEBUGS
    Serial.begin(115200);
    #endif
    #ifdef DEBUG
    Serial.println(F("Starting"));
    #endif

    pinMode(DONEPIN, OUTPUT);
    digitalWrite(DONEPIN, LOW);

    pinMode(PULSESWITCH, INPUT);
    enableInterrupt(PULSESWITCH, pulseCount, FALLING);
    
    pinMode(TRIGGER, INPUT);
    enableInterrupt(TRIGGER, nanoTimerTrigger, RISING);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
          LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
          LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
          LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    extern volatile unsigned long timer0_overflow_count;

    if (next == false) {
        os_runloop_once();
    } else {

        #ifdef DEBUG
        Serial.print(F("Enter sleeping forever "));
        Serial.flush(); // give the serial print chance to complete
        #endif
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        cli();
        timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
        sei();
    
        #ifdef DEBUG
        Serial.println(F("Sleep complete"));
        #endif        
    } 
}

void nanoTimerTrigger() { 
    mydata.energy = InterruptCount;
    mydata.batteryVoltage = batterycalc();  
    os_setCallback (&sendjob, do_send);
    next = false; 
    digitalWrite(DONEPIN,HIGH);
    digitalWrite(DONEPIN,LOW);       
    }

void pulseCount(){
    InterruptCount +=1;
    #ifdef DEBUG
    Serial.print(InterruptCount);
    #endif
    if(InterruptCount==12)
    {
      nanoTimerTrigger();
    }
    }

//Battery Monitor
int  batv = 0, bat_status = 0;

int batteryv(){
    int val = analogRead(A3) >> 2;     // read the input pin
    batv = (val * 2.00 * 3.3 / 255) * 1000; //1.85 is the voltage multiplier
    delay(0);
    return batv;
}

int batterycalc()
{
  int diff = 0;
  if (bat_status == 0) { //Baseline calibration
    for (int j = 0; j < 5; j++)  {
      bat_status = bat_status + batteryv();
    }
    bat_status = bat_status / 5;
  }
#ifdef DEBUG
  Serial.println("Battery_status = ");
  Serial.print(bat_status);
#endif
  for (int i = 0; i < 5; i++) { //Read current average
    batv = batv + batteryv();
  }
  batv = batv / 5;
  diff = abs(bat_status - batv);
#ifdef DEBUG
  Serial.println("Now Avg = ");
  Serial.println(batv);
  Serial.println("Value diff");
  Serial.println(diff);
#endif
  if (diff > 25 && diff < 70) { //Define Band Pass
    bat_status = batv;
  }
  batv = 0;
#ifdef DEBUG
  Serial.println("Battery Voltage");
  Serial.println(bat_status);
#endif
  return bat_status;
}
