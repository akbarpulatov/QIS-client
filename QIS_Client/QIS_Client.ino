#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

#define button1 4
#define button2 5
#define button3 6

#define ledRed      26
#define ledGreen    28
#define ledYellow   30

bool isIdDetected = false;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;
uint8_t pressedButton;
boolean toggle = false;

#define PN532_IRQ   (2)
#define PN532_RESET (3)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// ======================================================================
#include <EtherCard.h>
// ethernet interface mac address, must be unique on the LAN
byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

const char website[] PROGMEM = "192.168.233.96";
static byte session;
byte Ethernet::buffer[700];
Stash stash;

char UID[] = "5254454";
char score[] = "2";
uint32_t msTimer;
uint32_t msTimer1;
#define msTimerPeriod 2000
static uint32_t timer;

static void SetupHttp() {
//  Serial.begin(115200);
//  Serial.println("\n[Twitter Client]");

  // Change 'SS' to your Slave Select pin, if you arn't using the default pin
  if (ether.begin(sizeof Ethernet::buffer, mymac, SS) == 0)
    Serial.println(F("Failed to access Ethernet controller"));
  if (!ether.dhcpSetup())
    Serial.println(F("DHCP failed"));

  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);
  ether.printIp("DNS: ", ether.dnsip);

  if (!ether.dnsLookup(website))
    Serial.println(F("DNS failed"));

  ether.printIp("SRV: ", ether.hisip);

//  sendToTwitter();  
}

static void sendToTwitter () {
  Serial.println("Sending tweet...");
  byte sd = stash.create();

  stash.println("{");
  stash.print("\"UID\" : \"");
  for (uint8_t i=0; i < uidLength; i++) 
  {
    stash.print(uid[i], HEX); 
  }
  stash.println("\",");
  stash.print("\"Score\" : ");
  stash.print(pressedButton);
  stash.println("\"\r\n}");
  
  stash.save();
  int stash_size = stash.size();


  // Compose the http POST request, taking the headers below and appending
  // previously created stash in the sd holder.
  Stash::prepare(PSTR("POST / HTTP/1.0" "\r\n"
    "Host: $F" "\r\n"
    "Content-Length: $D" " "
    "$n"
    "\r\n"
    "$H"),
  website, website, stash_size, sd);

  // send the packet - this also releases all stash buffers once done
  // Save the session ID so we can watch for it in the main loop.
  session = ether.tcpSend();
}
// ======================================================================

void SetupNFC() {
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, LOW);
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1) {
      if(!msTimer1){
        toggle = !toggle;
        digitalWrite(ledRed, toggle);
        msTimer1 = 50;
      }  
    }; // halt
  }

  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

  nfc.setPassiveActivationRetries(0xFF);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  
  Serial.println("Waiting for an ISO14443A card");
}
//===============================================
void CheckNFC() {
  boolean success;
//  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
//  uint8_t uidLength;        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  
  if (success) {
    Serial.println("Found a card!");
    Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("UID Value: ");
    for (uint8_t i=0; i < uidLength; i++) 
    {
      Serial.print(" 0x");Serial.print(uid[i], HEX); 
    }
    Serial.println("");
  // Wait 1 second before continuing
//  delay(1000);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, HIGH);
  }
  else
  {
    // PN532 probably timed out waiting for a card
    Serial.println("Timed out waiting for a card");
    while (1) {
//      Serial.print("0");
      if(!msTimer1){
        Serial.print("0");
        toggle = !toggle;
        digitalWrite(ledRed, toggle);
        msTimer1 = 50;
      }
    };
  }
  
}
//===============================================
void setup() {

  Serial.begin(115200);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);

  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledYellow, LOW);

  SetupTimer();
  SetupNFC();
  CheckNFC();
  SetupHttp();
  PingSetup();
}
//===============================================
void loop() {
  PingCheck();


//  if (len > 0 && ether.packetLoopIcmpCheckReply("192.168.233.96")) {
//    Serial.print("  ");
//    Serial.print((micros() - timer) * 0.001, 3);
//    Serial.println(" ms");
//  }
//
//    // ping a remote server once every few seconds
//  if (micros() - timer >= 5000000) {
//    ether.printIp("Pinging: ", ether.hisip);
//    timer = micros();
//    ether.clientIcmpRequest(ether.hisip);
//  }



  if(!msTimer1) {
    toggle = !toggle;
//    digitalWrite(ledRed, toggle);
//    digitalWrite(ledGreen, toggle);
    digitalWrite(ledYellow, toggle);
//    PingCheck();
    
    msTimer1 = 1000;
  }

  if(!msTimer){
  
  bool input1 = digitalRead(button1);
  bool input2 = digitalRead(button2);
  bool input3 = digitalRead(button3);

    if (!input1 || !input2 || !input3) {
      
      if(!input1) {
        pressedButton = 1;
      } else if(!input2) {
        pressedButton = 2;
      } else {
        pressedButton = 3;
      }
          
      Serial.print("Pressed Button Was: ");
      Serial.println(pressedButton);
      Serial.print("uid is: ");
      
       for (uint8_t i=0; i < uidLength; i++) 
      {
        Serial.print(uid[i], HEX); 
      }
      Serial.println();
      sendToTwitter();    
      msTimer = msTimerPeriod;
    }
  }
}
//===============================================
// called when a ping comes in (replies to it are automatic)
static void gotPinged (byte* ptr) {
  ether.printIp(">>> ping from: ", ptr);
}
void PingSetup() {
  ether.parseIp(ether.hisip, "192.168.233.96");
//#endif
  ether.printIp("SRV: ", ether.hisip);
    
  // call this to report others pinging us
  ether.registerPingCallback(gotPinged);
  
  timer = -9999999; // start timing out right away
}
void PingCheck() {
  word len = ether.packetReceive(); // go receive new packets
  word pos = ether.packetLoop(len); // respond to incoming pings
  
  // report whenever a reply to our outgoing ping comes back
  if (len > 0 && ether.packetLoopIcmpCheckReply(ether.hisip)) {
    Serial.print("  ");
    Serial.print((micros() - timer) * 0.001, 3);
    Serial.println(" ms");
  }
  
  // ping a remote server once every few seconds
  if (micros() - timer >= 5000000) {
    ether.printIp("Pinging: ", ether.hisip);
    timer = micros();
    ether.clientIcmpRequest(ether.hisip);
  }
}
//===============================================
void SetupTimer() {
  // инициализация Timer1
    cli();  // отключить глобальные прерывания
    TCCR1A = 0;   // установить регистры в 0
    TCCR1B = 0;

    OCR1A = 15624; // установка регистра совпадения

    TCCR1B |= (1 << WGM12);  // включить CTC режим 
    TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024
//    TCCR1B |= (1 << CS12);

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
    sei(); // включить глобальные прерывания
}

ISR(TIMER1_COMPA_vect)
{
  if(msTimer)msTimer--;
  if(msTimer1)msTimer1--;
}
