#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <UIPEthernet.h>
#include <Adafruit_PN532.h>
#include <ArduinoUniqueID.h>

#define button1 4
#define button2 5
#define button3 6

#define buzzer 8
#define ledRed      26
#define ledGreen    28
#define ledYellow   30

#define BuzzerPlayTime 40
#define BuzzerFrequency 2700

#define msTimer2 NoConnectionTimout
#define msTimer3 NFCCardReadTime
#define msTimer4 ReconnectTime
#define ButtonBlockTimeout 1000

bool isIdDetected = false;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;
uint8_t pressedButton;
boolean toggle = false;
boolean isServerAvailable = false;

uint32_t msTimer;
uint32_t msTimer1;
uint32_t msTimer2;
uint32_t msTimer3;
uint32_t msTimer4;

byte mac[] = {0xAE, 0xB2, 0x26, 0xE4, 0x4A, 0x5C}; // MAC-адрес
byte ip[] = {192, 168, 233, 159}; // IP-адрес клиента
byte ipServ[] = {192, 168, 233, 96}; // IP-адрес сервера
#define PORT 4567
EthernetClient client; // создаем клиента

static uint32_t timer;
char tcpBuffer[100];

#define PN532_IRQ   (2)
#define PN532_RESET (3)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
// ======================================================================
void SetupNFC() 
{
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
void CheckNFC() 
{
  boolean success;
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

    tone(buzzer, BuzzerFrequency);
    delay(BuzzerPlayTime);
    noTone(buzzer);
  }
  else
  {
    // PN532 probably timed out waiting for a card
    Serial.println("Timed out waiting for a card");
    while (1) {
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
void setup() 
{
  Serial.begin(115200);

  Serial.print("UniqueID: ");
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    if (UniqueID[i] < 0x10)
      Serial.print("0");
    Serial.print(UniqueID[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

   for (size_t i = 0; i < 6; i++)
  {
    mac[i] = UniqueID[i+3];
  }

  Ethernet.begin(mac, ip); // инициализация контроллера

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledYellow, LOW);

  SetupTimer();
  SetupNFC();
  CheckNFC();
}
//===============================================
void loop() 
{
  // все, что приходит с сервера, печатаем в UART
  if (client.available()) {
    char chr = client.read();
    Serial.print(chr);
  }

  if(!client.connected() && !ReconnectTime) {
    ReconnectTime = 100;
    client.connect(ipServ, PORT);
    isServerAvailable = false;
  }

  if(!NoConnectionTimout) {
    isServerAvailable = true;
    NoConnectionTimout = 3000;
  }
  
  if(!msTimer1 && isServerAvailable) {
    toggle = !toggle;
    digitalWrite(ledYellow, toggle);
    msTimer1 = 200;
  }

  if(!NFCCardReadTime){
    noTone(buzzer);
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
      
      tone(buzzer, BuzzerFrequency);  
      NFCCardReadTime = BuzzerPlayTime;
      
      Serial.print("Pressed Button Was: ");
      Serial.println(pressedButton);
      Serial.print("uid is: ");
      
      for (uint8_t i=0; i < uidLength; i++) 
      {
        Serial.print(uid[i], HEX); 
      }
      Serial.println();
      
      int lenOfTcp = sprintf(tcpBuffer, "%d %02X%02X%02X%02X", pressedButton, uid[0], uid[1], uid[2], uid[3]);
      client.println(tcpBuffer);
      msTimer = ButtonBlockTimeout;
    }
  }
}
//===============================================
void SetupTimer() 
{
  // инициализация Timer1
    cli();  // отключить глобальные прерывания
    TCCR1A = 0;   // установить регистры в 0
    TCCR1B = 0;

    OCR1A = 15624; // установка регистра совпадения

    TCCR1B |= (1 << WGM12);  // включить CTC режим 
    TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
    sei(); // включить глобальные прерывания
}

ISR(TIMER1_COMPA_vect)
{
  if(msTimer)msTimer--;
  if(msTimer1)msTimer1--;
  if(msTimer2)msTimer2--;
  if(msTimer3)msTimer3--;
  if(msTimer4)msTimer4--;  
  
}
