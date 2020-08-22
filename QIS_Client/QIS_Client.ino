#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

#define button1 4
#define button2 5
#define button3 6

bool isIdDetected = false;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;


#define PN532_IRQ   (2)
#define PN532_RESET (3)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

void SetupTimer();

void SetupNFC() {
    nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
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
  delay(1000);
  }
  else
  {
    // PN532 probably timed out waiting for a card
    Serial.println("Timed out waiting for a card");
  }
  
}
//===============================================
void setup() {

  Serial.begin(115200);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  //SetupTimer();
  SetupNFC();
  CheckNFC();

}
//===============================================
void loop() {
  Serial.println("=======");
  delay(500);
  
  bool input1 = digitalRead(button1);
  bool input2 = digitalRead(button2);
  bool input3 = digitalRead(button3);

  uint8_t pressedButton;
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

//    char* uidsend = malloc(5);
//    sprintf(uidsend,"%X%X%X%X", uid[0],uid[1],uid[2],uid[3]);
//    Serial.println(uidsend);
    // send Http post request
    
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
    TCCR1B |= (1 << CS12);

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
    sei(); // включить глобальные прерывания
}

ISR(TIMER1_COMPA_vect)
{
  Serial.println("Timer is Fired!");
  //CheckNFC();
}
