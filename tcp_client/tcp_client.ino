// TCP клиент, передает данные из UART серверу, от сервера в UART
#include <SPI.h>
#include <UIPEthernet.h>

// определяем конфигурацию сети
byte mac[] = {0xAE, 0xB2, 0x26, 0xE4, 0x4A, 0x5C}; // MAC-адрес
byte ip[] = {192, 168, 233, 98}; // IP-адрес клиента
byte ipServ[] = {192, 168, 233, 96}; // IP-адрес сервера

EthernetClient client; // создаем клиента

void setup() {
  Ethernet.begin(mac, ip); // инициализация контроллера
  Serial.begin(115200);
  delay(1000);
  Serial.println("connecting...");
  // устанавливаем соединение с сервером
  if (client.connect(ipServ, 80)) {
    Serial.println("connected"); // успешно
  }
  else {
    Serial.println("connection failed"); // ошибка
  }
}

void loop() {
  // все, что приходит с сервера, печатаем в UART
  if (client.available()) {
    char chr = client.read();
    Serial.print(chr);
  }

  // все, что приходит из UART, передаем серверу
  while (Serial.available() > 0) {
    char inChr = Serial.read();
    if (client.connected()) {
      client.print(inChr);
    }
  }

  // если сервер отключился, останавливаем клиент
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    while (true); // останавливается
  }
}
