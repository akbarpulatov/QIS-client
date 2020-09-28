#include <SPI.h>
#include <UIPEthernet.h>

byte mac[] = {0xAE, 0xB2, 0x26, 0xE4, 0x4A, 0x5C}; // MAC-адрес

EthernetServer server(2000); // создаем сервер, порт 2000
EthernetClient client; // объект клиент
boolean clientAlreadyConnected= false; // признак клиент уже подключен

void setup() {
  Serial.begin(115200);
  Serial.println("Getting IP address using DHCP");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure using DHCP");
    while(true) ; // зависаем по ошибке
  }

  // вывод IP адреса
  Serial.print("IP address: ");
  IPAddress ip = Ethernet.localIP();
  for (byte i = 0; i < 4; i++) {
    Serial.print(ip[i], DEC);
    Serial.print(".");
  }
  Serial.println();
  server.begin(); // включаем ожидание входящих соединений
}

void loop() {
  client = server.available(); // ожидаем объект клиент
  if (client) {
    // есть данные от клиента
    if (clientAlreadyConnected == false) {
      // сообщение о подключении
      Serial.println("Client connected");
      client.println("Server ready"); // ответ клиенту
      clientAlreadyConnected= true;
    }

    while(client.available() > 0) {
      char chr = client.read(); // чтение символа
      server.write(chr); // передача клиенту
      Serial.write(chr);
    }
  }
}
