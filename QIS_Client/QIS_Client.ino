#include <UIPEthernet.h>

EthernetClient client;
uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

void setup() {
  Serial.begin(115200);

  char server[] = "192.168.233.96";

  String data = "\"score\" : ";
  data += "\"3\"";

  if(Ethernet.begin(mac) == 0){
    Serial.println("Failed to configure Ethernet using DHCP");
    while(1);
  }

  if (client.connect(server,8080)){
      Serial.println("Connected to server");
      client.println("POST / HTTP/1.1");
      client.println("Host: QIS");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(data.length());
      client.println("{");
      client.println(data);
      client.println("}");
      
  }else{
      Serial.println("Connection to server failed");
  }
}

void loop() {  
  while(client.connected()){
    if(client.available()){
      char c = client.read();
      Serial.print(c);  
    }
  }
}
