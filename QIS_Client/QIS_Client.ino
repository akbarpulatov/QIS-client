const int button1 = 4;
const int button2 = 5;
const int button3 = 6;


void setup() {

  Serial.begin(115200);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  

}

void loop() {
  int input1 = digitalRead(button1);
  int input2 = digitalRead(button2);
  int input3 = digitalRead(button3);

  Serial.print("Input 1 = ");
  Serial.println(input1);
  
  Serial.print("Input 2 = ");
  Serial.println(input2);
  
  Serial.print("Input 3 = ");
  Serial.println(input3);

  Serial.println("===========================================");

  delay(500);
}
