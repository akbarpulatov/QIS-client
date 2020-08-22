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
  delay(500);
  
  bool input1 = digitalRead(button1);
  bool input2 = digitalRead(button2);
  bool input3 = digitalRead(button3);

  Serial.print("Input 1 = ");
  Serial.println(input1);
  
  Serial.print("Input 2 = ");
  Serial.println(input2);
  
  Serial.print("Input 3 = ");
  Serial.println(input3);

  Serial.println("===========================================");

  if (!input1 || !input2 || !input3) {
    Serial.println("At least one of buttons were pressed!");
  }


  

  
}
