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
  Serial.println("=======");
  delay(500);
  
  bool input1 = digitalRead(button1);
  bool input2 = digitalRead(button2);
  bool input3 = digitalRead(button3);

  int pressedButton;
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
  
  
  
  }

}
