// constants 't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin,HIGH);
  

}

  uint8_t oldState = HIGH;


void loop() {
  // put your main code here, to run repeatedly:
  // read the state of the pushbutton value:
  // send data only when you receive data:
 // if (Serial.available() > 0) {
    // read the incoming byte:
 //    Serial.read();
  
  buttonState = digitalRead(buttonPin);
  if (buttonState != oldState) {
     delay(5); 
  buttonState = digitalRead(buttonPin);
  if (buttonState != oldState) {

  if (buttonState == HIGH) {
    Serial.print("0");
  } else {
     Serial.print("1");
  }
  
  oldState = buttonState;
 }}
}
