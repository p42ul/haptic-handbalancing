/*
  ESP8266 Blink by Simon Peter
  Blink the blue LED on the ESP-01 module
  This example code is in the public domain

  The blue LED on the ESP-01 module is connected to GPIO1
  (which is also the TXD pin; so we cannot use Serial.print() at the same time)

  Note that this sketch uses LED_BUILTIN to find the pin with the internal LED
*/

int button = 12;
int b2 = 14;
int buzzer = 13;
int switchState = 0;
int s2 = 0;
void setup() {
  pinMode(13, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  //pinMode(4, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, LOW);   // Turn the LED on (Note that LOW is the voltage level

  delay(1000);

  digitalWrite(13, HIGH);

  delay(1000);
  //digitalWrite(4, LOW);
//  switchState = digitalRead(button);
//  s2 = digitalRead(b2);
//  if (switchState == HIGH) { //pressed
//    digitalWrite(13, HIGH);
//  } else {
//    digitalWrite(13, LOW);
//  }
//  if (s2 == HIGH){
//    digitalWrite(4, HIGH);
//  } else {
//    digitalWrite(4, LOW);
//  }

}
