
/*
Vcc -> 3V3
CSN -> PA4
MOSI -> PA7
CE -> PB0
SCK -> PA5
MISO -> PA6
*/
void nRF_setup() {
  digitalWrite(PC13, LOW);
  if (!radio.begin()) {
    Serial.println("NRF failed");
    while (1);
    digitalWrite(PC13, HIGH);
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(addressA);
  radio.openReadingPipe(0, addressB);
  
  radio.startListening();
}