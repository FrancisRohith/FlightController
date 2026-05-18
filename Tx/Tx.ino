#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(PB0, PA4); // PB0 = CE, PA4 = CSN

// A 5 letter string to communicate with the other module.
const byte addressA[6] = "00001";
const byte addressB[6] = "00011";

struct Transmitter {
  uint16_t throttle;
};

struct Receiver {
  float roll;
  float pitch;
  float yaw;
};

Transmitter Tx;
Receiver Rx;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println("NRF failed");
    while (1);
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(addressB);   // write to this addressed module
  radio.openReadingPipe(0, addressA);   // read then write to this addredded module
  radio.startListening();
  Tx.throttle = 1000;
}

void loop() {
  radio.stopListening();  // first, stop listening so we can talk
  if(Serial.available()) {
    char c = Serial.read();
    if (c == 'w') Tx.throttle += 10;
    if (c == 's') Tx.throttle -= 10;
  }

  if (Tx.throttle > 2000) Tx.throttle = 2000;
  if (Tx.throttle < 1000) Tx.throttle = 1000;

  if (!radio.write(&Tx, sizeof(Tx))) {
    Serial.println(F("failed"));        // F() -> stores string in flash
  }
  
  radio.startListening();

  unsigned long started_waiting_at = micros();
  boolean timeout = false;

  while(!radio.available()) {
    if (micros() - started_waiting_at > 200000) {
      timeout = true;
      break;
    }
  }

  if (timeout) {
    Serial.println(F("Failed, response timeout"));
  } else {
    radio.read(&Rx, sizeof(Rx));
    Serial.print("Tx: ");
    Serial.print(Tx.throttle);
    Serial.print(" Rx: ");
    Serial.print(Rx.roll);
    Serial.print(" ");
    Serial.println(Rx.pitch);
    Serial.print(" ");
    Serial.println(Rx.yaw);

    delay(200);
  }

  
}
