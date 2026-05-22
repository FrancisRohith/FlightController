void error_signal() {
  if (error >= 100) red_led(HIGH);
  else if (error_timer < millis()) {
    error_timer = millis() + 250;           // set the next error_timer at 250ms apart
    if(error > 0 && error_counter > error + 3) error_counter = 0; // if there is an error and the error_counter > +3 reset the error.
    if(error_counter < error && error_led == 0 && error > 0) {  // if led=0 and led flashing remaining
      red_led(HIGH);
      error_led = 1;
    }
    else {
      red_led(LOW);
      error_counter++;    // error_counter is incremented till error+3
      error_led = 0;
    }
  }
}

// LED is blinks 1 time and pause, for error=1
// 1 -> error_counter = 0
//      error_counter < error && error_led=0
//        error_led=1
// 2 -> error_counter < error && error_led=1
//        error_led=0
//        error_counter = 1
// 3 -> error_counter = error && error_led=0
//        error_led=0
//        error_counter = 2
// 4 -> error_counter > error && error_led=0
//        error_led=0
//        error_counter = 3
// 5 -> error_counter > error && error_led=0
//        error_counter = 4
// 6 -> error_counter > error + 3
//        error_counter = 0

void red_led(int8_t level) {
  if (led) digitalWrite(PB4, !level);
  else digitalWrite(PB4, level);
}

void green_led(int8_t level) {
  if (led) digitalWrite(PB3, !level);
  else digitalWrite(PB3, level);
}