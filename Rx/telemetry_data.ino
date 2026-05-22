void transmit_data() {
  Serial.print(roll, 5);
  Serial.print("\t");
  Serial.print(pitch, 5);
  Serial.print("\t");
  Serial.print(yaw, 5);
  Serial.print("\t");
  Serial.print(esc_1);
  Serial.print("\t");
  Serial.print(esc_2);
  Serial.print("\t");
  Serial.print(esc_3);
  Serial.print("\t");
  Serial.println(esc_4);
}