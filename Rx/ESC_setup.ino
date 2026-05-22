void timer_setup() {
  Serial.println("Timer Setup");
  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA2, INPUT);
  pinMode(PA3, INPUT);
  // put your setup code here, to run once:
  Timer2.attachCompare1Interrupt(handler_channel_1);
  // handler_channel_1 is a function called when a interrupt occurs
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  // Control Register Counter Enable
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  // DMA/Interrupt Enable Register Capture/Compare 1 Interrupt Enable
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  // CC1 and CC2 configured as INPUT 
  // INPUT Capture 1 and 2 mapped to Timer unit 1 and 2 respectively
  // when Edge arrives in Timer unit 1 counter value copied to CCR and Interrupt called
  TIMER2_BASE->CCMR1 = 0b100000001;
  // CC3 and CC4 configured as INPUT 
  // INPUT Capture 3 and 4 mapped to Timer unit 3 and 4 respectively
  TIMER2_BASE->CCMR2 = 0b100000001;   
  // Capture Enable
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  // counter counts till FFFF at 1MHZ frequency
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0XFFFF;
  // DMA Control Registeer
  TIMER2_BASE->DCR = 0;

  // Timer3.attachCompare1Interrupt(handler_channel_5);
  // Timer3.attachCompare2Interrupt(handler_channel_6);
  // TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  // TIMER3_BASE->CR2 = 0;
  // TIMER3_BASE->SMCR = 0;
  // TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE;
  // TIMER3_BASE->EGR = 0;
  // TIMER3_BASE->CCMR1 = 0b100000001; 
  // TIMER3_BASE->CCMR2 = 0;
  // TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E;
  // TIMER3_BASE->PSC = 71;
  // TIMER3_BASE->ARR = 0xFFFF;
  // TIMER3_BASE->DCR = 0;

  pinMode(PA6, PWM);
  pinMode(PA7, PWM);
  pinMode(PB0, PWM);
  pinMode(PB1, PWM);
  // put your setup code here, to run once:
  // Timer2 Configuration Register 1 = (1U << 0) | (1U << 7)
  // TIMER_CR1_CEN - Counter Enable
  // TIMER_CR1_ARPE - Auto Reload Preload Enable
  TIMER3_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  // Configuration Register 2
  TIMER3_BASE->CR2 = 0;
  // Slave Mode Configuration Register
  TIMER3_BASE->SMCR = 0;
  // DMA/interrupt enable register
  TIMER3_BASE->DIER = 0;
  // Status Register
  TIMER3_BASE->SR = 0;
  // Event Generation Register
  TIMER3_BASE->EGR = 0;
  // Capture/Compatre Mode Register 1
  // Output compare 1 & 2 preload, fast enable
  TIMER3_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
  // Output compare 3 & 4 preload, fast enable
  TIMER3_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;;
  // Capture/Compatre Enable Register Capture Compare 1 Enable
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 20000;
  TIMER3_BASE->DCR = 0;

  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
} 

void ESC_setup() {
  TIMER3_BASE->CCR1 = 2000;
  TIMER3_BASE->CCR2 = 2000;
  TIMER3_BASE->CCR3 = 2000;
  TIMER3_BASE->CCR4 = 2000;
  delay(3000);
  TIMER3_BASE->CCR1 = 1000;
  TIMER3_BASE->CCR2 = 1000;
  TIMER3_BASE->CCR3 = 1000;
  TIMER3_BASE->CCR4 = 1000;
  delay(3000);
}

void handler_channel_1() {
  if (GPIOA_BASE->IDR & (0b1 << 0)) {
    channel_1_start = TIMER2_BASE->CCR1;
    // Capture Compare output Polarity Invert
    // after rising edge interrupt, next interrupt should be at falling edge
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;
    if (channel_1 < 0) channel_1 += 0xFFFF;
    // after falling edge interrupt, next interrupt should be at rising edge
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_2() {
  if (GPIOA_BASE->IDR & (0b1 << 1)) {
    channel_2_start = TIMER2_BASE->CCR2;
    // Capture Compare output Polarity Invert
    // after rising edge interrupt, next interrupt should be at falling edge
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;
  }
  else {
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;
    if (channel_2 < 0) channel_2 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}

void handler_channel_3() {
  if (GPIOA_BASE->IDR & (0b1 << 2)) {
    channel_3_start = TIMER2_BASE->CCR3;
    // Capture Compare output Polarity Invert
    // after rising edge interrupt, next interrupt should be at falling edge
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;
  }
  else {
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;
    if (channel_3 < 0) channel_3 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}

void handler_channel_4() {
  if (GPIOA_BASE->IDR & (0b1 << 3)) {
    channel_4_start = TIMER2_BASE->CCR4;
    // Capture Compare output Polarity Invert
    // after rising edge interrupt, next interrupt should be at falling edge
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;
  }
  else {
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;
    if (channel_4 < 0) channel_4 += 0xFFFF;
    // after falling edge interrupt, next interrupt should be at rising edge
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}

void handler_channel_5() {}
void handler_channel_6() {}
