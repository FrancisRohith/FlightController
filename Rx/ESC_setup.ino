
void ESC_setup() {
  pinMode(PA0, PWM);
  pinMode(PA1, PWM);
  pinMode(PA2, PWM);
  pinMode(PA3, PWM);
  // put your setup code here, to run once:
  // Timer2 Configuration Register 1 = (1U << 0) | (1U << 7)
  // TIMER_CR1_CEN - Counter Enable
  // TIMER_CR1_ARPE - Auto Reload Preload Enable
  TIMER2_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  // Configuration Register 2
  TIMER2_BASE->CR2 = 0;
  // Slave Mode Configuration Register
  TIMER2_BASE->SMCR = 0;
  // DMA/interrupt enable register
  TIMER2_BASE->DIER = 0;
  // Status Register
  TIMER2_BASE->SR = 0;
  // Event Generation Register
  TIMER2_BASE->EGR = 0;
  // Capture/Compatre Mode Register 1
  // Output compare 1 & 2 preload, fast enable
  TIMER2_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
  // Output compare 3 & 4 preload, fast enable
  TIMER2_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;;
  // Capture/Compatre Enable Register
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 20000;
  TIMER2_BASE->DCR = 0;

  TIMER2_BASE->CCR1 = 2000;
  TIMER2_BASE->CCR2 = 2000;
  TIMER2_BASE->CCR3 = 2000;
  TIMER2_BASE->CCR4 = 2000;
  delay(3000);
  TIMER2_BASE->CCR1 = 1000;
  TIMER2_BASE->CCR2 = 1000;
  TIMER2_BASE->CCR3 = 1000;
  TIMER2_BASE->CCR4 = 1000;
  delay(2000);
}
