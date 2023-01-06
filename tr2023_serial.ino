#include "Arduino.h"
uint16_t ADCval;
bool mapped;
unsigned long now;
unsigned long lastSecond = 0;
unsigned long isr_CATCH = 0;

ISR(PCINT2_vect){
}

ISR(INT0_vect){
}

// Catch ADC event
ISR(ADC_vect){
        uint8_t atmp = ADCL; // Load ADC Result's Low Byte to temp
        ADCval = atmp + (ADCH << 8);
        OCR1A = ADCval;
}

void bLNK () {
      if (mapped == HIGH) {digitalWrite(LED_BUILTIN, HIGH);mapped = LOW;}
      else if (mapped == LOW) {digitalWrite(LED_BUILTIN, LOW);mapped = HIGH;}
}

void setup() {
  uint8_t sreg = SREG;
  cli();
    PCICR = 
          0
          | (1 << PCIE2)  // Pin Change Interrupt Enable 2
          ;  
    // Pin Change Mask Register 2
    PCMSK2 = 
          0
          | (1 << PCINT18)  // PD2 (D2) Catch Pin change
          ;  
   ADCSRA =
          0
          | (1 << ADEN) | (1 << ADATE) // Enable ADC, ADC Auto Trigger Enable
          | (1 << ADIE)  // ADC Interrupt Enable, 
          | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) //ADC Prescaler /128
          ;  
   ADMUX =
        0
        | (0 << MUX0) | (1 << MUX1) | (1 << MUX2) // Select ADC6 ADC input 
        | (1 << REFS1) | (1 << REFS0)  // Refs 1.1v         
        ;            
   ADCSRB =
        0
        | (0 << ACME)   // Analog Comparator Multiplexer Disable
        | (0 << ADTS0) | (1 << ADTS1) | (0 << ADTS2) // ADC Auto Trigger Source : External Interrupt Request 0 
        ;

    // External Interrupt Mask Register
   EIMSK = 
          0
          | (1 << INT0)  // External Interrupt Request 0 Enable
          ;  
    // External Interrupt Control Register A
   EICRA = 
          0
          | (1 << ISC00) | (1 << ISC01)  // Rising edge on INT0 generates an interrupt request.
          ;    
// T/C1 Section
TCCR1B = 0;
DDRB |= _BV(PB1) | _BV(PB2); //Set pins as outputs (D9, D10 Arduino)
TCCR1A = _BV(COM1A1) | _BV(COM1B1) //Non-Inv PWM
| _BV(WGM11); // Mode 14: Fast PWM, TOP=ICR1
TCCR1B = _BV(WGM13) | _BV(WGM12)
| _BV(CS10); // Prescaler
ICR1 = 0x3ff; // TOP 10-bit 
OCR1A = 0x0;
OCR1B = 0x0;
// Start the timer with /8 prescaler
TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
SREG = sreg;
sei();

Serial.begin(57600);
}


void loop() {
  now = millis();
  if (now - lastSecond > ((1023 - ADCval)/2)) { 
  //isr_CATCH = OCR1A;
  bLNK(); 
  lastSecond = now; 
//   Serial.println(isr_CATCH);
  Serial.println(ADCval);
  
} 
}