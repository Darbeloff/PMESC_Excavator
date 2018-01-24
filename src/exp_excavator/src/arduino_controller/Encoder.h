
// Slave Select pins for encoders 1,2 and 3
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 53;
const int slaveSelectEnc2 = 9;

void initEncoders(int encoder_SSPin) {
  // Set slave selects as outputs
  pinMode(encoder_SSPin, OUTPUT);
  // Raise select pins
  // Communication begins when you drop the individual select signals
  digitalWrite(encoder_SSPin,HIGH);

}

uint16_t readEncoder(int encoder_SSPin){
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    unsigned int AS5147P_angle;
    noInterrupts();           // disable all interrupts
    
      digitalWrite(encoder_SSPin,LOW);      // Begin SPI conversation
      delayMicroseconds(1);
      SPI.transfer16(0xFFFF);                     // Request count
      digitalWrite(encoder_SSPin,HIGH);     // Terminate SPI conversation     
      
      digitalWrite(encoder_SSPin,LOW);      // Begin SPI conversation
      delayMicroseconds(1);
      AS5147P_angle = SPI.transfer16(0xC000);           // Read highest order byte
      digitalWrite(encoder_SSPin,HIGH);     // Terminate SPI conversation
      SPI.endTransaction();
      AS5147P_angle = (AS5147P_angle & (0x3FFF));
    }

    uint16_t AS5147P = ( (unsigned long) AS5147P_angle);
    interrupts();             // enable all interrupts

    return AS5147P;
}

