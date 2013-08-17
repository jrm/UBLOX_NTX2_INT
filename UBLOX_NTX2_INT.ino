#include <avr/pgmspace.h>
#include <TinyGPS_UBX.h>
#include <SoftwareSerial.h>
#include <util/crc16.h>
#include <stdlib.h>
 

SoftwareSerial GPS_Serial(3, 2); //RX pin 4, TX pin 5

TinyGPS gps;

#define PTTPIN 13
#define RADIOPIN1 10
#define RADIOPIN2 11

#define CALLSIGN "M3MRM"

#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 100    // Baud rate for use with RFM22B Max = 600

#define DEBUG true

byte gps_hour, gps_minute, gps_second;
long gps_lat, gps_lon;
unsigned long gps_fix_age, count;
char telemetry [80];
char lat_str[30], lon_str[30], alt_str[30];

char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;


// no need to store these in the RAM anyway
static char str_buffer[25];
prog_char GPSstr_poll[] PROGMEM = "$PUBX,00*33";
prog_char GPSstr_setup1[] PROGMEM = "$PUBX,40,ZDA,0,0,0,0*44";
prog_char GPSstr_setup2[] PROGMEM = "$PUBX,40,GLL,0,0,0,0*5C";
prog_char GPSstr_setup3[] PROGMEM = "$PUBX,40,VTG,0,0,0,0*5E";
prog_char GPSstr_setup4[] PROGMEM = "$PUBX,40,GSV,0,0,0,0*59";
prog_char GPSstr_setup5[] PROGMEM = "$PUBX,40,GSA,0,0,0,0*4E";
prog_char GPSstr_setup6[] PROGMEM = "$PUBX,40,GGA,0,0,0,0*5A";
prog_char GPSstr_setup7[] PROGMEM = "$PUBX,40,RMC,0,0,0,0*47";
PROGMEM const char *str_table[] = {
  GPSstr_poll, GPSstr_setup1, GPSstr_setup2, GPSstr_setup3, 
  GPSstr_setup4, GPSstr_setup5, GPSstr_setup6, GPSstr_setup7
};
 
 
void rtty_txbit (int bit) {
  if (bit)  {
    digitalWrite(RADIOPIN1, HIGH);
    digitalWrite(RADIOPIN2, LOW);

  } else {
    digitalWrite(RADIOPIN1, LOW);
    digitalWrite(RADIOPIN2, HIGH);
  }
}

uint16_t telem_CRC16_checksum (char *string) {
    size_t i;
    uint16_t crc;
    uint8_t c;
    crc = 0xFFFF;
    // Calculate checksum ignoring the first two $s
    for (i = 2; i < strlen(string); i++) {
      c = string[i];
      crc = _crc_xmodem_update (crc, c);
    }
    return crc;
}

ISR(TIMER1_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission. 
    strcpy(txstring,telemetry);
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else
    {
      txstatus=0; // Should be finished
      txj=0;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);   
      txc = txc >> 1;
      break;
    }
    else
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
 
  }
}


void initialise_interrupt() {
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

long readTemp() {
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = (result - 125) * 1075;
  return result;
}



void setup() {
  Serial.println("Initialising...");
  Serial.begin(9600);
  GPS_setup();  
  pinMode(RADIOPIN1, OUTPUT);
  pinMode(RADIOPIN2, OUTPUT);
  pinMode(PTTPIN, OUTPUT);
  digitalWrite(PTTPIN,HIGH);
  initialise_interrupt();
}

void loop() {
  count++;
  GPS_poll();
  gps.crack_time(&gps_hour, &gps_minute, &gps_second, &gps_fix_age);
  gps.get_position(&gps_lat, &gps_lon, &gps_fix_age);
  char time[8];
  sprintf(time, "%02d:%02d:%02d", gps_hour, gps_minute, gps_second);

  Serial.println();
  Serial.print("time: "); Serial.println(time);
  Serial.print("latitude: "); Serial.println(gps_lat/100000.0, 5);
  Serial.print("longitude: "); Serial.println(gps_lon/100000.0, 5);
  Serial.print("altitude: "); Serial.print(gps.altitude()/100.0, 0); Serial.println(" m");
  Serial.print("speed: "); Serial.print(gps.f_speed_kmph(), 2); Serial.println(" km/h");
  Serial.print("vert. speed: "); Serial.print(gps.vspeed(), DEC); Serial.println(" cm/s");
  Serial.print("bearing: "); Serial.println(gps.course()/100, DEC);
  Serial.print("satellites: "); Serial.println(gps.sats(), DEC);
  Serial.print("has fix: "); Serial.println(gps.has_fix(), DEC);
  Serial.print("fix quality: "); Serial.println(gps.fix_quality(), DEC);
  Serial.print("fix age: "); Serial.println(gps_fix_age, DEC);
  Serial.println("------------");
  
  double latitude = gps_lat/100000.0;
  double longitude = gps_lon/100000.0;
  double altitude = gps.altitude()/100.0;
  
  dtostrf(latitude, 8, 6, lat_str);
  dtostrf(longitude, 8, 6, lon_str);
  dtostrf(altitude, 1, 0, alt_str);
  
  telemetry[0] = '\0';
   
  if (gps.has_fix()) {
    sprintf(telemetry, 
            "$$%s,%ld,%02d:%02d:%02d,%s,%s,%s,%d,%d,%ld,%ld",
            CALLSIGN,
            count,
            gps_hour,
            gps_minute,
            gps_second,
            lat_str,
            lon_str,
            alt_str,
            gps.sats(), 
            (gps.has_fix()) ? 1 : 0,
            readVcc(),
            readTemp()
    );
  } else {
    sprintf(telemetry, 
            "$$%s,%ld,%02d:%02d:%02d,%s,%s,%s,%d,%d,%ld,%ld",
            CALLSIGN,
            count,
            gps_hour,
            gps_minute,
            gps_second,
            "0.0",
            "0.0",
            "0.0",
            gps.sats(), 
            (gps.has_fix()) ? 1 : 0,
            readVcc(),
            readTemp()

    );
  }
  sprintf(telemetry, "%s*%04X\n", telemetry, telem_CRC16_checksum(telemetry));
  Serial.println(telemetry);
  delay(5000);
}


void GPS_setup() {
  GPS_Serial.begin(9600);
  GPS_Serial.println("$PUBX,41,1,0007,0003,4800,0*13"); 
  GPS_Serial.begin(4800);
  GPS_Serial.flush();
  delay(500);
  // turn off all NMEA sentences for the uBlox GPS module
  // ZDA, GLL, VTG, GSV, GSA, GGA, RMC
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[1])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[2])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[3])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[4])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[5])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[6])));
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[7])));
  delay(500);
}

// request uBlox to give fresh data
boolean GPS_poll() {
  GPS_Serial.println(strcpy_P(str_buffer, (char*)pgm_read_word(&str_table[0])));
  delay(300);
  unsigned long starttime = millis();
  while (true) {
    if (GPS_Serial.available()) {
      char c = GPS_Serial.read();
      #if DEBUG
        Serial.print(c);
      #endif
      if (gps.encode(c))
        return true;
    }
    if (millis() - starttime > 1000) {
      #if DEBUG
        Serial.println("timeout");
      #endif
      break;
    }
  }
  return false;
}
