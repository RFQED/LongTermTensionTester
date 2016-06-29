/* PINGER PROGRAM
  This sketch is for the determination of the tension of a wire in
  a magnetic field by outputting a pulse train on pin 11 which switches a
  current into the wire which  displaces the wire in the magnetic field.
  The frequency is swept up and down and the signal amplitude is monitored to find
  a maximum and hence determine the resonant frequency.
  Thanks to Martin Nawrath - Kunsthochschule fuer Medien Koeln - Academy of Media Arts Cologne
  for DDS Sine Generator code.  It saved a lot of time reading the CPU manual.
   Timer2 generates the  31250 KHz Clock Interrupt
*/

/*      -----( Include required libs )-----       */
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

#include <SPI.h>
#include <Button.h> // needed for the buttons 

int RunBtn = 13;
int ChangeScanBtn = 9;

int buttonState = 0;
int changeState = 0; //looks for button to change the scan range to be pressed 
int loopCount = 0; //for carrying out tension test 3 times for each measurement 

/*
*/
/* table of 256 sine values / one half sine period / stored in flash memory
     Note: syntax of progmem changed from: PROGMEM  prog_uchar sine256[]  = */
const unsigned char sine256[] PROGMEM  = {


  //For this application the wire is driven only on one half cycle

  // Output is from pin 11 and starts high going low - this drives a common emitter
  // pnP transistor which drives current through the wire from the positive supply as pin 11 goes low. For the second half cycle
  // pin 11 is high and the transistor is off. The wire then will relax. At resonance the wire will vibrate and produce a small negative
  // signal. The signal is sampled when the first 255 value is loaded (note - the code ORs a
  253, 248, 242, 235, 229, 223, 217, 211,  204, 198, 192, 186, 180, 174, 168, 163, 157, 151, 145, 140, 134, 129, 123, 118, 113, 108, 103, 98, 93, 88, 83, 79,
  74, 70, 66, 62, 58, 54, 50, 46,   43, 39, 36, 33, 30, 27, 24, 22,   19, 17, 15, 13, 11,  9,  8,  6,    5,  4,  3,  2,  1,  1,  0,  0,
  0,  0,  0,  1,  1,  2,  3,  4,    5,  6,  8,  9, 11, 13, 15, 17,   19, 22, 24, 27, 30, 33, 36, 39,   43, 46, 50, 54, 58, 62, 66, 70,
  74, 79, 83, 88, 93, 98, 103, 108,  113, 118, 123, 129, 134, 140, 145, 151,  157, 163, 168, 174, 180, 186, 192, 198,  204, 211, 217, 223, 229, 235, 242, 248,
  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 254, 254, 254, 254,
  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 254, 254, 254, 255,
  255, 255, 255, 255, 255, 255, 255, 255,  255, 255, 255, 255, 255, 255, 255, 255,  255, 255, 255, 255, 255, 255, 255, 255,  255, 255, 255, 255, 255, 255, 255, 255,
  254, 254, 254, 254, 254, 254, 254, 254,  254, 254, 254, 254, 253, 253, 253, 253,  253, 253, 253, 253, 253, 253, 253, 253,  253, 253, 253, 253, 253, 253, 253, 253,
};

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


//int testPin = 9; //removed as pin 9 now in use
int t2Pin = 6;
byte sinevalue;
int clampPin = 8;

int analogval = 0;
int maxanalogval = 0;
boolean sweep;
double centrefreq = 0.0;
double centrefrequpKHz = 0.0;
double centrefreqdnKHz = 0.0;
double centrefreqavgKHz;
double avgcnt;
int overmaxcount; // used in test for max frequency - if n samples are all lower than max then stop and reverse direction to avoid catching a harmonic
boolean foundmaxup = false; //
boolean foundmaxdn = false; //
boolean connection = true; // assume there is a wire connected at start
const int maxcountlimit = 20;   // number of times analog value is lower than the max value before setting foundmax flags // DEFAULT IS 20! 
const int adcdiff = 2; //number of adc counts lower than max to trigger count for sweep reversal
const int cycles = 2;  //number of cycles at each frequency
const double freqinc = 4.0;


int scanRange = 0;

//LOW SCAN
double lowfreq = 800.0;  // 1115.0 calculated frequency for 0.3N ( 30gm) tension of 79mm wire ORIGINAL 
double highfreq =  1600.0;   //  1703.0 calculated frequency for 0.7N (70gm) tension  ORIGINAL

//HIGH SCAN
// double lowfreq = 1115.0;  // 1115.0 calculated frequency for 0.3N ( 30gm) tension of 79mm wire
// double highfreq = 1900.0; // upped frequency to reach above 70gm 

// actually start at 100Hz lower freq to allow supply stabilisation before starting to sample


//Linear Mass density (LMD) of Tungsten wire 25Um Diameter =0.00000967 Kg/metre
// wire length = 0.079 wire length in metres
//calculate resonant frequency f=1/(2*Length in metres ) * sqrt(Tension in Newtons / Linear mass density Kg/metre)

//Calculate Tension from freq   T = Linear mass density * 2**2 * Length**2 * f**2
// Because of limited precision multiply LMD by 10**6 and by a further 10**2 to convert to grams from newtons
// This is balanced by dividing the squared frequency in Hz by 1000 and using Khz
// Produce a constant (LDMX) - Tension is now found from LDMX * f**2 (f in KHz)
//const float LMDX = 967.0 * 4.0 * 0.086 * 0.086; ////         Length adjusted to calibrate tension at 50 gm on test setup

const float LMDX = 967.0 * 4.0 * 0.087 * 0.086; ////         Length adjusted to calibrate tension at 70 gm on test setup


double Tensionup;
double Tensiondn;

double TensionCalcd; //var holding the calculated tension


double dfreq;


// const double refclk=31372.549;  // =16MHz / 510
const double refclk = 31376.6;    // measured

// variables used inside interrupt service declared as volatile
volatile byte icnt;              // var inside interrupt
volatile byte icnt1;             // var inside interrupt
volatile byte c4ms;              // counter incremented every 4ms
volatile byte pulsecount;        // count output pulses
volatile unsigned long phaccu;   // phase accumulator
volatile unsigned long tword_m;  // dds tuning word m
volatile byte adcflag;        // set in int routn if 255 read as sinevalue - triggers an adc read roughly in centre of waveform from main prog


/*-----( Declare objects )-----*/
// set the LCD address to 0x20 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


/*-----( Declare Variables )-----*/


void setup()/*----( SETUP: RUNS ONCE )----*/
{
  lcd.begin(20, 4);        // initialize the lcd for 20 chars 4 lines and turn on backlight
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // --- button setup

  pinMode(RunBtn, INPUT);
  pinMode(ChangeScanBtn, INPUT);

  digitalWrite(RunBtn, HIGH);
  digitalWrite(ChangeScanBtn, HIGH);


  pinMode(clampPin, OUTPUT);      // sets the digital pin as OUTPUT to drive FET clamp)

  pinMode(5, OUTPUT);      // sets the digital pin as output
  pinMode(6, OUTPUT);      // sets the digital pin as output
  pinMode(7, OUTPUT);      // sets the digital pin as output
  pinMode(11, OUTPUT);     // pin11= PWM  output / frequency output //Will Turner - This could go to pin 5 from 11.  CHANGED TO 9 AS IT WOULDNT REACH OTHERS


  // ------- Quick 3 blinks of backlight  -------------
  for (int i = 0; i < 3; i++)
  {
    lcd.backlight();
    delay(20);
    lcd.noBacklight();
    delay(20);
  }
  lcd.backlight(); // finish with backlight on

  //-------- Initialise display ----------------
  // NOTE: Cursor Position: CHAR, LINE) start at 0
  lcd.setCursor(0, 0); //Start at character 0 on line 0
  lcd.print("Tension Tester v1.5 "); // sets up the screen for the wire number and layer number being recorded
  lcd.setCursor(0, 2);
  lcd.print("     Press Run      ");

  Setup_timer2();

  // disable interrupts to avoid timing distortion
  cbi (TIMSK0, TOIE0);             // disable Timer0 !!! delay() is now not available
  sbi (TIMSK2, TOIE2);             // enable Timer2 Interrupt

  dfreq = 1000.0;                  // initial output frequency = 1000.o Hz
  tword_m = pow(2, 32) * dfreq / refclk; // calulate DDS new tuning word

  // setup start and end frequencies of sweep

  // setup initial values for loop
  adcflag = false;

  analogval = 0;
  digitalWrite(clampPin, HIGH);    // sets the pin high to short out large signals before amp (using a FET as clamp)
  avgcnt = 0.0;
}


/*--(end setup )---*/
void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  loopCount = 0;
  changeState = digitalRead(ChangeScanBtn);
  
  if (!changeState)
 {
    if (scanRange == 1) //scan range start high
    {
    scanRange = 0;
    for (int i = 0; i < 100; i++){
      scanRange = 0; //terrible hack to add delay
       Serial.print("wait");
    } 
    //LOW SCAN
    double lowfreq = 800.0;  // 1115.0 calculated frequency for 0.3N ( 30gm) tension of 79mm wire ORIGINAL 
    double highfreq =  1600.0;   //  1703.0 calculated frequency for 0.7N (70gm) tension  ORIGINAL
    lcd.setCursor(19,0);//if button pressed and set to high currently
    lcd.print("L");//then change to low
    lcd.setCursor(0,3);
    lcd.print("LowFreq = 800-1600Hz");
    //change global vars here for scan range. 
    }

    
    else if (scanRange == 0)
    {
    scanRange = 1;
    for (int i = 0; i < 100; i++){
      scanRange = 1; //terrible hack to add delay
      Serial.print("wait") ;
    }     
  //HIGH SCAN
    double lowfreq = 1115.0;  // 1115.0 calculated frequency for 0.3N ( 30gm) tension of 79mm wire
    double highfreq = 1900.0; // upped frequency to reach above 70gm 
    //change global vars here
    lcd.setCursor(19,0);
    lcd.print("H");
    lcd.setCursor(0,3);
    lcd.print("HiFreq = 1115-1900Hz");
    }
    
  }
  
  buttonState = digitalRead(RunBtn);
  if (!buttonState)
  do
  {   
    if (loopCount == 0)
    {
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("                    ");
   
    }
      loopCount = loopCount+1;

      lcd.setCursor(0, 1);
      lcd.print("  UP    DOWN   AVG  ");
      lcd.setCursor(0, 2);
      lcd.print("F");
      lcd.setCursor(0, 3);
      lcd.print("T");
      
    TensionCalcd = FindTension();
    lcd.setCursor(15, 3);
    lcd.print(TensionCalcd);  


    if (loopCount == 2 && TensionCalcd != 0) {
       lcd.setCursor(0, 0);
       lcd.print("      FINISHED     ");
       break;
    }
    if (TensionCalcd == 0){
       lcd.setCursor(0, 0);
       lcd.print("      FINISHED     ");
       lcd.setCursor(0, 1);
       lcd.print("                   ");
       lcd.setCursor(0, 2);
       lcd.print("      NO SIGNAL     ");
       lcd.setCursor(0, 3);
       lcd.print("                    ");
       break;
      }
  }
 while (loopCount < 3);

}


/* --(end main loop )-- */

//******************************************************************
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
ISR(TIMER2_OVF_vect)
{ if (sinevalue < 255 )
  {
    adcflag = 0;
  }
  if (sinevalue >= 254 )
  { digitalWrite(clampPin, LOW);      // unclamp output
  }
  if (sinevalue < 254 )
  { digitalWrite(clampPin, HIGH);      // clamp output
  }
  phaccu = phaccu + tword_m; // soft DDS, phase accu with 32 bits
  icnt = phaccu >> 24;   // use upper 8 bits for phase accu as frequency information
  // read value fron ROM sine table and send to PWM DAC
  sinevalue = pgm_read_byte_near(sine256 + icnt);

  if (sinevalue == 255 ) // sets flag when at rough centre of quiet half cycle to trigger an adc read
  {
    adcflag++; //  at low freq may increment to 2 - will not trigger a second read
  }
  OCR2A = sinevalue | 1 ; // make sure 244 goes to 255 so no pulses during quiet half cycle (CRUDE - all pulse widths odd)
  if (icnt1++ == 125)       // increment variable c4ms every 4 milliseconds
  { c4ms++;
    icnt1 = 0;
  }
  if (icnt == 0)
  { pulsecount++;
    sbi(PORTD, 7); cbi(PORTD, 7); // output pulse on pin D7 for scope in phase with output frequency

  }
}
//******************************************************************
// timer2 setup
// set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
void Setup_timer2() {

  // Timer2 Clock Prescaler to : 1
  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);

  // Timer2 PWM Mode set to Phase Correct PWM
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);

  sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
}
//******************************************************************

double FindTension()
{

// ********* First part sweeps increasing frequency************
      connection = true; //assume a wire is connected abefore testing if it is not
      dfreq = lowfreq - 100.0;  //start at lower freq to allow supply stabilisation before starting to sample
      foundmaxdn = false;
      foundmaxup = false;
      centrefreq = 0;

       lcd.setCursor(0, 0);
       lcd.print("      RUNNING      ");

      sweep =  true;
      while (sweep == true)
      {
        if (sinevalue >= 254 )      // test if we are in the undriven half cycle
        {
    
          if (adcflag == 1 && dfreq <= lowfreq - 90.0)  // test if outside scan range and sample adc2
          {
            adcflag = 0;                // if adc2 is  30 * 4.9mV then there is no wire connected
            connection = true;          // assume wire is connected
            if (analogRead(2) > 30)
            { connection = false;        // no wire connected
              lcd.setCursor(0, 2);
              lcd.print("F     NO WIRE       ");
              lcd.setCursor(0, 3);
              lcd.print("T                   ");
            }
          }
          if (adcflag == 1 && dfreq > lowfreq)  // test if within scan range
          { // adcflag=0;
            sbi(PORTD, 6); cbi(PORTD, 6); // Test /reset PORTD,6 high to observe timing at start of analog read
            analogval = analogRead(0);   //   get value of signal from wire
    
            if (analogval > maxanalogval)
            { maxanalogval = analogval;
              centrefreq = dfreq;
              overmaxcount = 0;
            }
            if  (analogval < maxanalogval - 4)
            { overmaxcount++;
              if (overmaxcount >= maxcountlimit) {
                foundmaxup = true;
              }
            }
          }
        }
    
        if (pulsecount > cycles)    // keep same frequency for n cycles
        { pulsecount = 0;
          dfreq = dfreq + 4.0;              //Increment scan freq by a few Hz
          cbi (TIMSK2, TOIE2);             // disable Timer2 Interrupt
          tword_m = pow(2, 32) * dfreq / refclk; // calculate new tuning word
          sbi (TIMSK2, TOIE2);             // enable Timer2 Interrupt
          // test if sweep finished
          if (dfreq > highfreq  || overmaxcount > maxcountlimit )
          {
            centrefrequpKHz = 0;
            centrefrequpKHz = centrefreq / 1000.0;
            lcd.setCursor(15, 2);
            lcd.print("     ");
            lcd.setCursor(2, 2);
            if (connection == true && centrefrequpKHz != 0)
            { lcd.print(centrefrequpKHz);
            Tensionup = 0;
              Tensionup = LMDX * centrefrequpKHz * centrefrequpKHz; // in grams
              lcd.setCursor(2, 3);
              lcd.print(Tensionup);
              
            }
            maxanalogval = 0;
            overmaxcount = 0;
            lcd.setCursor(15, 2);
            lcd.print("     ");          //  now clear average values on LCD
            lcd.setCursor(15, 3);
            lcd.print("     ");
    
            sweep = false;
          }
        }
      }
      // ********* Second part sweeps decreasing frequency************
      dfreq = highfreq + 100.0;  //start at Higher freq to allow supply stabilisation before starting to sample
      maxanalogval = 0;
      overmaxcount = 0;
      sweep = true;
      while (sweep == true)
      {
        if (sinevalue >= 254 )      // test if we are in the undriven half cycle
        {
          if (adcflag == 1 && dfreq < highfreq)  // test if within scan range
          {
            sbi(PORTD, 6); cbi(PORTD, 6); // Test /reset PORTD,6 high to observe timing at start of analog read
            analogval = analogRead(0);
    
            if (analogval > maxanalogval)
            { maxanalogval = analogval;
              centrefreq = dfreq;
              overmaxcount = 0;
            }
            if  (analogval < maxanalogval - 4)
            { overmaxcount++;
              if (overmaxcount >= maxcountlimit) {foundmaxdn = true;}
            }
          }
        }
    
        if (pulsecount > cycles)    // keep same frequency for n cycles
        { pulsecount = 0;
          dfreq = dfreq - freqinc;              //Decrement scan freq by a few Hz
          cbi (TIMSK2, TOIE2);             // disable Timer2 Interrupt
          tword_m = pow(2, 32) * dfreq / refclk; // calculate new tuning word
          sbi (TIMSK2, TOIE2);             // enable Timer2 Interrupt
          // test if sweep finished
          if (dfreq < lowfreq   || overmaxcount > maxcountlimit )
          {
            if (foundmaxdn == true && foundmaxup == true)
            {
              centrefreqdnKHz = 0;
              centrefreqdnKHz = centrefreq / 1000.0;
              lcd.setCursor(8, 2);
              lcd.print(centrefreqdnKHz);
              Tensiondn = 0;
              Tensiondn = LMDX * centrefreqdnKHz * centrefreqdnKHz; // in grams
              lcd.setCursor(8, 3);
              lcd.print(Tensiondn);
            }
            else if (foundmaxdn == false || foundmaxup == false)
            {    
              connection = true;          // assume wire is connected
              if (analogRead(2) > 30)
              { connection = false;        // no wire connected
                lcd.setCursor(0, 2);
                lcd.print("F      NO WIRE      ");
                lcd.setCursor(0, 3);
                lcd.print("T                   ");
              }
              else
              { lcd.setCursor(0, 2);
                lcd.print("F     NO SIGNAL     ");
                lcd.setCursor(0, 3);
                lcd.print("T                   ");

              }
            }
            maxanalogval = 0;
            overmaxcount = 0;
            sweep = false;
          }
        }
    
      }
      avgcnt = avgcnt + 1.0;
      centrefreqavgKHz = 0;
      TensionCalcd = 0;
      if (foundmaxdn == true && foundmaxup == true)    
      {
        centrefreqavgKHz = ( centrefrequpKHz + centrefreqdnKHz) / 2.0;
        lcd.setCursor(15, 2);
        lcd.print(centrefreqavgKHz);
        TensionCalcd =  LMDX * centrefreqavgKHz * centrefreqavgKHz;
      }

      
// if (abs(Tensiondn - Tensionup) > 10)
//  {
//    lcd.setCursor(0, 2);
//    lcd.print("F     NO SIGNAL     "); 
//  }

 
 return TensionCalcd;
  }

void flashx3() {
// ------- Quick 3 blinks of backlight  -------------
for (int i = 0; i < 3; i++)
{
  lcd.backlight();
  delay(2000);
  lcd.noBacklight();
  delay(2000);
}
lcd.backlight(); // finish with backlight on

//-------- Initialise display ----------------
}
/* ( THE END ) */
