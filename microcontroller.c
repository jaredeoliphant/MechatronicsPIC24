#include<xc.h>
#define FCY 4000000UL
#include<libpic30.h>


//-----------------------------------------------------
// Select oscillator
#pragma config FNOSC = FRC     //8MHz
#pragma config ICS = PGx3  //debug tools if using pin 9 and 10 for programing
#pragma config OSCIOFNC = OFF  // disable clock out on pin 8 so I can use it. 

enum{center, dispenser, Rgoal, Cgoal, Lgoal} location;
enum{initial, drivinggoal, drivingdispenser, looking, returning, dispenserlooking, getballs, dumping, final} state;
enum{corner, notcorner} switchstate;
enum{ON, OFF} eye;

int foundagoal = 0;
int counter = 0;
int timer = 0; 

//-----------------------------------------------------
// Configuration functions
void configpins()
{
    _TRISA0 = _TRISA1 = _TRISB2 = _TRISA2 = _TRISA3 = _TRISB7 = 0; // dc motor outputs (pins 2,3,6,7,8,11)
    //pins 11 (enable), 2, 3 go to Left Right motors that control forward and backward.   
    //pins 6 (enable) ,7,8 go to Forward Backward motors that control left and right. 
    _TRISB8 = _TRISB9 = 1; // push button inputs (pin 12 to left button, pin 13 to right button)
    _TRISA6 = _TRISB14 = _TRISB15 = 0;  // stepper motor outputs.
    _TRISB12 = 0;   // LED output
    _TRISB0 = 1;    // IR sensor input (pin 4)
    _TRISB1 = 0;     // buzzer pwm output (pin 5)
    ANSA = 0x0000; // all A pins are digital
    ANSB = 0x0000; // all B pins are digital
}

void configtmr1()
{    
  _TON = 1;      // Turn on timer 1
  _TCS = 0;     // select internal clock as source
  _TCKPS = 0b11;  // divide by 256 = 15625 cycles per second
  _T1IP = 4;    // sets interrupt priority   (bits 12, 13 and 14 of IPC0: Interrupt priority 
  _T1IE = 1;   // Enable timer1 interrupt   (bit 3 of IEC0 : Interrupt enable control register 0)
  _T1IF = 0;   // clear interrupt flag (bit 3 of IFS0 : Interrupt flag status register 0)
  PR1 = 15000;  // set initial period to half a sec
  TMR1 = 0;       // reset timer 1 
}

void configtmr2()
{    
  _TON = 1;      // Turn on timer 1
  _TCS = 0;     // select internal clock as source
  _TCKPS = 0b11;  // divide by 256 = 15625 cycles per second
  _T2IP = 4;    // sets interrupt priority   (bits 12, 13 and 14 of IPC0: Interrupt priority 
  _T2IE = 1;   // Enable timer1 interrupt   (bit 3 of IEC0 : Interrupt enable control register 0)
  _T2IF = 0;   // clear interrupt flag (bit 3 of IFS0 : Interrupt flag status register 0)
  PR2 = 15000;  // set initial period to half a sec
  TMR2 = 0;       // reset timer 1 
}

void configCN()
{
  _CNIE = 1;    //Enable change notification interrupt (bit 3 of IEC1)
  _CNIP = 6;    //sets interrupt priority  (bits 12,13,14 of IPC4)
  _CN21IE = 1;   // Enable CN on pin 13 (CN21) (button)
  _CN22IE = 1;   // Enable CN on pin 12 (CN22) (button)
  _CN21PUE = 0;  //  Disable pull-up resistor on pin 13
  _CN22PUE = 0;  //  Disable pull-up resistor on pin 12
  _CNIF = 0;    // clear interrupt flag (bit 3 of IFS1)
}
void configpwm()
{
  //registers for pwm    OCxCON1, OCxCON2, OCxRS (period), OCxR (duty cycle), 

//***OC1 is for pin 14 for stepper motor!  (RA6)
  OC1CON1 = 0;
  OC1CON2 = 0;   //reset 

  //configure OC1 
  OC1CON1bits.OCTSEL = 0b111;     //system clock as timing source
  OC1CON2bits.SYNCSEL = 0x1F;    //self synchronization
  OC1CON2bits.OCTRIG = 0;       //synchronizes with OC1 source instead of trigging with the OC1 source
  OC1CON1bits.OCM = 0b110;    //edge-aligned PWM mode
  
  OC1RS = 39999;    //  period. 4M / 100 = 40000 cycles in a period for 100 Hz pwm freq.
  OC1R = 20000;     // will give you a ~50% duty cycle.
  
   //***OC3 is for pin 5 buzzer! (B1)
  
  OC3CON1 = 0;
  OC3CON2 = 0;   //reset 

  //configure OC1 
  OC3CON1bits.OCTSEL = 0b111;     //system clock as timing source
  OC3CON2bits.SYNCSEL = 0x1F;    //self synchronization
  OC3CON2bits.OCTRIG = 0;       //synchronizes with OC1 source instead of trigging with the OC1 source
  OC3CON1bits.OCM = 0b110;    //edge-aligned PWM mode
  
  OC3RS = 4999;    //  period. 4M / 200 = 20000 cycles in a period for 200 Hz pwm freq.
  OC3R = 0;     // initially a 0% duty cycle.

}
void configcomp()
{
    _CVROE = 0;                    // voltage reference output is internal only
    _CVRSS = 0;                    //Vdd and Vss as ref voltage
    //_CVR = 0b00101;                // set voltage reference at 5*3.3/32 = .515 V
    //_CVR = 0b00110;                // set voltage reference at 6*3.3/32 = .619 V
    //_CVR = 0b00111;                // set voltage reference at 7*3.3/32 = .721 V
    //_CVR = 0b01000;                // set voltage reference at 8*3.3/32 = .825 V
    _CVR = 0b01001;                // set voltage reference at 9*3.3/32 = .928 V
    //_CVR = 0b01010;                // set voltage reference at 10*3.3/32 = 1.03 V
    //_CVR = 0b01011;                // set voltage reference at 11*3.3/32 = 1.13 V
    //_CVR = 0b01100;                // set voltage reference at 12*3.3/32 = 1.24 V
    //_CVR = 0b01101;                // set voltage reference at 13*3.3/32 = 1.34 V
    //_CVR = 0b01110;                // set voltage reference at 14*3.3/32 = 1.44 V
    //_CVR = 0b01111;                // set voltage reference at 15*3.3/32 = 1.55 V
    //_CVR = 0b10000;                // set voltage reference at 16*3.3/32 = 1.65 V
    //_CVR = 0b10001;                // set voltage reference at 17*3.3/32 = 1.75 V
    //_CVR = 0b10010;                // set voltage reference at 18*3.3/32 = 1.85 V
    //_CVR = 0b10011;                // set voltage reference at 19*3.3/32 = 1.96 V
    //_CVR = 0b10100;                // set voltage reference at 20*3.3/32 = 2.06 V
    //_CVR = 0b10101;                // set voltage reference at 21*3.3/32 = 2.17 V
    //_CVR = 0b10110;                // set voltage reference at 22*3.3/32 = 2.27 V
    //_CVR = 0b10111;                // set voltage reference at 23*3.3/32 = 2.37 V
    //_CVR = 0b11000;                // set voltage reference at 24*3.3/32 = 2.48 V
    _CVREN = 1;                    //enable the module

    CM1CONbits.CON = 1;           // comparator is enabled
    CM1CONbits.COE = 0;           // comparator output is interal only
    CM1CONbits.CPOL = 1;          // comparator output is inverted
    CM1CONbits.EVPOL = 0b10;      // interrupt on low - high transition
    CM1CONbits.CREF = 1;          // Vin+ connected to internal Vref
    CM1CONbits.CCH = 0b10;        // Vin- connected to C1IND (pin 4))
    
    
    CM1CONbits.CEVT = 0;       // clear the comparator 1 event flag
    _CMIF = 0;                 // clear the global comparator interrupt flag
    _CMIE = 1;               //enable the global comparator interrupt 
}

//-----------------------------------------------------
// Buzzer functions
void tone(float freq, float duration)
{
  OC3RS = 4000000.0/freq;
  OC3R = 2000000.0/freq;    //50% D.C.
  __delay_ms(duration);
  OC3R = 0;                   //0 % D.C. = turn off after alloted time. 
}
void HARRYPOTTER()
{
  float beat = 600;   //lower number will be faster
  tone(493.88,beat);  // B4 quarter
  tone(659.25,1.5*beat);  // E5 dotted quarter
  tone(783.99,.5*beat);   // G5 eighth
  tone(739.99,beat);  //F#5 quarter
  tone(659.25,2*beat);  //E5 half
  tone(987.77,beat);  //B5 quarter
  tone(880.00,3*beat); //A5 dotted half
  tone(739.99,3*beat); //F#5 dotted half
  
  tone(659.25,1.5*beat);  // E5 dotted quarter
  tone(783.99,.5*beat);   // G5 eighth
  tone(739.99,beat);  //F#5 quarter
  tone(587.33,2*beat); //D5 half
  tone(698.46,beat); //F5 quarter
  tone(493.88,3*beat); //B4 dotted half
  
  tone(493.88,beat);  // B4 quarter
  tone(659.25,1.5*beat);  // E5 dotted quarter
  tone(783.99,.5*beat);   // G5 eighth
  tone(739.99,beat);  //F#5 quarter
  tone(659.25,2*beat);  //E5 half
  tone(987.77,beat);  //B5 quarter
  tone(1174.66,2*beat); //D6 half
  tone(1108.73,beat); //C#6 quarter
  tone(1046.5,2*beat); //C6 half
  
  tone(830.61,beat); //G#5 quarter
  tone(1046.5,1.5*beat); //C6 dotted quarter
  tone(987.77,.5*beat); //B5 eighth
  tone(932.33,beat); //Bb5 quarter
  tone(466.16,2*beat); //Bb4 half
  tone(783.99,beat); //G5 quarter
  tone(659.25,3*beat); //E5 dotted half
}


//-----------------------------------------------------
// Motor functions
void STOP()
{
    _LATB7 = _LATB2 = 0; // sleep mode
}
void RotateCCW()
{
    _LATB7 = _LATB2 = 1; // both on
    _LATA0 = _LATA1 = _LATA2 =_LATA3 = 0; // all pins set low
}
void RotateCW()
{
    _LATB7 = _LATB2 = 1; // both on
    _LATA0 = _LATA1 = _LATA2 = _LATA3 = 1; // all pins set high
}

void Moveleft()
{
    _LATB7 = 1; _LATB2 = 0; // one on
    _LATA0 = 0; _LATA1 = 1; // 
}
void Moveright()
{
     _LATB7 = 1; _LATB2 = 0; // one on
    _LATA0 = 1; _LATA1 = 0; // 
}
void Moveforward()
{
    _LATB7 = 0; _LATB2 = 1; // one on
    _LATA2 = 1; _LATA3 = 0; // 
}
void Movebackward()
{
    _LATB7 = 0; _LATB2 = 1; // one on
    _LATA2 = 0; _LATA3 = 1; // 
}
void Diagonalleft()
{
    _LATB7 = _LATB2 = 1; // both on
    _LATA2 = 1; _LATA3 = 0; //     forward
    _LATA0 = 0; _LATA1 = 1; //    left
}
void Diagonalright()
{
    _LATB7 = _LATB2 = 1; // both on
    _LATA2 = 1; _LATA3 = 0; // 
     _LATA0 = 1; _LATA1 = 0; //    right
}



//Stepper motor functions
void HopperSTOP()
{
    _LATB14 = 0;     // sleep   (1 = ON, 0 = OFF)
}

void HopperUP(float inches)
{
//1 rev = 9.5927"
//1 inch = 20.849 steps = 83.396 quarter steps
    _LATB14 = 1;     // sleep  (1 = ON, 0 = OFF)
    _LATB15 = 1;     // direction (1 = CCW , 0 = CW)
      __delay_ms(inches*208.49);
         HopperSTOP();
}

void HopperDOWN(float inches)
{
//1 rev = 9.5927"
//1 inch = 20.849 full steps 
//100 Hz = 100 steps/sec
//100/20.849 = 4.796393 inches/sec
//1000 ms/4.79  = 208.49 ms/inch 
    _LATB14 = 1;     // sleep   (1 = ON, 0 = OFF)
    _LATB15 = 0;     // direction  (1 = CCW , 0 = CW)
    __delay_ms(inches*208.49);
    HopperSTOP();
}

//-----------------------------------------------------
// Interrupt Service Routines
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;    // clear flag
    if (PR1 == 46875)
    {
        counter = counter + 1;
        if (counter == 5)    // 3*5 = 15 sec
        {
            timer = 1;
        }
        else if (counter == 38)     // 3*38 = 114 sec    end of round (Almost)
        {
            STOP();          // stop driver motors
            HopperSTOP();   // all the stop functions to halt everything
            _LATB12 = 0;      // led off
            state = final;
            eye = OFF; 
            CM1CONbits.CON = 0;           // comparator is disabled
            _CNIE = 0;    // disable change notification interrupt (bit 3 of IEC1)
            _T1IE = 0;   // disable timer1 interrupt   (bit 3 of IEC0 : Interrupt enable control register 0)
            PR1 = 15000;
            _CNIF = 0;    //  clear flag
            CM1CONbits.CEVT = 0;      //clear individual comparator flag
            _CMIF = 0;               // clear general interrupt flag
             _T1IF = 0;    // clear flag
            //_TRISB8 = _TRISB9 = 0; // make the button outputs so they can't be hit again
            while(1)
            {
              HARRYPOTTER();   // play a song (repeat forever)!     
            }
        }
        else
        {}
     }
    else
    {}
    //   
  
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
     _CNIF = 0;              //  clear flag
     //  starting condition; this should be the first thing that happens when a button is pushed
     if(state == initial)      
     {
       if(_RB8 == 0 && _RB9 == 0)
       {
         state = dispenserlooking;
         RotateCCW();
         location = center;
         PR1 = 46875;  // start timer for start of game (15625*3 sec = 46875)
         TMR1 = 0;       // reset timer 1 
       }
       else
       {}
     }
     //every time we drive forward looking for the dispenser corner, 
     //the state should be drivingdispenser, switchstate = notcorner
     else if(state == drivingdispenser)
     {
      if(_RB8 == 1 && _RB9 == 1)
      {
         STOP();
         switchstate = corner;
         location = dispenser;
         state = getballs;
      }
      else if(_RB8 == 1 && _RB9 == 0 && switchstate != corner)  //only left switch hits
      {
         Diagonalright();
         switchstate = notcorner;     // the switchstate variable allows us to find a corner
      }
      else if(_RB9 == 1 && _RB8 == 0 && switchstate != corner)
      {
         Diagonalleft();
         switchstate = notcorner;    // the switchstate variable allows us to find a corner
      }
      else
      {
          // do nothing
      }
     }
     //every time we drive forward looking for a goal corner, 
     //the state should be drivinggoal, switchstate = notcorner
     else if(state == drivinggoal)
     {
      if(_RB8 == 1 && _RB9 == 1)
      {
         STOP();
         switchstate = corner;
         state = dumping;
      }
      else if(_RB8 == 1 && _RB9 == 0 && switchstate != corner)  //only left switch hits
      {
         Diagonalright();
         switchstate = notcorner;     // the switchstate variable allows us to find a corner
      }
      else if(_RB9 == 1 && _RB8 == 0 && switchstate != corner)
      {
         Diagonalleft();
         switchstate = notcorner;    // the switchstate variable allows us to find a corner
      }
      else
      {
          // do nothing
      }
     }
     else
     {}
}

void __attribute__((interrupt, no_auto_psv)) _CompInterrupt(void)
{
    CM1CONbits.CEVT = 0;      //clear individual comparator flag
    _CMIF = 0;               // clear general interrupt flag
   
   // looking for an active goal 
    if (eye == ON && state == looking)      
    {
        foundagoal = 1;     //set a flag saying we saw an active goal
        _LATB12 = 1;        // turn the LEDs on so we know we found one
       // eye = OFF;         // stop looking 
    }
    // this should only be the case at the very beginning when we find the dispenser intially 
    else if (state == dispenserlooking)
    {
        RotateCW();
        __delay_ms(25);  // correct direction
        Moveforward();
        state = drivingdispenser;
        switchstate = notcorner;
        // as soon as we find the dispenser we can set the voltage reference higher and therfore be forever blind to the blinking IR led light.
         //_CVR = 0b10001;                // set voltage reference at 17*3.3/32 = 1.75 V
        //_CVR = 0b01111;                // set voltage reference at 15*3.3/32 = 1.55 V
         _CVR = 0b10000;                // set voltage reference at 16*3.3/32 = 1.65 V
        
    }
    else
    {}
}


//-----------------------------------------------------
// Main Function
int main()
{
  //config functions
  configpins();
  configCN();
  configtmr1();
  configpwm();
  configcomp();
 
  state = initial;
  int i,j;              // for "ball count" for loop and "shaking" for loop
  int goal;           // for counting which goal we are at
  eye = OFF;       // initial state of EYE

  // Continuous loop
    while(1)
    {
      
      // when we get to the dispenser and both buttons are pushed
      
        if(location == dispenser && state == getballs)
        {

          for(i = 0; i < 8; i++)     // 9 balls? 
          {
            _LATB12 = 1;      //LEDs on
            __delay_ms(400);  //wait
            _LATB12 = 0;      //LEDs off    this is where the ball should release (2 Hz)
            __delay_ms(400);  //wait 
          }
          Movebackward();
          __delay_ms(1700);  // get back to the center
          STOP();
          location = center;
          while (timer == 0)
          {
            //wait for 15 sec period to be up 
          }
          state = looking; 
        }
        
        // finding an active goal; currently only works if we see the goal on the first pass, 
        //we might need to implement a return trip if we miss the goal
        
        else if(state == looking && location == center)
        {
         // RotateCW();
         // //__delay_ms(900);   // 270 degree turn initially
         // __delay_ms(925);   // 270 degree turn initially
         // STOP();
         // __delay_ms(250); 
          eye = ON;   // turn the IR sensor interrupt ON after I turn away from the dispenser
          //__delay_ms(150); 
          while(state == looking)
          {
            RotateCCW();
            __delay_ms(200);           
            STOP();
            __delay_ms(500);
          goal = goal + 1;
          if(foundagoal == 1)
          {
            switch(goal)            // this switch statement is supposed to set location to the proper goal so we can return to the dispenser properly
            {
              case 1:
                location = Lgoal;
                foundagoal = 0;
                Moveforward();
                state = drivinggoal;
                switchstate = notcorner;
                goal = 0;                //reset goal variable
                eye = OFF;
                _LATB12 = 0;
                break;
              case 2:
                location = Cgoal;
                foundagoal = 0;
                Moveforward();
                state = drivinggoal;
                switchstate = notcorner;
                goal = 0;                //reset goal variable
                 eye = OFF;
                _LATB12 = 0;
                break;
              case 3:
                location = Rgoal;
                foundagoal = 0;
                Moveforward();
                state = drivinggoal;
                switchstate = notcorner;
                goal = 0;                //reset goal variable
                 eye = OFF;
                _LATB12 = 0;
                break; 
              case 4:                 // probably won't hit this one at all now.
                foundagoal = 0;
                goal = 0;                //reset goal variable
                _LATB12 = 0;
                eye = ON; 
                break;
              case 5:
                location = Lgoal;
                foundagoal = 0;
                Moveforward();
                state = drivinggoal;
                switchstate = notcorner;
                goal = 0;                //reset goal variable
                eye = OFF;
                _LATB12 = 0;
                break;
              case 6:
                location = Cgoal;
                foundagoal = 0;
                Moveforward();
                state = drivinggoal;
                switchstate = notcorner;
                goal = 0;                //reset goal variable
                 eye = OFF;
                _LATB12 = 0;
                break;
              case 7:
                location = Rgoal;
                foundagoal = 0;
                Moveforward();
                state = drivinggoal;
                switchstate = notcorner;
                goal = 0;                //reset goal variable
                 eye = OFF;
                _LATB12 = 0;
                break; 
              default:
                break;
            }
      
          }
          else
          {

          }
         }
       }
        
      // when we are at a goal and in the dumping state
      
        else if(state == dumping)
        {
          //__delay_ms(250);         // wait
          HopperUP(9.7);         // up 9.7 inches
          for (j = 0; j < 2; j++)         // make sure all the balls have time to roll down
            {
                HopperDOWN(1);   //shake
                HopperUP(1); 
            }    
          //HopperDOWN(9.7);       // 2017 ms so maybe I could back up and do this at the same time?
         Movebackward();
         HopperDOWN(8.17);    // 1700 ms
         STOP();              // in the center
         HopperDOWN(1.53);      // the rest of the way
          state = returning;    // go back to the dispenser that we remembered
        }
         
      // get back to the dispenser according to which goal we are at
      
        else if(state == returning)
        {
          switch(location)
          {
            case Lgoal:                  //goal locations are defined as if you are standing in the middle looking at the dispenser
              //Movebackward();
              //__delay_ms(1700);         // how long to reach the center
              RotateCW();
              __delay_ms(375);          // 90 degrees clockwise 
              Moveforward();
              state = drivingdispenser; 
              switchstate = notcorner;
              break;
            case Cgoal:
              //Movebackward();
              //__delay_ms(1700);         // how long to reach the center
              RotateCCW();
              __delay_ms(650);         // 180 degrees either direction 
              Moveforward();
              state = drivingdispenser;
              switchstate = notcorner;
              break;
            case Rgoal:
             // Movebackward();
              //__delay_ms(1700);
              RotateCCW();
              __delay_ms(300);
              Moveforward();
              state = drivingdispenser;
              switchstate = notcorner;
              break;
            default:
              break;
          }
        }
        else
        {
           //do nothing
        }
    }
    return 0;
}


