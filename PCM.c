/*
 * speaker_pcm
 * 
 * Plays 8-bit differential PCM audio on pin 11 and pin 3 using 
 * pulse-width modulation (PWM). This simplifies the hardware as speaker 
 * can be directly connected to arduino pins without a coupling capaitor.
 * For Arduino with Atmega168 at 16 MHz.
 *
 * Uses two timers. The first changes the sample value 8000 times a second.
 * The second holds pin 11 high for 0-255 ticks out of a 256-tick cycle,
 * depending on sample value. Pin 3 compements pin 11. 
 * The second timer repeats 62500 times per second
 * (16000000 / 256), much faster than the playback rate (8000 Hz), so
 * it almost sounds halfway decent, just really quiet on a PC speaker.
 *
 * Takes over Timer 1 (16-bit) for the 8000 Hz timer. This breaks PWM
 * (analogWrite()) for Arduino pins 9 and 10. Takes Timer 2 (8-bit)
 * for the pulse width modulation, breaking PWM for pins 11 & 3.
 *
 * References:
 *     Datasheet of ATmega328P
 *     http://web.csulb.edu/~hill/ee444/Lectures/11%20c%20Timer%20with%20PWM.pdf
 *     http://www.uchobby.com/index.php/2007/11/11/arduino-sound-part-1/
 *     http://www.atmel.com/dyn/resources/prod_documents/doc2542.pdf
 *     http://www.evilmadscientist.com/article.php/avrdac
 *     http://gonium.net/md/2006/12/27/i-will-think-before-i-code/
 *     http://fly.cc.fer.hr/GDM/articles/sndmus/speaker2.html
 *     http://www.gamedev.net/reference/articles/article442.asp
 *
 * Michael Smith <michael@hurts.ca>
 * Abu Bakar Siddique <mabs239@gmail.com> (Added differential output)
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PCM.h"

/*
 * The audio data needs to be unsigned, 8-bit, 8000 Hz, and small enough
 * to fit in flash. 10000-13000 samples is about the limit.
 *
 * sounddata.h should look like this:
 *     const int sounddata_length=10000;
 *     const unsigned char sounddata_data[] PROGMEM = { ..... };
 *
 * You can use wav2c from GBA CSS:
 *     http://thieumsweb.free.fr/english/gbacss.html
 * Then add "PROGMEM" in the right place. I hacked it up to dump the samples
 * as unsigned rather than signed, but it shouldn't matter.
 *
 * http://musicthing.blogspot.com/2005/05/tiny-music-makers-pt-4-mac-startup.html
 * mplayer -ao pcm macstartup.mp3
 * sox audiodump.wav -v 1.32 -c 1 -r 8000 -u -1 macstartup-8000.wav
 * sox macstartup-8000.wav macstartup-cut.wav trim 0 10000s
 * wav2c macstartup-cut.wav sounddata.h sounddata
 *
 * (starfox) nb. under sox 12.18 (distributed in CentOS 5), i needed to run
 * the following command to convert my wav file to the appropriate format:
 * sox audiodump.wav -c 1 -r 8000 -u -b macstartup-8000.wav
 */

int speakerPin = 11;
int speakerPin2 = 3; // abu
unsigned char const *sounddata_data=0;
int sounddata_length=0;
volatile uint16_t sample;
byte lastSample;

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
  if (sample >= sounddata_length) {
    if (sample == sounddata_length + lastSample) {
      stopPlayback();
    }
    else {
      // Ramp down to zero to reduce the click at the end of playback.
      OCR2A = sounddata_length + lastSample - sample;
	  OCR2B = sounddata_length + lastSample - sample; // abu
    }
  }
  else {
    OCR2A = pgm_read_byte(&sounddata_data[sample]);
	OCR2B = pgm_read_byte(&sounddata_data[sample]); // abu
  }
  
  ++sample;
}

void startPlayback(unsigned char const *data, int length)
{
  sounddata_data = data;
  sounddata_length = length;

  pinMode(speakerPin, OUTPUT);
  pinMode(speakerPin2, OUTPUT); // abu
  
  // Set up Timer 2 to do pulse width modulation on the speaker
  // pin.
  
  // ASSR Bit 6 – EXCLK: Enable External Clock Input
  // When EXCLK is written to one, and asynchronous clock is selected, the external clock input buffer is enabled and an
  // external clock can be input on timer oscillator 1 (TOSC1) pin instead of a 32kHz crystal. Writing to EXCLK should be done
  // before asynchronous operation is selected. Note that the crystal oscillator will only run when this bit is zero.

  // ASSR Bit 5 – AS2: Asynchronous Timer/Counter2
  // When AS2 is written to zero, Timer/Counter2 is clocked from the I/O clock, clk I/O . When AS2 is written to one,
  // Timer/Counter2 is clocked from a crystal oscillator connected to the timer oscillator 1 (TOSC1) pin. When the value of AS2 is
  // changed, the contents of TCNT2, OCR2A, OCR2B, TCCR2A and TCCR2B might be corrupted.
  
  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  
  // Set fast PWM mode 
  // WGM2[2:0]=011
  // Timer/Counter Mode of Operation=Fast PWM, TOP=0xFF, Update of OCRx at=BOTTOM, TOV Flag Set on=MAX
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);
  
  // Do non-inverting PWM on pin OC2A, On the Arduino this is pin 11.
  // COM2A[1:0]=10 , Clear OC2A on compare match, set OC2A at BOTTOM, (non-inverting mode).
  TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);

  // Do inverting PWM on pin OC2B, On the Arduino this is pin 3.
  // COM2B[1:0]=11 , Set OC2B on compare match, clear OC2B at BOTTOM, (inverting mode).
  TCCR2A = TCCR2A | _BV(COM2B1) | _BV(COM2B0); 
  //TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0); // Nom-inverting mode of pin 3
  
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20); // CS2[2:0]=001 --> clk T2S /(no prescaling)
  
  // Set initial pulse width to the first sample.
  OCR2A = pgm_read_byte(&sounddata_data[0]);
  OCR2B = pgm_read_byte(&sounddata_data[0]); // abu
  
  
  // Set up Timer 1 to send a sample every interrupt.
  
  cli();
  
  // Set CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  // WGM1[3:0]= 0100
  // Mode=CTC Top=OCR1A (Update Of OCR1x at)=Immediate (TOV1 Flag Set on)=MAX
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  
  
  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  
  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000
  
  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);
  
  lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]);
  sample = 0;
  sei();
}

void stopPlayback()
{
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);
  
  // Disable the per-sample timer completely.
  TCCR1B &= ~_BV(CS10);
  
  // Disable the PWM timer.
  TCCR2B &= ~_BV(CS10);
  
  digitalWrite(speakerPin, LOW);
  digitalWrite(speakerPin2, LOW); // abu
}
