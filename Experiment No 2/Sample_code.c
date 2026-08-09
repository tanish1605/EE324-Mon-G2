// -O0
// 7372800Hz
//
// PID line follower — see README.md for the full write-up of the design.
//
// NOTE: hand-written, not compiled — no AVR toolchain available in this
// environment. Check register/pin names against your exact board before
// flashing.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.c"

unsigned char ADC_Conversion(unsigned char);
unsigned char l = 0;
unsigned char c = 0;
unsigned char r = 0;
unsigned char PortBRestore = 0;

// ---- Per-sensor calibration ------------------------------------------
// TODO: measure each sensor's raw ADC reading fully OFF the line (MIN)
// and fully ON the line (MAX), then fill these in. See README.md ->
// "Calibration" for how to take the measurements. Left at 0/255 (a
// no-op) so the code behaves like uncalibrated raw values until you do.
unsigned char L_MIN = 0,   L_MAX = 255;
unsigned char C_MIN = 0,   C_MAX = 255;
unsigned char R_MIN = 0,   R_MAX = 255;

// ---- Tunable PID / behavior constants ---------------------------------
// error/position is on a normalized -255..+255 scale (see compute_position),
// so these are a reasonable starting point but still need retuning on the
// actual bot, especially after you plug in real calibration values.
#define KP              0.9f
#define KI              0.0005f
#define KD              6.0f      // acts on a *filtered* delta, so can run higher than a raw-derivative gain without going noisy
#define D_FILTER_ALPHA  0.5f      // 0..1, higher = trust the newest sample more (less smoothing)

#define BASE_SPEED      150
#define MAX_SPEED       255
#define MIN_SPEED       60
#define PIVOT_SPEED     140       // wheel speed used during a hard pivot turn
#define LINE_THRESH     20        // applied to NORMALIZED sensor values
#define RECOVERY_SPEED  100
#define SHARP_TURN_ERR  180.0f    // |error| beyond this -> pivot instead of differential steer

float error = 0, prev_error = 0, integral = 0;
float derivative = 0, filtered_derivative = 0, pid_output = 0;

// ---- Hardware setup (unchanged from your version) ----------------------

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;
 PORTB = PORTB & 0xF0;
 DDRD = DDRD | 0x30;
 PORTD = PORTD | 0x30;
}

void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;
 Direction &= 0x0F;
 PortBRestore = PORTB;
 PortBRestore &= 0xF0;
 PortBRestore |= Direction;
 PORTB = PortBRestore;
}

void forward (void)        { motion_set(0x06); }
void back (void)           { motion_set(0x09); }
void left (void)           { motion_set(0x05); }
void right (void)          { motion_set(0x0A); }
void soft_left (void)      { motion_set(0x04); }
void soft_right (void)     { motion_set(0x02); }
void soft_left_2 (void)    { motion_set(0x01); }
void soft_right_2 (void)   { motion_set(0x08); }
void hard_stop (void)      { motion_set(0x00); }
void soft_stop (void)      { motion_set(0x0F); }

void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;
 ACSR = 0x80;
 ADCSRA = 0x86;
}

void init_devices (void)
{
 cli();
 port_init();
 adc_init();
 sei();
}

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;
 PORTC = PORTC & 0x80;
}

void adc_pin_config (void)
{
 DDRA = 0x00;
 PORTA = 0x00;
}

void port_init()
{
 lcd_port_config();
 adc_pin_config();
 motion_pin_config();
}

void timer1_init(void)
{
 TCCR1B = 0x00;
 TCNT1H = 0xFF;
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D;
}

unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;
 while((ADCSRA&0x10)==0);
 a=ADCH;
 ADCSRA = ADCSRA|0x10;
 return a;
}

void velocity_control(unsigned char left_speed, unsigned char right_speed)
{
 OCR1AL = left_speed;
 OCR1BL = right_speed;
}

int cap_speed(int speed)
{
 if (speed > MAX_SPEED) return MAX_SPEED;
 if (speed < MIN_SPEED) return MIN_SPEED;
 return speed;
}

// ---- Per-sensor normalization -------------------------------------------
// Maps a raw ADC reading onto a common 0..255 scale using THIS sensor's
// measured min/max, so all three sensors become directly comparable
// despite having different raw sensitivities.
unsigned char normalize_sensor(unsigned char raw, unsigned char s_min, unsigned char s_max)
{
 if (s_max <= s_min) return raw;   // guards against unset/bad calibration data
 long scaled = ((long)raw - s_min) * 255L / (long)(s_max - s_min);
 if (scaled < 0)   scaled = 0;
 if (scaled > 255) scaled = 255;
 return (unsigned char)scaled;
}

// ---- Weighted-position error --------------------------------------------
// Treats each (normalized) sensor value as a weight and computes a
// centroid across the three sensor positions:
//   ~ -255  -> line fully under the left sensor
//   ~    0  -> line centered
//   ~ +255  -> line fully under the right sensor
float compute_position(unsigned char lv, unsigned char cv, unsigned char rv)
{
 unsigned int total = (unsigned int)lv + cv + rv;
 if (total == 0) return prev_error;   // guarded by caller, kept as a safety net
 long weighted = (long)lv * -255L + (long)rv * 255L;  // center contributes 0 weight
 return (float)weighted / (float)total;
}

// ---- Main ---------------------------------------------------------------

int main(void)
{
 int prev = 0;
 init_devices();
 timer1_init();

 lcd_set_4bit();
 lcd_init();

 while(1)
 {
  // Raw readings — shown on the LCD as-is so they're usable for calibration.
  unsigned char l_raw = ADC_Conversion(3);
  unsigned char c_raw = ADC_Conversion(4);
  unsigned char r_raw = ADC_Conversion(5);
  lcd_print(1, 1, l_raw, 3);
  lcd_print(1, 5, c_raw, 3);
  lcd_print(1, 9, r_raw, 3);
  lcd_print(1, 13, prev, 3);

  // Normalized readings — everything below this line uses these, not raw.
  l = normalize_sensor(l_raw, L_MIN, L_MAX);
  c = normalize_sensor(c_raw, C_MIN, C_MAX);
  r = normalize_sensor(r_raw, R_MIN, R_MAX);

  unsigned int total = (unsigned int)l + c + r;

  if (total < LINE_THRESH)
  {
   // Line fully lost: back off and swing toward the side we were last
   // curving to, rather than a fixed direction.
   hard_stop();
   velocity_control(RECOVERY_SPEED, RECOVERY_SPEED);
   if (prev_error > 0)      right();
   else if (prev_error < 0) left();
   _delay_ms(50);
   back();
   _delay_ms(50);
   hard_stop();

   // Recovery used a large, non-PID-loop delay — reset controller state
   // so the next PID cycle doesn't see a bogus derivative ("derivative
   // kick") from the time that passed.
   integral = 0;
   filtered_derivative = 0;
   prev_error = 0;
  }
  else if (l > LINE_THRESH && c > LINE_THRESH && r > LINE_THRESH)
  {
   // All sensors on line (thick line / intersection) — go straight.
   forward();
   velocity_control(BASE_SPEED, BASE_SPEED);
   integral = 0;
   prev_error = 0;
  }
  else
  {
   error = compute_position(l, c, r);

   if (fabsf(error) > SHARP_TURN_ERR)
   {
    // Sharp corner: differential PID steering can't turn fast enough
    // before the bot runs off the line, so pivot on the spot instead.
    if (error > 0) right();
    else           left();
    velocity_control(PIVOT_SPEED, PIVOT_SPEED);
    integral = 0;
    filtered_derivative = 0;
    prev_error = error;
   }
   else
   {
    integral += error;
    if (integral > 200)  integral = 200;
    if (integral < -200) integral = -200;

    derivative = error - prev_error;
    filtered_derivative = D_FILTER_ALPHA * derivative + (1.0f - D_FILTER_ALPHA) * filtered_derivative;
    prev_error = error;

    pid_output = (KP * error) + (KI * integral) + (KD * filtered_derivative);

    int left_speed  = cap_speed(BASE_SPEED - (int)pid_output);
    int right_speed = cap_speed(BASE_SPEED + (int)pid_output);

    forward();
    velocity_control((unsigned char)left_speed, (unsigned char)right_speed);
   }
  }

  _delay_ms(10);
 }
}
