#include <Arduino.h>
#include <LiquidCrystal.h>
#include <max6675.h>

//functions signitures
void regular_mocde();
void temp_select_mode_control();
void LCD_show(bool);
void read_endoder_status();
String convert_double_to_string(double);
double readTemperature();
int CalcPidValues();
void HotendUpdate();
void StepperMove(int);

//pins const setup
/*    LCD Module  ==>   Arduino
 *    1              ==>     Gnd
 *    2              ==>     5V
 *    3             ==>     5V or potensiometer
 *    4             ==>     D12
 *    5             ==>     Gnd
 *    6             ==>     D13
 *    7             ==>     Not Connected
 *    8             ==>     Not Connected
 *    9             ==>     Not Connected
 *    10             ==>    Not Connected
 *    11             ==>    A2
 *    12             ==>    A1
 *    13             ==>    A0
 *    14             ==>    D7
 *    15             ==>     5V
 *    16             ==>     Gnd      */
// LCD wireing shema in /img folder
const int lcd_rs_pin = 12;
const int lcd_en_pin = 13;
const int lcd_A2_pin = A2;
const int lcd_A1_pin = A1;
const int lcd_A0_pin = A0;
const int lcd_d7_pin = 7;

/*    Encoder Module  ==>   Arduino
 *    Clk              ==>     D0
 *    Gnd              ==>     Gnd
 *    Data             ==>     D1
 *    Switch_pin_A     ==>     D2
 *    Switch_Pin_B     ==>     Gnd (Pull off)
 *    */
const int encoder_A_pin = 0;
const int encoder_B_pin = 1;
const int encoder_C_pin = 2;

const int thermo_DO_pin = 4;
const int thermo_CS_pin = 5;
const int thermo_CLK_pin = 6;

const int stepper_dir_pin = 9;
const int stepper_step_pin = 10;

const int hotent_pwm_pin = 3;

const int stepper_control_switch_pin = A3;

//encoder controls consts
const int min_required_temp = 0;
const int max_required_temp = 250;
const int rotery_encoder_step = 10;


//encoder vars
int currentStateA;
int lastStateA;

//temperature and  pid  vars
double current_temp;
int requred_temp;
double last_temp_read;
float PID_error = 0;
float previous_error = 0;
int PID_value = 0;
float elapsed_PID_time;
float PID_time;
float PID_Time_prev;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;

// PID constants
int kp = 40.1;
int ki = 9.2;
int kd = 1.2;

//timers time stemp
unsigned long temp_select_blink_timestemp;
unsigned long display_refresh_timestemp;
unsigned long temp_read_timestemp;
unsigned long hotend_temp_update_timestemp;

//timers intervals
const int temp_select_blink_interval = 500;
const int display_refresh__interval = 500;
const int temp_read_interval = 500;
const int hotend_temp_update_interval = 300;

//menye vars
bool temp_select_mode = false;
bool show_required_temp = true;

const int stepper_delay_time = 1500;


LiquidCrystal lcd(lcd_rs_pin, lcd_en_pin, lcd_A2_pin, lcd_A1_pin, lcd_A0_pin, lcd_d7_pin);
MAX6675 thermocouple(thermo_CLK_pin, thermo_CS_pin, thermo_DO_pin);

void setup()
{
  // Define LCD Display
  pinMode(encoder_A_pin, INPUT);
  pinMode(encoder_B_pin, INPUT);
  pinMode(encoder_C_pin, INPUT);
  pinMode(hotent_pwm_pin,OUTPUT);
  pinMode(stepper_dir_pin,OUTPUT);
  pinMode(stepper_step_pin,OUTPUT);
  pinMode(stepper_control_switch_pin,INPUT);

  TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 and 11 PWM f
  digitalWrite(hotent_pwm_pin,HIGH);
  digitalWrite(stepper_dir_pin,HIGH);

  lcd.begin(16, 2);

  Serial.begin(9600);

  current_temp = 0;
  requred_temp = 0;
  temp_select_blink_timestemp = 0;
  display_refresh_timestemp = 0;
  temp_read_timestemp = 0;
  last_temp_read = 0;

  lcd.clear();
  LCD_show(true);

  lastStateA = digitalRead(encoder_A_pin);
}

void loop()
{
  // check for "click" encoder input
  if (digitalRead(encoder_C_pin))
  {
    temp_select_mode = !temp_select_mode;
  }

  current_temp = readTemperature();
  // choose display mode
  if (temp_select_mode)
  {
    read_endoder_status();
    temp_select_mode_control();
  }
  else
    regular_mocde();

  HotendUpdate();

  if(digitalRead(stepper_control_switch_pin) == HIGH)
    StepperMove(1);

}

void LCD_show(bool show_required_temp)
{
  String current_temp_str = convert_double_to_string(current_temp);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(current_temp_str);
  lcd.print("/");
  if (show_required_temp)
    lcd.print(int(requred_temp));
}

void read_endoder_status()
{
  currentStateA = digitalRead(encoder_A_pin);
  // If last and current state of outputA are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateA != lastStateA && currentStateA == 1)
  {

    // If the outputB state is different than the outputA state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(encoder_B_pin) != currentStateA)
    {
      if (requred_temp <= max_required_temp - rotery_encoder_step)
        requred_temp += rotery_encoder_step;
    }
    else
    {
      // Encoder is rotating CW so increment
      if (requred_temp >= min_required_temp + rotery_encoder_step)
        requred_temp -= rotery_encoder_step;
    }
    //Serial.println(requred_temp);
  }
  lastStateA = currentStateA;
}

void temp_select_mode_control()
{
  if (millis() - temp_select_blink_timestemp >= temp_select_blink_interval)
  {
    temp_select_blink_timestemp = millis();
    LCD_show(show_required_temp);
    show_required_temp = !show_required_temp;
  }
}

void regular_mocde()
{
  if (millis() - display_refresh_timestemp >= display_refresh__interval)
  {
    display_refresh_timestemp = millis();
    LCD_show(true);
  }
}

String convert_double_to_string(double num)
{
  char buffer[7];
  dtostrf(num, 4, 1, buffer);
  return String(buffer);
}

double readTemperature()
{
  if (millis() - temp_read_timestemp >= temp_read_interval)
  {
    last_temp_read = thermocouple.readCelsius();
    temp_read_timestemp = millis();
  }
  return last_temp_read;
}

int CalcPidValues(){
  PID_error = requred_temp - current_temp;
    // Calculate the P value
    PID_p = kp * PID_error;

    if (-3 < PID_error &&  PID_error < 3)
    {
      PID_i = PID_i + (ki * PID_error);
    }

    // For derivative we need real time to calculate speed change rate
    PID_Time_prev = PID_time; // the previous time is stored before the actual time read
    PID_time = millis();      // actual time read
    elapsed_PID_time = (PID_time - PID_Time_prev) / 1000;
    // Now we can calculate the D calue
    PID_d = kd * ((PID_error - previous_error) / elapsed_PID_time);
    // Final total PID value is the sum of P + I + D
    PID_value = PID_p + PID_i + PID_d;

    if (PID_value < 0)
      PID_value = 0;
    if (PID_value > 255)
      PID_value = 255;

    return PID_value;
}

void HotendUpdate(){
  if (requred_temp != 0)
  {
      if (millis() - hotend_temp_update_timestemp >= hotend_temp_update_interval)
      {

      int PID_value = CalcPidValues();
      // Now we can write the PWM signal to the mosfet on digital pin D3
      analogWrite(hotent_pwm_pin, 255 - PID_value);
      previous_error = PID_error;

      hotend_temp_update_timestemp = millis();
    }
  }
  else
  //hotend off
    digitalWrite(hotent_pwm_pin,HIGH);

}

void StepperMove(int stepes)
{
  for (int i = 0; i < stepes; i++)
  {
    digitalWrite(stepper_step_pin, HIGH);
    delayMicroseconds(stepper_delay_time);
    digitalWrite(stepper_step_pin, LOW);
    delayMicroseconds(stepper_delay_time);
    //TODO
    //Change this line to timers based delay instad of delay fumvrion
  }
}
