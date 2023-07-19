#include <Arduino.h>
#include <LiquidCrystal.h>
#include <max6675.h>

void regular_mocde();
void temp_select_mode_control();
void LCD_show(bool);
void read_endoder_status();
String convert_double_to_string(double);
double readTemperature();

//LCD wireing shema in /img folder
const int rs = 12, en = 13, d4 = 11, d5 = 10, d6 = 9, d7 = 7;
const int encoder_A_pin = 0, encoder_B_pin = 1,encoder_C_pin = 2;

int thermoDO = 4;
int thermoCS = 5;
int thermoCLK = 6;


double current_temp;
int requred_temp;
double last_temp_read;

unsigned long temp_select_blink_timestemp;
unsigned long display_refresh_timestemp;
unsigned long temp_read_timestemp;

const int temp_select_blink_interval = 500;
const int display_refresh__interval = 500;
const int temp_read_interval = 500;

const int current_temp_x = 0;
const int current_temp_y = 7;
const int required_temp_x = 0;
const int tequired_temp_y = 13;
const int min_required_temp =0;
const int max_required_temp = 250;
const int rotery_encoder_step = 10;

bool temp_select_mode = false;
bool show_required_temp = true;

int counter = 0;
int currentStateA;
int lastStateA;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {
  // Define LCD Display
  pinMode(encoder_A_pin,INPUT);
  pinMode(encoder_B_pin,INPUT);
  pinMode(encoder_C_pin,INPUT);

  lcd.begin(16,2);

  Serial.begin(9600);
  

  current_temp = 0;
  requred_temp=0;
  temp_select_blink_timestemp = 0;
  display_refresh_timestemp =0;
  temp_read_timestemp =0;
  last_temp_read=0;

  lcd.clear();
  LCD_show(true);

  lastStateA = digitalRead(encoder_A_pin);
}

void loop() {
  //check for "click" encoder input
  if(digitalRead(encoder_C_pin)){
    temp_select_mode = !temp_select_mode;
  }
  current_temp = readTemperature();

  //choose display mode
  if(temp_select_mode){
    read_endoder_status();
    temp_select_mode_control();
  }
  else
    regular_mocde();
}

void LCD_show(bool show_required_temp){
  String current_temp_str = convert_double_to_string(current_temp);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(current_temp_str);
  lcd.print("/");
  if(show_required_temp)
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
      if(requred_temp <= max_required_temp - rotery_encoder_step)
        requred_temp += rotery_encoder_step;
    }
    else
    {
      // Encoder is rotating CW so increment
      if(requred_temp >= min_required_temp + rotery_encoder_step)
        requred_temp -= rotery_encoder_step;
    }
    Serial.println(requred_temp);
  }
  lastStateA = currentStateA;
}

void temp_select_mode_control(){
  if(millis() - temp_select_blink_timestemp >= temp_select_blink_interval){
    temp_select_blink_timestemp = millis();
    LCD_show(show_required_temp);
    show_required_temp = !show_required_temp;
  }
}

void regular_mocde(){
  if(millis() - display_refresh_timestemp >= display_refresh__interval){
    display_refresh_timestemp = millis();
    LCD_show(true);
  }
}

String convert_double_to_string(double num){
  char buffer[7];
  dtostrf(num, 4, 1, buffer);
  return String(buffer);
}

double readTemperature() {
  if(millis() - temp_read_timestemp >= temp_read_interval){
    last_temp_read = thermocouple.readCelsius();
    temp_read_timestemp = millis();
  }
  return last_temp_read;
}
