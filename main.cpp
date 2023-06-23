#include <Arduino.h>
#include <LiquidCrystal.h>

void regular_mocde();
void temp_select_mode_control();
void LCD_show(bool);

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
const int encoder_A_pin = 6, encoder_B_pin = 7,encoder_C_pin = 8;
double current_temp;
double requred_temp;

unsigned long temp_select_blink_timestemp;
unsigned long display_refresh_timestemp;
const int temp_select_blink_interval = 500;
const int display_refresh__interval = 500;

const int current_temp_x = 0;
const int current_temp_y = 7;
const int required_temp_x = 0;
const int tequired_temp_y = 13;

bool temp_select_mode = true;
bool show_required_temp = true;

int counter = 0;
int currentStateA;
int lastStateA;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  // Define LCD Display
  pinMode(encoder_A_pin,INPUT);
  pinMode(encoder_B_pin,INPUT);
  pinMode(encoder_C_pin,INPUT);

  lcd.begin(16,2);
  current_temp = 0;
  requred_temp=0;
  temp_select_blink_timestemp = 0;
  display_refresh_timestemp =0;
  lcd.clear();
  LCD_show(true);

  lastStateA = digitalRead(encoder_A_pin);
}

void loop() {
  //check for "click" encoder input
  if(digitalRead(encoder_C_pin)){
    temp_select_mode = !temp_select_mode;
  }
  //choose display mode
  if(temp_select_mode){
    read_endoder_status();
    temp_select_mode_control();
  }
  else
    regular_mocde();
}

void LCD_show(bool show_required_temp){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(current_temp);
  lcd.print("/");
  if(show_required_temp)
    lcd.print(requred_temp);
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
      requred_temp += 10;
    }
    else
    {
      // Encoder is rotating CW so increment
      requred_temp -= 10;
    }
  }
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
