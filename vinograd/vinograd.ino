#define pushButton_pin   7
#define LED_pin   8
#include <stdio.h>
bool button1 = false;
int counter_wind, counter_rain = 0;
int val = 0;
void wind()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
  digitalWrite(LED_pin, !digitalRead(LED_pin));
  counter_wind++;
  button1 = true;
  }
  last_interrupt_time = interrupt_time;
}
void rain()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
  counter_rain++;
  button1 = true;
  }
  last_interrupt_time = interrupt_time;
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_pin, OUTPUT);
  pinMode(pushButton_pin, INPUT_PULLUP);
  attachInterrupt(pushButton_pin, wind, RISING);
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(6, rain, RISING);
} 
void loop()
{
  if(button1){
    Serial.print("Broj kise: ");
		Serial.println(counter_rain);
    Serial.print("Broj vjetra: ");
		Serial.println(counter_wind);
    button1 = false;
    val=analogRead(4);
  val = map(val,0,4100,0,100);
  Serial.print("Osvjetljenje: ");
  Serial.println(val);
	}
}