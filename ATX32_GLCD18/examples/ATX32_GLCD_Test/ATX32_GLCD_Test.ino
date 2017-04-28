#include <ATX32_GLCD18.h>

void setup() {
  // put your setup code here, to run once:
  glcdClear();
  glcdFillScreen(GLCD_BROWN);
  setTextColor(GLCD_GREEN);
  glcd(1,1,"Hello");
  glcdFillRect(10,30,90,90,GLCD_BLUE);
  setTextBackgroundTransparent();
  setTextColor(GLCD_YELLOW);
  glcd(4,3,"Text on blue");
}

void loop() {
  // put your main code here, to run repeatedly:

}