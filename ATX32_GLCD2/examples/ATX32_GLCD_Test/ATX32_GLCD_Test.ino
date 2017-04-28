#include <ATX32_GLCD2.h>

void setup() {
  // put your setup code here, to run once:
  glcdClear();
  glcdFillScreen(GLCD_BROWN);
  setTextSize(2);
  setTextColor(GLCD_GREEN);
  glcd(1,1,"Hello");
  glcdFillRect(20,60,180,180,GLCD_BLUE);
  setTextBackgroundTransparent();
  setTextColor(GLCD_YELLOW);
  glcd(4,3,"Text on blue");
}

void loop() {
  // put your main code here, to run repeatedly:

}