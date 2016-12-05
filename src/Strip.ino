#include <Arduino.h>

#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include <IRLib.h>
#include <TimerOne.h>

#define PIN 6
#define NUM_LEDS 90
#define BRIGHTNESS 100

#define ROT 0
#define GELB 30
#define GRUEN 80
#define TUERKIS 125
#define BLAU 170
#define VIOLETT 215

#define RECV_PIN 11
#define REWIND 0x6CE9
#define PLAY 0x2CE9
#define FORWARD 0x1CE9
#define LAST 0x1EE9
#define STOP 0xCE9
#define NEXT 0x5EE9

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);//Initialize Strip
uint8_t array[NUM_LEDS],array2[NUM_LEDS];
unsigned int Buffer[RAWBUF];
volatile bool recv=0, once=0, on=1, isr=0;//Variable for:IR signal received, function called once, strip is on, isr was triggered
volatile uint32_t count;//Loop count

const byte button1=2, button2=3, AnalogIn=A0; //Pins for Buttons and Lightsensor
int mode=0; //Global mode of Strip
double LightSensor=0, LightSensorOld=0; //Global Lightsensor variable for ISR

IRrecv My_Receiver(RECV_PIN);//Create Receiver
IRdecode My_Decoder;//Create Decoder

void setup() {
  //Start Serial
  Serial.begin(9600);

  //Start Strip
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();

  //Start Receiver
  My_Receiver.enableIRIn(); // Start the receiver
  My_Decoder.UseExtnBuf(Buffer);

  //Timer1.initialize();//Every sec
  //Timer1.attachInterrupt(timercallback);

  //Set Buttonpins and attach Interrupt
  pinMode(button1,INPUT);
  pinMode(button2,INPUT);
  attachInterrupt(0, button1isr, RISING);//Pin2 Hardware Interrupt
  attachInterrupt(1, button2isr, RISING);//Pin3 Hardware Interrupt

  randomSeed(analogRead(2));
}

void setMode(void){
  if(on){
    switch(mode){
    case 0:
      AllWhite();
      break;
    case 1:
      Aurora();
      break;
    case 2:
      RainbowFlow(50);
      break;
    case 3:
      RainbowOverWhite(30,50);
      break;
    case 4:
      WhiteOverRainbow(30,50);
      break;
    case 5:
      Christmas(10,8,255,10,500);//length,lengthon,white,rhytm,wait
      break;
    default:
      mode=0;
      break;
    }
  }
  else
    stripoff();
}


void stripoff(void){
  on=0;
  for(int i=0;i<strip.numPixels(); i++){
    strip.setPixelColor(i, 0,0,0);
  }
  strip.show();
  mode=0;
}

void button1isr(void){
  Serial.println("Button1ISR");
  isr=1;
  once=0;
  if(on){
    stripoff();
  }
  else {
    on=1;
  }
}

void button2isr(void){
  Serial.println("Button2ISR");
  isr=1;
  mode++;
  once=0;
}

//Dimm the strips brightness by the brightnss measured by the lightsensor, called by timer1
void dimmer(void){

  Serial.println("Dimmer");
  LightSensor=analogRead(AnalogIn);//Read LightSensor

  LightSensor=map(LightSensor,10,50, 0, 100);//map Lightsensor Value to 0-100 Range

  //catch out of bound values
  if(LightSensor<0)LightSensor=0;
  if(LightSensor>100)LightSensor=100;

  //make smooth transition between differences
  for(LightSensorOld; LightSensorOld!=LightSensor;){
    if(LightSensorOld<LightSensor)LightSensorOld++;
    if(LightSensorOld>LightSensor)LightSensorOld--;
    //calculate and set new brigtness
    strip.setBrightness(BRIGHTNESS*(1-LightSensorOld/100));
    Serial.println(strip.getBrightness());
    delay(50);
    strip.show();
  }
  LightSensorOld=LightSensor;
}

void checkirreceiver(void){
  unsigned long start;
  //Check IR as long as no interrupt occured or a signal was received, leave loop every x steps
  while(millis()-start>100){
    if (My_Receiver.GetResults(&My_Decoder)) {
      //Decode the IR signal
      My_Decoder.decode();
      if(My_Decoder.decode_type==SONY){
        My_Decoder.DumpResults();
        switch(My_Decoder.value){
        case PLAY:
          Serial.println("Play");
          on=1;
          setMode();
          break;
        case STOP:
          Serial.println("Stop");
          stripoff();
          break;
        case NEXT:
          Serial.println("Next");
          mode++;
          setMode();
          break;
        case LAST:
          Serial.println("Last");
          mode--;
          setMode();
          break;
        }
      }
      //Restart Receiver
      My_Receiver.resume();
    }
  }
}

void timercallback(void){
  Serial.println("Timer");
  checkirreceiver();
}

void loop() {
  //unsigned long start;
  /* //Check IR as long as no interrupt occured or a signal was received, leave loop every x steps
   while(millis()-start>100){//!isr && !recv){
   //check IR Receiver
   checkirreceiver();
   };
   recv=0;*/

  if(count%1000000==0)//ca. every 10 minutes
    dimmer();
  count++;

  //reset isr variable
  if(isr)
    isr=0;

  setMode();

}


void test(void){
  int32_t c=Wheel(20,1);
  c|=((int32_t)100<<24);//Add White

  for (int i = 0; i < 256; i++) {
    strip.setPixelColor(i, c);//Add White);//(i, 255, 70 , 0, 255)
    //strip.setPixelColor(i, 255, 225 , 0, 0);
  }

  printColor(c);

  strip.show();
}


//Set whole strip to white
void AllWhite(void) {
  if(!once){
    int32_t c = Wheel(20,1);//set Color
    c|=((int32_t)255<<24);//Add White

    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
    }

    strip.show();
    once=1;
  }
}

//Dynamic rainbow pattern
void RainbowFlow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256 && !isr; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(i+j,1));
    }
    strip.show();
    delay(wait);
  }
}

void Aurora(void) {
  uint16_t i,j,c;
  uint8_t start=GRUEN,ende=VIOLETT;
  //set initial state
  if(once==0){
    for(i=0; i<=strip.numPixels() && !isr; i++) {
      c=Wheel(random(start+20,ende-20),1);
      strip.setPixelColor(i, c);
      array[i]=c;
      array2[i]=i%2;
    }
    strip.show();
    once=1;
  }

  for(i=0; i<=strip.numPixels() && !isr; i++) {
    //check if decrease
    if(array2[i]==0)
      array[i]-=1;
    //check if increase
    if(array2[i]==1)
      array[i]+=1;
    //hit bottom set to increase
    if(array[i]==start)
      array2[i]=1;
    //hit top set to decrease
    if(array[i]==ende)
      array2[i]=0;

    strip.setPixelColor(i, Wheel(array[i],1));
  }
  strip.show();
  delay(50);
}

//White Glider over rainbow flow
//Input length of glider and wait period
void WhiteOverRainbow (int lenght, int wait){
  int skip=0;

  for(int j=0; j<256 && !isr; j++) {//Rainbow Flow
    for(int i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(i+j,1));
    }

    //Start new glider after one cycle
    int var=j%(strip.numPixels()+lenght);//limit value to strip lenght

      if((var==0 && (256-j)<(strip.numPixels()+lenght))||skip){//wait for end of loop if Glider cant make full cycle
      strip.show();
      skip=1;
      delay(wait);
      continue;
    }

    for(int l=var-lenght+1 && !isr; l<var; l++){//Set Glider
      strip.setPixelColor(l, 0,0,0,255);
    }
    strip.show();
    delay(wait);
  }

}

//Rainbow colored glider over white background
//Input length of glider and wait period
void RainbowOverWhite (int lenght, int wait) {
  //Set all white first
  for (int i = 0; i <= strip.numPixels() && !isr; i++) {
    strip.setPixelColor(i, 0, 0, 0, 255);
  }

  for (int j = 0; j < 256 && !isr; j++) {//Go trough color wheel
    //Start new glider after one cycle
    int var=j%(strip.numPixels()+lenght);//limit value to strip lenght

      if(var==0 && (256-j)<(strip.numPixels()+lenght))//Break if Glider cant make full cycle
      break;

    for(int l=var-lenght+1 && !isr; l<var; l++){//Set Glider
      strip.setPixelColor(l, Wheel(j,1));
    }
    strip.setPixelColor(var-lenght, 0, 0, 0, 255);//Set first Pixel before glider white

    strip.show();
    delay(wait);
  }
}

void Christmas(int lengthon, int length,int white, int rythm, int wait){
  uint16_t j;
  int32_t c;
  int8_t color=-1, var=1, i;
  static int8_t var2=1;
  static int32_t mov=0;

  for(i=0; i<=strip.numPixels() && !isr; i++) {
    if(i%lengthon==0){
      j=i;
      color+=var;
      if(color==1||color==-1)
        var*=-1;
    }
    if(i<j+length){
      if(color==1){
        if(var2==1)
          c=Wheel(GELB,1);
        else
          c=Wheel(ROT,1);
      }
      else if(color==0)
        c=Wheel(GRUEN,1);
      else{
        if(var2==1)
          c=Wheel(ROT,1);
        else
          c=Wheel(GELB,1);
      }
      strip.setPixelColor((i+mov)%strip.numPixels(), c);
    }
    else
      strip.setPixelColor((i+mov)%strip.numPixels(),((int32_t)white<<24));
  }
  if(mov%rythm==0)
    var2*=-1;
  strip.show();
  mov++;
  delay(wait);
}

//------------------------------------------------------------------------------------
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, byte maxCol) {
  WheelPos = 255 - WheelPos;

  int red, green, blue, white;

  if (WheelPos < 85) {
    red=255 - WheelPos * 3;
    green=0;
    blue=WheelPos * 3;

  }
  else if (WheelPos < 170) {
    WheelPos -= 85;

    red=0;
    green=WheelPos * 3;
    blue=255 - WheelPos * 3;
  }
  else{
    WheelPos -= 170;

    red=WheelPos * 3;
    green=255 - WheelPos * 3;
    blue=0;
  }

  if(maxCol)
    maxColor(red, green, blue);

  return strip.Color(red, green, blue, 0);
}

//Maximizes Saturation of Color
void maxColor(int& red, int& green, int& blue){
  float factor;

  if(!red){
    if(green>blue){
      factor=255.0/green;
    }
    else{
      factor=255.0/blue;
    }
  }

  if(!green){
    if(red>blue){
      factor=255.0/red;
    }
    else{
      factor=255.0/blue;
    }
  }

  if(!blue){
    if(red>green){
      factor=255.0/red;
    }
    else{
      factor=255.0/green;
    }
  }

  red*=factor;
  green*=factor;
  blue*=factor;
}

uint8_t white(uint32_t c){
  return (c>>24);
}

uint8_t red(uint32_t c) {
  return (c >> 16);
}
uint8_t green(uint32_t c) {
  return (c >> 8);
}
uint8_t blue(uint32_t c) {
  return (c);
}

void printColor(uint32_t c){
  Serial.print(red(c));
  Serial.print("\t");
  Serial.print(green(c));
  Serial.print("\t");
  Serial.print(blue(c));
  if(white(c)){
    Serial.print("\t");
    Serial.print(white(c));
  }
  Serial.print("\n");
}
