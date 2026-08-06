#include <Arduino.h>
#include <M5Unified.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>
#include <lgfx/v1/lgfx_fonts.hpp>

using namespace lgfx::v1;
namespace {
constexpr int SDA_A=32, SCL_A=33;
constexpr uint8_t PCA=0x40, TOF=0x29;
constexpr uint32_t I2C_HZ=400000, VESC_BAUD=115200;
constexpr float SHUNT_OHM=0.002f;
constexpr float DUTY_LIMIT_SAFE=0.03f, DUTY_LIMIT_FULL=1.00f;
constexpr uint32_t FULL_DUTY_ARM_HOLD_MS=1500;
enum class Mode:uint8_t { Servo, Tof, Ina, Vesc };
Mode mode=Mode::Servo, pageMode=Mode::Vesc;
HardwareSerial vesc(1);
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData{};
bool tofOn=false, pcaFound=false, pcaReady=false, pwmWriteOk=false, inaOk=false, vescOk=false;
uint8_t inaAddr=0; uint16_t centerMm=0;
float busV=NAN, currentA=NAN, vescV=NAN, vescA=NAN, vescDuty=NAN, vescErpm=NAN;
float voltageHistory[100]{}, currentHistory[100]{}; uint8_t histCount=0, histHead=0;
int servoAngle=90; uint16_t pwmCount=0; float dutySet=0;
bool fullDutyMode=false,fullDutyHoldTriggered=false;
uint32_t lastPoll=0,lastDraw=0,lastVescRequest=0,lastMotorTouch=0,fullDutyArmStart=0;
const char* const JP_TITLE=u8"ポートA 機器チェッカー";
const char* const JP_SERVO=u8"サーボ";
const char* const JP_TOF=u8"距離";
const char* const JP_INA=u8"電流・電圧";
const char* const JP_MOTOR=u8"モータ";

bool ack(uint8_t a){Wire.beginTransmission(a);return Wire.endTransmission()==0;}
bool read16(uint8_t a,uint8_t r,uint16_t& v){Wire.beginTransmission(a);Wire.write(r);if(Wire.endTransmission(false)!=0||Wire.requestFrom((int)a,2)!=2)return false;v=((uint16_t)Wire.read()<<8)|Wire.read();return true;}
bool read8(uint8_t a,uint8_t r,uint8_t& v){Wire.beginTransmission(a);Wire.write(r);if(Wire.endTransmission(false)!=0||Wire.requestFrom((int)a,1)!=1)return false;v=Wire.read();return true;}
void i2cStart(){vesc.end();Wire.end();Wire.begin(SDA_A,SCL_A,I2C_HZ);Wire.setTimeOut(25);}
uint16_t crc16(const uint8_t* p,size_t n){uint16_t c=0;while(n--){c^=(uint16_t)*p++<<8;for(uint8_t b=0;b<8;b++)c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}return c;}
int16_t i16(const uint8_t* p){return (int16_t)(((uint16_t)p[0]<<8)|p[1]);}
int32_t i32(const uint8_t* p){return (int32_t)(((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]);}

bool pcaWrite(uint8_t ch,uint16_t on,uint16_t off){uint8_t r=0x06+4*ch;Wire.beginTransmission(PCA);Wire.write(r);Wire.write(on&255);Wire.write(on>>8);Wire.write(off&255);Wire.write(off>>8);return Wire.endTransmission()==0;}
void servoStop(){if(pcaReady)pwmWriteOk=pcaWrite(0,0,0);}
bool pcaReg(uint8_t r,uint8_t v){Wire.beginTransmission(PCA);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
void pcaSetup(){
  pcaReady=false;
  pwmWriteOk=false;
  if(!ack(PCA))return;
  bool ok=pcaReg(0x00,0x10);
  ok=ok&&pcaReg(0x01,0x04);
  ok=ok&&pcaReg(0xFE,121);
  ok=ok&&pcaReg(0x00,0x21);
  ::delay(5);
  ok=ok&&pcaReg(0x00,0xA1);
  ::delay(1);
  uint8_t mode1=0,mode2=0,prescale=0;
  ok=ok&&read8(PCA,0x00,mode1)&&read8(PCA,0x01,mode2)&&read8(PCA,0xFE,prescale);
  ok=ok&&((mode1&0x10)==0)&&((mode1&0x40)==0)&&((mode2&0x04)!=0)&&(prescale==121);
  pcaReady=ok;
  if(pcaReady){
    pwmCount=map(servoAngle,0,180,102,512);
    pwmWriteOk=pcaWrite(0,0,pwmCount);
    if(!pwmWriteOk)pcaReady=false;
  }
}
void setServo(int angle){servoAngle=constrain(angle,0,180);if(!pcaReady){pcaSetup();if(!pcaReady)return;}pwmCount=map(servoAngle,0,180,102,512);pwmWriteOk=pcaWrite(0,0,pwmCount);if(!pwmWriteOk)pcaReady=false;}

float activeDutyLimit(){return fullDutyMode?DUTY_LIMIT_FULL:DUTY_LIMIT_SAFE;}
void sendVescDuty(float duty){const float limit=activeDutyLimit();int32_t v=(int32_t)(constrain(duty,-limit,limit)*100000.0f);uint8_t p[5]={5,(uint8_t)(v>>24),(uint8_t)(v>>16),(uint8_t)(v>>8),(uint8_t)v};uint16_t c=crc16(p,5);uint8_t f[10]={2,5,p[0],p[1],p[2],p[3],p[4],(uint8_t)(c>>8),(uint8_t)c,3};vesc.write(f,sizeof(f));}
void motorStop(){if(mode==Mode::Vesc)sendVescDuty(0);dutySet=0;}
void sendVescRequest(){const uint8_t p[]={4};uint16_t c=crc16(p,1);const uint8_t f[]={2,1,4,(uint8_t)(c>>8),(uint8_t)c,3};vesc.write(f,sizeof(f));}
void parseVesc(){static uint8_t state=0,n=0,i=0,p[80]{};static uint16_t got=0;while(vesc.available()){uint8_t b=vesc.read();if(state==0){if(b==2)state=1;}else if(state==1){n=b;i=0;state=(n&&n<=sizeof(p))?2:0;}else if(state==2){p[i++]=b;if(i==n)state=3;}else if(state==3){got=(uint16_t)b<<8;state=4;}else if(state==4){got|=b;state=5;}else{if(b==3&&got==crc16(p,n)&&n>=29&&p[0]==4){const uint8_t* q=p+1;vescA=i32(q+8)/100.0f;vescDuty=i16(q+20)/1000.0f;vescErpm=(float)i32(q+22);vescV=i16(q+26)/10.0f;vescOk=true;}state=0;}}}

void enter(Mode next){if(next==mode){if(next==Mode::Servo){i2cStart();pcaFound=false;pcaReady=false;pwmWriteOk=false;pcaSetup();pcaFound=pcaReady;}return;}servoStop();if(tofOn)tof.stopRanging();tofOn=false;motorStop();fullDutyMode=false;fullDutyArmStart=0;fullDutyHoldTriggered=false;mode=next;pcaFound=pcaReady=pwmWriteOk=inaOk=vescOk=false;if(mode==Mode::Vesc){Wire.end();vesc.begin(VESC_BAUD,SERIAL_8N1,SCL_A,SDA_A);}else{i2cStart();if(mode==Mode::Servo){pcaSetup();pcaFound=pcaReady;}else if(mode==Mode::Tof&&ack(TOF)&&tof.begin(TOF,Wire)&&tof.setResolution(64)&&tof.setRangingFrequency(10)&&tof.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS)&&tof.startRanging())tofOn=true;}}
void pollServo(){pcaFound=ack(PCA);if(!pcaFound){pcaReady=false;pwmWriteOk=false;return;}if(!pcaReady)pcaSetup();}
void pollTof(){if(tofOn&&tof.isDataReady()&&tof.getRangingData(&tofData))centerMm=tofData.distance_mm[32];}
void pollIna(){constexpr uint8_t a[]={0x40,0x41,0x44};inaOk=false;for(uint8_t x:a){uint16_t maker=0,die=0,b=0,s=0;if(read16(x,0xFE,maker)&&read16(x,0xFF,die)&&maker==0x5449&&die==0x2260&&read16(x,0x02,b)&&read16(x,0x01,s)){inaAddr=x;busV=b*0.00125f;currentA=(int16_t)s*0.0025f/1000.0f/SHUNT_OHM;inaOk=true;voltageHistory[histHead]=busV;currentHistory[histHead]=currentA;histHead=(histHead+1)%100;if(histCount<100)histCount++;return;}}}

void text(int x,int y,const char* s,uint16_t fg=0xFFFF,uint16_t bg=0x0000){M5.Display.setTextColor(fg,bg);M5.Display.setCursor(x,y);M5.Display.print(s);}
void tab(int x,const char* s,bool active){uint16_t bg=active?0x07E0:0x39E7;M5.Display.fillRoundRect(x,202,76,34,5,bg);M5.Display.setTextColor(active?0x0000:0xFFFF,bg);M5.Display.setCursor(x+5,212);M5.Display.print(s);}
void slider(int y,float position,uint16_t color){int left=18,right=301;int knob=left+(int)((right-left)*constrain(position,0.0f,1.0f));M5.Display.fillRoundRect(left,y,right-left,16,8,0x39E7);M5.Display.fillRoundRect(left,y,knob-left,16,8,color);M5.Display.fillCircle(knob,y+8,11,0xFFFF);M5.Display.drawCircle(knob,y+8,11,0x0000);}
void servoButton(int x,const char* label,bool neutral=false){uint16_t bg=neutral?0xFFE0:0x39E7;uint16_t fg=neutral?0x0000:0xFFFF;M5.Display.fillRoundRect(x,122,58,38,5,bg);M5.Display.drawRoundRect(x,122,58,38,5,0xFFFF);M5.Display.setTextColor(fg,bg);int16_t tw=M5.Display.textWidth(label);M5.Display.setCursor(x+(58-tw)/2,135);M5.Display.print(label);}
void drawPage(){M5.Display.fillScreen(0x0000);M5.Display.setTextSize(1);M5.Display.setTextColor(0xFFFF,0x0000);text(8,7,JP_TITLE,0xFFFF);if(mode==Mode::Servo){text(8,30,JP_SERVO,0x07FF);text(8,50,u8"PCA9685  /  I2C  G32=SDA  G33=SCL",0xBDF7);text(8,78,u8"ボタンで角度を調整",0xFFE0);text(8,172,u8"※ CH0、50Hz / 中立=90 deg",0xBDF7);}else if(mode==Mode::Tof){text(8,30,JP_TOF,0x07FF);text(8,50,u8"VL53L5CX  /  I2C  G32=SDA  G33=SCL",0xBDF7);text(8,78,u8"8x8 距離マップ (mm)",0xFFE0);}else if(mode==Mode::Ina){text(8,30,JP_INA,0x07FF);text(8,50,u8"INA226  /  I2C  G32=SDA  G33=SCL",0xBDF7);text(8,78,u8"黄: 電圧[V]  水: 電流[A]",0xFFE0);}else{text(8,30,JP_MOTOR,0x07FF);text(8,50,u8"VESC  /  UART  G32=TX  G33=RX",0xBDF7);text(8,78,u8"Dutyをスライド (安全上 +/-3%)",0xFFE0);text(8,172,u8"指を離すと 0% に戻ります",0xBDF7);}tab(2,JP_SERVO,mode==Mode::Servo);tab(82,JP_TOF,mode==Mode::Tof);tab(162,JP_INA,mode==Mode::Ina);tab(242,JP_MOTOR,mode==Mode::Vesc);pageMode=mode;}
void drawServo(){M5.Display.fillRect(6,96,308,70,0x0000);M5.Display.setTextColor(0xFFFF,0x0000);M5.Display.setCursor(8,98);M5.Display.printf(u8"角度: %3d deg   %s",servoAngle,pcaReady?u8"接続済":u8"未接続");servoButton(8,"-5");servoButton(70,"-1");servoButton(132,u8"中立",true);servoButton(194,"+1");servoButton(256,"+5");}
uint16_t rangeColor(uint16_t mm){if(mm==0||mm>4000)return 0x0000;uint8_t r=constrain((int)(255-(mm*255/2000)),0,255),b=255-r;return M5.Display.color565(r,32,b);}
void drawTof(){M5.Display.fillRect(8,96,304,96,0x0000);M5.Display.setTextColor(0xFFFF,0x0000);M5.Display.setCursor(8,96);M5.Display.printf(u8"中心: %u mm  %s",centerMm,tofOn?u8"計測中":u8"未接続");for(int y=0;y<8;y++)for(int x=0;x<8;x++){uint16_t d=tofData.distance_mm[y*8+x];M5.Display.fillRect(55+x*26,112+y*10,24,8,rangeColor(d));}}
void drawIna(){M5.Display.fillRect(6,94,308,98,0x0000);M5.Display.setTextColor(0xFFFF,0x0000);M5.Display.setCursor(8,96);M5.Display.printf("%.3f V  %.3f A  %s",busV,currentA,inaOk?"OK":"--");M5.Display.drawRect(8,116,304,70,0x7BEF);if(histCount<2)return;for(int j=1;j<histCount;j++){int a=(histHead-histCount+j-1+100)%100,b=(histHead-histCount+j+100)%100;int x1=9+(j-1)*302/(histCount-1),x2=9+j*302/(histCount-1);int yv1=185-constrain((int)(voltageHistory[a]/30.0f*68),0,68),yv2=185-constrain((int)(voltageHistory[b]/30.0f*68),0,68);int yi1=151-constrain((int)(currentHistory[a]/5.0f*34),-34,34),yi2=151-constrain((int)(currentHistory[b]/5.0f*34),-34,34);M5.Display.drawLine(x1,yv1,x2,yv2,0xFFE0);M5.Display.drawLine(x1,yi1,x2,yi2,0x07FF);}}
void drawVesc(){const float limit=activeDutyLimit();M5.Display.fillRect(6,70,308,91,0x0000);uint16_t buttonColor=fullDutyMode?0xF800:0xFD20;M5.Display.fillRoundRect(8,72,304,22,5,buttonColor);M5.Display.setTextColor(0x0000,buttonColor);M5.Display.setCursor(14,79);if(fullDutyMode)M5.Display.print(u8"上限解除中: +/-100%  (タップで解除)");else if(fullDutyArmStart)M5.Display.printf(u8"上限解除へ長押し: %lu / %lu ms",(unsigned long)(::millis()-fullDutyArmStart),(unsigned long)FULL_DUTY_ARM_HOLD_MS);else M5.Display.print(u8"上限解除: 1.5秒長押しで有効化");M5.Display.setTextColor(0xFFFF,0x0000);M5.Display.setCursor(8,98);M5.Display.printf("%.1fV  %.2fA  %.0f ERPM",vescV,vescA,vescErpm);M5.Display.setCursor(8,110);M5.Display.printf("Duty: %+0.1f %% / +/-%.0f %%  %s",dutySet*100,limit*100,vescOk?"OK":"...");slider(125,(dutySet+limit)/(2*limit),fullDutyMode?0xF800:0xFD20);}
void draw(){if(pageMode!=mode)drawPage();if(mode==Mode::Servo)drawServo();else if(mode==Mode::Tof)drawTof();else if(mode==Mode::Ina)drawIna();else drawVesc();}
void handleTouch(){auto t=M5.Touch.getDetail();if(t.wasClicked()&&t.y>=198){if(t.x<80)enter(Mode::Servo);else if(t.x<160)enter(Mode::Tof);else if(t.x<240)enter(Mode::Ina);else enter(Mode::Vesc);return;}if(mode==Mode::Servo&&t.wasClicked()&&t.y>=118&&t.y<=165){if(t.x<66)setServo(servoAngle-5);else if(t.x<128)setServo(servoAngle-1);else if(t.x<190)setServo(90);else if(t.x<252)setServo(servoAngle+1);else setServo(servoAngle+5);return;}if(mode==Mode::Vesc&&t.y>=72&&t.y<=94){if(fullDutyMode){if(t.wasClicked()){motorStop();fullDutyMode=false;}return;}if(t.isPressed()){if(!fullDutyArmStart)fullDutyArmStart=::millis();if(!fullDutyHoldTriggered&&::millis()-fullDutyArmStart>=FULL_DUTY_ARM_HOLD_MS){motorStop();fullDutyMode=true;fullDutyHoldTriggered=true;}}else{fullDutyArmStart=0;fullDutyHoldTriggered=false;}return;}if(!t.isPressed()){fullDutyArmStart=0;fullDutyHoldTriggered=false;}if((!t.isPressed()&&!t.wasClicked())||t.y<110||t.y>155)return;float p=constrain((t.x-18)/283.0f,0.0f,1.0f);if(mode==Mode::Vesc){const float limit=activeDutyLimit();dutySet=(p*2-1)*limit;sendVescDuty(dutySet);lastMotorTouch=::millis();}}
}
void setup(){auto c=M5.config();c.serial_baudrate=115200;M5.begin(c);M5.Display.setFont(&lgfx::v1::fonts::lgfxJapanGothic_12);M5.Display.setTextSize(1);Serial.begin(115200);i2cStart();}
void loop(){M5.update();handleTouch();uint32_t now=::millis();if(mode==Mode::Vesc){if(now-lastVescRequest>400){sendVescRequest();lastVescRequest=now;}parseVesc();if(dutySet!=0&&now-lastMotorTouch>250)motorStop();}else if(now-lastPoll>150){if(mode==Mode::Servo)pollServo();else if(mode==Mode::Tof)pollTof();else pollIna();lastPoll=now;}if(now-lastDraw>120){draw();lastDraw=now;}::delay(4);}
