import processing.serial.*;
import controlP5.*; // controlP5 library
import processing.opengl.*;

ControlP5 controlP5;
//The serial port:
Serial myPort;

int xPP        = 80;
int yPP        = 100;
int xPD        = 200;
int yPD        = 100;
int xRP        = 80;
int yRP        = 150;
int xRD        = 200;
int yRD        = 150;
int xYP        = 80;
int yYP        = 200;
int xYD        = 200;
int yYD        = 200;
int xPPWR        = 375;
int yPPWR        = 100;
int xRPWR        = 375;
int yRPWR        = 150;
int xYPWR        = 375;
int yYPWR        = 200;
int xRollCal     = 480;
int yRollCal     = 200;
int xRollZero    = 540;
int yRollZero    = 200;

// Constants
int Y_AXIS = 1;
int X_AXIS = 2;
color b1, b2, c1, c2;


Numberbox PitchP;
Numberbox PitchD;
Numberbox RollP;
Numberbox RollD;
Numberbox YawP;
Numberbox YawD;
Numberbox PitchPWR;
Numberbox RollPWR;
Numberbox YawPWR;
Numberbox RollCal;
Button buttonREAD,buttonWRITE, buttonCONFon, buttonCONFoff, buttonP0, buttonP1, buttonP2, buttonRCOff, buttonRCOn, buttonYawRC, buttonYawAut;
Button buttonZeroRoll;
int m=10, i, commListMax;

color green_ = color(0, 120, 170), gray_ = color(60, 60, 60); ///30 120 30
boolean writeEnable = false;
boolean readEnable = false;
boolean printlist = true;
boolean portopen = false;
char RCcontrol = '0';
char YawRCon = '0';
String readStatus = "";
String Version = "0.4.1.2";

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}


void setup()
{

  size(600, 280);
  background(75);

  textSize(28);
  fill(0, 120, 170);//blue
  text("EvvGC GUI",20,50); text(Version, 170,50);


  // Define colors
  b1 = color(80);
  b2 = color(60);
  c1 = color(80, 80, 80);
  c2 = color(60, 61, 59);

  controlP5 = new ControlP5(this); // initialize the GUI controls

  //List all the available serial ports:
  println(Serial.list());



  commListMax = -1;
  for(int i=0;i<Serial.list().length;i++) {
    commListMax = i;
  }



  /******************************PID cells*************************************************************/
  PitchP = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("PitchP",0,xPP,yPP,40,16));
  PitchP.setColorBackground(gray_);PitchP.setMin(0);PitchP.setDirection(Controller.HORIZONTAL);PitchP.setDecimalPrecision(2);PitchP.setMultiplier(0.01);PitchP.setMax(2.54);

  PitchD = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("PitchD",0,xPD,yPD,40,16));
  PitchD.setColorBackground(gray_);PitchD.setMin(0);PitchD.setDirection(Controller.HORIZONTAL);PitchD.setDecimalPrecision(2);PitchD.setMultiplier(0.01);PitchD.setMax(1);

  RollP = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("RollP",0,xRP,yRP,40,16));
  RollP.setColorBackground(gray_);RollP.setMin(0);RollP.setDirection(Controller.HORIZONTAL);RollP.setDecimalPrecision(2);RollP.setMultiplier(0.01);RollP.setMax(2.54);

  RollD = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("RollD",0,xRD,yRD,40,16));
  RollD.setColorBackground(gray_);RollD.setMin(0);RollD.setDirection(Controller.HORIZONTAL);RollD.setDecimalPrecision(2);RollD.setMultiplier(0.01);RollD.setMax(1);

  YawP = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("YawP",0,xYP,yYP,40,16));
  YawP.setColorBackground(gray_);YawP.setMin(0);YawP.setDirection(Controller.HORIZONTAL);YawP.setDecimalPrecision(2);YawP.setMultiplier(0.01);YawP.setMax(2.54);

  YawD = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("YawD",0,xYD,yYD,40,16));
  YawD.setColorBackground(gray_);YawD.setMin(0);YawD.setDirection(Controller.HORIZONTAL);YawD.setDecimalPrecision(2);YawD.setMultiplier(0.01);YawD.setMax(1);


  /******************************Power cells*************************************************************/
  PitchPWR = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("PitchPWR",0,xPPWR,yPPWR,40,16));
  PitchPWR.setColorBackground(gray_);PitchPWR.setMin(0);PitchPWR.setDirection(Controller.HORIZONTAL);PitchPWR.setDecimalPrecision(0);PitchPWR.setMultiplier(1);PitchPWR.setMax(100);

  RollPWR = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("RollPWR",0,xRPWR,yRPWR,40,16));
  RollPWR.setColorBackground(gray_);RollPWR.setMin(0);RollPWR.setDirection(Controller.HORIZONTAL);RollPWR.setDecimalPrecision(0);RollPWR.setMultiplier(1);RollPWR.setMax(100);

  YawPWR = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("YawPWR",0,xYPWR,yYPWR,40,16));
  YawPWR.setColorBackground(gray_);YawPWR.setMin(0);YawPWR.setDirection(Controller.HORIZONTAL);YawPWR.setDecimalPrecision(0);YawPWR.setMultiplier(1);YawPWR.setMax(100);


  /******************************Calibration cells*************************************************************/
  RollCal = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("RollCal",0,xRollCal,yRollCal,40,16));
  RollCal.setColorBackground(gray_);RollCal.setMin(-10);RollCal.setDirection(Controller.HORIZONTAL);RollCal.setDecimalPrecision(1);RollCal.setMultiplier(0.2);RollCal.setMax(10);
  buttonZeroRoll =      controlP5.addButton("ZERO",1,535,200,40,16); buttonZeroRoll.setColorBackground(gray_);


  /****************************Buttons*********************************************************************/
   buttonWRITE =     controlP5.addButton("WRITE",1,500,248,60,20);buttonWRITE.setColorBackground(gray_);
   buttonREAD =     controlP5.addButton("READ",1,40,248,60,20);buttonREAD.setColorBackground(gray_);

   buttonCONFon =     controlP5.addButton("CONFIGon",1,400,10,60,60);buttonCONFon.setColorBackground(gray_);
   buttonCONFoff =     controlP5.addButton("CONFIGoff",1,330,10,60,60);buttonCONFoff.setColorBackground(gray_);

   buttonP0 =     controlP5.addButton("P0",1,490, 5,20,20);buttonP0.setColorBackground(gray_);
   buttonP1 =     controlP5.addButton("P1",1,490,30,20,20);buttonP1.setColorBackground(gray_);
   buttonP2 =     controlP5.addButton("P2",1,490,55,20,20);buttonP2.setColorBackground(gray_);

   buttonRCOff  =     controlP5.addButton("RC_OFF",1,450,122,65,20);buttonRCOff.setColorBackground(gray_);
   buttonRCOn   =     controlP5.addButton("RC_ON",1,450,100,65,20);buttonRCOn.setColorBackground(gray_);

   buttonYawRC  =     controlP5.addButton("Yaw_RC_Pan",1,520,100,75,20);buttonYawRC.setColorBackground(gray_);
   buttonYawAut =     controlP5.addButton("Yaw_Auto_Pan",1,520,122,75,20);buttonYawAut.setColorBackground(gray_);

   buttonCONFoff.setColorBackground(green_);

}

void draw() {
  //String Serial.list()[];
  //size(600, 280);
  //background(80);

  fill(70); strokeWeight(0);stroke(35); //75
  rect(0, 80, 600, 155, 0);

  fill(70); strokeWeight(0);stroke(35);
  rect(440, 80, 600, 155, 0);

  fill(75); strokeWeight(0);stroke(80);
  rect(0, 240, 600, 100, 0);

  fill(255);

  // Background
 // setGradient(0, 0, width, 80, c2, c1, Y_AXIS);

  textSize(12);
  text("Pitch P:",35,112);
  textSize(12);
  text("Pitch D:",155,112);

  textSize(12);
  text("Roll P:",38,162);
  textSize(12);
  text("Roll D:",158,162);

  textSize(12);
  text("Yaw P:",38,211);
  textSize(12);
  text("Yaw D:",158,211);

  textSize(12);
  text("Pitch Power:             %",301,112);
  textSize(12);
  text("Roll Power:             %",307,162);
  textSize(12);
  text("Yaw Power:             %",305,212);

  textSize(12);
  text("Roll Calibration (degr.)",460,190);

   textSize(16);
  //fill(0, 102, 153, 204);
 // text("Connected to:",400,45); text(Serial.list()[0],515,45);
 // text("Connected to:",400,45); text(Serial.list()[1],515,65);
  //text("Connected to:",400,45); text(Serial.list()[2],515,65);


    if(printlist==true){
      for(int i=0;i<=commListMax;i++) {
      text(Serial.list()[i],515,21+i*25);
      }
      printlist=false;
    }

    textSize(12);
    text(readStatus,110,263);

    if(PitchP.value()==0)PitchP.setValue(0.01);
    if(RollP.value()==0)RollP.setValue(0.01);
    if(YawP.value()==0)YawP.setValue(0.01);

    if(PitchD.value()==0)PitchD.setValue(0.01);
    if(RollD.value()==0)RollD.setValue(0.01);
    if(YawD.value()==0)YawD.setValue(0.01);

    if(PitchPWR.value()==0)PitchPWR.setValue(1);
    if(RollPWR.value()==0)RollPWR.setValue(1);
    if(YawPWR.value()==0)YawPWR.setValue(1);

    //if(RollCal.value()==-9.9)RollCal.setValue(-9.9);

}


public void WRITE() {
  if(writeEnable == false) return;

  myPort.write("h");

  myPort.write (int (PitchP.value()*100));
  myPort.write (int (RollP.value()*100));
  myPort.write (int (YawP.value()*100));

  myPort.write (int (PitchD.value()*100));
  myPort.write (int (RollD.value()*100));
  myPort.write (int (YawD.value()*100));

  myPort.write (int (PitchPWR.value()));
  myPort.write (int (RollPWR.value()));
  myPort.write (int (YawPWR.value()));

  myPort.write (RCcontrol);
  myPort.write (YawRCon);

  myPort.write (int (RollCal.value()*10+100));
  //println (RollCal.value());
  //println (int (RollCal.value()*10+100));

 readStatus = "Write OK";

}

public void READ() {
  if(readEnable == false) return;


myPort.write("g");//sends get values command

while(i<200000000){i++;}//delay
i=0;

//myPort.write("OK");
readStatus="Can't Read";

if(myPort.read()=='x'){
PitchP.setValue(myPort.read()/100.00);
RollP.setValue(myPort.read()/100.00);
YawP.setValue(myPort.read()/100.00);

PitchD.setValue(myPort.read()/100.00);
RollD.setValue(myPort.read()/100.00);
YawD.setValue(myPort.read()/100.00);

PitchPWR.setValue(myPort.read());
RollPWR.setValue(myPort.read());
YawPWR.setValue(myPort.read());

RCcontrol=char(myPort.read());
YawRCon  =char(myPort.read());

RollCal.setValue((myPort.read()-100.00)/10.00);

if (RCcontrol == '0'){  buttonRCOff.setColorBackground(green_); buttonRCOn.setColorBackground(gray_);}
if (RCcontrol == '1'){  buttonRCOff.setColorBackground(gray_);  buttonRCOn.setColorBackground(green_);}

if (YawRCon   == '0'){  buttonYawRC.setColorBackground(gray_);  buttonYawAut.setColorBackground(green_);}
if (YawRCon   == '1'){  buttonYawRC.setColorBackground(green_);  buttonYawAut.setColorBackground(gray_);}

readStatus="Read OK";
}


}

public void CONFIGon() {

  myPort.write("i"); //enter config command
  buttonWRITE.setColorBackground(green_);
  buttonREAD.setColorBackground(green_);
  buttonCONFon.setColorBackground(green_);
  buttonCONFoff.setColorBackground(gray_);
  writeEnable = true;
  readEnable = true;

}

public void CONFIGoff() {

  myPort.write("j"); //exit config command
  buttonWRITE.setColorBackground(gray_);
  buttonREAD.setColorBackground(gray_);
  buttonCONFon.setColorBackground(gray_);
  buttonCONFoff.setColorBackground(green_);
  writeEnable = false;
  readEnable = false;
}

public void P0() {

  buttonP0.setColorBackground(green_);
  buttonP1.setColorBackground(gray_);
  buttonP2.setColorBackground(gray_);
  // Open the port you are using at the rate you want:
  if(portopen==true){ myPort.stop();}
  myPort = new Serial(this, Serial.list()[0], 9600);
  portopen=true;

}

public void P1() {

  buttonP0.setColorBackground(gray_);
  buttonP1.setColorBackground(green_);
  buttonP2.setColorBackground(gray_);
  // Open the port you are using at the rate you want:
  if(portopen==true){ myPort.stop();}
  myPort = new Serial(this, Serial.list()[1], 9600);
  portopen=true;

}

public void P2() {

  buttonP0.setColorBackground(gray_);
  buttonP1.setColorBackground(gray_);
  buttonP2.setColorBackground(green_);
  // Open the port you are using at the rate you want:
  if(portopen==true){ myPort.stop();}
  myPort = new Serial(this, Serial.list()[2], 9600);
  portopen=true;

}

public void RC_OFF() {
  buttonRCOn.setColorBackground(gray_);
  buttonRCOff.setColorBackground(green_);
  RCcontrol = '0';
}

public void RC_ON() {
  buttonRCOff.setColorBackground(gray_);
  buttonRCOn.setColorBackground(green_);
  RCcontrol = '1';
}

public void Yaw_RC_Pan() {
  buttonYawAut.setColorBackground(gray_);
  buttonYawRC.setColorBackground(green_);
  YawRCon = '1';
}

public void Yaw_Auto_Pan() {
  buttonYawRC.setColorBackground(gray_);
  buttonYawAut.setColorBackground(green_);
  YawRCon = '0';
}

public void ZERO() {
  RollCal.setValue(0.0);
  //println (RollCal.value());
  //println (int (RollCal.value()*10+100));

}







