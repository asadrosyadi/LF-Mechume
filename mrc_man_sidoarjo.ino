#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 10); // RX, TX
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo grab,dof1,dof2,grip;

int pos2=90, pos1=90;
int grab_ball=0,mod;
int speed_ = 1,kec;
int speed_right=0;
int speed_left=0;
const int en=39;
const int S0=41;
const int S1=43;
const int S2=45;
const int S3=47;
const int sig=A0;
const int en_hor=39;
const int S0_hor=41;
const int S1_hor=43;
const int S2_hor=45;
const int S3_hor=47;
const int sig_hor=A1;
const int button1=27;
const int button2=25;
const int button3=23;
const int atas_in1= 46;
const int atas_in2 = 44;
const int atas_in3 = 42;
const int atas_in4 = 40;
const int atas_ena=3;
const int atas_enb=2;
const int bawah_in1=31;
const int bawah_in2=33;
const int bawah_in3=35;
const int bawah_in4=37;
const int bawah_ena=7;
const int bawah_enb=8;
int enb=13;
int in4=12;
int in3=9;
//int selenoid=13;
//int selenoid2=12;
unsigned char tombol1=0;
unsigned char tombol2=0;
unsigned char tombol3=0;
int data_sensor[12];
unsigned int counter=0,i,j,count=0,up=0,lintasan=0;
int xsensor,var,jumlah_indeks,jum_indeks;
int s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,flag;
int ref[12],sens[12];
int sensor_now[12],maks[12];
int minimal[12]={1100,1100,1100,1100,1100,1100,1100,1100,1100,1100,1100,1100};
int address=0,address_jumlah_indeks=50,address_speed=15,address_kons_p=20,address_kons_d=30,address_kons_i=40;
unsigned char pwm_ka,pwm_ki,indeks=0;
int laju,kecepatan,pi,di,ai,kons_p,kons_d,kons_i;

void maju(int kece){
  kecepatan=kece;
  analogWrite(atas_ena,kecepatan);     analogWrite(atas_enb,kecepatan);
  digitalWrite(atas_in1, HIGH);         digitalWrite(atas_in3, HIGH);
  digitalWrite(atas_in2, LOW);        digitalWrite(atas_in4, LOW);    
  analogWrite(bawah_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(bawah_in1, HIGH);      digitalWrite(bawah_in3, HIGH);
  digitalWrite(bawah_in2, LOW);     digitalWrite(bawah_in4, LOW); 
}
void mundur(int kece){
  kecepatan=kece;
  analogWrite(atas_ena,kecepatan);     analogWrite(atas_enb,kecepatan);
  digitalWrite(atas_in1, LOW);        digitalWrite(atas_in3, LOW);
  digitalWrite(atas_in2, HIGH);         digitalWrite(atas_in4, HIGH);    
  analogWrite(bawah_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(bawah_in1, LOW);     digitalWrite(bawah_in3, LOW);
  digitalWrite(bawah_in2, HIGH);      digitalWrite(bawah_in4, HIGH); 
}
void bel_ka(int kece){
  kecepatan=kece;
  analogWrite(atas_ena,kecepatan);     analogWrite(atas_enb,kecepatan);
  digitalWrite(atas_in1, LOW);        digitalWrite(atas_in3, HIGH);
  digitalWrite(atas_in2, HIGH);         digitalWrite(atas_in4, LOW);    
  analogWrite(bawah_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(bawah_in1, LOW);     digitalWrite(bawah_in3, HIGH);
  digitalWrite(bawah_in2, HIGH);      digitalWrite(bawah_in4, LOW); 
}
void bel_ki(int kece){
  kecepatan=kece;
  analogWrite(atas_ena,kecepatan);     analogWrite(atas_enb,kecepatan);
  digitalWrite(atas_in1, HIGH);         digitalWrite(atas_in3, LOW);
  digitalWrite(atas_in2, LOW);        digitalWrite(atas_in4, HIGH);    
  analogWrite(bawah_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(bawah_in1, HIGH);      digitalWrite(bawah_in3, LOW);
  digitalWrite(bawah_in2, LOW);     digitalWrite(bawah_in4, HIGH); 
}
void geser_kiri(int kece){
  kecepatan=kece;
  analogWrite(atas_ena,kecepatan);     analogWrite(atas_enb,kecepatan);
  digitalWrite(atas_in1, HIGH);         digitalWrite(atas_in3, LOW);
  digitalWrite(atas_in2, LOW);        digitalWrite(atas_in4, HIGH);    
  analogWrite(bawah_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(bawah_in1, LOW);     digitalWrite(bawah_in3, HIGH);
  digitalWrite(bawah_in2, HIGH);      digitalWrite(bawah_in4, LOW); 
}
void geser_kanan(int kece){
  kecepatan=kece;
  analogWrite(atas_ena,kecepatan);     analogWrite(atas_enb,kecepatan);
  digitalWrite(atas_in1, LOW);        digitalWrite(atas_in3, HIGH);
  digitalWrite(atas_in2, HIGH);         digitalWrite(atas_in4, LOW);    
  analogWrite(bawah_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(bawah_in1, HIGH);      digitalWrite(bawah_in3, LOW);
  digitalWrite(bawah_in2, LOW);     digitalWrite(bawah_in4, HIGH); 
}
void diagonal_kanan(int kece){
  kecepatan=kece;           
  analogWrite(bawah_ena,kecepatan);  analogWrite(atas_enb,kecepatan);
  digitalWrite(bawah_in1, HIGH);      digitalWrite(atas_in3, HIGH);
  digitalWrite(bawah_in2, LOW);     digitalWrite(atas_in4, LOW);
}
void diagonal_kiri(int kece){
  kecepatan=kece;           
  analogWrite(atas_ena,kecepatan);  analogWrite(bawah_enb,kecepatan);
  digitalWrite(atas_in1, HIGH);      digitalWrite(bawah_in3, HIGH);
  digitalWrite(atas_in2, LOW);     digitalWrite(bawah_in4, LOW);
}
void stopped(){
  analogWrite(atas_ena,0);     analogWrite(atas_enb,0);
  analogWrite(bawah_ena,0);  analogWrite(bawah_enb,0);
}
void default_pos(){
  analogWrite(enb,170);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  delay(100);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
void shoot(){
  analogWrite(enb,255);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  delay(100);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
void baca_button(){
  tombol1=digitalRead(button1);
  tombol2=digitalRead(button2);
  tombol3=digitalRead(button3);
}
void sensing(){
  digitalWrite(en, LOW);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  data_sensor[0]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  data_sensor[1]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  data_sensor[2]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  data_sensor[3]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  data_sensor[4]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  data_sensor[5]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  data_sensor[6]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  data_sensor[7]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  data_sensor[8]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  data_sensor[9]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  data_sensor[10]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  data_sensor[11]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
}
void sensing_hor(){
  digitalWrite(en_hor, LOW);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, LOW);
  data_sensor[0]=analogRead(sig_hor);
  digitalWrite(S0_hor, HIGH);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, LOW);
  data_sensor[1]=analogRead(sig_hor);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, HIGH);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, LOW);
  data_sensor[2]=analogRead(sig_hor);
  digitalWrite(S0_hor, HIGH);
  digitalWrite(S1_hor, HIGH);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, LOW);
  data_sensor[3]=analogRead(sig_hor);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, HIGH);
  digitalWrite(S3_hor, LOW);
  data_sensor[4]=analogRead(sig_hor);
  digitalWrite(S0_hor, HIGH);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, HIGH);
  digitalWrite(S3_hor, LOW);
  data_sensor[5]=analogRead(sig_hor);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, HIGH);
  digitalWrite(S2_hor, HIGH);
  digitalWrite(S3_hor, LOW);
  data_sensor[6]=analogRead(sig_hor);
  digitalWrite(S0_hor, HIGH);
  digitalWrite(S1_hor, HIGH);
  digitalWrite(S2_hor, HIGH);
  digitalWrite(S3_hor, LOW);
  data_sensor[7]=analogRead(sig_hor);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, HIGH);
  data_sensor[8]=analogRead(sig_hor);
  digitalWrite(S0_hor, HIGH);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, HIGH);
  data_sensor[9]=analogRead(sig_hor);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, HIGH);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, HIGH);
  data_sensor[10]=analogRead(sig_hor);
  digitalWrite(S0_hor, HIGH);
  digitalWrite(S1_hor, HIGH);
  digitalWrite(S2_hor, LOW);
  digitalWrite(S3_hor, HIGH);
  data_sensor[11]=analogRead(sig_hor);
  digitalWrite(S0_hor, LOW);
  digitalWrite(S1_hor, LOW);
  digitalWrite(S2_hor, HIGH);
  digitalWrite(S3_hor, HIGH);
}
void sensing_kalibrasi(){
  digitalWrite(en, LOW);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  sensor_now[0]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  sensor_now[1]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  sensor_now[2]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  sensor_now[3]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  sensor_now[4]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  sensor_now[5]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  sensor_now[6]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  sensor_now[7]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  sensor_now[8]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  sensor_now[9]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  sensor_now[10]=analogRead(sig);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  sensor_now[11]=analogRead(sig);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
}
void read_sensor(){
  sensing();
  xsensor=63;
  xsensor&=63;
  if(data_sensor[0]<(sens[0]*4)){s0=0; var=1;}       else{s0=1; var=0; flag=1;} xsensor&=~var; 
  if(data_sensor[1]<(sens[1]*4)){s1=0; var=2;}       else{s1=1; var=0;} xsensor&=~var; 
  if(data_sensor[2]<(sens[2]*4)){s2=0; var=4;}       else{s2=1; var=0;} xsensor&=~var;
  if(data_sensor[3]<(sens[3]*4)){s3=0; var=8;}       else{s3=1; var=0;} xsensor&=~var;
  if(data_sensor[4]<(sens[4]*4)){s4=0; var=16;}      else{s4=1; var=0;} xsensor&=~var;
  if(data_sensor[5]<(sens[5]*4)){s5=0; var=32;}      else{s5=1; var=0; flag=0;} xsensor&=~var;
  /*if(data_sensor[6]<(sens[6]*4)){s6=0; var=64;}      else{s6=1; var=0;} xsensor&=~var;
  if(data_sensor[7]<(sens[7]*4)){s7=0; var=128;}     else{s7=1; var=0;} xsensor&=~var;
  if(data_sensor[8]<(sens[8]*4)){s8=0; var=256;}     else{s8=1; var=0;} xsensor&=~var; 
  if(data_sensor[9]<(sens[9]*4)){s9=0; var=512;}     else{s9=1; var=0;} xsensor&=~var; 
  if(data_sensor[10]<(sens[10]*4)){s10=0; var=1024;} else{s10=1;var=0;} xsensor&=~var;
  if(data_sensor[11]<(sens[11]*4)){s11=0; var=2048;} else{s11=1;var=0;} xsensor&=~var;*/
}
void read_sensor_belakang(){
  sensing();
  xsensor=63;
  xsensor&=63;
  if(data_sensor[6]<(sens[6]*4)){s6=0; var=1;}       else{s6=1; var=0; flag=1;} xsensor&=~var; 
  if(data_sensor[7]<(sens[7]*4)){s7=0; var=2;}       else{s7=1; var=0;} xsensor&=~var; 
  if(data_sensor[8]<(sens[8]*4)){s8=0; var=4;}       else{s8=1; var=0;} xsensor&=~var;
  if(data_sensor[9]<(sens[9]*4)){s9=0; var=8;}       else{s9=1; var=0;} xsensor&=~var;
  if(data_sensor[10]<(sens[10]*4)){s10=0; var=16;}   else{s10=1; var=0;} xsensor&=~var;
  if(data_sensor[11]<(sens[11]*4)){s11=0; var=32;}   else{s11=1; var=0; flag=0;} xsensor&=~var;
}
void tampil_all_sensor(){
  read_sensor();
  lcd.setCursor(0,0);
  lcd.print(s0);
  lcd.print(s1);
  lcd.print(s2);
  lcd.print(s3);
  lcd.print(s4);
  lcd.print(s5);
  lcd.print(s11);
  lcd.print(s10);
  lcd.print(s9);
  lcd.print(s8);
  lcd.print(s7);
  lcd.print(s6);
  lcd.setCursor(0,1);
  lcd.print(xsensor);
  lcd.print("   ");
}
void tampil_raw_sensor(){
  lcd.setCursor(0,0);
  lcd.print(sensor_now[6]);
  lcd.setCursor(4,0);
  lcd.print(sensor_now[7]);
  lcd.setCursor(8,0);
  lcd.print(sensor_now[8]);
  lcd.setCursor(0,1);
  lcd.print(sensor_now[9]);
  lcd.setCursor(4,1);
  lcd.print(sensor_now[10]);
  lcd.setCursor(8,1);
  lcd.print(sensor_now[11]);
}
void kalibrasi2(){
  sensing_kalibrasi();
  tampil_raw_sensor();
  for(int i=0;i<=11;i++){
    if(sensor_now[i]>maks[i]){
      maks[i]=sensor_now[i];
    }
    else if(sensor_now[i]<minimal[i]){
      minimal[i]=sensor_now[i];
    }
    ref[i]=(((maks[i]-minimal[i])/2)+minimal[i])/4;
    EEPROM.write(i,ref[i]); 
  }
}
void inverse_pid(unsigned char simpan){
  unsigned int kec_maks=150;
  unsigned int kec_min=90;
  int kp=kons_p;    //14  9.6      13.2
  int kd=kons_d;   //76  18.3     250    230
  int ki=kons_i;    //146.5              //300        1130       1730    750                                                         
  static int error,lastError,Error,LastError,SumError,right_speed,left_speed;
             
  read_sensor();                
  switch(xsensor){
  case 1 : error =  -10; break;
  case 7 : error =  -9; break;
  case 6 : error =  -8; break;
  case 12 : error = -7; break;
  case 24 : error = -6; break;
  case 16 : error = -5; break;
  case 48 : error = -4; break;
  case 32 : error = -3; break;
  case 96 : error = -2; break;
  case 64 : error = -1; break;
  case 192 : error = 0; break;
  case 128 : error =  1; break;
  case 384 : error =  2; break;
  case 256 : error =  3; break;
  case 768 : error =  4; break;
  case 512 : error =  5; break;
  case 1536 : error =  6; break;
  case 3072 : error =  7; break;
  case 6144 : error =  8; break;
  case 14336 : error =  9; break;
  case 8192 : error =  10; break;
  case 0:  
    if(flag==0){error=11;}
    else{error=-11;} 
      break;
    }  
  int SetPoint = 0;                      // Setpoint yang diinginkan
  Error = SetPoint - error; 
  int outPID = kp*0.1*Error + kd*0.1*(Error - lastError) + ki*0.1;
  lastError = Error;
  
  double motorKi = simpan - outPID;     // Motor Kiri
  double motorKa = simpan + outPID;     // Motor Kanan
  
  if (motorKi > kec_maks)motorKi = kec_maks;
  if (motorKi < kec_min)motorKi = kec_min;
  if (motorKa > kec_maks)motorKa = kec_maks;
  if (motorKa < kec_min)motorKa = kec_min;
   
  if(motorKi==motorKa){  
    maju(simpan);
  }
  else if(motorKi>motorKa){
    bel_ka((motorKi+motorKa)/2);
  }
  else if(motorKa>motorKi){
    bel_ki((motorKi+motorKa)/2);
  }
}
void pid(unsigned char simpan){
  unsigned int kec_maks=225;
  unsigned int kec_min=150; //30
  int kp=kons_p;    //7
  int kd=kons_d;    //4
  int ki=kons_i;    //0                                                         
  static int error,lastError,Error,LastError,SumError,right_speed,left_speed;
             
  read_sensor();                
  switch(xsensor){
  case 32 : error = -5; break;
  case 48 : error = -4; break;
  case 56 : error = -3; break;
  case 24 : error = -2; break;
  case 8 : error = -1; break;
  case 12 : error = 0; break;
  case 4 : error =  1; break;
  case 6 : error =  2; break;
  case 7 : error =  3; break;
  case 3 : error =  4; break;
  case 1 : error =  5; break;
  case 0:  
    if(flag==0){error=6;}
    else{error=-6;} 
      break;
    } 
  int SetPoint = 0;                      // Setpoint yang diinginkan
  Error = SetPoint - error; 
  int outPID = kp*0.1*Error + kd*0.1*(Error - lastError) + ki*0.1;
  lastError = Error;
  int laju;
  
  double motorKi = simpan - outPID;     // Motor Kiri
  double motorKa = simpan + outPID;     // Motor Kanan
  
  if (motorKi > kec_maks)motorKi = kec_maks;
  if (motorKi < kec_min)motorKi = kec_min;
  if (motorKa > kec_maks)motorKa = kec_maks;
  if (motorKa < kec_min)motorKa = kec_min;
  tampil_all_sensor(); 
  laju=(motorKi + motorKa)/2;
  if(motorKi==motorKa){  
    maju(simpan);
    lcd.setCursor(12,1);
    lcd.print("F ");
  }
  else if(motorKi>motorKa){
    diagonal_kanan(laju);
    lcd.setCursor(12,1);
    lcd.print("R ");
  }
  else if(motorKa>motorKi){
    diagonal_kiri(laju);
    lcd.setCursor(12,1);
    lcd.print("L ");
  }
}

void pid_timer(int waktu){
  for(i=0;i<=waktu;i++){
    for(j=0;j<=50;j++){
      //pid(kecepatan);
    }
  }
}
void berhenti_hitam(){
  while(1){
    pid(kecepatan);
    if(s0 && s5)break;
  }
  stopped();
}
void line(){
  read_sensor(); 
  if(s2 && s3){
    maju(180);
    lcd.setCursor(12,1);
    lcd.print("F ");
  }
  else if(s2 && s1){
    diagonal_kiri(140);
    lcd.setCursor(12,1);
    lcd.print("L_");
  }
  else if(s2 && s1 && s0){
    diagonal_kiri(165);
    lcd.setCursor(12,1);
    lcd.print("L_");
  }
  else if(s0 && s1){
    bel_ki(140);
    lcd.setCursor(12,1);
    lcd.print("L ");
  }
  else if(s0){
    bel_ki(184);
  }
  else if(s3 && s4){
    diagonal_kanan(140);
  }
  else if(s3 && s4 && s5){
    diagonal_kanan(165);
  }
  else if(s4 && s5){
    bel_ka(130);
  }
  else if(s5){
    bel_ka(140);
  }
}
void finished(){
  while(1){
    //pid(kecepatan);
    if(s4 || s5 || s6 || s7 || s8 || s9);
    break;
  }
  while(1){
    stopped();   
  }
}
void arm_up(){
  pos1=pos1+3;
  if(pos1>180){pos1=180;}
  dof1.write(pos1);
  //delay(5);
}
void arm_down(){
  pos1=pos1-3;
  if(pos1<0){pos1=0;}
  dof1.write(pos1);
  //delay(5);
}
void grip_up(){
  pos2=pos2+3;
  if(pos2>180){pos2=180;}
  dof2.write(pos2);
  //delay(5);
}
void grip_down(){
  pos2=pos2-3;
  if(pos2<0){pos2=0;}
  dof2.write(pos2);
  //delay(5);
}
void grip_open(){
  grip.write(0);
}
void grip_close(){
  grip.write(50); //
}
void ambil_objek1(){
  dof2.write(0); 
  dof1.write(165);
  delay(1500);
  grip_close();
  delay(500);
}
void ambil_objek2(){
  dof2.write(0); 
  dof1.write(150);
  delay(1000);
  grip_close();
  delay(500);
}
void ambil_objek3(){
  dof2.write(0); 
  dof1.write(120);
  delay(1000);
  grip_close();
  delay(500);
}
void taruh_objek1(){
  dof2.write(0); 
  dof1.write(180);
  delay(1000);
  grip_open();
}
void taruh_objek2(){
  dof2.write(0); 
  dof1.write(150);
  delay(1000);
  grip_open();
}
void taruh_objek3(){
  dof2.write(0); 
  dof1.write(120);
  delay(1000);
  grip_open();
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  lcd.begin();
  pinMode(en,OUTPUT);
  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);
  pinMode (atas_in1, OUTPUT);
  pinMode (atas_in2, OUTPUT);
  pinMode (atas_in3, OUTPUT);
  pinMode (atas_in4, OUTPUT);
  pinMode (atas_ena, OUTPUT);
  pinMode (atas_enb, OUTPUT);
  pinMode (bawah_in1, OUTPUT);
  pinMode (bawah_in2, OUTPUT);
  pinMode (bawah_in3, OUTPUT);
  pinMode (bawah_in4, OUTPUT);
  pinMode (bawah_ena, OUTPUT);
  pinMode (bawah_enb, OUTPUT);
  pinMode (in3, OUTPUT);
  pinMode (in4, OUTPUT);
  pinMode (enb, OUTPUT);
  //pinMode (selenoid, OUTPUT);
  pinMode (button1, INPUT_PULLUP); 
  pinMode (button2, INPUT_PULLUP);
  pinMode (button3, INPUT_PULLUP);
  //digitalWrite(selenoid,LOW);
  //digitalWrite(selenoid2,LOW);
  //grab.attach(9);
  dof1.attach(6); //makin kecil makin kebawah, makin besar makin keatas
  dof2.attach(5); //makin kecil makin kebawah, makin besar makin keatas
  grip.attach(4);
  grip_close();
  dof2.write(pos2); //90
  dof1.write(pos1); //0 kebawah
  //grab.write(30);
  delay(1000);
  dof1.detach();
  grab.detach();
}

void loop(){
  awal:
  lcd.setCursor(0,0);
  lcd.print("1.Setting ");
  lcd.setCursor(0,1);
  lcd.print("2.Sensor "); 
  lcd.setCursor(10,0);
  lcd.print("3.Play");
  baca_button();
  while(tombol1 && tombol2 && tombol3){baca_button();}//tampil_sensor();} 
  if(!tombol1){
    delay(200);
    lcd.clear();
    delay(200);
    while(!tombol1){
      baca_button();
      lcd.setCursor(0,0);
      lcd.print("1.Kalib ");
      lcd.setCursor(0,1);
      lcd.print("2.Speed ");
      lcd.setCursor(10,0);
      lcd.print("3.PID");
    }
    while(1){
      while(tombol1 && tombol2 && tombol3){baca_button();}
      if(!tombol1){
        lcd.clear();
        delay(200);
        while(!tombol1){
          delay(50);
          while(1){
            baca_button();
            kalibrasi2();
            if(!tombol1){
              delay(200);
              lcd.clear();
              goto awal;
            }
          } 
        } 
      }
      else if(!tombol2){
        delay(100);
        kecepatan=EEPROM.read(address_speed);
        laju=kecepatan;
        while(!tombol2){baca_button();}
        lcd.clear();
        while(1){
          lcd.setCursor(0,0);
          lcd.print("Speed:");
          lcd.print(kecepatan);
          lcd.print("  ");
          baca_button();
          if(!tombol1){
            delay(200);
            laju++;
            if(laju>255){laju=0;}
            lcd.print(laju);
            lcd.print("  ");
          }
          else if(!tombol2){
            delay(200);
            laju--;
            if(laju<0){laju=255;}
            lcd.print(laju);
            lcd.print("  ");
          }
          else if(!tombol3){
            delay(200);
            lcd.clear();
            EEPROM.write(address_speed,laju);
            delay(500);
            lcd.setCursor(0,0);
            lcd.print("OK");
            delay(500);
            lcd.clear();
            goto awal;
          }
        }  
      }
      else if(!tombol3){
        delay(100);
        lcd.clear();
        delay(200);
        kons_p=EEPROM.read(address_kons_p);
        kons_d=EEPROM.read(address_kons_d);
        kons_i=EEPROM.read(address_kons_i);
        pi=kons_p;
        di=kons_d;
        ai=kons_i;
        lcd.setCursor(0,0);
        lcd.print("Kp:");
        lcd.print(pi);  
        lcd.setCursor(0,1);
        lcd.print("Kd:");
        lcd.print(di); 
        lcd.setCursor(8,0);
        lcd.print("Ki:");
        lcd.print(ai);  
        while(!tombol1){baca_button();}
        while(1){
          baca_button();
          if(!tombol1 && indeks==0){
            delay(200);
            pi++;
            if(pi>255){pi=0;}
            lcd.setCursor(3,0);
            lcd.print(pi);
            lcd.print("  ");
          }
          else if(!tombol2 && indeks==0){
            delay(200);
            pi--;
            if(pi<0){pi=255;}
            lcd.setCursor(3,0);
            lcd.print(pi);
            lcd.print("  ");
          }
          else if(!tombol3 && indeks==0){
            delay(200);
            EEPROM.write(address_kons_p,pi);
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("OK");
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("  ");
            indeks=1;
            delay(500);
          }
          while(indeks==1){
          baca_button();
          if(!tombol1 && indeks==1){
            delay(200);
            di++;
            if(di>255){di=0;}
            lcd.setCursor(3,1);
            lcd.print(di);
            lcd.print("  ");
          }
          else if(!tombol2 && indeks==1){
            delay(200);
            di--;
            if(di<0){di=255;}
            lcd.setCursor(3,1);
            lcd.print(di);
            lcd.print("  ");
          }
          else if(!tombol3 && indeks==1){
            delay(200);
            EEPROM.write(address_kons_d,di);
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("OK");
            indeks=2;
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("OK");
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("  ");
          }
          }
          while(indeks==2){
          baca_button();
          if(!tombol1 && indeks==2){
            delay(200);
            ai++;
            if(ai>255){ai=0;}
            lcd.setCursor(11,0);
            lcd.print(ai);
            lcd.print("  ");
          }
          else if(!tombol2 && indeks==2){
            delay(200);
            ai--;
            if(ai<0){ai=255;}
            lcd.setCursor(11,0);
            lcd.print(ai);
            lcd.print("  ");
          }
          else if(!tombol3 && indeks==2){
            delay(200);
            EEPROM.write(address_kons_i,ai);
            delay(500);
            indeks=0;
            lcd.setCursor(8,1);
            lcd.print("OK");
            delay(500);
            lcd.setCursor(8,1);
            lcd.print("  ");
            delay(1000);
            lcd.clear();
            goto awal;
          }
          }
        }
      }
    }
  }
  else if(!tombol2){
    delay(100);
    lcd.clear();
    kons_p=EEPROM.read(address_kons_p);
    kons_d=EEPROM.read(address_kons_d);
    kons_i=EEPROM.read(address_kons_i);
    kecepatan=EEPROM.read(address_speed);
    sens[0]=EEPROM.read(0);
    sens[1]=EEPROM.read(1);
    sens[2]=EEPROM.read(2);
    sens[3]=EEPROM.read(3);
    sens[4]=EEPROM.read(4);
    sens[5]=EEPROM.read(5);
    sens[6]=EEPROM.read(6);
    sens[7]=EEPROM.read(7);
    sens[8]=EEPROM.read(8);
    sens[9]=EEPROM.read(9);
    sens[10]=EEPROM.read(10);
    sens[11]=EEPROM.read(11);
    sens[12]=EEPROM.read(12);
    sens[13]=EEPROM.read(13);
    while(1){
      tampil_all_sensor();
    }  
  }
  else if(!tombol3){
    delay(100);
    kons_p=EEPROM.read(address_kons_p);
    kons_d=EEPROM.read(address_kons_d);
    kons_i=EEPROM.read(address_kons_i);
    kecepatan=EEPROM.read(address_speed);
    sens[0]=EEPROM.read(0);
    sens[1]=EEPROM.read(1);
    sens[2]=EEPROM.read(2);
    sens[3]=EEPROM.read(3);
    sens[4]=EEPROM.read(4);
    sens[5]=EEPROM.read(5);
    sens[6]=EEPROM.read(6);
    sens[7]=EEPROM.read(7);
    sens[8]=EEPROM.read(8);
    sens[9]=EEPROM.read(9);
    sens[10]=EEPROM.read(10);
    sens[11]=EEPROM.read(11);
    sens[12]=EEPROM.read(12);
    sens[13]=EEPROM.read(13);
    lcd.clear();
    delay(100);
    while(!tombol3){
      baca_button();
      lcd.setCursor(0,0);
      lcd.print("1.Start ");
      lcd.setCursor(0,1);
      lcd.print("2.Retry ");  
      while(tombol1 && tombol2){baca_button();}
      if(!tombol1){
        lcd.clear();
        delay(200);
        while(!tombol1){
          baca_button();
          lcd.setCursor(0,0);
          lcd.print("1.Start1 ");
          lcd.setCursor(0,1);
          lcd.print("2.Start2 "); 
          while(tombol1 && tombol2){baca_button();}
          lom:
          if(!tombol1){   //program untuk start1
            lcd.clear();
            delay(200);
            while(!tombol1){
              //shoot();
              //goto lom;
              pid(kecepatan);
              /*berhenti_hitam();
              ambil_objek1();
              dof1.write(60);
              maju(130);
              delay(500);
              berhenti_hitam();
              taruh_objek1();
              dof1.write(60);
              while(1){
                stopped();
              }*/
            }
          }
          else if(!tombol2){
            lcd.clear();
            delay(200);
            while(!tombol2){  //program untuk start2
              if(mySerial.available()){
                delay(25);
                while(mySerial.available()){
                  char c = mySerial.read();
                  //dof1.detach();
                  dof2.detach();
                  //grip.detach();
                  if(c=='>'){
                    speed_ += 1;
                    if(speed_ > 4){speed_ = 4;}
                    if(speed_==1){
                      kec=150;  
                    }
                    else if(speed_==2){
                      kec=190;  
                    }
                    else if(speed_==3){
                      kec=220;  
                    }
                    else if(speed_==4){
                      kec=255;  
                    }       
          
                    Serial.print("Speed: ");
                    Serial.println(kec);
                  }
                  else if(c=='<'){
                    speed_ -= 1;
                    if(speed_ < 1){speed_ = 1;}
                    if(speed_==1){
                      kec=150;  
                    }
                    else if(speed_==2){
                      kec=190;  
                    }
                    else if(speed_==3){
                      kec=220;  
                    }
                    else if(speed_==4){
                      kec=255;  
                    }
          
                    Serial.print("Speed: ");
                    Serial.println(kec);
                  }
                  else if(c=='^'){
                    c = mySerial.read();
                    kecepatan=kec;
                    if(c=='1'){                      
                      if(mySerial.available()){
                        c=mySerial.read();
                        if(c=='0'){ 
                          Serial.println("==> arm turun");
                          dof1.attach(6);
                          arm_down();
                          lcd.setCursor(0,0);
                          lcd.print("arm turun ");
                        }
                        else if(c=='1'){ 
                          Serial.println("==> arm naik");
                          dof1.attach(6);
                          arm_up();
                          lcd.setCursor(0,0);
                          lcd.print("arm naik");
                        }                        
                      }
                      else{
                        Serial.print("==> maju :");
                        Serial.println(kecepatan);
                        maju(kecepatan);
                      }
                    }
                    else if(c=='2'){
                      Serial.print("==> mundur :");
                      Serial.println(kecepatan);
                      mundur(kecepatan);
                    }
                    else if(c=='3'){
                      Serial.print("==> kanan :");
                      Serial.println(kecepatan);
                      bel_ka(kecepatan);
                    }
                    else if(c=='4'){   
                      Serial.print("==> kiri :");
                      Serial.println(kecepatan);
                      bel_ki(kecepatan);
                    }
                    else if(c=='5'){ 
                      Serial.println("==> berhenti");
                      kecepatan=0;
                      stopped();
                    }
                    else if(c=='6'){ 
                      Serial.print("==> geser kanan :");
                      Serial.println(kecepatan);
                      geser_kanan(kecepatan);
                    }
                    else if(c=='7'){ 
                      Serial.print("==> geser kiri :");
                      Serial.println(kecepatan);
                      geser_kiri(kecepatan);
                    }
                    else if(c=='8'){ 
                      Serial.println("==> griper naik");
                      dof2.attach(5);
                      grip_up();
                      lcd.setCursor(0,0);
                      lcd.print("up    ");
                    }
                    else if(c=='9'){ 
                      Serial.println("==> griper turun");
                      dof2.attach(5);
                      grip_down();
                      lcd.setCursor(0,0);
                      lcd.print("down   ");
                    }                    
                  }
                  else if(c=='('){
                    Serial.println("==> griper buka");
                    grip.attach(4);
                    grip_open();
                    lcd.setCursor(0,0);
                    lcd.print("open   ");
                  }
                  else if(c==')'){
                    Serial.println("==> griper tutup");
                    grip.attach(4);
                    grip_close();
                    lcd.setCursor(0,0);
                    lcd.print("close  ");
                  }
                  else if(c=='!'){
                    Serial.println("==> grab");
                    default_pos();
                  }
                  else if(c=='$'){
                    Serial.println("==> shoot");
                    shoot();
                  }
                }
              } 
            }
          }
        }
      }
      else if(!tombol2){
        lcd.clear();
        delay(200);
        while(!tombol2){
          baca_button();
          lcd.setCursor(0,0);
          lcd.print("1.Retry1 ");
          lcd.setCursor(0,1);
          lcd.print("2.Retry2 "); 
          while(tombol1 && tombol2){baca_button();}  
          if(!tombol1){
            lcd.clear();
            delay(200);
            while(!tombol1){
              baca_button();
              lcd.setCursor(0,0);
              lcd.print("1.CP1 ");
              lcd.setCursor(0,1);
              lcd.print("2.CP2 "); 
              lcd.setCursor(8,0);
              lcd.print("3.CP3 ");
              while(tombol1 && tombol2 && tombol3){baca_button();}  
              if(!tombol1){     //program untuk cek poin 1 start 1
                lcd.clear();
                delay(200);
                while(!tombol1){              
                  line(); 
                  
                }   
              }
              else if(!tombol2){  //program untuk cek poin 2 start 1
                lcd.clear();
                delay(200);
                while(!tombol2){  
                  
                }     
              }
              else if(!tombol3){   //program untuk cek poin 3 start 1
                lcd.clear();
                delay(200);
                while(!tombol3){  
                    
                }     
              }
            }
          }
          else if(!tombol2){
            lcd.clear();
            delay(200);
            while(!tombol2){
              baca_button();
              lcd.setCursor(0,0);
              lcd.print("1.linede");
              lcd.setCursor(0,1);
              lcd.print("2.linebe"); 
              lcd.setCursor(9,0);
              lcd.print("3.shoot");
              while(tombol1 && tombol2 && tombol3){baca_button();}  
              if(!tombol1){     //program untuk cek poin 1 start 2
                delay(100);
                lcd.clear();
                while(1){ 
                  sensing();
                  lcd.setCursor(0,0);
                  lcd.print(data_sensor[0]);
                  lcd.setCursor(4,0);
                  lcd.print(data_sensor[1]);
                  lcd.setCursor(8,0);
                  lcd.print(data_sensor[2]);
                  lcd.setCursor(0,1);
                  lcd.print(data_sensor[3]);
                  lcd.setCursor(4,1);
                  lcd.print(data_sensor[4]);
                  lcd.setCursor(8,1);
                  lcd.print(data_sensor[5]); 
                }
              }
              else if(!tombol2){     //program untuk cek poin 2 start 2
                delay(100);
                lcd.clear();
                while(1){
                  sensing();
                  lcd.setCursor(0,0);
                  lcd.print(data_sensor[6]);
                  lcd.setCursor(4,0);
                  lcd.print(data_sensor[7]);
                  lcd.setCursor(8,0);
                  lcd.print(data_sensor[8]);
                  lcd.setCursor(0,1);
                  lcd.print(data_sensor[9]);
                  lcd.setCursor(4,1);
                  lcd.print(data_sensor[10]);
                  lcd.setCursor(8,1);
                  lcd.print(data_sensor[11]); 
                }
              }
              else if(!tombol3){     //program untuk cek poin 3 start 2
                delay(100);
                lcd.clear();
                while(1){
                  geser_kanan(250);
                  delay(1000);
                  geser_kiri(250);
                  delay(1000);
                }
              }
            }
          }
        }
      }
    }
  }
}
