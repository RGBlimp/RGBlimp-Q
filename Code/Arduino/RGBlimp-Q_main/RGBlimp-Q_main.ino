// RGBlimp-Q Main Controller
// Last Edit: 2024-6-19

#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <string.h>
#include <math.h>
#include <EEPROM.h>

#define MIN(i, j) (((i) < (j)) ? (i) : (j))
#define MAX(i, j) (((i) > (j)) ? (i) : (j))

#define LED_BUILTIN 2
#define THRUST 27
#define GDLX_A 32
#define GDLX_B 33
#define GDLY_A 22
#define GDLY_B 23
#define IMUA_T 13
#define IMUA_R 14
#define IMUB_T 16
#define IMUB_R 17
#define GRIPPER 25

#define THRUST_MAX 300

#define GD_CLS  0x0
#define GD_X    0x1
#define GD_Y    0x2
#define GD_XY   0x3

#define FB_GD   0x0  // Gondola Distance Feedback Control
#define FB_SF   0x1  // Straight Feedback Flight     (Yaw Adjustment)
#define FB_ES   0x2  // Eular Spiral Flight (Following a Ramp Signal)
#define FB_SQ   0x3  // Square End Motion Trajectory of the Continuum Arm   (Following a piecewise function composed of different trigonometric functions)
#define FB_TR   0x4  // Triangular End Motion Trajectory of the Continuum Arm   (Following a piecewise function composed of different trigonometric functions)



// IMU
unsigned char buffer[40];
sensor_msgs::Imu IMU;
geometry_msgs::Quaternion IMUA,IMUB,q;
float psi = 0.0, psi_ = 0.0; // Yaw & Ratio of Body Frame 
float phiA=0,thetaA=0,phiB=0,thetaB=0;

// Parameters
std_msgs::UInt64 params; // params = deltaX + deltaY*2^8

// Calibration
float deltaX = 0.0, deltaY = 0.0;

// Feedback Control
ushort isGD = 0;  // Gondola Distance Mode  (GD_CLS,GD_X,GD_Y,GD_XY)
ushort fbCN = 0;  // Gondola Feedback Control Mode Number
struct pid{
  float target;
  float measure;
  float err;
  float err_last;
  float Kp,Ki,Kd;
  float integral;
} Gx,Gy,sf; 
float stMax = 0.0;
float slope = 0.0;  // [mm/s] The slope of target function for Eular Spiral (fbCN=2)
float AxLemn=0.0, AyLemn=0.0, taLemn = 0.0, tbLemn = 0.0;  // [mm,ms]  The amplitude and period time for Lemniscate (fbCN=3)
bool loopLemn=0;
uint32_t startTime = 0, cntLemn = 0;


// Wi-Fi
const char* ssid     = "RGBlimp";
const char* password = "robot2021";

// ROS SETUP
IPAddress server(192,168,0,101); //ros server IP ros master
const uint16_t serverPort = 11411; 
ros::NodeHandle nh;


uint16_t crc16(unsigned char *addr, int num, uint16_t crc){
 ushort i;
  for (; num > 0; num--){           /* Step through bytes in memory */
    crc = crc ^ (*addr++ << 8);     /* Fetch byte from memory, XOR into CRC top byte*/
    for (i = 0; i < 8; i++){        /* Prepare to rotate 8 bits */
     if (crc & 0x8000)              /* b15 is set... */
      crc = (crc << 1) ^ 0x1021;    /* rotate and XOR with polynomic */
     else                           /* b15 is clear... */
      crc <<= 1;                    /* just rotate */
    }                               /* Loop for 8 bits */
    crc &= 0xFFFF;                  /* Ensure CRC remains 16-bit value */
  }                                 /* Loop until num=0 */
  return(crc);                      /* Return updated CRC */
}


ros::Subscriber<std_msgs::UInt64> sub("/command", &control);
ros::Publisher imu("IMU",&IMU);
ros::Publisher imua("IMUA",&IMUA);
ros::Publisher imub("IMUB",&IMUB);
ros::Publisher param("param",&params);
// CONTROL FUNCTION  "/command"
void control(const std_msgs::UInt64 &msg){  
  // msg.data = A + B*2^10 + C*2^20 + D*2^30 + E*2^40 + F*2^50 + SIGN*2^60
  uint16_t A =  msg.data%1024;      // in [0,2^10) = [0,1024)
  uint16_t B = (msg.data>>10)%1024; // in [0,2^10) = [0,1024)
  uint16_t C = (msg.data>>20)%1024; // in [0,2^10) = [0,1024)
  uint16_t D = (msg.data>>30)%1024; // in [0,2^10) = [0,1024)
  uint16_t E = (msg.data>>40)%1024; // in [0,2^10) = [0,1024)
  uint16_t F = (msg.data>>50)%1024; // in [0,2^10) = [0,1024)
  uint16_t S =  msg.data>>60;       // in [0,2^4)  = [0,16)
  if(S==0){thrust(0);              // [0] Emergency Stop
           gondola(500,500);
           fbCN=FB_GD;isGD=GD_CLS;}
  if(S==1) thrust(A);               // [1] Thrust             (A)      A>>[0,500]
  if(S==2) gripper(A);              // [2] Gripper            (A)      A>>[0,90] deg
  if(S==3){fbCN=FB_GD;isGD=GD_CLS;  // [3] Gondola Throttle   (A,B)    A,B>>[0,1000]->[-500,500]
           gondola(A,B);}
  if(S==4) fbCtrl(A,B,C,D,E,F);     // [4] Gondola Feedback   (A,B,C,D,E,F)   F:(0 FB_GD, 1 FB_SF, 2 FB_ES, 3 FB_SQ)
  if(S==9){if(!A)calib();           // [9] Calibrate the deviation of the Gondola origin
           else{EEPROM.write(0, uint8_t(A));  // [0,255]->[0,240]->[-120,120]->[-3.0:0.025:3.0] uint8_t(()*40 + 120)  [-1.35,2.00]
                EEPROM.write(1, uint8_t(B));
                EEPROM.commit();
                deltaX = (float(EEPROM.read(0))-120)/40;
                deltaY = (float(EEPROM.read(1))-120)/40;
                digitalWrite(LED_BUILTIN,LOW);delay(200); 
                digitalWrite(LED_BUILTIN,HIGH);}}
  if(S==10)param.publish(&params);
}


void calib(){
  deltaX = 0;
  deltaY = 0;
  for(int i=0; i<1000; i++,delay(5)){
    gondola( thetaB*(800)+500, phiB*(-800)+500 );
    //Serial.printf("[thetaB,phiB]=[%.3f,%.3f]\n",thetaB,phiB);
  }
  gondola( 500, 500);
  EEPROM.write(0, uint8_t((MAX(-2.0,MIN(2.0, Gx.measure )))*40 + 120) );  // [0,255]->[0,240]->[-120,120]->[-3.0:0.025:3.0]
  EEPROM.write(1, uint8_t((MAX(-2.0,MIN(2.0, Gy.measure )))*40 + 120) );
  EEPROM.commit();
  deltaX = (float(EEPROM.read(0))-120)/40;
  deltaY = (float(EEPROM.read(1))-120)/40;
  //Serial.printf("Calib.[x,y]=[%.3f,%.3f],   G.[x,y]=[%.3f,%.3f]\n",deltaX,deltaY,(MAX(-2.0,MIN(2.0, Gx.measure )))*40 + 120,(MAX(-2.0,MIN(2.0, Gy.measure )))*40 + 120);
  params.data = EEPROM.read(0) + EEPROM.read(1)*256;
  digitalWrite(LED_BUILTIN,LOW);
  delay(200); 
  digitalWrite(LED_BUILTIN,HIGH);
}

// THRUST  (A)    A>>[0,500]
void thrust(uint16_t T){
  T = T>THRUST_MAX?THRUST_MAX:T; 
  ledcWrite(0, 520+T); 
}

// GONDOLA Throttle (A,B)  A,B>>[0,1000]  ->  [-500,500]
void gondola(uint16_t x, uint16_t y){
  int xx = (x>1000?1000:x) - 500; 
  int yy = (y>1000?1000:y) - 500; 
  // x-o-z Plane
  if (xx>0){
    ledcWrite(8, 0);
    ledcWrite(9,  int(xx*2.046));    // MX FORWARD
    }
  else {
    ledcWrite(8, -int(xx*2.046));    // MX BACKWARD
    ledcWrite(9, 0);
    }
  // y-o-z Plane
  if (yy>0){
    ledcWrite(10, 0);
    ledcWrite(11,  int(yy*2.046));   // MY FORWARD
    }
  else {
    ledcWrite(10, -int(yy*2.046));   // MY BACKWARD
    ledcWrite(11, 0);
    }
}

// GRIPPER  (A)    A>>[0,90]
void gripper(uint16_t d){
  d = d>90?90:d; 
  ledcWrite(12, 51+int(d*0.56666)); 
}

// Feedback Control
void fbCtrl(uint16_t A, uint16_t B, uint16_t C, uint16_t D, uint16_t E, uint16_t F){
  if (F==0){ // Gondola Distance(xy) Feedback Control
    Gx.integral=0.0, Gy.integral=0.0;
    Gx.target=((A-500.0)/8.0);  // A [0,1000] -> [-62.5,62.5] [mm]
    Gy.target=((B-500.0)/8.0);  // B [0,1000] -> [-62.5,62.5] [mm]
    Gx.Kp=C?C:50.0;             // C [0,1000]
    Gy.Kp=C?C:50.0;         
    Gx.Ki=D?(D/10):0.10;        // D [0,1000] -> [0.0,100.0]
    Gy.Ki=D?(D/10):0.10;
    Gx.Kd=E?(E/10):0.10;        // E [0,1000] -> [0.0,100.0]
    Gy.Kd=E?(E/10):0.10;
    fbCN = FB_GD; 
    isGD = GD_XY; 
  }
  if (F==1){ // Straight Flight Feedback Control
    // Set the yaw angle in this time as straight flight direction
    sf.target=psi;              // [deg]
    sf.Kp=A;//?A:50;            // A [0,1000]
    sf.Kd=B;//?(B/10):0.10;     // B [0,1000] -> [0.0,100.0]
    stMax=C?MIN(45,C):20;       // C [0,40]
    fbCN = FB_SF; 
    isGD = GD_Y; 
  }
  if (F==2){ // Gy.target = A*t + B
    // Following a ramp signal
    Gy.integral=0.0;
    Gy.target=B;                // B [0,1000]
    slope=(A-500)/8.0;          // A [0,1000] -> [-62.5,62.5] [mm]
    fbCN = FB_ES; 
    isGD = GD_Y; 
  }
  if (F==3){ // Square End Motion Trajectory
    AxLemn=A;              //[mm] A [0,1000]
    AyLemn=B;              //[mm] B [0,1000] 
    taLemn=C?C*100:10000;       //[ms] C [0,1000] -> [0.0,100.0] [s] -> [0.0, 100000] [ms]
    tbLemn=D?D*100:10000;       //[ms] D [0,1000] -> [0.0,100.0] [s] -> [0.0, 100000] [ms]
    loopLemn=E?1:0;
    startTime = esp_log_timestamp();
    cntLemn = 0;
    fbCN = FB_SQ;
    isGD = (AxLemn<1)?GD_Y:GD_XY;
  }
  if (F==4){ // Triangular End Motion Trajectory 
    AxLemn=A;              //[mm] A [0,1000]
    AyLemn=B;              //[mm] B [0,1000] 
    taLemn=C?C*100:10000;       //[ms] C [0,1000] -> [0.0,100.0] [s] -> [0.0, 100000] [ms]
    tbLemn=D?D*100:10000;       //[ms] D [0,1000] -> [0.0,100.0] [s] -> [0.0, 100000] [ms]
    loopLemn=E?1:0;
    startTime = esp_log_timestamp();
    cntLemn = 0;
    fbCN = FB_TR;
    isGD = (AxLemn<1)?GD_Y:GD_XY;
  }
}

void setup() { 
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Close the low-voltage detecter to avoid reboot
  xTaskCreatePinnedToCore(imuUpdate_, "imuUpdate", 10000, NULL, 1, NULL,  1);  //  IMU Update Process   PRO_CPU -- 0 | APP_CPU -- 1
  xTaskCreatePinnedToCore(   fbCtrl_,    "fbCtrl", 10000, NULL, 1, NULL,  1);  //  fbCtrl Process
  
  // Debug
  Serial.begin(115200); 

  // Calibration
  EEPROM.begin(2);
  deltaX = (float(EEPROM.read(0))-120)/40;
  deltaY = (float(EEPROM.read(1))-120)/40;
  Serial.printf("Calib.[x,y]=[%.3f,%.3f]mm\n",deltaX,deltaY);
  params.data = EEPROM.read(0) + EEPROM.read(1)*256;

  // Initialize Thrust PWM
  // freq   1000/125 = 8ms
  // rang   [0,4095] -> [0,8ms]
  // [1ms,2ms]  ->  [4096*1/8,4096*2/8]=[512,1024] -> [520,1020]
  ledcSetup(0, 125, 12); // channel, freq, resolution 
  ledcAttachPin(THRUST, 0); // Pin, channel
  ledcWrite(0, 0); 

  // Initialize Gondola PWM
  ledcSetup(8, 50, 10); // channel, freq, resolution
  ledcSetup(9, 50, 10); // channel, freq, resolution
  ledcSetup(10, 50, 10); // channel, freq, resolution
  ledcSetup(11, 50, 10); // channel, freq, resolution
  ledcAttachPin(GDLX_A, 8); // Pin, channel
  ledcAttachPin(GDLX_B, 9); // Pin, channel
  ledcAttachPin(GDLY_A, 10); // Pin, channel
  ledcAttachPin(GDLY_B, 11); // Pin, channel
  ledcWrite(8, 0); 
  ledcWrite(9, 0);
  ledcWrite(10, 0);
  ledcWrite(11, 0);

  // Initialize Gripper PWM
  // [1ms,2ms]  ->  [1024*1/20,1024*2/20]=[51.2,102.4] -> [51,102]
  ledcSetup(12, 50, 10); // channel, freq, resolution 
  ledcAttachPin(GRIPPER, 12); // Pin, channel
  ledcWrite(12, 51); 

  
  // Initialize the LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW); // Close the LED
  // WiFi Connecting
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(300); digitalWrite(LED_BUILTIN,HIGH);delay(200); digitalWrite(LED_BUILTIN,LOW);}
  digitalWrite(LED_BUILTIN,HIGH); // Open the LED when WiFi connected
  // Rosserial Socket Connecting
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub); // Ctrl
  nh.advertise(imu); // IMU full data of Based Frame
  nh.advertise(imua); // IMUA Tool  Frame
  nh.advertise(imub); // IMUB Based Frame
  nh.advertise(param); // Param

  // Initialize IMUs
  Serial1.begin(115200, SERIAL_8N1, IMUA_T, IMUA_R);
  Serial2.begin(115200, SERIAL_8N1, IMUB_T, IMUB_R);
  Serial1.write("AT+SETPTL=D1\r\n", 14);
  Serial2.write("AT+SETPTL=A0,B0,D1\r\n", 20);
  Serial1.write("AT+MODE=0\r\n", 11);  // 0 -- 6 axis; 1 -- 9 axis
  Serial2.write("AT+MODE=0\r\n", 11);  // 
  
  Serial1.write("AT+ODR=50\r\n", 11);  // 50 Hz
  Serial2.write("AT+ODR=50\r\n", 11);  // 50 Hz

  q.w = 1; q.x = 0; q.y = 0; q.z = 0;
  IMUA = q; IMUB = q;
  
  
  // Initialize Thruster BLDC
  /*ledcWrite(0, 1020); 
  delay(3000);*/
  ledcWrite(0, 520); 
}

void loop(){
  nh.spinOnce();
  delay(5);
}


// IMU UPDATE
void imuUpdate_(void *pvParameters) {
  uint16_t crc = 0;
  geometry_msgs::Quaternion tmp, tmpA, tmpB;
  while (1) {
    vTaskDelay(10);
    // IMUB -- Body Frame
    if (Serial2.available() > 0) {
      crc = 0;
      Serial2.readBytes(buffer, 37);
      while (Serial2.read() >= 0);  // Clear the buffer
      if ((buffer[0] << 8) + buffer[1] == 0x5AA5) {
        crc = crc16(buffer, 4, crc);
        crc = crc16(buffer + 6, buffer[2] + (buffer[3] << 8), crc);
        if (crc == (buffer[5] << 8) + buffer[4]) {
          // BUFFER: 
          //   [6]  A0, [7-8]   X, [9-10]  Y, [11-12] Z, 
          //   [13] B0, [14-15] X, [16-17] Y, [18-19] Z, 
          //   [20] D1, [21-24] w, [25,28] x, [29,32] y, [33,36] z
          int16_t tmpimu;
          memcpy(&tmpimu, &buffer[7], sizeof(int16_t));
          IMU.linear_acceleration.y = float(tmpimu)/1000; // [m/s^2]
          memcpy(&tmpimu, &buffer[9], sizeof(int16_t));
          IMU.linear_acceleration.x =-float(tmpimu)/1000; // [m/s^2]
          memcpy(&tmpimu, &buffer[11], sizeof(int16_t));
          IMU.linear_acceleration.z =-float(tmpimu)/1000; // [m/s^2]
          memcpy(&tmpimu, &buffer[14], sizeof(int16_t));
          IMU.angular_velocity.y =-float(tmpimu)/10;      // [deg/s]
          memcpy(&tmpimu, &buffer[16], sizeof(int16_t));
          IMU.angular_velocity.x = float(tmpimu)/10;      // [deg/s]
          memcpy(&tmpimu, &buffer[18], sizeof(int16_t));
          IMU.angular_velocity.z = float(tmpimu)/10;      // [deg/s]
          memcpy(&tmp.w, &buffer[21], sizeof(float));
          memcpy(&tmp.x, &buffer[25], sizeof(float));
          memcpy(&tmp.y, &buffer[29], sizeof(float));
          memcpy(&tmp.z, &buffer[33], sizeof(float));
          IMUB.w = 1 / sqrt(2) * (tmp.x + tmp.y);
          IMUB.x = 1 / sqrt(2) * (tmp.z - tmp.w);
          IMUB.y = 1 / sqrt(2) * (tmp.z + tmp.w);
          IMUB.z = 1 / sqrt(2) * (tmp.x - tmp.y);
          psi  =  atan2(2*(IMUB.w*IMUB.z+IMUB.x*IMUB.y), 1-2*(IMUB.y*IMUB.y+IMUB.z*IMUB.z))*180/3.14; // deg
          psi_ =-IMU.angular_velocity.z;  // [deg/s]  note: following the legacy code
          //Serial.printf("IMUB.[w,x,y,z]=[%.3f,%.3f,%.3f,%.3f], [psi,psi_]=[%.3f,%.3f]\n",IMUB.w,IMUB.x,IMUB.y,IMUB.z,psi,psi_);
          IMU.header.stamp = nh.now();
          IMU.orientation = IMUB;
          imub.publish(&IMUB);
          imu.publish(&IMU);
        }
      }
    }

    // IMUA & q -- Tool Frame
    if (Serial1.available() > 0) {
      crc = 0;
      Serial1.readBytes(buffer, 23);
      while (Serial1.read() >= 0);  // Clear the buffer
      if ((buffer[0] << 8) + buffer[1] == 0x5AA5) {
        crc = crc16(buffer, 4, crc);
        crc = crc16(buffer + 6, buffer[2] + (buffer[3] << 8), crc);
        if (crc == (buffer[5] << 8) + buffer[4]) {
          // BUFFER: [6] D1, [7-10] w, [11,14] x, [15,18] y, [19,22] z
          memcpy(&tmp.w, &buffer[7], sizeof(float));
          memcpy(&tmp.x, &buffer[11], sizeof(float));
          memcpy(&tmp.y, &buffer[15], sizeof(float));
          memcpy(&tmp.z, &buffer[19], sizeof(float));
          IMUA.w = 1 / sqrt(2) * (tmp.z - tmp.w);
          IMUA.x =-1 / sqrt(2) * (tmp.y + tmp.x);
          IMUA.y = 1 / sqrt(2) * (tmp.y - tmp.x);
          IMUA.z = 1 / sqrt(2) * (tmp.z + tmp.w);
          phiA = atan2(2 * (IMUA.w * IMUA.x + IMUA.y * IMUA.z), 1 - 2 * (IMUA.x * IMUA.x + IMUA.y * IMUA.y));
          thetaA = asin(2 * (IMUA.w * IMUA.y - IMUA.x * IMUA.z));
          //Serial.printf("IMUA.[w,x,y,z]=[%.3f,%.3f,%.3f,%.3f]\n",IMUA.w,IMUA.x,IMUA.y,IMUA.z);
          //Serial.printf("IMUA.[Phi,Theta,Psi]=[%.3f,%.3f,%.3f]\n", phiA*180/3.14, thetaA*180/3.14, atan2(2*(IMUA.w*IMUA.z+IMUA.x*IMUA.y), 1-2*(IMUA.y*IMUA.y+IMUA.z*IMUA.z))*180/3.14);
          
          // Compute the Configuration of Continuum Arm
          // 1. Set the yaw of the IMUB as 0
          phiB = atan2(2 * (IMUB.w * IMUB.x + IMUB.y * IMUB.z), 1 - 2 * (IMUB.x * IMUB.x + IMUB.y * IMUB.y));
          thetaB = asin(2 * (IMUB.w * IMUB.y - IMUB.x * IMUB.z));
          //Serial.printf("IMUB.[Phi,Theta,Psi]=[%.3f,%.3f,%.3f]\n",phiB*180/3.14,thetaB*180/3.14,psi);
          tmpA.w = cos(phiA / 2) * cos(thetaA / 2);  // Modified IMUA data
          tmpA.x = sin(phiA / 2) * cos(thetaA / 2);
          tmpA.y = cos(phiA / 2) * sin(thetaA / 2);
          tmpA.z =-sin(phiA / 2) * sin(thetaA / 2);
          tmpB.w = cos(phiB / 2) * cos(thetaB / 2);  // Modified IMUB data
          tmpB.x = sin(phiB / 2) * cos(thetaB / 2);
          tmpB.y = cos(phiB / 2) * sin(thetaB / 2);
          tmpB.z =-sin(phiB / 2) * sin(thetaB / 2);
          // 2. Get the Configuration of Continuum Arm
          tmp.w = tmpB.w*tmpA.w + tmpB.x*tmpA.x + tmpB.y*tmpA.y + tmpB.z*tmpA.z;
          tmp.x = tmpB.w*tmpA.x - tmpB.x*tmpA.w - tmpB.y*tmpA.z + tmpB.z*tmpA.y;
          tmp.y = tmpB.w*tmpA.y + tmpB.x*tmpA.z - tmpB.y*tmpA.w - tmpB.z*tmpA.x;
          tmp.z = tmpB.w*tmpA.z - tmpB.x*tmpA.y + tmpB.y*tmpA.x - tmpB.z*tmpA.w;
          // 3. Adjust the Configuration of Continuum Arm
          q.w = sqrt(tmp.w * tmp.w + tmp.z * tmp.z);
          q.x = 1 / q.w * ( tmp.w * tmp.x + tmp.z * tmp.y);
          q.y = 1 / q.w * (-tmp.z * tmp.x + tmp.w * tmp.y);
          q.z = 0.0;
          imua.publish(&IMUA);
          //Serial.printf("q.[w,x,y,z]=[%.3f,%.3f,%.3f,%.3f]\n",q.w,q.x,q.y,q.z);
          //Serial.printf("q.[Phi,Theta]=[%.3f,%.3f]\n",atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),  asin(2 * (q.w * q.y - q.x * q.z)));

          // 4. Dx Dy Parameters
          Gx.measure = ((q.y>0)?1:-1) * 2 * 40 * acos(q.w) / sqrt(1 + q.x * q.x / q.y / q.y);  // d = 40mm 
          Gy.measure = -q.x/q.y * Gx.measure; 
          Gx.measure = isnan(Gx.measure)?0.0:Gx.measure;
          Gy.measure = isnan(Gy.measure)?0.0:Gy.measure;
          Gx.measure -= deltaX;  // Calibration
          Gy.measure -= deltaY;
          //Serial.printf("[Gx.measure,Gy.measure]=[%.3f,%.3f]\n", Gx.measure, Gy.measure);
        }
      }
    }
    
  }
}

// fbCtrl loop
void fbCtrl_(void *pvParameters) {
  int Mx=0,My=0;
  while (1) {
    vTaskDelay(100);  // 1/0.100 = 10 Hz

    // Straight Flight (Outer Loop)
    if(fbCN==FB_SF){
      sf.err = sf.target - psi;
      sf.err_last = 0 + psi_*50;  // Here set yaw ratio times det_t psi_*(1/50) as diffential item
      Serial.printf("sf.[err,err']=[%.3f,%.3f]\n", sf.err, sf.err_last);
      Gy.target = (MAX(-stMax, MIN(stMax, sf.Kp*sf.err + sf.Kd*sf.err_last) ));
      //gondola(500, uint16_t(MAX(0, MIN(1000, My+500) )));
    }

    // Eular Spiral
    if(fbCN==FB_ES){
      Gy.target = (MAX(-50, MIN(50, Gy.target+slope) ));  // Linear Increase
    }

    // Square End Motion Trajectory
    if(fbCN==FB_SQ){
      if(!loopLemn && (esp_log_timestamp()-startTime)>2*(taLemn+tbLemn)){
         gondola(500,500);
         fbCN=FB_GD;isGD=GD_CLS;
      }
      float time_ = (esp_log_timestamp()-startTime)%int(2*(taLemn+tbLemn));
      if (time_ < taLemn) cntLemn = 0;
      else if(time_ < (taLemn+tbLemn)) cntLemn = 1;
      else if(time_ < (2*taLemn+tbLemn)) cntLemn = 2;
      else cntLemn = 3;
      switch(cntLemn){
      case 0:
         Gx.target = AxLemn*cos(6.283/(taLemn*2)*time_ - 3.1415);
         Gy.target = AyLemn;
         break;
      case 1:
         Gx.target = AxLemn;
         Gy.target = AyLemn*cos(6.283/(tbLemn*2)*(time_-taLemn));
         break;
      case 2:
         Gx.target = AxLemn*cos(6.283/(taLemn*2)*(time_-taLemn-tbLemn));
         Gy.target =-AyLemn;
         break;
      case 3:
         Gx.target =-AxLemn;
         Gy.target = AyLemn*cos(6.283/(tbLemn*2)*(time_-2*taLemn-tbLemn) - 3.1415);
         break;
      }
      //Serial.printf("%.3f,%.3f\n", Gx.target, Gy.target);
    }

    // Triangular End Motion Trajectory
    if(fbCN==FB_TR){
      if(!loopLemn && (esp_log_timestamp()-startTime)>(taLemn+2*tbLemn)){
         gondola(500,500);
         fbCN=FB_GD;isGD=GD_CLS;
      }
      float time_ = (esp_log_timestamp()-startTime)%int(taLemn+2*tbLemn);
      if (time_ < tbLemn) cntLemn = 0;
      else if(time_ < (tbLemn + taLemn)) cntLemn = 1;
      else cntLemn = 2;
      switch(cntLemn){
      case 0:
         Gx.target = AxLemn*sin(6.283/(tbLemn*4)*time_ - 3.1415);
         Gy.target = AyLemn*sin(6.283/(tbLemn*4)*time_ - 3.1415);
         break;
      case 1:
         Gx.target = AxLemn*cos(6.283/(taLemn*2)*(time_-tbLemn)-3.1415);
         Gy.target =-AyLemn;
         break;
      case 2:
         Gx.target = AxLemn*cos(6.283/(tbLemn*4)*(time_-tbLemn-taLemn));
         Gy.target =-AyLemn*cos(6.283/(tbLemn*4)*(time_-tbLemn-taLemn));
         break;
      }
      //Serial.printf("%.3f,%.3f\n", Gx.target, Gy.target);
    }
    
    // Gondola Distance PID Ctrl
    if(isGD){
      // PID Gondola X
      Gx.err = Gx.target-Gx.measure;
      Gx.integral = MAX(-500, MIN(500, Gx.integral+Gx.err) );
      Mx = Gx.Kp*Gx.err + Gx.Ki*Gx.integral + Gx.Kd*(Gx.err-Gx.err_last);
      Mx = ((abs(Mx)<6) or    !(isGD%2))?0:Mx;
      Gx.err_last = Gx.err;

      // PID Gondola Y
      Gy.err = Gy.target-Gy.measure;
      Gy.integral = MAX(-500, MIN(500, Gy.integral+Gy.err) );
      My = Gy.Kp*Gy.err + Gy.Ki*Gy.integral + Gy.Kd*(Gy.err-Gy.err_last);
      My = ((abs(My)<6) or !(isGD>>1%2))?0:My;
      Gy.err_last = Gy.err;

      gondola(uint16_t(MAX(0, MIN(1000, Mx+500) )), uint16_t(MAX(0, MIN(1000, My+500) )));
      //Serial.printf("[Dx,Dy]=[%.3f,%.3f], [Mx,My,DY]=[%d,%d,%d]\n", Gx.measure, Gy.measure, Mx, My, Gy.target);
    }
  }
}
