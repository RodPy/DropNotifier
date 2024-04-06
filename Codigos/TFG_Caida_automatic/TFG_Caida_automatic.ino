#include "Wire.h"

#include "math.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFi.h>

/*--------------------------MPU 6050---------------------------------*/
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
//MPU6050 sensor_ang;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
#define SDA 4
#define SCL 15
#define btn 21

#define red 27
#define blue 12
#define green 14
const int MPU_addr=0x68; // I2C address of the MPU-6050

int16_t ax, ay, az;
int16_t gx, gy, gz, Tmp;
double ax_d, ay_d, az_d;
double  ang_x, ang_y;
float ang_x_prev, ang_y_prev;
long tiempo_prev;
float dt;
int d_max;
float accel_ang_y,accel_ang_x; 

int lastState = HIGH; 
int currentState;   

#define WLAN_SSID   "RS_PC"
#define WLAN_PASS   "chocolate"
// #define WLAN_SSID   "Flia.Arriola"
// #define WLAN_PASS   "enpala649"

//MQTT broker settings https://proyectoinfo.mybluemix.net
//#define HOST        "192.168.100.32"
//#define HOST        "192.168.100.17"
#define HOST        "192.168.100.137"

#define PORT        1883
#define USERNAME    "linux"
#define PASSWORD    "linux"

//time out loop count
const int timeout = 200;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, HOST, PORT, USERNAME, PASSWORD);
Adafruit_MQTT_Publish acelera_x = Adafruit_MQTT_Publish(&mqtt, "acelerometro/ax");
Adafruit_MQTT_Publish acelera_y = Adafruit_MQTT_Publish(&mqtt, "acelerometro/ay");
Adafruit_MQTT_Publish acelera_z = Adafruit_MQTT_Publish(&mqtt, "acelerometro/az");
Adafruit_MQTT_Publish button = Adafruit_MQTT_Publish(&mqtt, "acelerometro/button");


void MQTT_connect();
void init_MPU_sensor();
void MPU_sensor();

bool detectarMovimientoFurso(int16_t ax_actual, int16_t ay_actual, int16_t az_actual, int16_t ax_anterior, int16_t ay_anterior, int16_t az_anterior) {
    // Calcula la diferencia absoluta entre las mediciones actuales y anteriores en cada eje
    int16_t diff_ax = abs(ax_actual - ax_anterior);
    int16_t diff_ay = abs(ay_actual - ay_anterior);
    int16_t diff_az = abs(az_actual - az_anterior);
    int delta = 100
    // Comprueba si alguna de las diferencias es mayor que 100 puntos
    if (diff_ax > delta || diff_ay > delta || diff_az > delta) {
        return true; // Hay un movimiento brusco
    } else {
        return false; // No hay movimiento brusco
    }
}

void setup() { 
  delay(1); 
  WiFi.mode(WIFI_STA);  
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  pinMode(btn, INPUT_PULLUP); // config GPIO21 as input pin and enable the internal pull-up resistor
  pinMode(red, OUTPUT); // config GPIO21 as input pin and enable the internal pull-up resistor
  pinMode(green, OUTPUT); // config GPIO21 as input pin and enable the internal pull-up resistor
  pinMode(blue, OUTPUT); // config GPIO21 as input pin and enable the internal pull-up resistor

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  int i = 0;
  for (; i < timeout; i++)
  {
    if(WiFi.status() == WL_CONNECTED) break;
    delay(100);
    Serial.print(".");
  }
  if(i == timeout)
    Serial.println("No Conectado");
  LedConect(100, red);

  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  LedConect(100, green);

  MQTT_connect();
  init_MPU_sensor();
}
  void loop() 
{
  currentState = digitalRead(btn);
  Serial.println( currentState);
  if (currentState == LOW){
      button.publish(currentState);
        digitalWrite(green, LOW);
        digitalWrite(red, LOW);
        digitalWrite(blue, HIGH);
  }
  else{
        digitalWrite(green, HIGH);
        digitalWrite(red, LOW);
        digitalWrite(blue, LOW);
  }

  delay(150);/*
  int x = random(-10,10);
  int y = random(-10,10);
  int z = random(-10,10);*/

  acelera_x.publish(ax_d);
  delay(1);
  acelera_y.publish(ay_d);
  delay(1);
  acelera_z.publish(az_d);

    // Guarda las mediciones anteriores antes de obtener nuevas mediciones
    int16_t ax_anterior = ax_d;
    int16_t ay_anterior = ay_d;
    int16_t az_anterior = az_d;

    // Obtiene nuevas mediciones del sensor MPU6050
    MPU_sensor();

    // Comprueba si hay un movimiento brusco comparando las mediciones actuales con las anteriores
    bool movimiento_brusco = detectarMovimientoFurso(ax, ay, az, ax_anterior, ay_anterior, az_anterior);

    // Si hay un movimiento brusco, realiza alguna acción
    if (movimiento_brusco) {
        // Realiza alguna acción, como enviar un mensaje MQTT, encender un LED, etc.
        Serial.println("¡Movimiento brusco detectado!");
        button.publish(LOW);

    }

  Serial.print(ax_d); Serial.print("\t");Serial.print(ay_d); Serial.print("\t");Serial.print(az_d); Serial.println("\t");
  MPU_sensor();

}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 10;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 1 second...");
       mqtt.disconnect();
       delay(1000);
       retries--;
       if (retries == 0)
         Serial.println("No Conectado");
           LedConect(200, red);

  }
  Serial.println("MQTT Connected!");
   LedConect(100, blue);
}

void init_MPU_sensor(){
  Wire.begin(SDA, SCL);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void MPU_sensor(){
  /* Leer las aceleraciones y velocidades angulares
  sensor_ang.getAcceleration(&ax, &ay, &az);
  sensor_ang.getRotation(&gx, &gy, &gz);
  // Correxion de Offsets*/
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  ax=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  az_d = az * (9.81/16384.0);
  ay_d = ay * (9.81/16384.0);
  ax_d = ax * (9.81/16384.0);
}

void LedConect(int time, int led){
  digitalWrite(led, HIGH);
  delay(time);
  ledOFF(100);
}

void ledOFF(int time){
  digitalWrite(green, LOW);
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  delay(time);
}
