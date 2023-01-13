//Тестовый скетч для визуализации траектории в питоне

#define DEBUG_MODE_INIT
//Константы
#define BAUDRATE 115200
/* задержка между кадрами */
#define BNO055_SAMPLERATE_DELAY_MS 100
/* Длина UART Команды */
#define COMMAND_LEN 4
//Библиотеки
#include <Wire.h>
#include <Adafruit_Sensor.h>  
#include <Adafruit_BNO055.h>  
#include <utility/imumaths.h>



int i;
double time1,time2,time_val;
String console;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

sensors_event_t  event_acel;
//sensors_event_t  event_euler;
imu::Quaternion Rotate, Vector, Rotate_reverse, Ready;
String Packet[9];
String ReadyPacket;
//char Packet[5][10];
//char ReadyPacket[50];
double Acel[3];  //[x,y,z]
double Vel[3];
double Pos[3];
void PC_init();

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
  for (i = 0; i<3; i++)
  {
    Acel[i] = 0.0;        
  }  
  for (i = 0; i<3; i++)
  {
    Vel[i] = 0.0;
  }  
  for (i = 0; i<3; i++)
  {
    Pos[i] = 0.0;
  }  
}

void setup() {
  Serial.begin(115200);
  while (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay (5000);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  //displaySensorDetails();
  /*
  Функция записи в регистр с каким то хуем спрятана в приват. Програмисты на плюсах конченные
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (1 << 2) | // Euler = Rad
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  bno.Adafruit_BNO055::write8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR, unitsel);// Это я на радианы переключил эйлеры
  */
  time1 = (long)(millis());
  
  
}


void loop() {
  
  //PC_init();
  
  
  //bno.getEvent(&event_euler);
  time2 = time1;   
  time1 = (long)(millis());
  //time_val = time1 - time2;
  Rotate = bno.getQuat();
  bno.getEvent(&event_acel, Adafruit_BNO055::VECTOR_LINEARACCEL);
  /*
  Vector = imu::Quaternion(0,(double)event_acel.acceleration.x,(double)event_acel.acceleration.y,(double)event_acel.acceleration.z);
  Rotate_reverse = Rotate.conjugate() / (double)(Rotate.w()*Rotate.w()+Rotate.x()*Rotate.x()+Rotate.y()*Rotate.y()+Rotate.z()*Rotate.z());
  Ready = Rotate * Vector * Rotate_reverse;
  */
  //Для удобной работы избавляюсь от обьекта кватерниона. 

  /*
  Packet[0] = Ready.x();
  Packet[1] = Ready.y();
  Packet[2] = Ready.z();
  Packet[3] = time1;
  Packet[4] = time2;
  */
  /*
  dtostrf(Ready.x(),4,4,Packet[0]);
  dtostrf(Ready.y(),4,4,Packet[1]);
  dtostrf(Ready.z(),4,4,Packet[2]);
  dtostrf(time1,4,4,Packet[3]);
  dtostrf(time2,4,4,Packet[4]);
  memcpy(&ReadyPacket[0],Packet[0],9);
  memcpy(&ReadyPacket[9],Packet[1],9);
  memcpy(&ReadyPacket[19],Packet[2],9);
  memcpy(&ReadyPacket[29],Packet[3],9);
  memcpy(&ReadyPacket[39],Packet[4],9);
  */
  /*
  //Правильная отправка
  Packet[0] = String(Ready.x(), 4);
  Packet[1] = String(Ready.y(), 4);
  Packet[2] = String(Ready.z(), 4);
  Packet[3] = String(time1);
  Packet[4] = String(time2);
  ReadyPacket = Packet[0] + " " + Packet[1] + " " + Packet[2] + " " + Packet[3] + " " + Packet[4]; 
  */


  /*
  //Дебаговая отправка без поворота
  Packet[0] = String(event_acel.acceleration.x, 4);
  Packet[1] = String(event_acel.acceleration.y, 4);
  Packet[2] = String(event_acel.acceleration.z, 4);
  Packet[3] = String(time1);
  Packet[4] = String(time2);
  ReadyPacket = Packet[0] + " " + Packet[1] + " " + Packet[2] + " " + Packet[3] + " " + Packet[4]; 
  */
  /*
  //Дебаговая отправка кватерниона
  Packet[0] = String((double)Rotate.w(), 4);
  Packet[1] = String((double)Rotate.x(), 4);
  Packet[2] = String((double)Rotate.y(), 4);
  Packet[3] = String((double)Rotate.z(), 4);
  Packet[4] = String(time1);
  ReadyPacket = Packet[0] + " " + Packet[1] + " " + Packet[2] + " " + Packet[3] + " " + Packet[4]; 
  */

  //Новая главная отправка. Отныне вся обработка на сервере
  Packet[0] = String(event_acel.acceleration.x, 4);
  Packet[1] = String(event_acel.acceleration.y, 4);
  Packet[2] = String(event_acel.acceleration.z, 4);
  Packet[3] = String(time1);
  Packet[4] = String(time2);
  Packet[5] = String((double)Rotate.w(), 4);
  Packet[6] = String((double)Rotate.x(), 4);
  Packet[7] = String((double)Rotate.y(), 4);
  Packet[8] = String((double)Rotate.z(), 4);
  ReadyPacket = Packet[0] + " " + Packet[1] + " " + Packet[2] + " " + Packet[3] + " " + Packet[4] + " " + Packet[5] + " " + Packet[6] + " " + Packet[7] + " " + Packet[8]; 
  Serial.println(ReadyPacket);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
/*
  //Первое интегрирование
  for (i = 0; i < 3; i++)
  {
    Vel[i] = (double)(Vel[i] + Acel[i] * (time_val/1000));
  }
  //Второе интегрирование
  for (i = 0; i < 3; i++)
  {
    Pos[i] = (Pos[i] + Vel[i] * (time_val/1000));
  }
*/
  
/*
  Serial.print("Position: x = ");
  Serial.print(Pos[0]);
  Serial.print(" y = ");
  Serial.print(Pos[1]);
  Serial.print(" z = ");
  Serial.println(Pos[2]);  
*/ 
  //Serial.println(sizeof(Packet));


  //Serial.write((char*)Packet,sizeof(Packet));
       
}


void PC_init()
{
  char await[4] = {0xC0,0xFF,0x01,0xC0};
  char buf[4] = {0,0,0,0};
  #ifdef DEBUG_MODE_INIT
    char debug[4] = {0x22,0x33,0x44,0x55};
  #endif  
  while(1)
  {
    Serial.write(await,COMMAND_LEN);
    if (Serial.readBytes(buf,COMMAND_LEN) > 0)
    {
      
      Serial.write(debug,COMMAND_LEN);
      Serial.write(buf,COMMAND_LEN);
            
      if (buf[0] == 0xC0)
      {
        if (buf[1] == 0x01)
        {
          if (buf[2] == 0x01)
          {
            if (buf[3] == 0xC0)
            {
              Serial.write(buf,COMMAND_LEN);
              break;            
            }          
          }       
        }
      }  
    }  
  }

}