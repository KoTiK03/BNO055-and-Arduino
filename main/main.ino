//Эта программа считывает и обрабатывает данные библиотекой адафрута. Я планирую её буквально разобрать на исходники чтобы глубоко понять что к чему.

//Константы
#define BAUDRATE 115200
/* задержка между кадрами */
#define BNO055_SAMPLERATE_DELAY_MS (100)
//Библиотеки
#include <Wire.h>
#include <Adafruit_Sensor.h>  //Библиотека для работы с разного рода датчиками всякими
#include <Adafruit_BNO055.h>  //Её производная
#include <utility/imumaths.h>

//Переменные
int i;
//Это обьект из библиотеки
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Это огромная структура которая по сути отвечает за адреса всяких штук в датчике
// class Adafruit_BNO055 : public Adafruit_Sensor {
// public:
//   /** BNO055 Registers **/
//   typedef enum {
//     /* Page id register definition */
//     BNO055_PAGE_ID_ADDR = 0X07,

//     /* PAGE0 REGISTER DEFINITION START*/
//     BNO055_CHIP_ID_ADDR = 0x00,
//     BNO055_ACCEL_REV_ID_ADDR = 0x01,
//     BNO055_MAG_REV_ID_ADDR = 0x02,
//     BNO055_GYRO_REV_ID_ADDR = 0x03,
//     BNO055_SW_REV_ID_LSB_ADDR = 0x04,
//     BNO055_SW_REV_ID_MSB_ADDR = 0x05,
//     BNO055_BL_REV_ID_ADDR = 0X06,

//     /* Accel data register */
//     BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
//     BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
//     BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
//     BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
//     BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
//     BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

//     /* Mag data register */
//     BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
//     BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
//     BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
//     BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
//     BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
//     BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

//     /* Gyro data registers */
//     BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
//     BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
//     BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
//     BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
//     BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
//     BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

//     /* Euler data registers */
//     BNO055_EULER_H_LSB_ADDR = 0X1A,
//     BNO055_EULER_H_MSB_ADDR = 0X1B,
//     BNO055_EULER_R_LSB_ADDR = 0X1C,
//     BNO055_EULER_R_MSB_ADDR = 0X1D,
//     BNO055_EULER_P_LSB_ADDR = 0X1E,
//     BNO055_EULER_P_MSB_ADDR = 0X1F,

//     /* Quaternion data registers */
//     BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
//     BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
//     BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
//     BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
//     BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
//     BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
//     BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
//     BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

//     /* Linear acceleration data registers */
//     BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
//     BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
//     BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
//     BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
//     BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
//     BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

//     /* Gravity data registers */
//     BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
//     BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
//     BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
//     BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
//     BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
//     BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

//     /* Temperature data register */
//     BNO055_TEMP_ADDR = 0X34,

//     /* Status registers */
//     BNO055_CALIB_STAT_ADDR = 0X35,
//     BNO055_SELFTEST_RESULT_ADDR = 0X36,
//     BNO055_INTR_STAT_ADDR = 0X37,

//     BNO055_SYS_CLK_STAT_ADDR = 0X38,
//     BNO055_SYS_STAT_ADDR = 0X39,
//     BNO055_SYS_ERR_ADDR = 0X3A,

//     /* Unit selection register */
//     BNO055_UNIT_SEL_ADDR = 0X3B,

//     /* Mode registers */
//     BNO055_OPR_MODE_ADDR = 0X3D,
//     BNO055_PWR_MODE_ADDR = 0X3E,

//     BNO055_SYS_TRIGGER_ADDR = 0X3F,
//     BNO055_TEMP_SOURCE_ADDR = 0X40,

//     /* Axis remap registers */
//     BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
//     BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

//     /* SIC registers */
//     BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
//     BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
//     BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
//     BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
//     BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
//     BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
//     BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
//     BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
//     BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
//     BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
//     BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
//     BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
//     BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
//     BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
//     BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
//     BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
//     BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
//     BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

//     /* Accelerometer Offset registers */
//     ACCEL_OFFSET_X_LSB_ADDR = 0X55,
//     ACCEL_OFFSET_X_MSB_ADDR = 0X56,
//     ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
//     ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
//     ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
//     ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

//     /* Magnetometer Offset registers */
//     MAG_OFFSET_X_LSB_ADDR = 0X5B,
//     MAG_OFFSET_X_MSB_ADDR = 0X5C,
//     MAG_OFFSET_Y_LSB_ADDR = 0X5D,
//     MAG_OFFSET_Y_MSB_ADDR = 0X5E,
//     MAG_OFFSET_Z_LSB_ADDR = 0X5F,
//     MAG_OFFSET_Z_MSB_ADDR = 0X60,

//     /* Gyroscope Offset register s*/
//     GYRO_OFFSET_X_LSB_ADDR = 0X61,
//     GYRO_OFFSET_X_MSB_ADDR = 0X62,
//     GYRO_OFFSET_Y_LSB_ADDR = 0X63,
//     GYRO_OFFSET_Y_MSB_ADDR = 0X64,
//     GYRO_OFFSET_Z_LSB_ADDR = 0X65,
//     GYRO_OFFSET_Z_MSB_ADDR = 0X66,

//     /* Radius registers */
//     ACCEL_RADIUS_LSB_ADDR = 0X67,
//     ACCEL_RADIUS_MSB_ADDR = 0X68,
//     MAG_RADIUS_LSB_ADDR = 0X69,
//     MAG_RADIUS_MSB_ADDR = 0X6A
//   } adafruit_bno055_reg_t;

//   /** BNO055 power settings */
//   typedef enum {
//     POWER_MODE_NORMAL = 0X00,
//     POWER_MODE_LOWPOWER = 0X01,
//     POWER_MODE_SUSPEND = 0X02
//   } adafruit_bno055_powermode_t;

//   /** Remap settings **/
//   typedef enum {
//     REMAP_CONFIG_P0 = 0x21,
//     REMAP_CONFIG_P1 = 0x24, // default
//     REMAP_CONFIG_P2 = 0x24,
//     REMAP_CONFIG_P3 = 0x21,
//     REMAP_CONFIG_P4 = 0x24,
//     REMAP_CONFIG_P5 = 0x21,
//     REMAP_CONFIG_P6 = 0x21,
//     REMAP_CONFIG_P7 = 0x24
//   } adafruit_bno055_axis_remap_config_t;

//   /** Remap Signs **/
//   typedef enum {
//     REMAP_SIGN_P0 = 0x04,
//     REMAP_SIGN_P1 = 0x00, // default
//     REMAP_SIGN_P2 = 0x06,
//     REMAP_SIGN_P3 = 0x02,
//     REMAP_SIGN_P4 = 0x03,
//     REMAP_SIGN_P5 = 0x01,
//     REMAP_SIGN_P6 = 0x07,
//     REMAP_SIGN_P7 = 0x05
//   } adafruit_bno055_axis_remap_sign_t;

//   /** A structure to represent revisions **/
//   typedef struct {
//     uint8_t accel_rev; /**< acceleration rev */
//     uint8_t mag_rev;   /**< magnetometer rev */
//     uint8_t gyro_rev;  /**< gyroscrope rev */
//     uint16_t sw_rev;   /**< SW rev */
//     uint8_t bl_rev;    /**< bootloader rev */
//   } adafruit_bno055_rev_info_t;

//   /** Vector Mappings **/
//   typedef enum {
//     VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
//     VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
//     VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
//     VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
//     VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
//     VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
//   } adafruit_vector_type_t;

//   Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A,
//                   TwoWire *theWire = &Wire);

//   bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
//   void setMode(adafruit_bno055_opmode_t mode);
//   adafruit_bno055_opmode_t getMode();
//   void setAxisRemap(adafruit_bno055_axis_remap_config_t remapcode);
//   void setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign);
//   void getRevInfo(adafruit_bno055_rev_info_t *);
//   void setExtCrystalUse(boolean usextal);
//   void getSystemStatus(uint8_t *system_status, uint8_t *self_test_result,
//                        uint8_t *system_error);
//   void getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel,
//                       uint8_t *mag);

//   imu::Vector<3> getVector(adafruit_vector_type_t vector_type);
//   imu::Quaternion getQuat();
//   int8_t getTemp();

//   /* Adafruit_Sensor implementation */
//   bool getEvent(sensors_event_t *);
//   bool getEvent(sensors_event_t *, adafruit_vector_type_t);
//   void getSensor(sensor_t *);

//   /* Functions to deal with raw calibration data */
//   bool getSensorOffsets(uint8_t *calibData);
//   bool getSensorOffsets(adafruit_bno055_offsets_t &offsets_type);
//   void setSensorOffsets(const uint8_t *calibData);
//   void setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type);
//   bool isFullyCalibrated();

//   /* Power managments functions */
//   void enterSuspendMode();
//   void enterNormalMode();

// private:
//   byte read8(adafruit_bno055_reg_t);
//   bool readLen(adafruit_bno055_reg_t, byte *buffer, uint8_t len);
//   bool write8(adafruit_bno055_reg_t, byte value);

//   Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

//   int32_t _sensorID;
//   adafruit_bno055_opmode_t _mode;
// };



void displaySensorDetails(void) {
  sensor_t sensor;
// Это внутренняя шняга библиотеки которая по сути просто содержит тип сенсора и спецификацию на него. Наш тип вроде (3) (константа SENSOR_TYPE_ORIENTATION)
//   typedef struct {
//   char name[12];     /**< sensor name */
//   int32_t version;   /**< version of the hardware + driver */
//   int32_t sensor_id; /**< unique sensor identifier */
//   int32_t type;      /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
//   float max_value;   /**< maximum value of this sensor's value in SI units */
//   float min_value;   /**< minimum value of this sensor's value in SI units */
//   float resolution; /**< smallest difference between two values reported by this
//                        sensor */
//   int32_t min_delay; /**< min delay in microseconds between events. zero = not a
//                         constant rate */
// } sensor_t;

  bno.getSensor(&sensor);

//   void Adafruit_BNO055::getSensor(sensor_t *sensor) {
//   /* Clear the sensor_t object */
//   memset(sensor, 0, sizeof(sensor_t));

//   /* Insert the sensor name in the fixed length char array */
//   strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name) - 1] = 0;
//   sensor->version = 1;
//   sensor->sensor_id = _sensorID;
//   sensor->type = SENSOR_TYPE_ORIENTATION;
//   sensor->min_delay = 0;
//   sensor->max_value = 0.0F;
//   sensor->min_value = 0.0F;
//   sensor->resolution = 0.01F;
// }

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
}

void setup() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  /* Запуск сенсора */
  if (!bno.begin()) {

//     bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode) {

//   if (!i2c_dev->begin()) {
//     return false;
//   }

//   /* Make sure we have the right device */
//   uint8_t id = read8(BNO055_CHIP_ID_ADDR);
//   if (id != BNO055_ID) {
//     delay(1000); // hold on for boot
//     id = read8(BNO055_CHIP_ID_ADDR);
//     if (id != BNO055_ID) {
//       return false; // still not? ok bail
//     }
//   }

//   /* Switch to config mode (just in case since this is the default) */
//   setMode(OPERATION_MODE_CONFIG);

//   /* Reset */
//   write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
//   /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
//   delay(30);
//   while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
//     delay(10);
//   }
//   delay(50);

//   /* Set to normal power mode */
//   write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
//   delay(10);

//   write8(BNO055_PAGE_ID_ADDR, 0);

//   /* Set the output units */
//   /*
//   uint8_t unitsel = (0 << 7) | // Orientation = Android
//                     (0 << 4) | // Temperature = Celsius
//                     (0 << 2) | // Euler = Degrees
//                     (1 << 1) | // Gyro = Rads
//                     (0 << 0);  // Accelerometer = m/s^2
//   write8(BNO055_UNIT_SEL_ADDR, unitsel);
//   */

//   /* Configure axis mapping (see section 3.4) */
//   /*
//   write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
//   delay(10);
//   write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
//   delay(10);
//   */

//   write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
//   delay(10);
//   /* Set the requested operating mode (see section 3.3) */
//   setMode(mode);
//   delay(20);

//   return true;
// }

    /* Что-то пошло не так */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  /* Это отвечает за использование внешнего кристала. Я пока не понимаю как и куда оно работает */
  bno.setExtCrystalUse(true);

  /* Функция выводит некоторые базовые характеристики сенсора */
  displaySensorDetails();
}

void loop() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);  //если убрать второй аргумент (с одним аргументов запустить) в функции будут углы эйлера.

  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */

  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
