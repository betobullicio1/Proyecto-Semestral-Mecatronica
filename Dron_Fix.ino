#include "BluetoothSerial.h"   // Para comunicacion Bluetooth 
#include <Wire.h>              // Para comunicacion I2C con el MPU6050
#include <Adafruit_MPU6050.h>  // Libreria para el MPU6050
#include <Adafruit_Sensor.h>   // Dependencia de la libreria Adafruit_MPU6050

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it.
#endif

// Crea un objeto BluetoothSerial
BluetoothSerial SerialBT; 

// --- Configuracion de Pines GPIO y Canales PWM ---
// Pines de la ESP32 conectados al pin 'PWM' de cada modulo MOSFET.
// AJUSTA ESTOS PIN ES SEGÚN TUS CONEXIONES REALES.
const int motor1Pin = 2;  // Motor Frontal Izquierdo
const int motor2Pin = 4;  // Motor Frontal Derecho
const int motor3Pin = 5;  // Motor Trasero Izquierdo
const int motor4Pin = 18; // Motor Trasero Derecho

// Configuración de PWM
const int freq = 5000;      // Frecuencia del PWM en Hz (5000 Hz es un buen punto de partida para motores DC)
const int resolution = 8;   // Resolución del PWM (8 bits = 0-255 valores)

// Canales PWM (0-15)
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

// --- MPU6050 ---
Adafruit_MPU6050 mpu; // Crea un objeto MPU6050

// Variables para los ángulos de Roll y Pitch
float roll = 0.0;
float pitch = 0.0;
// float yaw = 0.0; // Yaw es más complejo de estabilizar con solo MPU6050 sin magnetómetro

// --- Control PID ---
// ¡¡¡ESTOS VALORES SON EJEMPLOS Y NECESITARÁN AJUSTE MANUAL INTENSIVO!!!
// Kp (Proporcional): Responde a la diferencia actual.
// Ki (Integral): Responde a la acumulación de error (para eliminar el error estacionario).
// Kd (Derivativo): Responde a la tasa de cambio del error (para amortiguar oscilaciones).

// PID para Roll
float Kp_roll = 1.0;  // Empieza con valores bajos (ej: 0.1 a 1.0)
float Ki_roll = 0.0;
float Kd_roll = 0.0;

// PID para Pitch
float Kp_pitch = 1.0; // Empieza con valores bajos
float Ki_pitch = 0.0;
float Kd_pitch = 0.0;

// Error acumulado para el término Integral
float error_roll_sum = 0.0;
float error_pitch_sum = 0.0;

// Error previo para el término Derivativo
float prev_error_roll = 0.0;
float prev_error_pitch = 0.0;

// Tiempo para cálculo derivativo e integral (ms)
unsigned long last_loop_time = 0;
float dt = 0; // Delta tiempo en segundos

// --- Variables de Vuelo ---
int throttle = 0; // Acelerador principal (0-255)
bool armed = false; // Estado de seguridad: false = motores apagados, true = motores armados

// Velocidad base para los motores cuando están armados
const int MIN_THROTTLE_ARMED = 50; // Velocidad mínima a la que los motores giran cuando armados
const int MAX_THROTTLE = 255;      // Velocidad máxima (255 para 8-bit PWM)

// --- Funciones Auxiliares (Definiciones o Prototipos) ---
// Función auxiliar para controlar un motor individual
void setMotorSpeed(int channel, int speed) {
  ledcWrite(channel, constrain(speed, 0, MAX_THROTTLE));
}

// Prototipos de las otras funciones
void setAllMotorsSpeed(int speed);
void readMPU6050Data();
void calculatePID();
void applyMotorMix();


// --- SETUP ---
void setup() {
  Serial.begin(115200);
  SerialBT.begin("MiDroneESP32_MPU"); // Nuevo nombre Bluetooth
  Serial.println("MiDroneESP32_MPU listo. Esperando comandos.");

  // Configurar comunicación I2C para MPU6050
  Wire.begin(); // ESP32 por defecto SDA=GPIO21, SCL=GPIO22

  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println("Fallo al encontrar MPU6050, revisa conexiones!");
    while (1) delay(10); // Queda en loop si no encuentra el sensor
  }
  Serial.println("MPU6050 encontrado y configurado.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Rango del acelerómetro
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // Rango del giroscopio
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Filtro para suavizar lecturas

  // Calibracion Basica (idealmente deberias calcular tus propios offsets)
  // mpu.setGyroOffsets(X, Y, Z);
  // mpu.setAccelOffsets(X, Y, Z);

  // Configurar PWM para motores
  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(motor1Pin, pwmChannel1); 
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(motor2Pin, pwmChannel2); 
  ledcSetup(pwmChannel3, freq, resolution);
  ledcAttachPin(motor3Pin, pwmChannel3); 
  ledcSetup(pwmChannel4, freq, resolution);
  ledcAttachPin(motor4Pin, pwmChannel4); 

  // Apagar motores al inicio
  setAllMotorsSpeed(0);
}

// --- LOOP ---
void loop() {
  unsigned long current_time = millis();
  dt = (current_time - last_loop_time) / 1000.0; // Delta tiempo en segundos
  last_loop_time = current_time;

  readMPU6050Data(); // Lee datos del sensor y calcula Roll/Pitch

  // --- Procesamiento de comandos Bluetooth ---
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();

    Serial.print("Comando BT: ");
    Serial.println(command);

    if (command == "armar") {
      armed = true;
      Serial.println("DRON ARMADO. Motores listos.");
      setAllMotorsSpeed(MIN_THROTTLE_ARMED); // Motores a velocidad minima al armar
    } else if (command == "desarmar") {
      armed = false;
      Serial.println("DRON DESARMADO. Motores apagados.");
      setAllMotorsSpeed(0);
      throttle = 0; // Reinicia el acelerador
    } else if (command.startsWith("throttle_")) {
      // Formato: "throttle_100" para ajustar la aceleracion
      throttle = command.substring(9).toInt();
      throttle = constrain(throttle, 0, MAX_THROTTLE);
      Serial.print("Throttle ajustado a: "); Serial.println(throttle);
    } else if (command.startsWith("pid_roll_kp_")) {
      Kp_roll = command.substring(12).toFloat();
      Serial.print("Kp_roll ajustado a: "); Serial.println(Kp_roll);
    } else if (command.startsWith("pid_pitch_kp_")) {
      Kp_pitch = command.substring(13).toFloat();
      Serial.print("Kp_pitch ajustado a: "); Serial.println(Kp_pitch);
    }
    // Puedes añadir comandos para ajustar Ki, Kd o todos a la vez.
  }

  // --- Lógica de Control de Vuelo ---
  if (armed) {
    calculatePID();     // Calcula las correcciones PID
    applyMotorMix();    // Aplica las correcciones a las velocidades de los motores
  } else {
    // Si no está armado, los motores permanecen apagados (setAllMotorsSpeed(0) ya se encarga de esto)
  }

  // Pequeño delay para no saturar el loop, especialmente si no hay comandos BT
  delay(5); 
}

// --- Funciones de Dron ---

// Establece una velocidad base para todos los motores (usado para armar o desarmar)
void setAllMotorsSpeed(int speed) {
  setMotorSpeed(pwmChannel1, speed);
  setMotorSpeed(pwmChannel2, speed);
  setMotorSpeed(pwmChannel3, speed);
  setMotorSpeed(pwmChannel4, speed);
}

// Lee datos del MPU6050 y calcula Roll y Pitch usando un filtro complementario simple
void readMPU6050Data() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convertir giroscopio de rad/s a deg/s para la integracion
  float gyroX = g.gyro.x * (180.0 / PI); // Roll rate
  float gyroY = g.gyro.y * (180.0 / PI); // Pitch rate

  // Ángulos del acelerómetro (estáticos)
  float accRoll = atan2(a.acceleration.y, a.acceleration.z) * (180.0 / PI);
  float accPitch = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * (180.0 / PI);

  // Filtro Complementario (ajusta el factor alfa según tu preferencia)
  // Alfa más alto: más confianza en el acelerómetro (más lento a cambios rápidos, menos ruido)
  // Alfa más bajo: más confianza en el giroscopio (más rápido a cambios rápidos, más deriva)
  float alpha = 0.98; // Típicamente entre 0.95 y 0.98
  roll = alpha * (roll + gyroX * dt) + (1.0 - alpha) * accRoll;
  pitch = alpha * (pitch + gyroY * dt) + (1.0 - alpha) * accPitch;

  // Imprimir datos para depuración (puedes comentar esto si causa mucha salida)
  // Serial.print("Roll: "); Serial.print(roll);
  // Serial.print(" Pitch: "); Serial.print(pitch);
  // Serial.print(" AccX: "); Serial.print(a.acceleration.x);
  // Serial.print(" GyroX: "); Serial.println(g.gyro.x);
}

// Calcula las salidas PID para Roll y Pitch
void calculatePID() {
  // Target (objetivo) de angulo para vuelo nivelado es 0 grados.
  float target_roll = 0.0;
  float target_pitch = 0.0; // En esta version, se mantiene nivelado

  // Error actual
  float error_roll = target_roll - roll;
  float error_pitch = target_pitch - pitch;

  // Termino Proporcional (P)
  float p_out_roll = Kp_roll * error_roll;
  float p_out_pitch = Kp_pitch * error_pitch;

  // Termino Integral (I) - Acumulación de error a lo largo del tiempo
  error_roll_sum += error_roll * dt;
  error_pitch_sum += error_pitch * dt;
  
  // Limitar la suma del error integral para evitar "wind-up"
  error_roll_sum = constrain(error_roll_sum, -100, 100); // Ajusta estos limites
  error_pitch_sum = constrain(error_pitch_sum, -100, 100); // Ajusta estos limites

  float i_out_roll = Ki_roll * error_roll_sum;
  float i_out_pitch = Ki_pitch * error_pitch_sum;

  // Termino Derivativo (D) - Tasa de cambio del error
  float d_out_roll = Kd_roll * ((error_roll - prev_error_roll) / dt);
  float d_out_pitch = Kd_pitch * ((error_pitch - prev_error_pitch) / dt);

  // Actualizar error previo
  prev_error_roll = error_roll;
  prev_error_pitch = error_pitch;
}

// Aplica las correcciones PID a las velocidades de los motores
void applyMotorMix() {
  // Las velocidades base ya estan en 'throttle'
  // Las correcciones PID se aplican como ajustes a ese throttle.
  // Este es un esquema MIXER simplificado para un Quadcopter X/+:
  // (La direccion de rotacion de los motores es importante para el yaw, pero aqui solo Roll/Pitch)

  // Asumimos que los p_out_roll y p_out_pitch son calculados en calculatePID()
  // y que Kp_roll y Kp_pitch actuan como "ganancias" para los errores.
  // Para esta implementacion, estamos aplicando las correcciones directamente a la mezcla:

  float roll_correction_value = (roll * Kp_roll);   // Usa el angulo actual * Kp
  float pitch_correction_value = (pitch * Kp_pitch); // Usa el angulo actual * Kp

  int currentMotor1Speed = throttle - pitch_correction_value + roll_correction_value; 
  int currentMotor2Speed = throttle - pitch_correction_value - roll_correction_value;
  int currentMotor3Speed = throttle + pitch_correction_value + roll_correction_value;
  int currentMotor4Speed = throttle + pitch_correction_value - roll_correction_value;

  // Escribe las velocidades a los canales PWM usando la nueva función setMotorSpeed
  setMotorSpeed(pwmChannel1, currentMotor1Speed);
  setMotorSpeed(pwmChannel2, currentMotor2Speed);
  setMotorSpeed(pwmChannel3, currentMotor3Speed);
  setMotorSpeed(pwmChannel4, currentMotor4Speed);

  // Imprime las velocidades para depuración
  Serial.print("M1:"); Serial.print(currentMotor1Speed);
  Serial.print(" M2:"); Serial.print(currentMotor2Speed);
  Serial.print(" M3:"); Serial.print(currentMotor3Speed);
  Serial.print(" M4:"); Serial.println(currentMotor4Speed);

  // Imprime angulos para depuración del sensor
  Serial.print("Roll_deg: "); Serial.print(roll);
  Serial.print(", Pitch_deg: "); Serial.println(pitch);
}