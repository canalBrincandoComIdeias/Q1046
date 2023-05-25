/*
     CÓDIGO:	  Q1046
     AUTOR:		  BrincandoComIdeias
     APRENDA: 	https://cursodearduino.net/
     SKETCH:    Teste 1x Helice - Projeto Drone
     DATA:		  25/04/2023
*/

// INCLUSÃO DE BIBLIOTECAS
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>

// DEFINIÇÕES DE PINOS
#define pinESC 8
#define pinStop 12
#define pinPot A0

#define APERTADO LOW
#define VEL_MIN 0
#define VEL_MAX 60 /* Limitador de velocidade para previnir acidentes */

// INSTANCIANDO OBJETOS
Servo esc;
Adafruit_MPU6050 mpu;

// DECLARANDO VARIAVEIS GLOBAIS
float angulo = 0.0F;
int velocidade = 0;

// DECLARANDO FUNÇÕES
void stop();
void setupMPU();
float accAngulo(sensors_event_t *acc);

void setup() {
  Serial.begin(115200);

  pinMode(pinStop, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  setupMPU();

  /* Aguarda enquanto o botão STOP estiver acionado */
  while (digitalRead(pinStop) == APERTADO) {
    digitalWrite(LED_BUILTIN, bitRead(millis(), 5));
  }
  digitalWrite(LED_BUILTIN, LOW);

  /* Define o pulso do pwm entre 1000us e 2000us */
  esc.attach(pinESC, 1000, 2000);
  esc.write(0);
  delay(3000);

  /* Espera por uma partida segura */
  while (map(analogRead(pinPot), 0, 1024, 0, VEL_MAX) > 10) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

void loop() {
  /* Leitura do Sensor */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo = accAngulo(&a);

  /* Exibe os valores para plotter serial */
  Serial.print("Ang:" + String(angulo));
  Serial.print(" , ");
  Serial.println("Vel:" + String(velocidade));

  /* Controle da velocidade do motor */
  velocidade = map(analogRead(pinPot), 0, 1024, 0, VEL_MAX);
  velocidade = constrain(velocidade, VEL_MIN, VEL_MAX);

  esc.write(velocidade);

  /* Botao de parada */
  if (digitalRead(pinStop) == APERTADO) {
    /* Caso precise parar o motor no meio do loop */
    stop();
  }
}

// IMPLEMENTO DAS FUNÇÕES
void stop() {
  velocidade = 0;
  esc.write(velocidade);
  esc.detach();

  /* Aguarda enquanto o botão STOP estiver acionado */
  do {
    digitalWrite(LED_BUILTIN, bitRead(millis(), 5));
  } while (digitalRead(pinStop) == APERTADO);
  digitalWrite(LED_BUILTIN, LOW);

  /* Define o pulso do pwm entre 1000us e 2000us */
  esc.attach(pinESC, 1000, 2000);
  esc.write(0);
  delay(3000);

  /* Espera por uma partida segura */
  while (map(analogRead(pinPot), 0, 1024, 0, VEL_MAX) > 10) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

void setupMPU() {

  if (!mpu.begin()) {
    Serial.println("Erro na inicialização do MPU");

    while (1) {
      /* Pisca indicando erro no sensor */
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }

  /* Utilizando a mesma escala que o projeto do YMFC-AL */
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float accAngulo(sensors_event_t *acc) {
  float ang = 0;

  /* Calculo com dados do Accelerometro */
  unsigned long ax2 = acc->acceleration.x * acc->acceleration.x;
  unsigned long ay2 = acc->acceleration.y * acc->acceleration.y;
  unsigned long az2 = acc->acceleration.z * acc->acceleration.z;

  /* módulo = raiz da soma dos quadrados das componentes */
  float modulo = sqrt(ax2 + ay2 + az2);

  /* valor da componente z / módulo do vetor resultado = cos do angulo em relação ao eixo z */
  float cos = acc->acceleration.z / modulo;
  //float sin = acc->acceleration.x / modulo;

  ang = acos(cos);
  //ang = asin(sin);

  //evita resultado nan
  if (isnan(ang)) return 0;

  //converte de radianos para graus
  ang = ang * 180 / PI;

  return ang;
}
