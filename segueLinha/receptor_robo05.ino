// =====================================================================
//  RECEPTOR DO ROBO (ESP32-S3) - ESP-NOW  (versao completa)
//  - Modo MANUAL: D-pad (MSG_CMD), pulsos (MSG_PULSOS), LEDs RGB (MSG_LED),
//    garra (MSG_GARRA, so se a PCA9685 estiver conectada)
//  - Modo OBR: MSG_OBR a=OBR_INICIAR roda o obr010.ino;
//    MSG_OBR a=OBR_PARAR reinicia o robo (reconectar em BUSCAR ROBOS)
//  - Telemetria a cada 200 ms: sensores, fimC1/fimC2, estado
//
//  Pasta do sketch: MotorControl.h, MotorControl.cpp,
//                   IReight_model.hpp, IReight_model.cpp
//  Bibliotecas: Adafruit VL53L0X, Adafruit TCS34725,
//               Adafruit NeoPixel, Adafruit PWM Servo Driver
// =====================================================================
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "MotorControl.h"
#include "IReight_model.hpp"

#define CANAL                 1
#define DEVICE_NAME           "ROBO-01"
#define TIMEOUT_SINAL_MS      400
#define TIMEOUT_PULSOS_MS     15000
#define INTERVALO_ANNOUNCE_MS 300
#define INTERVALO_TELEM_MS    200
#define INTERVALO_PCA_MS      1000
#define LED_BRILHO            60
#define VEL_GARRA             180
#define SEM_VALOR             0xFFFF

// ---------- Pinos e canais (como no obr010.ino) ----------
#define I2C_SDA  15
#define I2C_SCL  16
#define TCAADDR  0x70
#define PCA_ADDR 0x40
#define PIN_LED_1 2
#define NUMPIXELS 8
#define SERVOMIN 150
#define SERVOMAX 600

const int fimC1 = 9;     // fim de curso 1 (INPUT_PULLUP, pressionado = LOW)
const int fimC2 = 10;    // fim de curso 2

const uint8_t CANAL_VLX_FRENTE   = 2;
const uint8_t CANAL_VLX_DIREITO  = 5;
const uint8_t CANAL_VLX_ESQUERDO = 4;
const uint8_t CANAL_COR_ESQ      = 0;
const uint8_t CANAL_COR_DIR      = 1;
const uint8_t CANAL_PCA          = 3;

const int TRIG_1 = 12, ECHO_1 = 11;   // U1 frontal
const int TRIG_2 = 21, ECHO_2 = 47;   // U2 lado 1
const int TRIG_3 = 13, ECHO_3 = 14;   // U3 lado 2
const int TRIG_US[3] = {TRIG_1, TRIG_2, TRIG_3};
const int ECHO_US[3] = {ECHO_1, ECHO_2, ECHO_3};

// ---------- Protocolo (mesmos valores do controle) ----------
enum : uint8_t { MSG_DISCOVER = 1, MSG_ANNOUNCE = 2, MSG_COLOR = 3, MSG_CMD = 4,
                 MSG_PULSOS = 5, MSG_TELEM = 6, MSG_OBR = 7, MSG_LED = 8, MSG_GARRA = 9 };
enum : uint8_t { CMD_STOP = 0, CMD_FRENTE, CMD_TRAS, CMD_PIVO_ESQ, CMD_PIVO_DIR, CMD_CURVA_ESQ, CMD_CURVA_DIR };
enum : uint8_t { DIR_FRENTE = 0, DIR_TRAS, DIR_GIRO_ESQ, DIR_GIRO_DIR };
enum : uint8_t { PULSO_OCIOSO = 0, PULSO_EXECUTANDO, PULSO_CONCLUIDO, PULSO_TIMEOUT, PULSO_PARADO };
enum : uint8_t { CLASSE_NADA = 0, CLASSE_VERDE, CLASSE_VERMELHO, CLASSE_PRETO };
enum : uint8_t { OBR_PARAR = 0, OBR_INICIAR = 1 };
enum : uint8_t { LED_ESQ = 0, LED_DIR = 1, LED_AMBOS = 2 };
enum : uint8_t { GARRA_ABAIXA = 0, GARRA_DERRUBA_DIR, GARRA_DERRUBA_ESQ, GARRA_PCOLETA, GARRA_PNEUTRA };

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t a;          // CMD: comando | OBR: OBR_* | GARRA: GARRA_*
  uint8_t b;          // CMD: velocidade
  uint8_t c;
  char name[16];
} Msg;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  dir;       // DIR_*
  uint16_t pulsos;
  uint8_t  vel;
} PacotePulsos;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t lado;       // LED_ESQ, LED_DIR, LED_AMBOS
  uint8_t r, g, b;
} PacoteLed;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  estadoPulso;
  uint8_t  okMask;    // bit0 cor E, bit1 cor D, bit2 VLX F, bit3 VLX D, bit4 VLX E, bit5 PCA
  uint8_t  estado;    // bit0 OBR ativo, bit1 vermelho detectado
  uint8_t  fim;       // bit0 fimC1 pressionado, bit1 fimC2 pressionado
  uint8_t  classe[2];
  uint16_t pulsosE;
  uint16_t pulsosD;
  uint16_t cor[2][4]; // R, G, B, C
  uint16_t ir[8];
  uint16_t ultra[3];  // mm, SEM_VALOR = sem eco
  uint16_t vlx[3];    // mm, SEM_VALOR = erro
} Telem;

// ---------- Motores ----------
MotorControl mEsq(5, 4, 40, 39);
MotorControl mDir(7, 6, 42, 41);
void IRAM_ATTR isrEsq() { mEsq.handleInterrupt(); }
void IRAM_ATTR isrDir() { mDir.handleInterrupt(); }

// ---------- Sensores ----------
Adafruit_VL53L0X loxFrente, loxDireito, loxEsquerdo;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA_ADDR);
Adafruit_NeoPixel led1(NUMPIXELS, PIN_LED_1, NEO_GRB + NEO_KHZ800);

bool     corOk[2] = {false, false};
bool     vlxOk[3] = {false, false, false};
bool     pcaOk = false;
uint16_t corRaw[2][4] = {{0}};
uint8_t  classeCor[2] = {CLASSE_NADA, CLASSE_NADA};
uint16_t ultraMM[3] = {SEM_VALOR, SEM_VALOR, SEM_VALOR};
uint16_t vlxMM[3]   = {SEM_VALOR, SEM_VALOR, SEM_VALOR};
uint32_t ultimoPcaMs = 0;

// ---------- Variaveis do obr010.ino ----------
int sensorValues[8];
const int limiar  = 800;
const int limiarD = 1800;
int dir = 1;
const int N_LINHA = 1;
int t = 1;
int t2 = 3;
int nv = 2;
int TErro = 0, TErro1 = 0, TErro2 = 0;
int Va = 300;
int lP = 190;
int nT = 2;
int d1 = 10, d2 = 5, d3 = 15;
int tentativas = 1;
const int nL = 30;
int nL2 = 5;
int leiturasObstaculo = 0;
int av = 0, av1 = 430, av2 = 800;
int PG = 1130;
int velMedia = 200;
int velBusca = 250;
int velLenta = 100;
volatile int n1 = 0, n2 = 0, comandoVerde = 0;
int e = 0, nE = 50;
char ladoPrioridade = 'F';
bool achou = true;
volatile bool vermelho = false;
const uint8_t CANAL_COR_ESQ_OBR = 0;   // mesmo canal de CANAL_COR_ESQ

bool Terminou = false;
int distLateralAnterior = -1;
enum LadoParede { LADO_ESQUERDA, LADO_DIREITA };
LadoParede ladoSeguido;
const int DIST_FRONTAL_MIN_MM   = 170;
const long PULSOS_GIRO_DESVIO   = 500;
const int DELTA_DIST_ABRUPTO_MM = 100;
const int velManobra = 255;
const int velBase    = 255;
const unsigned long PERIODO_LOOP_MS = 20;
unsigned long ultimoLoopServos = 0;
int direction = 1;

enum State { SEGUE_LINHA, SERVOS_ESQUEMAS };
State estadoAtual = SEGUE_LINHA;

SemaphoreHandle_t i2cMutex = NULL;
TaskHandle_t taskCorHandle = NULL;

// ---------- Comunicacao (escrito no callback, lido no loop) ----------
uint8_t  ctrlMac[6] = {0};
volatile bool ctrlConhecido = false;
volatile bool pendingAnnounce = false;
uint32_t ultimoAnnounceMs = 0, ultimoTelemMs = 0;

volatile uint8_t  cmdAtual = CMD_STOP;
volatile uint8_t  velAtual = 0;
volatile uint32_t ultimoCmdMs = 0;
volatile bool     pedidoParar = false;
volatile bool     novoPulso = false;
PacotePulsos      pulsoPendente;
volatile bool     novoLed = false;
PacoteLed         ledPendente;
volatile uint8_t  garraPendente = 0xFF;   // 0xFF = nenhum
volatile bool     obrIniciarPedido = false;
volatile bool     obrPararPedido = false;
volatile bool     obrAtivo = false;

// ---------- Estado dos pulsos ----------
bool     pulsoAtivo = false;
uint32_t pulsoInicioMs = 0;
uint8_t  estadoPulso = PULSO_OCIOSO;

// =====================================================================
//  MULTIPLEXADOR E SENSORES
// =====================================================================
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// Leitura de cor protegida por mutex (mesma logica do obr010)
bool lerCorSeguro(uint8_t canal, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  tcaSelect(canal);
  delay(t);
  tcs.getRawData(&r, &g, &b, &c);
  xSemaphoreGive(i2cMutex);
  return true;
}

// Filtros de cor (copiados do obr010.ino)
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 105) return false;
  float L = (r + g + b);
  return (g > L * 0.38 && r < L * 0.33 && b < L * 0.28);
}

bool ehVrmlh(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 80) return false;
  float L = (r + g + b);
  return (g < L * 0.37 && r > L * 0.48);
}

uint8_t classificaCor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (ehVerde(r, g, b, c)) return CLASSE_VERDE;
  if (ehVrmlh(r, g, b, c)) return CLASSE_VERMELHO;
  if (c < lP)              return CLASSE_PRETO;
  return CLASSE_NADA;
}

void lerCorManual(uint8_t idx) {
  uint16_t r, g, b, c;
  uint8_t canal = (idx == 0) ? CANAL_COR_ESQ : CANAL_COR_DIR;
  if (!lerCorSeguro(canal, r, g, b, c)) return;
  corRaw[idx][0] = r; corRaw[idx][1] = g; corRaw[idx][2] = b; corRaw[idx][3] = c;
  classeCor[idx] = classificaCor(r, g, b, c);
}

// Ultrassonico (copiado do obr010.ino)
float lerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracao = pulseIn(echoPin, HIGH, 30000);
  if (duracao == 0) return -1.0;
  return (duracao * 0.0333) / 2;   // cm
}

void lerUltra(uint8_t i) {
  float d = lerDistancia(TRIG_US[i], ECHO_US[i]);
  ultraMM[i] = (d > 0) ? (uint16_t)(d * 10) : SEM_VALOR;
}

// VL53L0X (copiado do obr010.ino, com mutex)
int lerDistanciaVLX(uint8_t canal, Adafruit_VL53L0X &sensor) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return -1;
  tcaSelect(canal);
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  xSemaphoreGive(i2cMutex);
  if (measure.RangeStatus != 4) return measure.RangeMilliMeter;
  return -1;
}

void lerVlx(uint8_t i) {
  if (!vlxOk[i]) { vlxMM[i] = SEM_VALOR; return; }
  Adafruit_VL53L0X &s = (i == 0) ? loxFrente : (i == 1 ? loxDireito : loxEsquerdo);
  uint8_t canal = (i == 0) ? CANAL_VLX_FRENTE : (i == 1 ? CANAL_VLX_DIREITO : CANAL_VLX_ESQUERDO);
  int d = lerDistanciaVLX(canal, s);
  vlxMM[i] = (d >= 0) ? (uint16_t)d : SEM_VALOR;
}

// Uma leitura por chamada (round-robin), so no modo manual
void lerSensoresManual() {
  static uint8_t passo = 0;
  switch (passo) {
    case 0: if (corOk[0]) lerCorManual(0); break;
    case 1: if (corOk[1]) lerCorManual(1); break;
    case 2: lerUltra(0); break;
    case 3: lerUltra(1); break;
    case 4: lerUltra(2); break;
    case 5: lerVlx(0); break;
    case 6: lerVlx(1); break;
    case 7: lerVlx(2); break;
  }
  passo = (passo + 1) % 8;
}

// IR (IReight): copia para sensorValues[] (mesma funcao do obr010)
void atualizarSensores() {
  recv_data();
  if (g_new_package_flag == 1) {
    g_new_package_flag = 0;
    Deal_Usart_AData();
    for (int i = 0; i < 8; i++) sensorValues[i] = IR_Data_Anglo[i];
  }
}

int calcularPesoEsquerda() {
  int soma = 0;
  for (int i = 0; i < 4; i++) if (sensorValues[i] >= limiar) soma += (4 - i);
  return soma;
}

int calcularPesoDireita() {
  int soma = 0;
  for (int i = 4; i < 8; i++) if (sensorValues[i] >= limiar) soma += (i - 3);
  return soma;
}

bool detectarPrata() {
  for (int i = 0; i < 8; i++) if (sensorValues[i] >= 220) return false;
  return true;
}

// =====================================================================
//  MOTORES
// =====================================================================
// velEsq / velDir positivos = frente, negativos = re
void controleM(int velEsq, int velDir) {
  velEsq = constrain(velEsq, -255, 255);
  velDir = constrain(velDir, -255, 255);
  if (velEsq >= 0) { ledcWrite(40, velEsq); ledcWrite(39, 0); }
  else             { ledcWrite(40, 0);      ledcWrite(39, -velEsq); }
  if (velDir >= 0) { ledcWrite(42, velDir); ledcWrite(41, 0); }
  else             { ledcWrite(42, 0);      ledcWrite(41, -velDir); }
}

void pararMotores() {
  controleM(0, 0);
  mEsq.stop();
  mDir.stop();
}

// Comando continuo do controle (modo manual)
void aplicaComando(uint8_t cmd, int v) {
  v = constrain(v, 0, 255);
  switch (cmd) {
    case CMD_FRENTE:    controleM( v,  v); break;
    case CMD_TRAS:      controleM(-v, -v); break;
    case CMD_PIVO_ESQ:  controleM(-v,  v); break;
    case CMD_PIVO_DIR:  controleM( v, -v); break;
    case CMD_CURVA_ESQ: controleM( v / 3, v); break;
    case CMD_CURVA_DIR: controleM( v, v / 3); break;
    default:            controleM(0, 0); break;
  }
}

// Pulsos (modo manual)
void iniciaPulsos(uint8_t d, uint16_t p, uint8_t v) {
  if (p == 0) return;
  long s = p;
  switch (d) {
    case DIR_FRENTE:   mEsq.move( s, v); mDir.move( s, v); break;
    case DIR_TRAS:     mEsq.move(-s, v); mDir.move(-s, v); break;
    case DIR_GIRO_ESQ: mEsq.move(-s, v); mDir.move( s, v); break;
    case DIR_GIRO_DIR: mEsq.move( s, v); mDir.move(-s, v); break;
    default: return;
  }
  pulsoAtivo = true;
  pulsoInicioMs = millis();
  estadoPulso = PULSO_EXECUTANDO;
}

void abortaPulsos() {
  mEsq.stop();
  mDir.stop();
  pulsoAtivo = false;
}

void atualizaPulsos() {
  mEsq.update();
  mDir.update();
  if (!mEsq.isMoving() && !mDir.isMoving()) {
    pulsoAtivo = false;
    estadoPulso = PULSO_CONCLUIDO;
  } else if (millis() - pulsoInicioMs > TIMEOUT_PULSOS_MS) {
    abortaPulsos();
    estadoPulso = PULSO_TIMEOUT;
  }
}

// =====================================================================
//  GARRA (PCA9685 no canal 3 do TCA9548A) - macros do servosEsquemas05
// =====================================================================
bool pcaPresente() {
  tcaSelect(CANAL_PCA);
  Wire.beginTransmission(PCA_ADDR);
  return Wire.endTransmission() == 0;
}

// Verifica a PCA a cada INTERVALO_PCA_MS; inicializa quando aparece
void atualizaPCA(bool forca) {
  if (!forca && millis() - ultimoPcaMs < INTERVALO_PCA_MS) return;
  ultimoPcaMs = millis();
  bool presente = pcaPresente();
  if (presente && !pcaOk) {
    tcaSelect(CANAL_PCA);
    pca.begin();
    pca.setOscillatorFrequency(27000000);
    pca.setPWMFreq(50);
    delay(10);
    Serial.println("[GARRA] PCA9685 conectada");
  }
  if (!presente && pcaOk) Serial.println("[GARRA] PCA9685 desconectada");
  pcaOk = presente;
}

void moverServo(uint8_t numServo, int angulo) {
  angulo = constrain(angulo, 0, 180);
  int pulse = map(angulo, 0, 180, SERVOMIN, SERVOMAX);
  tcaSelect(CANAL_PCA);
  pca.setPWM(numServo, 0, pulse);
}

void Pneutra() {
  moverServo(1, 90);
  moverServo(2, 90);
  delay(500);
  moverServo(0, 2);
}

void Pcoleta() {
  moverServo(1, 135);
  moverServo(2, 35);
  delay(300);
  moverServo(0, 109);
}

void DerrubaDireita() {
  delay(1000);
  moverServo(1, 135);
  moverServo(2, 95);
  delay(1000);
}

void DerrubaEsquerda() {
  delay(1000);
  moverServo(2, 30);
  moverServo(1, 80);
  delay(1000);
}

void AbaixaColeta() {
  Pcoleta();
  delay(300);
  mEsq.move(300, VEL_GARRA);
  mDir.move(300, VEL_GARRA);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  Pneutra();
}

// Executa um comando de garra (bloqueante: STOP so e lido depois)
void executaGarra(uint8_t cod) {
  if (!pcaOk) { Serial.println("[GARRA] PCA9685 nao conectada"); return; }
  if (pulsoAtivo) return;
  switch (cod) {
    case GARRA_ABAIXA:       AbaixaColeta();    break;
    case GARRA_DERRUBA_DIR:  DerrubaDireita();  break;
    case GARRA_DERRUBA_ESQ:  DerrubaEsquerda(); break;
    case GARRA_PCOLETA:      Pcoleta();         break;
    case GARRA_PNEUTRA:      Pneutra();         break;
    default: break;
  }
  controleM(0, 0);
}

// =====================================================================
//  LEDs RGB (pixels 0-3 = direita, 4-7 = esquerda)
// =====================================================================
void rgbEsq(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 4; i < NUMPIXELS; i++) led1.setPixelColor(i, led1.Color(r, g, b));
  led1.show();
}

void rgbDir(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < 4; i++) led1.setPixelColor(i, led1.Color(r, g, b));
  led1.show();
}

void aplicaLed(const PacoteLed &p) {
  if (p.lado == LED_ESQ || p.lado == LED_AMBOS) rgbEsq(p.r, p.g, p.b);
  if (p.lado == LED_DIR || p.lado == LED_AMBOS) rgbDir(p.r, p.g, p.b);
}

// =====================================================================
//  PROGRAMA OBR (port do obr010.ino)
// =====================================================================
void alinhamentoFC() {
  int estadoFC1 = digitalRead(fimC1);
  int estadoFC2 = digitalRead(fimC2);
  controleM(-velMedia, -velMedia);
  while (estadoFC1 || estadoFC2) {
    estadoFC1 = digitalRead(fimC1);
    estadoFC2 = digitalRead(fimC2);
  }
}

void alinhamento() {
  unsigned long tempoAnterior = millis();
  while (1) {
    if ((millis() - tempoAnterior) >= 10000) break;
    atualizarSensores();
    int todosAtivos = 0;
    for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) todosAtivos++;
    if (todosAtivos >= 6) { controleM(0, 0); break; }
    int pesoEsq = calcularPesoEsquerda();
    int pesoDir = calcularPesoDireita();
    if (pesoEsq == pesoDir)      controleM(-100, -100);
    else if (pesoDir > pesoEsq)  controleM(-180, 180);
    else                         controleM(180, -180);
  }
}

void alinhamentoF() {
  mEsq.move(80, velBusca);
  mDir.move(80, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  while (1) {
    atualizarSensores();
    int todosAtivos = 0;
    for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) todosAtivos++;
    if (todosAtivos >= 6) { controleM(0, 0); break; }
    int pesoEsq = calcularPesoEsquerda();
    int pesoDir = calcularPesoDireita();
    if (pesoEsq == pesoDir)      controleM(110, 110);
    else if (pesoDir > pesoEsq)  controleM(180, -180);
    else                         controleM(-180, 180);
  }
}

void executarSegueLinhaInterno() {
  int ativosEsq = 0, ativosDir = 0;
  for (int i = 0; i < 4; i++) if (sensorValues[i] >= limiar) ativosEsq += (4 - i);
  for (int i = 4; i < 8; i++) if (sensorValues[i] >= limiar) ativosDir += (i - 3);
  if (ativosEsq >= 6 || ativosDir >= 6) {
    for (int i = 0; i < 3; i++) if (sensorValues[i] >= limiar) ladoPrioridade = 'E';
    for (int i = 5; i < 8; i++) if (sensorValues[i] >= limiar) ladoPrioridade = 'D';
    e = 0;
  } else {
    e++;
    if (e >= nE) ladoPrioridade = 'F';
  }
  int pesoEsq = calcularPesoEsquerda();
  int pesoDir = calcularPesoDireita();
  if (pesoEsq == pesoDir || (sensorValues[4] >= limiar && sensorValues[5] >= limiar)) {
    if (ativosEsq >= 10 && ativosDir >= 10) encruzilhada();
    int maxLeituraSensor = 4095;
    int velMinimaCorrecao = velMedia - 30;
    int velMaximaCorrecao = velMedia + 50;
    int velAlinhamentoDir = map(sensorValues[4], 600, maxLeituraSensor, velMinimaCorrecao, velMaximaCorrecao);
    int velAlinhamentoEsq = map(sensorValues[5], 600, maxLeituraSensor, velMinimaCorrecao, velMaximaCorrecao);
    velAlinhamentoDir = constrain(velAlinhamentoDir, velMinimaCorrecao, velMaximaCorrecao);
    velAlinhamentoEsq = constrain(velAlinhamentoEsq, velMinimaCorrecao, velMaximaCorrecao);
    controleM(velAlinhamentoEsq, velAlinhamentoDir);
  } else if (pesoEsq > pesoDir) {
    int diff = pesoEsq - pesoDir;
    if (diff < 3)      controleM(130, 255);
    else if (diff < 5) controleM(-130, 130);
    else               controleM(-255, 255);
  } else {
    int diff = pesoDir - pesoEsq;
    if (diff < 3)      controleM(255, 130);
    else if (diff < 5) controleM(200, -200);
    else               controleM(255, -255);
  }
}

// Uma iteracao do modo SEGUE LINHA. Retorna true se detectar prata.
bool executarSegueLinha() {
  float distFrontal = lerDistancia(TRIG_1, ECHO_1);
  if (distFrontal > 0 && distFrontal < d1) {
    leiturasObstaculo++;
    if (leiturasObstaculo <= nL && distFrontal < d2 - 1) {
      mEsq.move(-150, velBusca);
      mDir.move(-150, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    }
    if (leiturasObstaculo >= nL && distFrontal < d2) {
      leiturasObstaculo = 0;
      desvioObstaculo();
    }
  } else {
    leiturasObstaculo = 0;
  }

  if (comandoVerde != 0) executarDesvioVerde(comandoVerde);

  atualizarSensores();

  if (detectarPrata()) {
    pararMotores();
    Serial.println("PRATA detectada! Indo para SERVOS_ESQUEMAS.");
    return true;
  }

  int todosAtivos = 0;
  for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) todosAtivos++;

  if (todosAtivos >= 6) {
    // Interseccao: verifica cores
    controleM(velLenta, velLenta);
    rgbEsq(255, 255, 255);
    rgbDir(255, 255, 255);
    bool verdeS1 = false, verdeS2 = false;
    bool linhaS1 = false, linhaS2 = false;
    while (1) {
      uint16_t r1, g1, b1, c1, r2, g2, b2, c2;
      lerCorSeguro(CANAL_COR_ESQ, r1, g1, b1, c1);
      lerCorSeguro(CANAL_COR_DIR, r2, g2, b2, c2);
      if (ehVerde(r1, g1, b1, c1) || comandoVerde == 1) verdeS1 = true;
      if (ehVerde(r2, g2, b2, c2) || comandoVerde == 2) verdeS2 = true;
      if (c1 < lP) linhaS1 = true;
      if (c2 < lP) linhaS2 = true;
      if (verdeS1 && verdeS2) break;
      if (verdeS1 || verdeS2 || linhaS1 || linhaS2) {
        delay(20);
        lerCorSeguro(CANAL_COR_ESQ, r1, g1, b1, c1);
        lerCorSeguro(CANAL_COR_DIR, r2, g2, b2, c2);
        if (ehVerde(r1, g1, b1, c1) || comandoVerde == 1) verdeS1 = true;
        if (ehVerde(r2, g2, b2, c2) || comandoVerde == 2) verdeS2 = true;
        if (c1 < lP) linhaS1 = true;
        if (c2 < lP) linhaS2 = true;
        break;
      }
      controleM(velLenta, velLenta);
    }
    if (verdeS1 && verdeS2) {
      comandoVerde = 1;
      tentativas = 0;
    } else if (((verdeS1 != verdeS2) || (!verdeS1 && !verdeS2 && linhaS1 && linhaS2)) && tentativas <= nT) {
      mEsq.move(-250, velBusca);
      mDir.move(-250, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      n1 = 0; n2 = 0; comandoVerde = 0;
      pararMotores();
      tentativas++;
    } else if ((verdeS1 != verdeS2) && tentativas >= nT) {
      comandoVerde = verdeS1 ? 1 : 2;
      tentativas = 0;
    } else if (!verdeS1 && !verdeS2 && linhaS1 && linhaS2 && tentativas >= nT) {
      mEsq.move(250, velBusca);
      mDir.move(250, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      n1 = 0; n2 = 0; comandoVerde = 0;
      pararMotores();
      tentativas = 0;
    }
    rgbEsq(0, 0, 0);
    rgbDir(0, 0, 0);
  } else {
    int totalAtivos = 0;
    for (int i = 2; i < 7; i++) if (sensorValues[i] >= limiar) totalAtivos++;
    if (totalAtivos == 0) {
      mEsq.move(Va, velLenta);
      mDir.move(Va, velLenta);
      bool encontrou = false;
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        if (comandoVerde != 0) { encontrou = true; break; }
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return true; }
        int ativos = 0;
        for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiar) ativos++;
        if (ativos > 0) { encontrou = true; break; }
      }
      pararMotores();
      if (!encontrou) encontrarLinha();
    } else {
      executarSegueLinhaInterno();
    }
  }
  return false;
}

// -------- Modo SERVOS ESQUEMAS --------
void inicializarServos() {
  rgbDir(0, 0, 255);
  rgbEsq(0, 0, 255);

  mEsq.move(1150, velBusca);
  mDir.move(1150, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  Terminou = false;
  distLateralAnterior = -1;

  int distDireita = -1, distEsquerda = -1;
  for (int i = 0; i < 6 && (distDireita == -1 || distEsquerda == -1); i++) {
    int d = lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);
    if (d != -1) distDireita = d;
    int ee = lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo);
    if (ee != -1) distEsquerda = ee;
    delay(10);
  }
  if (distEsquerda <= distDireita && distEsquerda != -1) {
    ladoSeguido = LADO_ESQUERDA;
  } else {
    ladoSeguido = LADO_DIREITA;
    dir = -1;
  }

  mEsq.move( PG * dir, velBusca);
  mDir.move(-PG * dir, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  pararMotores();
  delay(100);
  alinhamentoFC();
  mEsq.move(1000, velBusca);
  mDir.move(1000, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  delay(200);
  alinhamentoF();
  ultimoLoopServos = millis();
}

void desviarFrontal() {
  Serial.println("[SERVOS] Desvio frontal!");
  if (ladoSeguido == LADO_ESQUERDA) {
    mEsq.move( PULSOS_GIRO_DESVIO, velManobra);
    mDir.move(-PULSOS_GIRO_DESVIO, velManobra);
  } else {
    mEsq.move(-PULSOS_GIRO_DESVIO, velManobra);
    mDir.move( PULSOS_GIRO_DESVIO, velManobra);
  }
  int distAtual = (ladoSeguido == LADO_ESQUERDA)
                    ? lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo)
                    : lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);
  int distAnterior = distAtual;
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update();
    mDir.update();
    distAtual = (ladoSeguido == LADO_ESQUERDA)
                  ? lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo)
                  : lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);
    int variacao = distAtual - distAnterior;
    if (variacao >= DELTA_DIST_ABRUPTO_MM) break;
    distAnterior = distAtual;
  }
}

void executarIteracaoServos() {
  if (Terminou) return;
  if (millis() - ultimoLoopServos < PERIODO_LOOP_MS) return;
  ultimoLoopServos = millis();

  atualizarSensores();
  int sensoresComLimiarD = 0;
  for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) sensoresComLimiarD++;
  if (sensoresComLimiarD >= 2) {
    pararMotores();
    Serial.println("[SERVOS] Refletancia limiarD detectada -> Terminou");
    Terminou = true;
    return;
  }

  int distFrente = lerDistanciaVLX(CANAL_VLX_FRENTE, loxFrente);
  if (distFrente != -1 && distFrente < DIST_FRONTAL_MIN_MM) {
    desviarFrontal();
    return;
  }

  int distAtual = (ladoSeguido == LADO_ESQUERDA)
                    ? lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo)
                    : lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);

  if (distAtual != -1) {
    if (distLateralAnterior != -1) {
      int variacao = distAtual - distLateralAnterior;
      if (variacao >= DELTA_DIST_ABRUPTO_MM) {
        Serial.println("[SERVOS] Aumento abrupto! Manobra final...");
        mEsq.move( PG * dir, velBusca);
        mDir.move(-PG * dir, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        mEsq.move( PG * dir + 200 * dir, velBusca);
        mDir.move(-PG * dir + 200 * dir, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        alinhamentoFC();
        mEsq.move( PG * dir, velBusca);
        mDir.move(-PG * dir, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        mEsq.move(-400, velBusca);
        mDir.move(-400, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        alinhamentoF();
        pararMotores();
        Serial.println("[SERVOS] Manobra final concluida -> Terminou");
        Terminou = true;
        return;
      }
    }
    distLateralAnterior = distAtual;
  }

  controleM(velBase, velBase);
}

// -------- Busca de linha / desvios --------
void encontrarLinha() {
  if (ladoPrioridade == 'F') {
    mEsq.move(130, 130);
    mDir.move(130, 130);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    mEsq.move(600, velBusca);
    mDir.move(600, velBusca);
    achou = false;
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return; }
      int ativos = 0;
      for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) {
        delay(t);
        atualizarSensores();
        ativos = 0;
        for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA) { achou = true; break; }
      }
    }
    if (!achou) {
      mEsq.move(-1000, velLenta);
      mDir.move(-1000, velLenta);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return; }
        int ativos = 0;
        for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA) {
          delay(t);
          atualizarSensores();
          ativos = 0;
          for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
          if (ativos >= N_LINHA) break;
        }
      }
      mEsq.move(-150, 130);
      mDir.move(-150, 130);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    }
    ladoPrioridade = 'E';
  } else if (ladoPrioridade == 'E') {
    if (tentarGirarEBuscar('E', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('D', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('D', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('E', PG)) return;
  } else {
    if (tentarGirarEBuscar('D', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('E', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('E', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('D', PG)) return;
  }
  pararMotores();
}

bool tentarGirarEBuscar(char direcao, long pulsos) {
  if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move( pulsos, velBusca); }
  else                { mEsq.move( pulsos, velBusca); mDir.move(-pulsos, velBusca); }
  bool achouLinha = false;
  if (direcao == 'E') {
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return false; }
      int ativos = 0;
      for (int i = 0; i < 4; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { achouLinha = true; break; }
    }
  } else {
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return false; }
      int ativos = 0;
      for (int i = 4; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { achouLinha = true; break; }
    }
  }
  if (achouLinha) {
    if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move( pulsos, velBusca); }
    else                { mEsq.move( pulsos, velBusca); mDir.move(-pulsos, velBusca); }
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return false; }
      int ativos = 0;
      for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { pararMotores(); return true; }
    }
  }
  return false;
}

void executarDesvioVerde(int lado) {
  uint16_t r, g, b, c;
  int canalSensor = (lado == 1) ? CANAL_COR_ESQ : CANAL_COR_DIR;
  int canalOposto = (lado == 1) ? CANAL_COR_DIR : CANAL_COR_ESQ;
  int limiarC = 240;
  bool duploVerde = false;
  mEsq.move(100, velLenta);
  mDir.move(100, velLenta);
  bool detectouPreto = false;
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    lerCorSeguro(canalOposto, r, g, b, c);
    if (ehVerde(r, g, b, c)) {
      lerCorSeguro(canalOposto, r, g, b, c);
      if (ehVerde(r, g, b, c)) duploVerde = true;
    }
    lerCorSeguro(canalSensor, r, g, b, c);
    if (c < limiarC) { detectouPreto = true; break; }
  }
  pararMotores();
  delay(50);
  lerCorSeguro(canalOposto, r, g, b, c);
  if (ehVerde(r, g, b, c)) {
    lerCorSeguro(canalOposto, r, g, b, c);
    if (ehVerde(r, g, b, c)) duploVerde = true;
  }
  if (!detectouPreto) { n1 = 0; n2 = 0; comandoVerde = 0; return; }

  if (duploVerde) {
    rgbEsq(0, 255, 0); rgbDir(0, 255, 0);
    mEsq.move(100, velBusca); mDir.move(100, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    rgbEsq(0, 0, 0); rgbDir(0, 0, 0);
    mEsq.move(1600, velBusca + TErro); mDir.move(-1600, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    for (int i = 0; i < 2; i++) {
      mEsq.move(1500, velBusca); mDir.move(-1500, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return; }
        int ativos = 0;
        for (int j = 3; j < 5; j++) if (sensorValues[j] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
    }
    mEsq.move(1500, velBusca); mDir.move(-1500, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return; }
      int ativos = 0;
      for (int j = 3; j < 5; j++) if (sensorValues[j] >= limiarD) ativos++;
      if (ativos >= N_LINHA) break;
    }
    mEsq.move(150, velBusca); mDir.move(150, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  } else {
    mEsq.move(-300, 130); mDir.move(-300, 130);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    if (lado == 1) {
      rgbEsq(0, 255, 0); rgbDir(0, 0, 0);
      ladoPrioridade = 'D';
      mEsq.move(-400, velBusca); mDir.move(400, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(50, velBusca); mDir.move(50, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      rgbEsq(0, 0, 0); rgbDir(0, 0, 0);
      mEsq.move(500, velBusca); mDir.move(500, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return; }
        int ativos = 0;
        for (int j = 0; j < 2; j++) if (sensorValues[j] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
      mEsq.move(250, 130); mDir.move(250, 130);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(-700, velBusca); mDir.move(700, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return; }
        int ativos = 0;
        for (int j = 6; j < 8; j++) if (sensorValues[j] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
    } else {
      rgbEsq(0, 0, 0); rgbDir(0, 255, 0);
      ladoPrioridade = 'E';
      mEsq.move(400, velBusca + TErro2); mDir.move(-400, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(50, velBusca + TErro2); mDir.move(50, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      rgbEsq(0, 0, 0); rgbDir(0, 0, 0);
      mEsq.move(500, velBusca + TErro2); mDir.move(500, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return; }
        int ativos = 0;
        for (int j = 6; j < 8; j++) if (sensorValues[j] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
      mEsq.move(250, 130); mDir.move(250, 130);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(700, velBusca + TErro2); mDir.move(-700, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        if (detectarPrata()) { pararMotores(); return; }
        int ativos = 0;
        for (int j = 0; j < 2; j++) if (sensorValues[j] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
    }
    mEsq.move(-100, velBusca); mDir.move(-100, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  }
  pararMotores();
  delay(10);
  n1 = 0; n2 = 0; comandoVerde = 0;
}

// Versao do obr010 sem while(1): se vermelho, apenas retorna (loop principal para)
void desvioObstaculo() {
  if (vermelho) { pararMotores(); return; }
  rgbEsq(255, 0, 255); rgbDir(255, 0, 255);
  pararMotores();
  delay(300);
  rgbEsq(0, 0, 0); rgbDir(0, 0, 0);
  Serial.println("Obstaculo detectado! Desviando...");
  mEsq.move(-PG * direction, velBusca); mDir.move( PG * direction, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  mEsq.move(-150, velBusca); mDir.move(-150, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  alinhamento();
  desvio();
  alinhamentoF();
  mEsq.move(150, velBusca); mDir.move(150, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  alinhamento();
  mEsq.move(350, velBusca); mDir.move(350, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  mEsq.move(-PG * direction, velBusca); mDir.move( PG * direction, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  mEsq.move(-PG * direction, velBusca); mDir.move( PG * direction, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    atualizarSensores();
    if (detectarPrata()) { pararMotores(); return; }
    int ativos = 0;
    for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
    if (ativos >= N_LINHA) { delay(t2); atualizarSensores(); if (ativos >= N_LINHA) break; }
  }
  mEsq.move(-100, velBusca); mDir.move(-100, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  mEsq.move(-500, velBusca); mDir.move(-500, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    atualizarSensores();
    if (detectarPrata()) { pararMotores(); return; }
    int ativos = 0;
    for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
    if (ativos >= N_LINHA) { delay(t); atualizarSensores(); if (ativos >= N_LINHA) break; }
  }
  pararMotores();
  delay(10);
  n1 = 0; n2 = 0; comandoVerde = 0;
}

void desvio() {
  av = av1;
  for (int i = 0; i < 2; i++) {
    int leituras = 0;
    controleM(velMedia, velMedia);
    while (leituras < nL2) {
      float dist = (direction == 1) ? lerDistancia(TRIG_2, ECHO_2) : lerDistancia(TRIG_3, ECHO_3);
      if (dist > 0 && dist <= d3) leituras++;
      else leituras = 0;
      delay(15);
    }
    leituras = 0;
    controleM(velMedia, velMedia);
    while (leituras < nL2) {
      float dist = (direction == 1) ? lerDistancia(TRIG_2, ECHO_2) : lerDistancia(TRIG_3, ECHO_3);
      if (dist > d3 || dist < 0) leituras++;
      else leituras = 0;
      delay(15);
    }
    mEsq.move(av, velMedia); mDir.move(av, velMedia);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    mEsq.move( PG * direction, velBusca); mDir.move(-PG * direction, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    av = av2;
  }
  ladoPrioridade = 'E';
}

void encruzilhada() {
  mEsq.move(Va, velLenta);
  mDir.move(Va, velLenta);
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    if (comandoVerde != 0) break;
    atualizarSensores();
    if (detectarPrata()) { pararMotores(); return; }
    int ativos = 0;
    for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiarD) ativos++;
    if (ativos > 0) break;
  }
}

// Task de cor (nucleo 0): so existe depois do primeiro MSG_OBR iniciar
void TaskContadorVerde(void *pvParameters) {
  static int consecutivosEsq = 0;
  static int consecutivosDir = 0;
  for (;;) {
    uint16_t r, g, b, c;

    if (lerCorSeguro(CANAL_COR_ESQ, r, g, b, c)) {
      if (ehVrmlh(r, g, b, c)) {
        vermelho = true;
        Serial.println("[COR] Vermelho no sensor esquerdo! Parando.");
        for (;;) vTaskDelay(pdMS_TO_TICKS(10));
      }
      if (comandoVerde == 0) {
        if (ehVerde(r, g, b, c)) {
          consecutivosEsq++;
          if (consecutivosEsq >= nv) { n1++; comandoVerde = 1; consecutivosEsq = 0; }
        } else {
          consecutivosEsq = 0;
        }
      }
    }

    if (lerCorSeguro(CANAL_COR_DIR, r, g, b, c)) {
      if (ehVrmlh(r, g, b, c)) {
        vermelho = true;
        Serial.println("[COR] Vermelho no sensor direito! Parando.");
        for (;;) vTaskDelay(pdMS_TO_TICKS(10));
      }
      if (comandoVerde == 0) {
        if (ehVerde(r, g, b, c)) {
          consecutivosDir++;
          if (consecutivosDir >= nv) { n2++; comandoVerde = 2; consecutivosDir = 0; }
        } else {
          consecutivosDir = 0;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =====================================================================
//  INICIO / EXECUCAO DO PROGRAMA OBR
// =====================================================================
void iniciaObr() {
  abortaPulsos();
  controleM(0, 0);
  if (taskCorHandle == NULL) {
    xTaskCreatePinnedToCore(TaskContadorVerde, "TaskCor", 4096, NULL, 1, &taskCorHandle, 0);
  }
  n1 = 0; n2 = 0; comandoVerde = 0;
  vermelho = false;
  tentativas = 1;
  leiturasObstaculo = 0;
  e = 0;
  ladoPrioridade = 'F';
  achou = true;
  Terminou = false;
  distLateralAnterior = -1;
  ultimoLoopServos = 0;
  estadoAtual = SEGUE_LINHA;
  rgbEsq(0, 0, 0);
  rgbDir(0, 0, 0);
  obrAtivo = true;
  Serial.println("OBR iniciado (SEGUE_LINHA)");
}

// Uma iteracao do loop() original do obr010
void obrLoop() {
  if (vermelho) {
    static bool avisou = false;
    pararMotores();
    if (!avisou) { rgbEsq(255, 0, 0); rgbDir(255, 0, 0); avisou = true; }
    return;
  }

  switch (estadoAtual) {
    case SEGUE_LINHA:
      if (executarSegueLinha()) {
        pararMotores();
        inicializarServos();
        estadoAtual = SERVOS_ESQUEMAS;
        Serial.println("Transicao: SEGUE_LINHA -> SERVOS_ESQUEMAS");
      }
      break;

    case SERVOS_ESQUEMAS:
      executarIteracaoServos();
      if (Terminou) {
        pararMotores();
        mEsq.move(700, velBusca);
        mDir.move(700, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        pararMotores();
        estadoAtual = SEGUE_LINHA;
        Terminou = false;
        distLateralAnterior = -1;
        Serial.println("Transicao: SERVOS_ESQUEMAS -> SEGUE_LINHA");
      }
      break;
  }
}

// Modo manual (comandos do controle)
void loopManual() {
  atualizaPCA(false);

  if (novoLed) { novoLed = false; aplicaLed(ledPendente); }

  if (garraPendente != 0xFF) {
    uint8_t cod = garraPendente;
    garraPendente = 0xFF;
    executaGarra(cod);
  }

  if (novoPulso) {
    novoPulso = false;
    if (!pulsoAtivo) iniciaPulsos(pulsoPendente.dir, pulsoPendente.pulsos, pulsoPendente.vel);
  }
  if (pedidoParar) {
    pedidoParar = false;
    if (pulsoAtivo) { abortaPulsos(); estadoPulso = PULSO_PARADO; }
  }

  if (pulsoAtivo) {
    atualizaPulsos();
  } else {
    lerSensoresManual();
    bool sinalOk = (millis() - ultimoCmdMs) < TIMEOUT_SINAL_MS;
    if (sinalOk) aplicaComando(cmdAtual, velAtual);
    else         controleM(0, 0);
  }
}

// =====================================================================
//  ESP-NOW
// =====================================================================
bool ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  p.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&p) == ESP_OK;
}

void sendAnnounce() {
  if (!ctrlConhecido) return;
  ensurePeer(ctrlMac);
  Msg a = {};
  a.type = MSG_ANNOUNCE;
  strncpy(a.name, DEVICE_NAME, 15);
  esp_now_send(ctrlMac, (uint8_t *)&a, sizeof(a));
}

void enviaTelemetria() {
  ensurePeer(ctrlMac);
  Telem tm = {};
  tm.type = MSG_TELEM;
  tm.estadoPulso = estadoPulso;
  tm.okMask = (corOk[0] ? 1 : 0) | (corOk[1] ? 2 : 0) |
              (vlxOk[0] ? 4 : 0) | (vlxOk[1] ? 8 : 0) | (vlxOk[2] ? 16 : 0) |
              (pcaOk ? 32 : 0);
  tm.estado = (obrAtivo ? 1 : 0) | (vermelho ? 2 : 0);
  tm.fim = (digitalRead(fimC1) == LOW ? 1 : 0) | (digitalRead(fimC2) == LOW ? 2 : 0);
  tm.classe[0] = classeCor[0];
  tm.classe[1] = classeCor[1];
  tm.pulsosE = (uint16_t)mEsq.getCount();
  tm.pulsosD = (uint16_t)mDir.getCount();
  memcpy(tm.cor, corRaw, sizeof(corRaw));
  for (int i = 0; i < 8; i++) tm.ir[i] = (sensorValues[i] > 0) ? (uint16_t)sensorValues[i] : 0;
  memcpy(tm.ultra, ultraMM, sizeof(ultraMM));
  memcpy(tm.vlx, vlxMM, sizeof(vlxMM));
  esp_now_send(ctrlMac, (uint8_t *)&tm, sizeof(tm));
}

// Callback do WiFi: so copia dados e marca flags. Motores e garra rodam no loop().
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t tipo = data[0];

  if (tipo == MSG_DISCOVER && len == sizeof(Msg)) {
    memcpy(ctrlMac, info->src_addr, 6);
    ctrlConhecido = true;
    pendingAnnounce = true;
    return;
  }

  // Demais mensagens: registra o controle como remetente
  memcpy(ctrlMac, info->src_addr, 6);
  ctrlConhecido = true;

  if (tipo == MSG_CMD && len == sizeof(Msg)) {
    if (obrAtivo) return;
    Msg m;
    memcpy(&m, data, sizeof(Msg));
    cmdAtual = m.a;
    velAtual = m.b;
    ultimoCmdMs = millis();
    if (m.a == CMD_STOP) pedidoParar = true;
  }
  else if (tipo == MSG_PULSOS && len == sizeof(PacotePulsos)) {
    if (obrAtivo) return;
    memcpy(&pulsoPendente, data, sizeof(PacotePulsos));
    novoPulso = true;
  }
  else if (tipo == MSG_LED && len == sizeof(PacoteLed)) {
    memcpy(&ledPendente, data, sizeof(PacoteLed));
    novoLed = true;
  }
  else if (tipo == MSG_GARRA && len == sizeof(Msg)) {
    if (obrAtivo) return;
    Msg m;
    memcpy(&m, data, sizeof(Msg));
    garraPendente = m.a;
  }
  else if (tipo == MSG_OBR && len == sizeof(Msg)) {
    Msg m;
    memcpy(&m, data, sizeof(Msg));
    if (m.a == OBR_INICIAR) obrIniciarPedido = true;
    else if (obrAtivo)      obrPararPedido = true;
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  led1.begin();
  led1.setBrightness(LED_BRILHO);
  rgbEsq(0, 0, 0);
  rgbDir(0, 0, 0);

  serial_init();                 // IR: Serial1 nos pinos 18/17
  delay(100);
  SET_Eight_Mode(0, 1, 0);

  i2cMutex = xSemaphoreCreateMutex();

  mEsq.begin(isrEsq);
  mDir.begin(isrDir);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  tcaSelect(CANAL_VLX_FRENTE);   delay(10); vlxOk[0] = loxFrente.begin();
  tcaSelect(CANAL_VLX_DIREITO);  delay(10); vlxOk[1] = loxDireito.begin();
  tcaSelect(CANAL_VLX_ESQUERDO); delay(10); vlxOk[2] = loxEsquerdo.begin();

  tcaSelect(CANAL_COR_ESQ); delay(t); corOk[0] = tcs.begin();
  tcaSelect(CANAL_COR_DIR); delay(t); corOk[1] = tcs.begin();

  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_US[i], OUTPUT);
    pinMode(ECHO_US[i], INPUT);
  }

  pinMode(fimC1, INPUT_PULLUP);
  pinMode(fimC2, INPUT_PULLUP);

  atualizaPCA(true);
  Serial.println(pcaOk ? "[GARRA] PCA9685 OK" : "[GARRA] PCA9685 NAO encontrada");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  Serial.print("MAC do robo: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Falha ao iniciar ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("Aguardando controle...");
}

void loop() {
  atualizarSensores();

  // Parar o OBR = reiniciar o robo (limpa o estado do programa)
  if (obrPararPedido) {
    Serial.println("OBR parado: reiniciando");
    delay(50);
    ESP.restart();
  }
  if (obrIniciarPedido) {
    obrIniciarPedido = false;
    if (!obrAtivo) iniciaObr();
  }

  if (pendingAnnounce) {
    pendingAnnounce = false;
    sendAnnounce();
  }
  if (ctrlConhecido && millis() - ultimoAnnounceMs >= INTERVALO_ANNOUNCE_MS) {
    ultimoAnnounceMs = millis();
    sendAnnounce();
  }

  if (obrAtivo) obrLoop();
  else          loopManual();

  if (ctrlConhecido && millis() - ultimoTelemMs >= INTERVALO_TELEM_MS) {
    ultimoTelemMs = millis();
    enviaTelemetria();
  }

  delay(5);
}