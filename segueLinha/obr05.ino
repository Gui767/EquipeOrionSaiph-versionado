#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include "MotorControl.h"
#include "IReight_model.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// =================================================================
//  CONSTANTES E PINAGEM
// =================================================================
#define I2C_SDA 15
#define I2C_SCL 16
#define TCAADDR 0x70

//Fim de curso
const int fimC1 = 9;
const int fimC2 = 10;
// Canais do multiplexador para os VL53L0X
const uint8_t CANAL_VLX_FRENTE   = 2;
const uint8_t CANAL_VLX_DIREITO  = 5;
const uint8_t CANAL_VLX_ESQUERDO = 4;

// Pinos dos ultrassônicos (usados no modo segue linha)
const int TRIG_1 = 12;
const int ECHO_1 = 11;
const int TRIG_2 = 21;
const int ECHO_2 = 47;

// Pino e número de LEDs RGB
#define PIN_LED_1 2
#define NUMPIXELS 8

// =================================================================
//  OBJETOS GLOBAIS
// =================================================================
// Motores
MotorControl mEsq(5, 4, 40, 39);
MotorControl mDir(7, 6, 42, 41);

// Sensores de distância VL53L0X
Adafruit_VL53L0X loxFrente   = Adafruit_VL53L0X();
Adafruit_VL53L0X loxDireito  = Adafruit_VL53L0X();
Adafruit_VL53L0X loxEsquerdo = Adafruit_VL53L0X();

// Sensor de cor TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

// LED RGB
Adafruit_NeoPixel led1(NUMPIXELS, PIN_LED_1, NEO_GRB + NEO_KHZ800);

// =================================================================
//  VARIÁVEIS COMPARTILHADAS (IR)
// =================================================================
int sensorValues[8];
const int limiar     = 800;
const int limiarD    = 1400;   // Limiar para desvio / refletância forte


int dir = 1;

// =================================================================
//  VARIÁVEIS DO MODO SEGUE LINHA
// =================================================================
const int N_LINHA = 1;
int t = 1;
int t2 = 3;
int nv = 2;
int TErro = 0, TErro1 = 0, TErro2 = 0;
int Va = 300;
int lP = 200;           // Limiar de preto para o sensor de cor
int nT = 2;
int d1 = 10, d2 = 5, d3 = 15;
int tentativas = 1;
const int nL = 30;      // Leituras consecutivas para obstáculo
int nL2 = 5;
int leiturasObstaculo = 0;
int av = 0, av1 = 320, av2 = 800;
int PG = 1150;          // Pulso de giro
int velMedia = 200;
int velBusca = 250;
int velLenta = 100;

// Controle de cor
volatile int n1 = 0, n2 = 0, comandoVerde = 0;
int reta = 0, nR = 10, e = 0, nE = 50;
char ladoPrioridade = 'E';
bool achou = true;
volatile bool vermelho = false;   // true = vermelho detectado -> parada absoluta (igual ao 046 original)

// Canais do multiplexador usados pelo sensor de cor (mesma nomenclatura do 046 original)
const uint8_t CANAL_COR_ESQ = 0;
const uint8_t CANAL_COR_DIR = 1;

// =================================================================
//  VARIÁVEIS DO MODO SERVOS ESQUEMAS
// =================================================================
bool Terminou = false;
int distLateralAnterior = -1;
enum LadoParede { LADO_ESQUERDA, LADO_DIREITA };
LadoParede ladoSeguido;

const int DIST_FRONTAL_MIN_MM  = 170;
const long PULSOS_GIRO_DESVIO   = 500;
const long PULSOS_GIRO_CONTRARIO = 1500;
const int DELTA_DIST_ABRUPTO_MM = 100;
const int velManobra = 255;
const int velBase    = 255;
const unsigned long PERIODO_LOOP_MS = 20;
unsigned long ultimoLoopServos = 0;

// =================================================================
//  MÁQUINA DE ESTADOS
// =================================================================
enum State {
  SEGUE_LINHA,
  SERVOS_ESQUEMAS
};
State estadoAtual = SEGUE_LINHA;

// =================================================================
//  DUAL-CORE: LEITURA DE VERDE/VERMELHO (NÚCLEO 0) - lógica do 046
// =================================================================
// O TCS34725 (cor) e os VL53L0X (distância) estão atrás do mesmo
// multiplexador TCA9548A, no mesmo barramento I2C. Como a leitura de
// cor passa a rodar em paralelo, no núcleo 0, todo acesso ao barramento
// (tanto o novo, quanto o já existente em lerDistanciaVLX e nas leituras
// de cor que continuam no núcleo 1) precisa ser protegido por um mutex.
SemaphoreHandle_t i2cMutex = NULL;
TaskHandle_t taskCorHandle = NULL;
void TaskContadorVerde(void *pvParameters);
bool lerCorSeguro(uint8_t canal, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c);

// =================================================================
//  PROTÓTIPOS DE FUNÇÕES
// =================================================================
// IR
void atualizarSensores();
int calcularPesoEsquerda();
int calcularPesoDireita();

// Motores
void controleM(int velEsq, int velDir);
void pararMotores();

// Alinhamentos (unificados)
void alinhamento();
void alinhamentoF();

// Segue linha
bool executarSegueLinha();          // retorna true se detectar "prata"
void encontrarLinha();
bool tentarGirarEBuscar(char direcao, long pulsos);
void executarDesvioVerde(int lado);
void desvioObstaculo();
void desvio();
void encruzilhada();
bool detectarPrata();               // todos os sensores < 220

// Servos esquemas
void inicializarServos();
void executarIteracaoServos();      // executa uma iteração e atualiza Terminou
void desviarFrontal();

// Sensores VL53L0X
void tcaSelect(uint8_t channel);
int lerDistanciaVLX(uint8_t canal, Adafruit_VL53L0X &sensor);

// Ultrassom
float lerDistancia(int trigPin, int echoPin);

// RGB
void rgbEsq(uint8_t r, uint8_t g, uint8_t b);
void rgbDir(uint8_t r, uint8_t g, uint8_t b);

// Cor
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
bool ehVrmlh(uint16_t r, uint16_t g, uint16_t b, uint16_t c);

// Interrupções
void IRAM_ATTR isrEsq() { mEsq.handleInterrupt(); }
void IRAM_ATTR isrDir() { mDir.handleInterrupt(); }

// =================================================================
//  IMPLEMENTAÇÃO DAS FUNÇÕES
// =================================================================



void alinhamentoFC(){
  int estadoFC1 = digitalRead(fimC1);
  int estadoFC2 = digitalRead(fimC2);
  controleM(-velMedia, -velMedia);
  while((estadoFC1 || estadoFC2)){
  estadoFC1 = digitalRead(fimC1);
  estadoFC2 = digitalRead(fimC2);
  }
}

// -------------------- IR --------------------
void atualizarSensores() {
  recv_data();
  if (g_new_package_flag == 1) {
    g_new_package_flag = 0;
    Deal_Usart_AData();
    for (int i = 0; i < 8; i++) {
      sensorValues[i] = IR_Data_Anglo[i];
    }
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

// -------------------- Motores --------------------
void controleM(int velEsq, int velDir) {
  velEsq = constrain(velEsq, -255, 255);
  velDir = constrain(velDir, -255, 255);
  if (velEsq >= 0) { ledcWrite(40, velEsq); ledcWrite(39, 0); }
  else             { ledcWrite(40, 0); ledcWrite(39, abs(velEsq)); }
  if (velDir >= 0) { ledcWrite(42, velDir); ledcWrite(41, 0); }
  else             { ledcWrite(42, 0); ledcWrite(41, abs(velDir)); }
}

void pararMotores() {
  controleM(0, 0);
  mEsq.stop();
  mDir.stop();
}

// -------------------- Detecção de "Prata" --------------------
bool detectarPrata() {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= 220) return false;
  }
  return true;
}

// -------------------- Alinhamentos (versão sem verificarPrata) --------------------
void alinhamento() {
  while (1) {
    atualizarSensores();
    int todosAtivos = 0;
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] >= limiarD) todosAtivos++;
    }
    if (todosAtivos >= 6) {
      controleM(0, 0);
      break;
    }
    int pesoEsq = calcularPesoEsquerda();
    int pesoDir = calcularPesoDireita();
    if (pesoEsq == pesoDir) {
      controleM(-100, -100);
    } else if (pesoDir > pesoEsq) {
      controleM(-180, 180);
    } else {
      controleM(180, -180);
    }
  }
}

void alinhamentoF() {
  mEsq.move(80, velBusca);
  mDir.move(80, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  while (1) {
    atualizarSensores();
    int todosAtivos = 0;
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] >= limiarD) todosAtivos++;
    }
    if (todosAtivos >= 6) {
      controleM(0, 0);
      break;
    }
    int pesoEsq = calcularPesoEsquerda();
    int pesoDir = calcularPesoDireita();
    if (pesoEsq == pesoDir) {
      controleM(110, 110);
    } else if (pesoDir > pesoEsq) {
      controleM(180, -180);
    } else {
      controleM(-180, 180);
    }
  }
}

// -------------------- Funções do modo SEGUE LINHA --------------------
bool executarSegueLinha() {
  // Verifica obstáculo frontal (ultrassônico)
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

  // Se detectou verde, executa desvio
  if (comandoVerde != 0) {
    executarDesvioVerde(comandoVerde);
  }

  // Atualiza sensores IR
  atualizarSensores();

  // DETECÇÃO DE PRATA (todos os sensores < 220)
  if (detectarPrata()) {
    pararMotores();
    Serial.println("PRATA detectada! Avançando 1000 pulsos e indo para SERVOS_ESQUEMAS.");
    // Avança 1000 pulsos
    
    
    return true;   // transição para SERVOS_ESQUEMAS
  }

  // Lógica de seguir linha
  int todosAtivos = 0;
  for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) todosAtivos++;

  if (todosAtivos >= 6) {
    // Interseção: verifica cores
    controleM(velLenta, velLenta);
    rgbEsq(255, 255, 255);
    rgbDir(255, 255, 255);
    bool verdeS1 = false, verdeS2 = false;
    bool linhaS1 = false, linhaS2 = false;
    while (1) {
      uint16_t r1, g1, b1, c1, r2, g2, b2, c2;
      lerCorSeguro(0, r1, g1, b1, c1);
      lerCorSeguro(1, r2, g2, b2, c2);
      if (ehVerde(r1, g1, b1, c1) || comandoVerde == 1) verdeS1 = true;
      if (ehVerde(r2, g2, b2, c2) || comandoVerde == 2) verdeS2 = true;
      if (c1 < lP) linhaS1 = true;
      if (c2 < lP) linhaS2 = true;
      if (verdeS1 && verdeS2) break;
      if (verdeS1 || verdeS2 || linhaS1 || linhaS2) {
        delay(20);
        lerCorSeguro(0, r1, g1, b1, c1);
        lerCorSeguro(1, r2, g2, b2, c2);
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
        if (detectarPrata()) { pararMotores(); return true; } // verifica prata durante a busca
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
    if (diff < 3)      controleM(-130, 255);
    else if (diff < 5) controleM(-130, 130);
    else               controleM(-255, 255);
  } else {
    int diff = pesoDir - pesoEsq;
    if (diff < 3)      controleM(255, -130);
    else if (diff < 5) controleM(200, -200);
    else               controleM(255, -255);
  }
}

// -------------------- Funções do modo SERVOS ESQUEMAS --------------------
void inicializarServos() {
  rgbDir(0,0,255);   rgbEsq(0,0,255);

  mEsq.move(1000, velBusca);
  mDir.move(1000, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  Terminou = false;
  distLateralAnterior = -1;
  // Define o lado a ser seguido (exemplo: baseado na distância inicial)
  int distDireita = -1, distEsquerda = -1;
  for (int i = 0; i < 6 && (distDireita == -1 || distEsquerda == -1); i++) {
    int d = lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);
    if (d != -1) distDireita = d;
    int e = lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo);
    if (e != -1) distEsquerda = e;
    delay(10);
  }
  if (distEsquerda <= distDireita && distEsquerda != -1) {
    ladoSeguido = LADO_ESQUERDA;
  } else {
    ladoSeguido = LADO_DIREITA;
    dir = -1;
  }
  
  mEsq.move( PG*dir, velBusca);
  mDir.move(-PG*dir, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  pararMotores();
  delay(100);
  alinhamentoFC();
  mEsq.move(300, velBusca);
  mDir.move(300, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  mEsq.move((-PG + 150)*dir, velBusca);
  mDir.move(( PG + 150)*dir, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }

  Serial.print("Lado monitorado: ");
  Serial.println(ladoSeguido == LADO_ESQUERDA ? "ESQUERDA" : "DIREITA");
  ultimoLoopServos = millis();
}

void executarIteracaoServos() {
  if (Terminou) return; // já terminou, aguarda transição

  if (millis() - ultimoLoopServos < PERIODO_LOOP_MS) return;
  ultimoLoopServos = millis();

  // Verifica refletância (2 ou mais sensores com limiarD)
  atualizarSensores();
  int sensoresComLimiarD = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= limiarD) sensoresComLimiarD++;
  }
  if (sensoresComLimiarD >= 2) {
    pararMotores();
    Serial.println("[SERVOS] Refletância limiarD detectada -> Terminou = true");
    Terminou = true;
    return;
  }

  // Obstáculo frontal
  int distFrente = lerDistanciaVLX(CANAL_VLX_FRENTE, loxFrente);
  if (distFrente != -1 && distFrente < DIST_FRONTAL_MIN_MM) {
    desviarFrontal();
    return;
  }

  // Aumento abrupto lateral
  int distAtual = (ladoSeguido == LADO_ESQUERDA)
                    ? lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo)
                    : lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);

  if (distAtual != -1) {
    if (distLateralAnterior != -1) {
      int variacao = distAtual - distLateralAnterior;
      if (variacao >= DELTA_DIST_ABRUPTO_MM) {
        Serial.println("[SERVOS] Aumento abrupto! Executando manobra final...");
        mEsq.move( PG*dir, velBusca);
        mDir.move(-PG*dir, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        mEsq.move( PG*dir + 200*dir, velBusca);
        mDir.move(-PG*dir + 200*dir , velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        
        alinhamentoFC();
        mEsq.move( PG*dir, velBusca);
        mDir.move(-PG*dir, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        mEsq.move(-400, velBusca);
        mDir.move(-400, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        alinhamentoF();
        pararMotores();
        Serial.println("[SERVOS] Manobra final concluída -> Terminou = true");
        Terminou = true;
        return;
      }
    }
    distLateralAnterior = distAtual;
  }

  // Avanço direto
  controleM(velBase, velBase);
}

void desviarFrontal() {
  Serial.println("[SERVOS] Desvio frontal!");
  if (ladoSeguido == LADO_ESQUERDA) {
    mEsq.move(PULSOS_GIRO_DESVIO, velManobra);
    mDir.move(-PULSOS_GIRO_DESVIO, velManobra);
  } else {
    mEsq.move(-PULSOS_GIRO_DESVIO, velManobra);
    mDir.move(PULSOS_GIRO_DESVIO, velManobra);
  }
  int distAtual = (ladoSeguido == LADO_ESQUERDA)
                    ? lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo)
                    : lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);
  int distLateralAnterior = distAtual;
  while (mEsq.isMoving() || mDir.isMoving()) {
     mEsq.update(); 
     mDir.update(); 
     // Aumento abrupto lateral
     distAtual = (ladoSeguido == LADO_ESQUERDA)
                    ? lerDistanciaVLX(CANAL_VLX_ESQUERDO, loxEsquerdo)
                    : lerDistanciaVLX(CANAL_VLX_DIREITO, loxDireito);
     int variacao = distAtual - distLateralAnterior;
      if (variacao >= DELTA_DIST_ABRUPTO_MM) {
        break;
     }
     distLateralAnterior = distAtual;
  }
}

// -------------------- VL53L0X --------------------
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

int lerDistanciaVLX(uint8_t canal, Adafruit_VL53L0X &sensor) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return -1;
  tcaSelect(canal);
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  xSemaphoreGive(i2cMutex);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  }
  return -1;
}

// -------------------- Ultrassom --------------------
float lerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracao = pulseIn(echoPin, HIGH, 30000);
  if (duracao == 0) return -1.0;
  return (duracao * 0.0333) / 2;
}

// -------------------- RGB --------------------
void rgbEsq(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 4; i < NUMPIXELS; i++) led1.setPixelColor(i, led1.Color(r, g, b));
  led1.show();
}
void rgbDir(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < 4; i++) led1.setPixelColor(i, led1.Color(r, g, b));
  led1.show();
}

// -------------------- Cor --------------------
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 100) return false;
  float Lcor = (r + g + b);
  return (g > Lcor * 0.39 && r < Lcor * 0.33 && b < Lcor * 0.27);
}
bool ehVrmlh(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 80) return false;
  float Lcor = (r + g + b);
  return (g < Lcor * 0.40 && r > Lcor * 0.45);
}

// Leitura de cor protegida por mutex: seleciona o canal do TCA9548A e lê
// o TCS34725 sem permitir que outra tarefa mude o canal no meio da leitura.
bool lerCorSeguro(uint8_t canal, uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  tcaSelect(canal);
  delay(t);
  tcs.getRawData(&r, &g, &b, &c);
  xSemaphoreGive(i2cMutex);
  return true;
}


void TaskContadorVerde(void *pvParameters) {
  static int consecutivosEsq = 0;
  static int consecutivosDir = 0;

  for (;;) {
    uint16_t r, g, b, c;

    // --- Sensor esquerdo ---
    if (lerCorSeguro(CANAL_COR_ESQ, r, g, b, c)) {
      if (ehVrmlh(r, g, b, c)) {

        vermelho = true;

        Serial.println("[COR] Vermelho detectado no sensor esquerdo! Parando robo...");
        for (;;) vTaskDelay(pdMS_TO_TICKS(10)); // evita reset por watchdog
      }
      if (comandoVerde == 0) {
        if (ehVerde(r, g, b, c)) {
          consecutivosEsq++;
          if (consecutivosEsq >= nv) {
            n1++;
            comandoVerde = 1;
            consecutivosEsq = 0;
          }
        } else {
          consecutivosEsq = 0;
        }
      }
    }

    // --- Sensor direito ---
    if (lerCorSeguro(CANAL_COR_DIR, r, g, b, c)) {
      if (ehVrmlh(r, g, b, c)) {

        vermelho = true;

        Serial.println("[COR] Vermelho detectado no sensor direito! Parando robo...");
        for (;;) vTaskDelay(pdMS_TO_TICKS(10));
      }
      if (comandoVerde == 0) {
        if (ehVerde(r, g, b, c)) {
          consecutivosDir++;
          if (consecutivosDir >= nv) {
            n2++;
            comandoVerde = 2;
            consecutivosDir = 0;
          }
        } else {
          consecutivosDir = 0;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =================================================================
//  SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Inicializa LED RGB
  led1.begin();
  led1.setBrightness(30);
  rgbEsq(0, 0, 0);
  rgbDir(0, 0, 0);

  // Inicializa sensores IR
  serial_init();
  delay(100);
  SET_Eight_Mode(0, 1, 0);

  // Inicializa motores
  mEsq.begin(isrEsq);
  mDir.begin(isrDir);

  // Mutex que protege todo acesso ao barramento I2C compartilhado
  // (TCA9548A -> TCS34725 e VL53L0X), usado tanto pelo loop principal
  // quanto pela task de cor no núcleo 0.
  i2cMutex = xSemaphoreCreateMutex();

  // Inicializa I2C e sensores VL53L0X (já disponíveis para o modo servos)
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);
  tcaSelect(CANAL_VLX_FRENTE);   delay(10); loxFrente.begin();
  tcaSelect(CANAL_VLX_DIREITO);  delay(10); loxDireito.begin();
  tcaSelect(CANAL_VLX_ESQUERDO); delay(10); loxEsquerdo.begin();

  // Inicializa sensor de cor (usado no modo segue linha)
  Wire.begin(I2C_SDA, I2C_SCL); // já iniciado
  tcaSelect(CANAL_COR_ESQ); delay(t); tcs.begin();
  tcaSelect(CANAL_COR_DIR); delay(t); tcs.begin();

  // Inicializa ultrassônicos
  pinMode(TRIG_1, OUTPUT);
  pinMode(ECHO_1, INPUT);
  pinMode(TRIG_2, OUTPUT);
  pinMode(ECHO_2, INPUT);

  pinMode(fimC1, INPUT_PULLUP);
  pinMode(fimC2, INPUT_PULLUP);

  // Lógica de leitura de verde/vermelho do 046, rodando em segundo plano
  // no núcleo 0, exatamente como no arquivo original (TaskContadorVerde).
  xTaskCreatePinnedToCore(
    TaskContadorVerde,
    "TaskCor",
    4096,
    NULL,
    1,
    &taskCorHandle,
    0   // núcleo 0
  );

  Serial.println("Setup concluído. Estado inicial: SEGUE_LINHA");
}

// =================================================================
//  LOOP PRINCIPAL (MÁQUINA DE ESTADOS)
// =================================================================
void loop() {
  // Parada absoluta por vermelho (lógica do 046): a task do núcleo 0 já
  // trava sozinha ao detectar vermelho, mas o loop principal (núcleo 1)
  // também precisa parar de emitir novos comandos de movimento.
  if (vermelho) {
    rgbEsq(255,0,0); rgbDir(255,0,0); 
    pararMotores();
    for (;;) delay(100);
  }

  switch (estadoAtual) {
    case SEGUE_LINHA: {
      bool deveTransicionar = executarSegueLinha();
      if (deveTransicionar) {
        // Transição para SERVOS_ESQUEMAS
        pararMotores();
        inicializarServos();
        estadoAtual = SERVOS_ESQUEMAS;
        Serial.println("Transição: SEGUE_LINHA -> SERVOS_ESQUEMAS");
      }
      break;
    }

    case SERVOS_ESQUEMAS: {
      executarIteracaoServos();
      if (Terminou) {
        // Transição de volta para SEGUE_LINHA
        pararMotores();
        Serial.println("Terminou no modo SERVOS. Avançando 700 pulsos e voltando para SEGUE_LINHA.");
        mEsq.move(700, velBusca);
        mDir.move(700, velBusca);
        while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
        pararMotores();
        estadoAtual = SEGUE_LINHA;
        Terminou = false;
        distLateralAnterior = -1;
        Serial.println("Transição: SERVOS_ESQUEMAS -> SEGUE_LINHA");
      }
      break;
    }
  }
}

// =================================================================
//  IMPLEMENTAÇÃO DAS FUNÇÕES FALTANTES DO MODO SEGUE LINHA
//  (código extraído do original e adaptado)
// =================================================================

void encontrarLinha() {
  if (ladoPrioridade == 'F') {
    mEsq.move(80, 130);
    mDir.move(80, 130);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    mEsq.move(600, velBusca);
    mDir.move(600, velBusca);
    achou = false;
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return; } // interrompe se prata
      int ativos = 0;
      for (int i = 2; i < 6; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) {
        delay(t);
        atualizarSensores();
        ativos = 0;
        for (int i = 2; i < 6; i++) if (sensorValues[i] >= limiarD) ativos++;
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
  if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move(pulsos, velBusca); }
  else { mEsq.move(pulsos, velBusca); mDir.move(-pulsos, velBusca); }
  bool achou = false;
  if (direcao == 'E') {
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return false; }
      int ativos = 0;
      for (int i = 0; i < 4; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { achou = true; break; }
    }
  } else {
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      if (detectarPrata()) { pararMotores(); return false; }
      int ativos = 0;
      for (int i = 4; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { achou = true; break; }
    }
  }
  if (achou) {
    if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move(pulsos, velBusca); }
    else { mEsq.move(pulsos, velBusca); mDir.move(-pulsos, velBusca); }
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
  int canalSensor = (lado == 1) ? 0 : 1;
  int canalOposto = (lado == 1) ? 1 : 0;
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
    rgbEsq(0,0,0); rgbDir(0,0,0);
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
    while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    }
  } else {
    mEsq.move(-300, 130); mDir.move(-300, 130);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    if (lado == 1) {
      rgbEsq(0,255,0); rgbDir(0,0,0);
      ladoPrioridade = 'D';
      mEsq.move(-400, velBusca); mDir.move(400, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(50, velBusca); mDir.move(50, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      rgbEsq(0,0,0); rgbDir(0,0,0);
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
      rgbEsq(0,0,0); rgbDir(0,255,0);
      ladoPrioridade = 'E';
      mEsq.move(400, velBusca + TErro2); mDir.move(-400, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(50, velBusca + TErro2); mDir.move(50, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      rgbEsq(0,0,0); rgbDir(0,0,0);
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

void desvioObstaculo() {
  if(vermelho) {pararMotores(); while(1) delay(1000);}
  else{
  rgbEsq(255,0,0); rgbDir(255,0,0);
  pararMotores();
  delay(300);
  rgbEsq(0,0,0); rgbDir(0,0,0);
  Serial.println("Obstaculo detectado! Girando para a esquerda...");
  mEsq.move(-PG, velBusca); mDir.move(PG, velBusca);
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
  mEsq.move(-PG, velBusca); mDir.move(PG, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  mEsq.move(-PG, velBusca); mDir.move(PG, velBusca);
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
}

void desvio() {
  av = av1;
  for (int i = 0; i < 2; i++) {
    int leituras = 0;
    controleM(velMedia, velMedia);
    while (leituras < nL2) {
      float dist = lerDistancia(TRIG_2, ECHO_2);
      if (dist > 0 && dist <= d3) leituras++;
      else leituras = 0;
      delay(15);
    }
    leituras = 0;
    controleM(velMedia, velMedia);
    while (leituras < nL2) {
      float dist = lerDistancia(TRIG_2, ECHO_2);
      if (dist > d3 || dist < 0) leituras++;
      else leituras = 0;
      delay(15);
    }
    mEsq.move(av, velMedia); mDir.move(av, velMedia);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    mEsq.move(PG, velBusca); mDir.move(-PG, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    av = av2;
  }
  ladoPrioridade = 'E';
}

void encruzilhada() {
  mEsq.move(Va, velLenta); mDir.move(Va, velLenta);
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
