#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "MotorControl.h"
#include "IReight_model.hpp" 

// --- Definições RGB e Periféricos ---
#define PIN_LED_1 2
#define NUMPIXELS 8

Adafruit_NeoPixel led1(NUMPIXELS, PIN_LED_1, NEO_GRB + NEO_KHZ800);

// --- Configurações de Hardware (Motores e Sensores IR) ---
MotorControl mEsq(5, 4, 40, 39);
MotorControl mDir(7, 6, 42, 41);
int sensorValues[8];
const int limiar = 800;
const int limiarD = 2000; // Limiar de Desvio

const int N_LINHA = 1;
int t = 1;
int t2 = 3;
int nv = 2;
int TErro = 0;
int TErro1 = 0;
int TErro2 = 0;

int Va = 150;

int lP = 250; // Limiar de preto para 'c'
int nT = 1;   // Número de tentativas
int d1 = 10;
int d2 = 5;
int d3 = 15;
int tentativas = 1;

// --- VERIFICAÇÃO CONSECUTIVA DE OBSTÁCULO ---
const int nL = 30; // Número de verificações seguidas exigidas
int nL2 = 5;
int leiturasObstaculo = 0;

int av = 0;
int av1 = 150;
int av2 = 600;

// --- VARIÁVEL DE PULSO DE GIRO ---
int PG = 1000;

// --- Configurações de Cor (Núcleo 0) ---
#define I2C_SDA 15
#define I2C_SCL 16
#define TCAADDR 0x70

const uint8_t esq = 0;
const uint8_t dir = 1;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
volatile int n1 = 0;
volatile int n2 = 0;
volatile int comandoVerde = 0;
int reta = 0;
int nR = 10;
int e = 0;
int nE = 50;

char ladoPrioridade = 'E';
bool achou = true;
bool vermelho = false;
const int velMedia = 180;
const int velBusca = 220;
const int velLenta = 100;

// ================================================================
// CONFIGURAÇÕES DOS SENSORES ULTRASSÔNICOS
// ================================================================
const int TRIG_1 = 12;
const int ECHO_1 = 11;
const int TRIG_2 = 21;
const int ECHO_2 = 47;

// --- Interrupções dos Motores ---
void IRAM_ATTR isrEsq() { mEsq.handleInterrupt(); }
void IRAM_ATTR isrDir() { mDir.handleInterrupt(); }

// --- Funções Auxiliares de Cor ---
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void rgbEsq(uint8_t m, uint8_t n, uint8_t p){
  for(int i = 4; i < NUMPIXELS; i++) {
    led1.setPixelColor(i, led1.Color(m, n, p));
  }
  led1.show();
}

void rgbDir(uint8_t m, uint8_t n, uint8_t p){
  for(int i = 0; i < 4; i++) {
    led1.setPixelColor(i, led1.Color(m, n, p));
  }
  led1.show();
}

bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 80) return false;
  float Lcor = (r + g + b);
  return (g > Lcor * 0.37 && r < Lcor * 0.34 && b < Lcor * 0.26);
}

bool ehVrmlh(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 80) return false;
  float Lcor = (r + g + b);
  return (g < Lcor * 0.40 && r > Lcor * 0.48);
}

// ================================================================
// LEITURA FÍSICA DO ULTRASSÔNICO
// ================================================================
float lerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duracao = pulseIn(echoPin, HIGH, 30000); 
  if (duracao == 0) {
    return -1.0;
  }
  
  return (duracao * 0.0333) / 2;
}

// ================================================================
// CONTROLE DOS MOTORES (PONTE H)
// ================================================================
void controleM(int velEsq, int velDir) {
  if (velEsq >= 0) { 
    ledcWrite(40, velEsq);
    ledcWrite(39, 0);      
  } else { 
    ledcWrite(40, 0); 
    ledcWrite(39, abs(velEsq));
  }
  
  if (velDir >= 0) { 
    ledcWrite(42, velDir); 
    ledcWrite(41, 0);
  } else { 
    ledcWrite(42, 0);      
    ledcWrite(41, abs(velDir));
  }
}

// Protótipos de funções
void atualizarSensores();
int calcularPesoEsquerda();
int calcularPesoDireita();
void desvio();
void verificarPrata();

// ================================================================
// NOFA FUNÇÃO: PARADA E PISCAR AZUL
// ================================================================
void verificarPrata() {
  bool todosMenores = true;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= 220) {
      todosMenores = false;
      break;
    }
  }

  if (todosMenores) {
    controleM(0, 0);
    mEsq.stop();
    mDir.stop();
    Serial.println("Todos os sensores < 220! Parando e piscando em azul...");
    while (1) {
      rgbEsq(0, 0, 255);
      rgbDir(0, 0, 255);
      delay(250);
      rgbEsq(0, 0, 0);
      rgbDir(0, 0, 0);
      delay(250);
    }
  }
}

// ================================================================
// ALINHAMENTO NA LINHA
// ================================================================
void alinhamento() {
  while(1){
    atualizarSensores();
    verificarPrata();
    int todosAtivos = 0;
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] >= limiarD) {
        todosAtivos++;
      }
    }

    if (todosAtivos >= 6) {
      controleM(0, 0);
      break;
    }
  
    int pesoEsq = calcularPesoEsquerda();
    int pesoDir = calcularPesoDireita();
    if (pesoEsq == pesoDir ) {
      controleM(-100, -100);
    } 
    else if (pesoDir > pesoEsq) {
      controleM(-180, 180);
    } 
    else {
      controleM(180, -180);
    }
  }
}

void alinhamentoF() {
  mEsq.move(80, velBusca);
  mDir.move(80, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  while(1){
    atualizarSensores();
    verificarPrata();
    int todosAtivos = 0;
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] >= limiarD) { 
        todosAtivos++;
      }
    }

    if (todosAtivos >= 6) {
      controleM(0, 0);
      break;
    }
  
    int pesoEsq = calcularPesoEsquerda();
    int pesoDir = calcularPesoDireita();
    if (pesoEsq == pesoDir ) {
      controleM(110, 110);
    } 
    else if (pesoDir > pesoEsq) {
      controleM(180, -180);
    }
    else {
      controleM(-180, 180);
    }
  }
}

// ================================================================
// DESVIO DE OBSTÁCULO
// ================================================================
void desvioObstaculo() {
  rgbEsq(255,0,0);
  rgbDir(255,0,0);
  mEsq.stop();
  mDir.stop();
  controleM(0, 0);
  delay(200);
  rgbEsq(0,0,0);
  rgbDir(0,0,0);
  Serial.println("Obstaculo detectado! Girando para a esquerda...");
  
  mEsq.move(-PG, velBusca);
  mDir.move(PG, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  
  mEsq.move(-150, velBusca); mDir.move(-150, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  
  alinhamento();
  desvio();
  alinhamentoF();

  mEsq.move(150, velBusca); mDir.move(150, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { 
    mEsq.update(); 
    mDir.update(); 
  }

  alinhamento();

  mEsq.move(350, velBusca); mDir.move(350, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { 
    mEsq.update(); 
    mDir.update(); 
  }

  mEsq.move(-PG, velBusca); mDir.move(PG, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { 
    mEsq.update();
    mDir.update(); 
  }

  mEsq.move(-PG, velBusca); mDir.move(PG, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
    int ativos = 0;
    for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
    if (ativos >= N_LINHA){delay(t2); atualizarSensores(); verificarPrata();
      if (ativos >= N_LINHA) break;
    }
  }
  mEsq.move(-100, velBusca); mDir.move(-100, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) { 
    mEsq.update(); 
    mDir.update(); 
  }
  mEsq.move(-500, velBusca); mDir.move(-500, velBusca);
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
    int ativos = 0;
    for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
    if (ativos >= N_LINHA){delay(t); atualizarSensores(); verificarPrata();
      if (ativos >= N_LINHA) break;
    }
  }
  mEsq.stop(); mDir.stop(); delay(10);
  n1 = 0; n2 = 0; comandoVerde = 0;
}

void desvio() {
  av = av1;
  for(int i = 0; i < 2; i++){
    int leituras = 0;
    controleM(velMedia, velMedia);
    while (leituras < nL2) {
      float dist = lerDistancia(TRIG_2, ECHO_2);
      if (dist > 0 && dist <= d3) {
        leituras++;
      } else {
        leituras = 0;
      }
      delay(15);
    }
    
    leituras = 0;
    controleM(velMedia, velMedia);
    while (leituras < nL2) {
      float dist = lerDistancia(TRIG_2, ECHO_2);
      if (dist > d3 || dist < 0) {
        leituras++;
      } else {
        leituras = 0;
      }
      delay(15);
    }
    
    mEsq.move(av, velMedia);
    mDir.move(av, velMedia);
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update();
      mDir.update();
    }
    
    mEsq.move(PG, velBusca);
    mDir.move(-PG, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { 
      mEsq.update(); 
      mDir.update(); 
    }
    av = av2;
  }
  ladoPrioridade = 'E';
}

// ================================================================
// TAREFA DO NÚCLEO 0: LEITURA DE VERDE E VERMELHO (FREE-RTOS)
// ================================================================
void encruzilhada(){
  mEsq.move(Va, velLenta);
  mDir.move(Va, velLenta);
  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    if (comandoVerde != 0) { break; }
    atualizarSensores();
    verificarPrata();
    int ativos = 0;
    for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiarD) ativos++;
    if (ativos > 0) { break; }
  }
}

void TaskContadorVerde(void * pvParameters) {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(t);
  tcaSelect(esq); delay(t); tcs.begin();
  tcaSelect(dir); delay(t); tcs.begin();
  
  static int consecutivosS1 = 0;
  static int consecutivosS2 = 0;
  
  for(;;) {
    uint16_t r, g, b, c;

    // --- CHECAGEM SENSOR ESQUERDO ---
    tcaSelect(esq);
    delay(t);
    tcs.getRawData(&r, &g, &b, &c);

    // Detecção de Vermelho (Esquerda) -> Parada Absoluta
    if (ehVrmlh(r, g, b, c)) {
      controleM(0, 0);
      mEsq.stop();
      mDir.stop();
      vermelho = true;
      rgbEsq(255, 0, 0);
      rgbDir(255, 0, 0);
      Serial.println("Vermelho detectado no sensor esquerdo! Parando robo...");
      while (1) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Evita reset por Watchdog Timer
      }
    }

    // Detecção de Verde (Esquerda)
    if (comandoVerde == 0) {
      if (ehVerde(r, g, b, c)) {
        consecutivosS1++;
        if (consecutivosS1 >= nv) {
          n1++; comandoVerde = 1; consecutivosS1 = 0;
        }
      } else { 
        consecutivosS1 = 0; 
      }
    }

    // --- CHECAGEM SENSOR DIREITO ---
    tcaSelect(dir);
    delay(t);
    tcs.getRawData(&r, &g, &b, &c);

    // Detecção de Vermelho (Direita) -> Parada Absoluta
    if (ehVrmlh(r, g, b, c)) {
      controleM(0, 0);
      mEsq.stop();
      mDir.stop();
      vermelho = true;
      rgbEsq(255, 0, 0);
      rgbDir(255, 0, 0);
      Serial.println("Vermelho detectado no sensor direito! Parando robo...");
      while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }

    // Detecção de Verde (Direita)
    if (comandoVerde == 0) {
      if (ehVerde(r, g, b, c)) {
        consecutivosS2++;
        if (consecutivosS2 >= nv) {
          n2++; comandoVerde = 2; consecutivosS2 = 0;
        }
      } else { 
        consecutivosS2 = 0; 
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ================================================================
// SENSORES IR E NAVEGAÇÃO
// ================================================================
void atualizarSensores() {
  recv_data();
  if(g_new_package_flag == 1) {
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

void executarSegueLinha() {
  int ativosEsq = 0, ativosDir = 0;
  for (int i = 0; i < 4; i++) if (sensorValues[i] >= limiar) ativosEsq += (4 - i);
  for (int i = 4; i < 8; i++) if (sensorValues[i] >= limiar) ativosDir += (i - 3);
  if(ativosEsq >= 6 || ativosDir >= 6){
    for (int i = 0; i < 2; i++) if (sensorValues[i] >= limiar) ladoPrioridade = 'E';
    for (int i = 6; i < 8; i++) if (sensorValues[i] >= limiar) ladoPrioridade = 'D'; 
    e = 0;
  } else {
    e++;
    if(e >= nE){
      ladoPrioridade = 'F';
    }
  }

  int pesoEsq = calcularPesoEsquerda();
  int pesoDir = calcularPesoDireita();
  if (pesoEsq == pesoDir || (sensorValues[4] >= limiar && sensorValues[5] >= limiar)) { 
    if(ativosEsq >= 10 && ativosDir >= 10){
      encruzilhada();
    }
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
    if (diff < 3)      controleM(-80, 190);
    else if (diff < 5) controleM(-130, 130);
    else               controleM(0, 180);
  } else {
    int diff = pesoDir - pesoEsq;
    if (diff < 3)      controleM(190, -80);
    else if (diff < 5) controleM(130, -130);
    else               controleM(180, 0);
  }
}

bool tentarGirarEBuscar(char direcao, long pulsos) {
  if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move(pulsos, velBusca); }
  else { mEsq.move(pulsos, velBusca); mDir.move(-pulsos, velBusca); }
 
  bool achou = false;
  if(direcao == 'E'){
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      verificarPrata();
      int ativos = 0;
      for (int i = 0; i < 4; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { achou = true; break;}
    }
  } else {
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      verificarPrata();
      int ativos = 0;
      for (int i = 4; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { achou = true; break;}
    }
  }
  if(achou == true){
    if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move(pulsos, velBusca); }
    else { mEsq.move(pulsos, velBusca); mDir.move(-pulsos, velBusca); }
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      verificarPrata();
      int ativos = 0;
      for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) { mEsq.stop(); mDir.stop(); return true; }
    }
  }
  return false;
}

void encontrarLinha() {
  if (ladoPrioridade == 'F'){
    mEsq.move(80, 130); mDir.move(80, 130);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }

    mEsq.move(650, velBusca); mDir.move(650, velBusca);
    while (1) {
      if(!mEsq.isMoving() || !mDir.isMoving()){ achou = false; break; }
      mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
      int ativos = 0;
      for (int i = 1; i < 7; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA){ delay(t); atualizarSensores(); verificarPrata();
        int ativos = 0;
        for (int i = 1; i < 7; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA){ achou = true; break; }
      }
    }
    if(achou == false){
      mEsq.move(-1000, velLenta); mDir.move(-1000, velLenta);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
        int ativos = 0;
        for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA){ delay(t); atualizarSensores(); verificarPrata();
          int ativos = 0;
          for (int i = 0; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
          if (ativos >= N_LINHA){ break; }
        }
      }
      mEsq.move(-150, 130); mDir.move(-150, 130);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    }
    ladoPrioridade = 'E';
  }
  else if (ladoPrioridade == 'E') {
    if (tentarGirarEBuscar('E', PG)) return;
    delay(200);
    if (tentarGirarEBuscar('D', PG)) return;
    delay(200); if (tentarGirarEBuscar('D', PG)) return;
    delay(200); if (tentarGirarEBuscar('E', PG)) return;
  } else {
    if (tentarGirarEBuscar('D', PG)) return;
    delay(200); if (tentarGirarEBuscar('E', PG)) return;
    delay(200); if (tentarGirarEBuscar('E', PG)) return;
    delay(200); if (tentarGirarEBuscar('D', PG)) return;
  }
  mEsq.stop(); mDir.stop();
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
    tcaSelect(canalOposto); delay(t);
    tcs.getRawData(&r, &g, &b, &c);
    if (ehVerde(r, g, b, c)){
      tcaSelect(canalOposto); delay(t); tcs.getRawData(&r, &g, &b, &c);
      if (ehVerde(r, g, b, c)) duploVerde = true;
    }
    tcaSelect(canalSensor); delay(t); tcs.getRawData(&r, &g, &b, &c);
    if (c < limiarC) { detectouPreto = true; break; }
  }
  
  mEsq.stop(); mDir.stop(); delay(50);
  tcaSelect(canalOposto); delay(t);
  tcs.getRawData(&r, &g, &b, &c);
  if (ehVerde(r, g, b, c)){
    tcaSelect(canalOposto); delay(t); tcs.getRawData(&r, &g, &b, &c);
    if (ehVerde(r, g, b, c)) duploVerde = true;
  } 
  if (!detectouPreto) { n1 = 0; n2 = 0; comandoVerde = 0; return; }

  if (duploVerde) {
    rgbEsq(0, 255, 0); rgbDir(0, 255, 0); 
    mEsq.move(100, velBusca);
    mDir.move(100, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    rgbEsq(0, 0, 0); rgbDir(0, 0, 0);
    mEsq.move(1600, velBusca + TErro); mDir.move(-1600, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }

    for (int i = 0; i < 2; i++){
      mEsq.move(1500, velBusca); mDir.move(-1500, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        atualizarSensores();
        verificarPrata();
        int ativos = 0;
        for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
    }
    mEsq.move(1500, velBusca); mDir.move(-1500, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      verificarPrata();
      int ativos = 0;
      for (int i = 3; i < 5; i++) if (sensorValues[i] >= limiarD) ativos++;
      if (ativos >= N_LINHA) break;
    }

  } else {
    mEsq.move(-300, 130); mDir.move(-300, 130);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    
    if (lado == 1) { 
      rgbEsq(0, 255, 0);
      rgbDir(0, 0, 0); ladoPrioridade = 'D';
      mEsq.move(-400, velBusca); mDir.move(400, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(50, velBusca); mDir.move(50, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      rgbEsq(0, 0, 0); rgbDir(0, 0, 0); 
      mEsq.move(500, velBusca); mDir.move(500, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
        int ativos = 0;
        for (int i = 0; i < 2; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
      mEsq.move(250, 130); mDir.move(250, 130);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(-700, velBusca); mDir.move(700, velBusca + TErro1);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
        int ativos = 0;
        for (int i = 6; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
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
        mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
        int ativos = 0;
        for (int i = 6; i < 8; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
      mEsq.move(250, 130); mDir.move(250, 130);
      while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
      mEsq.move(700, velBusca + TErro2); mDir.move(-700, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update(); atualizarSensores(); verificarPrata();
        int ativos = 0;
        for (int i = 0; i < 2; i++) if (sensorValues[i] >= limiarD) ativos++;
        if (ativos >= N_LINHA) break;
      }
    }
    mEsq.move(-100, velBusca); mDir.move(-100, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
  }
  mEsq.stop(); mDir.stop(); delay(10);
  n1 = 0; n2 = 0; comandoVerde = 0;
}

// ================================================================
// SETUP PRINCIPAL
// ================================================================
void setup() {
  delay(500);
  led1.begin(); led1.setBrightness(30);

  serial_init();
  delay(100);
  SET_Eight_Mode(0, 1, 0); 

  mEsq.begin(isrEsq); mDir.begin(isrDir);
  pinMode(TRIG_1, OUTPUT);
  pinMode(ECHO_1, INPUT);
  pinMode(TRIG_2, OUTPUT);
  pinMode(ECHO_2, INPUT);
  xTaskCreatePinnedToCore(TaskContadorVerde, "TaskVerde", 4096, NULL, 1, NULL, 0);

  rgbEsq(0,0,0);
  rgbDir(0,0,0);
}

// ================================================================
// LOOP PRINCIPAL
// ================================================================
void loop() {
  float distFrontal = lerDistancia(TRIG_1, ECHO_1);
      if (vermelho == true) {
        controleM(0, 0);
        mEsq.stop();
        mDir.stop();
        rgbEsq(255, 0, 0);
        rgbDir(255, 0, 0);
        Serial.println("Vermelho detectado na intersecao! Parando robo...");
        while (1) {
          delay(100);
        }
      }else if (distFrontal > 0 && distFrontal < d1) { 
    leiturasObstaculo++;
    if (leiturasObstaculo <= nL && distFrontal < d2 - 1){
      mEsq.move(-150, velBusca);
      mDir.move(-150, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update();
        mDir.update();
      }
    }
    if (leiturasObstaculo >= nL && distFrontal < d2) { 
      leiturasObstaculo = 0;
      desvioObstaculo();
    }
  } else {
    leiturasObstaculo = 0;
  }

  if (comandoVerde != 0) {
    executarDesvioVerde(comandoVerde);
  }

  atualizarSensores();
  verificarPrata(); // Checagem executada a cada leitura
  
  int todosAtivos = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= limiarD) todosAtivos++;
  }

  if (todosAtivos >= 6) {
    controleM(velLenta, velLenta);
    rgbEsq(255,255,255); rgbDir(255,255,255);

    bool verdeS1 = false;
    bool verdeS2 = false;
    bool linhaS1 = false;
    bool linhaS2 = false;
    while (1) {
      uint16_t r1, g1, b1, c1;
      uint16_t r2, g2, b2, c2;
      
      tcaSelect(esq); delay(t);
      tcs.getRawData(&r1, &g1, &b1, &c1);
      
      tcaSelect(dir); delay(t);
      tcs.getRawData(&r2, &g2, &b2, &c2);

      // --- CHECAGEM DE VERMELHO (Parada em Interseção) ---
      
      if (ehVerde(r1, g1, b1, c1) || comandoVerde == 1) verdeS1 = true;
      if (ehVerde(r2, g2, b2, c2) || comandoVerde == 2) verdeS2 = true;
      
      if (c1 < lP) linhaS1 = true;
      if (c2 < lP) linhaS2 = true;
      
      if (verdeS1 && verdeS2) {
        break;
      }
      
      if (verdeS1 || verdeS2 || linhaS1 || linhaS2) {
        delay(20);
        tcaSelect(esq); delay(t); tcs.getRawData(&r1, &g1, &b1, &c1);
        tcaSelect(dir); delay(t); tcs.getRawData(&r2, &g2, &b2, &c2);

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
    }
    else if (((verdeS1 != verdeS2) || (!verdeS1 && !verdeS2 && linhaS1 && linhaS2)) && tentativas <= nT) {
      mEsq.move(-250, velBusca);
      mDir.move(-250, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update();
        mDir.update();
      }
      n1 = 0; n2 = 0; comandoVerde = 0;
      mEsq.stop();
      mDir.stop();
      controleM(0, 0);
      tentativas++;
    }
    else if ((verdeS1 != verdeS2) && tentativas >= nT) {
      comandoVerde = verdeS1 ? 1 : 2;
      tentativas = 0;
    }
    else if (!verdeS1 && !verdeS2 && linhaS1 && linhaS2 && tentativas >= nT) {
      mEsq.move(250, velBusca);
      mDir.move(250, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update();
        mDir.update();
      }
      n1 = 0; n2 = 0; comandoVerde = 0;
      mEsq.stop();
      mDir.stop();
      controleM(0, 0);
      tentativas = 0;
    }
  }
  else {
    int totalAtivos = 0;
    for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiar) totalAtivos++;
    if (totalAtivos == 0) {
      mEsq.move(Va, velLenta);
      mDir.move(Va, velLenta);
      bool encontrouAlgo = false;
      while (mEsq.isMoving() || mDir.isMoving()) {
        mEsq.update(); mDir.update();
        if (comandoVerde != 0) { encontrouAlgo = true; break; }
        atualizarSensores();
        verificarPrata();
        int ativos = 0;
        for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiar) ativos++;
        if (ativos > 0) { encontrouAlgo = true; break; }
      }
      mEsq.stop();
      mDir.stop();
      if (!encontrouAlgo) { encontrarLinha(); }
    } else {
      executarSegueLinha();
    }
  }
}