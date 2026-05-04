//teste mudando a lógica do fitro de verde 

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "MotorControl.h"

// --- Configurações de Hardware (Segue Linha) ---
MotorControl mEsq(42, 41, 13, 14);
MotorControl mDir(16, 17, 47, 21);
const int sensorPins[6] = {8, 3, 9, 10, 7, 4};
int sensorValues[6];
const int limiar = 4090;
const int N_LINHA = 3;
int t = 3;
int nv = 2;

// --- Configurações de Cor (Núcleo 0) ---
#define I2C_SDA 2
#define I2C_SCL 1
#define TCAADDR 0x70
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

// Variáveis de controle
volatile int n1 = 0; 
volatile int n2 = 0;
volatile int comandoVerde = 0; // 0=Nada, 1=Verde Esq (S0), 2=Verde Dir (S1)

char ladoPrioridade = 'E'; 
const int velMedia = 130;
const int velBusca = 110;
const int velLenta = 80;  // Velocidade para o avanço após verde

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

bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 50) return false;
  float Lcor = (r + g + b) * 0.40; // Limite de cor com 40% da claridade "c"
  return (g > Lcor && r < Lcor);  //só retorna 1 se verde(g) for maior que 40% de 'c', e vermelho(r) for menor que 40% de 'c'
}


// ================================================================
// TAREFA DO NÚCLEO 0: LEITURA DE VERDE
// ================================================================
void TaskContadorVerde(void * pvParameters) {
  Wire.begin(I2C_SDA, I2C_SCL);
  tcaSelect(0); tcs.begin();
  tcaSelect(1); tcs.begin();
  
  static int consecutivosS1 = 0;
  static int consecutivosS2 = 0;

  for(;;) {
    if (comandoVerde != 0) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

    uint16_t r, g, b, c;

    // Sensor 0 (Canal 0) - Esquerda
    tcaSelect(0);
    delay(t);
    tcs.getRawData(&r, &g, &b, &c);
    if (ehVerde(r, g, b, c)) {
      consecutivosS1++;
      if (consecutivosS1 >= nv) {
        n1++; comandoVerde = 1; consecutivosS1 = 0;
        Serial.printf("\n[S0] VERDE! C=%d | Total n1: %d\n", c, n1);
      }
    } else { consecutivosS1 = 0; }
    Serial.print("S1 -> R:"); Serial.print(r); Serial.print(" G:"); Serial.print(g); 
    Serial.print(" B:"); Serial.print(b); Serial.print(" C:"); Serial.print(c);
    // Sensor 1 (Canal 1) - Direita
    tcaSelect(1);
    delay(t);
    tcs.getRawData(&r, &g, &b, &c);
    Serial.print(" | S2 -> R:"); Serial.print(r); Serial.print(" G:"); Serial.print(g);
    Serial.print(" B:"); Serial.print(b); Serial.print(" C:"); Serial.print(c);
    Serial.print("    n1="); Serial.print(n1); Serial.print("    n2="); Serial.println(n2);
    if (ehVerde(r, g, b, c)) {
      consecutivosS2++;
      if (consecutivosS2 >= nv) {
        n2++; comandoVerde = 2; consecutivosS2 = 0;
        Serial.printf("\n[S1] VERDE! C=%d | Total n2: %d\n", c, n2);
      }
    } else { consecutivosS2 = 0; }
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }

}

// ================================================================
// FUNÇÕES DE MOVIMENTAÇÃO E SENSORES ORIGINAIS
// ================================================================
void atualizarSensores() {
  for (int i = 0; i < 6; i++) sensorValues[i] = analogRead(sensorPins[i]);
}

int calcularPesoEsquerda() {
  int soma = 0;
  for (int i = 0; i < 3; i++) if (sensorValues[i] >= limiar) soma += (3 - i);
  return soma;
}

int calcularPesoDireita() {
  int soma = 0;
  for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiar) soma += (i - 2);
  return soma;
}

void acionarMotores(int pEsq, int pDir) {
  if (pEsq >= 0) { ledcWrite(13, pEsq); ledcWrite(14, 0); }
  else { ledcWrite(13, 0); ledcWrite(14, abs(pEsq)); }
  
  if (pDir >= 0) { ledcWrite(47, pDir); ledcWrite(21, 0); }
  else { ledcWrite(47, 0); ledcWrite(21, abs(pDir)); }
}

void executarSegueLinha() {
  int ativosEsq = 0, ativosDir = 0;
  for (int i = 0; i < 3; i++) if (sensorValues[i] >= limiar) ativosEsq++;
  for (int i = 3; i < 6; i++) if (sensorValues[i] >= limiar) ativosDir++;
  
  if (ativosEsq >= 3 && ativosEsq > ativosDir) ladoPrioridade = 'E';
  else if (ativosDir >= 3 && ativosDir > ativosEsq) ladoPrioridade = 'D';

  int pesoEsq = calcularPesoEsquerda();
  int pesoDir = calcularPesoDireita();
  
  if (pesoEsq == pesoDir) {
    acionarMotores(velMedia, velMedia);
  } else if (pesoEsq > pesoDir) {
    int diff = pesoEsq - pesoDir;
    if (diff < 3)      acionarMotores(70, 130);
    else if (diff < 5) acionarMotores(-110, 130);
    else               acionarMotores(-130, 180);
  } else {
    int diff = pesoDir - pesoEsq;
    if (diff < 3)      acionarMotores(130, 70);
    else if (diff < 5) acionarMotores(130, -110);
    else               acionarMotores(180, -130);
  }
}

bool tentarGirarEBuscar(char direcao, long pulsos) {
  if (direcao == 'E') { mEsq.move(-pulsos, velBusca); mDir.move(pulsos, velBusca); }
  else { mEsq.move(pulsos, velBusca); mDir.move(-pulsos, velBusca); }

  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();
    atualizarSensores();
    int ativos = 0;
    for (int i = 0; i < 6; i++) if (sensorValues[i] >= limiar) ativos++;
    if (ativos >= N_LINHA) { mEsq.stop(); mDir.stop(); return true; }
  }
  return false;
}

void encontrarLinha() {
  if (ladoPrioridade == 'E') {
    if (tentarGirarEBuscar('E', 1000)) return;
    delay(200); if (tentarGirarEBuscar('D', 1000)) return;
    delay(200); if (tentarGirarEBuscar('D', 1000)) return;
    delay(200); if (tentarGirarEBuscar('E', 1000)) return;
  } else {
    if (tentarGirarEBuscar('D', 1000)) return;
    delay(200); if (tentarGirarEBuscar('E', 1000)) return;
    delay(200); if (tentarGirarEBuscar('E', 1000)) return;
    delay(200); if (tentarGirarEBuscar('D', 1000)) return;
  }
  mEsq.stop(); mDir.stop();

}

// ================================================================
// LÓGICA DE DESVIO (EXECUTADA NO NÚCLEO 1)
// ================================================================
void executarDesvioVerde(int lado) {
  uint16_t r, g, b, c;
  int canalSensor = (lado == 1) ? 0 : 1;
  int canalOposto = (lado == 1) ? 1 : 0;
  int limiarC = (lado == 1) ? 300 : 200;

  Serial.println("--- Iniciando Fase 1: Avanco Lento ---");
  
  // 1. Avança devagar por 30 pulsos ou até detectar preto no sensor de cor atual
  mEsq.move(30, velLenta);
  mDir.move(30, velLenta);

  while (mEsq.isMoving() || mDir.isMoving()) {
    mEsq.update(); mDir.update();

    tcaSelect(canalSensor);
    tcs.getRawData(&r, &g, &b, &c);

    if (c < limiarC) {
      Serial.printf("Preto detectado (C=%d). Parando avanco.\n", c);
      break;
    }
  }
  mEsq.stop(); mDir.stop();
  delay(50);

  Serial.println("--- Fase 2: Analise do Oposto e Giro ---");
  
  // 2. Lê o sensor oposto para checar Duplo Verde
  bool duploVerde = false;
  tcaSelect(canalOposto);
  delay(t);
  tcs.getRawData(&r, &g, &b, &c);

  if (canalOposto == 0 && ehVerde(r, g, b, c)) duploVerde = true;
  else if (canalOposto == 1 && ehVerde(r, g, b, c)) duploVerde = true;

  // 3. Zera as variáveis de decisão, como pedido

  if (duploVerde) {
    Serial.println("Verde detectado no lado OPOSTO (Duplo Verde)! Iniciando meia volta...");
    // frente
    mEsq.move(100, velBusca); mDir.move(100, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    // Meia volta de 1000 pulsos inicial (Girando para a direita como padrão)
    mEsq.move(1200, velBusca); mDir.move(-1200, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    //ré
    mEsq.move(-200, velBusca); mDir.move(-200, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    // Continua girando até detectar linha ou até o limite de 2000 pulsos
    mEsq.move(1500, velBusca); mDir.move(-1500, velBusca);
    while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      int ativos = 0;
      for (int i = 0; i < 6; i++) if (sensorValues[i] >= limiar) ativos++;
      
      if (ativos >= N_LINHA) {
        Serial.println("Linha detectada após duplo verde!");
        break;
      }
    }
  } else {
    Serial.println("Apenas um verde. Giro de 90 graus.");
   mEsq.move(350, 130);
   mDir.move(350, 130);
  
   // Enquanto os motores não atingirem o alvo, atualiza a contagem
   while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }
    // Gira de 1000 pulsos para o lado detectado originalmente
    if (lado == 1) { // Esquerda
      mEsq.move(-1000, velBusca); mDir.move(1000, velBusca);
      //até achar a linha
      mEsq.move(-1000, velBusca); mDir.move(1000, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      int ativos = 0;
      for (int i = 0; i < 6; i++) if (sensorValues[i] >= limiar) ativos++;
      
      if (ativos >= N_LINHA) {
        Serial.println("Linha detectada após duplo verde!");
        break;
      }
    }
    } else { // Direita
      mEsq.move(1000, velBusca); mDir.move(-1000, velBusca);
      //até achar a linha
      mEsq.move(1000, velBusca); mDir.move(-1000, velBusca);
      while (mEsq.isMoving() || mDir.isMoving()) {
      mEsq.update(); mDir.update();
      atualizarSensores();
      int ativos = 0;
      for (int i = 0; i < 6; i++) if (sensorValues[i] >= limiar) ativos++;
      
      if (ativos >= N_LINHA) {
        Serial.println("Linha detectada após duplo verde!");
        break;
      }
    }
    }
    while (mEsq.isMoving() || mDir.isMoving()) { mEsq.update(); mDir.update(); }


  }

  mEsq.stop(); mDir.stop();
  delay(10);
  n1 = 0;
  n2 = 0;
  comandoVerde = 0;

}

void setup() {
  Serial.begin(115200);
  mEsq.begin(isrEsq); mDir.begin(isrDir);
  for (int i = 0; i < 6; i++) pinMode(sensorPins[i], INPUT);
  xTaskCreatePinnedToCore(TaskContadorVerde, "TaskVerde", 4096, NULL, 1, NULL, 0);
}

void loop() {
  if (comandoVerde != 0) {
    executarDesvioVerde(comandoVerde);
  }

  // Segue linha
  atualizarSensores();
  int totalAtivos = 0;
  for (int i = 0; i < 6; i++) if (sensorValues[i] >= limiar) totalAtivos++;

  if (totalAtivos == 0) {
    encontrarLinha();
  } else {
    executarSegueLinha();
  }
}
