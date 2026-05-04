#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ====== DEFINIÇÕES DE PINOS ESP32-S3 ======
#define I2C_SDA 2
#define I2C_SCL 1

// Endereço padrão do Multiplexador
#define TCAADDR 0x70

// Inicialização dos sensores (mesmo objeto, trocamos o canal antes de usar)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
int i = 0;

// Variáveis para armazenar leituras
uint16_t r1, g1, b1, c1;
uint16_t r2, g2, b2, c2;

// --- FUNÇÃO ESSENCIAL: SELECIONAR CANAL DO MULTIPLEXADOR ---
uint8_t d = 5;
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);

  // Inicia o I2C nos pinos 2 (SDA) e 1 (SCL) da ESP32-S3
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(500);

  Serial.println("Iniciando varredura do Multiplexador...");

  // Inicializa Sensor 1 (Porta 0 do TCA)
  tcaSelect(0); 
  delay(d);
  if (tcs.begin()) {
    Serial.println("Sensor 1 (Porta 0) OK");
  } else {
    Serial.println("ERRO: Sensor 1 não encontrado");
    i++;
  }

  // Inicializa Sensor 2 (Porta 1 do TCA)
  tcaSelect(1);
  delay(d);
  if (tcs.begin()) {
    Serial.println("Sensor 2 (Porta 1) OK");
  } else {
    Serial.println("ERRO: Sensor 2 não encontrado");
    while(1);
  }
  if(i >= 1) while(1);
}

void lerSensores() {
  // LER SENSOR 1
  tcaSelect(0);
  delay(d);
  tcs.getRawData(&r1, &g1, &b1, &c1);
  
  // LER SENSOR 2
  tcaSelect(1);
  delay(d);
  tcs.getRawData(&r2, &g2, &b2, &c2);

  // Exibição no Monitor Serial
  Serial.print("S1 -> R:"); Serial.print(r1); Serial.print(" G:"); Serial.print(g1); 
  Serial.print(" B:"); Serial.print(b1); Serial.print(" C:"); Serial.print(c1);
  
  Serial.print(" | S2 -> R:"); Serial.print(r2); Serial.print(" G:"); Serial.print(g2); 
  Serial.print(" B:"); Serial.print(b2); Serial.print(" C:"); Serial.println(c2);
}

void loop() {
  lerSensores();
  delay(10);
}