#include <Wire.h>
#include <Adafruit_TCS34725.h>

#define I2C_SDA 2
#define I2C_SCL 1
#define TCAADDR 0x70

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

uint16_t r1, g1, b1, c1;
uint16_t r2, g2, b2, c2;
uint8_t d = 5;
const int n = 0; 

// --- FUNÇÃO PARA IDENTIFICAR VERDE ---
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // 1. Verifica se há luz suficiente (evita falso positivo no escuro)
  if (c < 50) return false; 
//r e b = 30 ficou bom 
  if (g > (r + 20) && g > (b + 30)) {
    return true;
    +n;
  }
  return false;
}

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(500);

  tcaSelect(0); 
  delay(d);
  tcs.begin();

  tcaSelect(1);
  delay(d);
  tcs.begin();
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

  // --- APLICAÇÃO DO FILTRO ---
  bool s1Verde = ehVerde(r1, g1, b1, c1);
  bool s2Verde = ehVerde(r2, g2, b2, c2);

  // Impressão dos dados
  Serial.print("S1:"); Serial.print(c1);
  if(s1Verde) Serial.print(" [VERDE!] ");
  
  Serial.print(" | S2:"); Serial.print(c2);
  if(s2Verde) Serial.print(" [VERDE!] ");
  
  Serial.println();
    // Exibição no Monitor Serial
  Serial.print("S1 -> R:"); Serial.print(r1); Serial.print(" G:"); Serial.print(g1); 
  Serial.print(" B:"); Serial.print(b1); Serial.print(" C:"); Serial.print(c1);
  
  Serial.print(" | S2 -> R:"); Serial.print(r2); Serial.print(" G:"); Serial.print(g2); 
  Serial.print(" B:"); Serial.print(b2); Serial.print(" C:"); Serial.print(c2);
  Serial.print("    "); Serial.println(n);
}

void loop() {
  lerSensores();
  delay(50); 
}