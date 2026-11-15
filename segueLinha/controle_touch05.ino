// =====================================================================
//  CONTROLE TOUCH -> ROBO (ESP-NOW) - ESP32-S3 + LCD 240x320
//  Menu: Mover, Giro, Sensores (COR/REFL/ULTRA/VLX/FIM),
//        Motores (pulsos confirmados), Garra, RGB, OBR, Robos,
//        Camera (ESP-NOW): video + controle do robo
// =====================================================================
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include <stdio.h>
#include <TJpg_Decoder.h>

#define CANAL 1

// ---------- Protocolo (igual ao receptor do robo) ----------
enum : uint8_t { MSG_DISCOVER = 1, MSG_ANNOUNCE = 2, MSG_COLOR = 3, MSG_CMD = 4, MSG_PULSOS = 5,
                 MSG_TELEM = 6, MSG_OBR = 7, MSG_LED = 8, MSG_GARRA = 9 };
enum : uint8_t { CMD_STOP = 0, CMD_FRENTE, CMD_TRAS, CMD_PIVO_ESQ, CMD_PIVO_DIR, CMD_CURVA_ESQ, CMD_CURVA_DIR };
enum : uint8_t { DIR_FRENTE = 0, DIR_TRAS, DIR_GIRO_ESQ, DIR_GIRO_DIR };
enum : uint8_t { PULSO_OCIOSO = 0, PULSO_EXECUTANDO, PULSO_CONCLUIDO, PULSO_TIMEOUT, PULSO_PARADO };
enum : uint8_t { CLASSE_NADA = 0, CLASSE_VERDE, CLASSE_VERMELHO, CLASSE_PRETO };
enum : uint8_t { GARRA_ABAIXA_COLETA = 0, GARRA_DERRUBA_DIR, GARRA_DERRUBA_ESQ, GARRA_PCOLETA, GARRA_PNEUTRA };
enum : uint8_t { LED_ESQ = 0, LED_DIR, LED_AMBOS };
enum : uint8_t { MSG_CAM_CTRL = 10, MSG_FRAME = 11 };

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t a;
  uint8_t b;
  uint8_t c;
  char name[16];
} Msg;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  dir;
  uint16_t pulsos;
  uint8_t  vel;
} PacotePulsos;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t lado;
  uint8_t r, g, b;
} PacoteLed;

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  estadoPulso;
  uint8_t  okMask;      // bit0 corE, bit1 corD, bit2 VLX F, bit3 VLX D, bit4 VLX E, bit5 PCA
  uint8_t  estado;      // bit0 OBR ativa, bit1 linha vermelha
  uint8_t  fim;         // bit0 fimC1 pressionado, bit1 fimC2 pressionado
  uint8_t  classe[2];
  uint16_t pulsosE;
  uint16_t pulsosD;
  uint16_t cor[2][4];
  uint16_t ir[8];
  uint16_t ultra[3];
  uint16_t vlx[3];
} Telem;

// ---------- Camera (ESP-NOW) ----------
#define CAM_CHUNK      200
#define CAM_CAB        10
#define CAM_MAX_FRAME  24000
#define CAM_MAX_PARTES 128
#define CAM_TIMEOUT_MS 2000
#define CAM_PING_MS    500
#define CAM_VIEW_X     40     // area da imagem (QQVGA 160x120 centralizada)
#define CAM_VIEW_Y     36
#define CAM_VIEW_W     160
#define CAM_VIEW_H     120

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint16_t frameId;
  uint16_t seq;
  uint16_t total;
  uint16_t tamanho;
  uint8_t  len;
  uint8_t  dados[CAM_CHUNK];
} FrameChunk;

uint8_t  camRx[CAM_MAX_FRAME];
uint8_t  camShow[CAM_MAX_FRAME];
uint8_t  camMapa[CAM_MAX_PARTES];
volatile uint16_t camLenShow = 0;
volatile bool     camPronto = false;
volatile bool     camBusy = false;
volatile uint32_t camUltimoMs = 0;
uint16_t camFrameRx = 0xFFFF, camLenRx = 0, camTotalRx = 0, camRecebidas = 0;
uint32_t camPingMs = 0, camFpsMs = 0, camFpsCont = 0, camFps = 0;

#define VEL_PADRAO         255
#define VEL_PULSO          200
#define INTERVALO_ENVIO_MS 100
#define ONLINE_MS          1000
#define TELEM_MS           1000
#define MAX_ROBOS          4
#define SEM_VALOR          0xFFFF
#define PULSO_MIN          10
#define PULSO_MAX          5000

uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------- Pinos ----------
#define LCD_SCLK 39
#define LCD_MOSI 38
#define LCD_MISO -1
#define LCD_DC   42
#define LCD_CS   45
#define LCD_RST  0
#define LCD_BL   1
#define I2C_SDA  48
#define I2C_SCL  47
#define LCD_W    240
#define LCD_H    320

// ---------- Cores (RGB565) ----------
#define COR_FUNDO    0x0000
#define COR_CARD     0x18E3
#define COR_ACENTO   0x06FF
#define COR_TEXTO    0xFFFF
#define COR_CINZA    0x8410
#define COR_VERDE    0x07E0
#define COR_VERMELHO 0xF800
#define COR_AMARELO  0xFFE0

// Paleta de 16 cores para os LEDs (RGB 8 bits)
const uint8_t PALETA[16][3] = {
  {255, 0, 0}, {255, 128, 0}, {255, 255, 0}, {128, 255, 0},
  {0, 255, 0}, {0, 255, 128}, {0, 255, 255}, {0, 128, 255},
  {0, 0, 255}, {128, 0, 255}, {255, 0, 255}, {255, 0, 128},
  {255, 255, 255}, {128, 128, 128}, {255, 200, 0}, {128, 0, 32}
};
const char *NOME_COR[16] = {
  "Vermelho", "Laranja", "Amarelo", "Verde-limao", "Verde", "Turquesa", "Ciano", "Azul-claro",
  "Azul", "Roxo", "Magenta", "Rosa", "Branco", "Cinza", "Dourado", "Vinho"
};

// ---------- Tipos de tela e area ----------
struct Rect { int x, y, w, h; };
enum Tela { TELA_MENU = 0, TELA_MOVER, TELA_GIRO, TELA_SENSORES, TELA_MOTORES, TELA_GARRA, TELA_RGB, TELA_OBR, TELA_ROBOS, TELA_CAMERA };
enum SubSensor { SUB_COR = 0, SUB_REFL, SUB_ULTRA, SUB_VLX, SUB_FIM };

// ---------- Robos encontrados ----------
struct Robo {
  uint8_t mac[6];
  char name[16];
  volatile uint32_t lastSeen;
  bool usado;
};
Robo robos[MAX_ROBOS];
int selected = -1;
volatile bool needRedraw = false;
const char *statusText = "Toque em BUSCAR ROBOS";

// ---------- Telemetria do robo ----------
Telem telem;
volatile uint32_t telemMs = 0;

// ---------- Estados das telas ----------
uint16_t pulsoSel = 100;
uint8_t  dirSel = DIR_FRENTE;
uint16_t pulsoEnviado = 0;
uint8_t  ladoRgb = LED_DIR;
int      corRgbSel = -1;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, LCD_MISO);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_W, LCD_H);

// ---------- Areas de toque ----------
const Rect AREA_VOLTAR = {6, 4, 64, 20};
const char *NOME_MENU[9] = {"Mover", "Giro", "Sensores", "Motores", "Garra", "RGB", "OBR", "Robos", "Camera"};

const Rect AREA_MOV[6] = {
  {97, 40, 46, 44}, {97, 140, 46, 44}, {45, 90, 46, 44},
  {149, 90, 46, 44}, {45, 196, 68, 40}, {117, 196, 68, 40}
};
// Botoes de movimento na aba da camera (mesmos comandos do modo Mover)
const Rect AREA_CAM_MOV[4] = {
  {97, 170, 46, 34},   // FRENTE
  {97, 250, 46, 34},   // TRAS
  {45, 210, 46, 34},   // PIVO ESQ
  {149, 210, 46, 34}   // PIVO DIR
};
const uint8_t CMD_DO_BOTAO[6] = {
  CMD_FRENTE, CMD_TRAS, CMD_PIVO_ESQ, CMD_PIVO_DIR, CMD_CURVA_ESQ, CMD_CURVA_DIR
};
const char *ROTULO_SUB[5] = {"COR", "REFL", "ULTRA", "VLX", "FIM"};
const Rect AREA_BUSCAR = {10, 36, 220, 30};
const Rect AREA_PASSO[4] = {
  {6, 84, 54, 30}, {64, 84, 54, 30}, {122, 84, 54, 30}, {180, 84, 54, 30}
};
const char *ROTULO_PASSO[4] = {"-100", "-10", "+10", "+100"};
const int   VALOR_PASSO[4]  = {-100, -10, 10, 100};
const Rect AREA_DIR[4] = {
  {6, 128, 110, 32}, {124, 128, 110, 32}, {6, 166, 110, 32}, {124, 166, 110, 32}
};
const char *ROTULO_DIR[4] = {"FRENTE", "TRAS", "GIRO ESQ", "GIRO DIR"};
const Rect AREA_CONFIRMA = {6, 226, 150, 36};
const Rect AREA_PARAR    = {162, 226, 72, 36};

const char *ROTULO_GARRA[5] = {"AbaixaColeta()", "DerrubaDireita()", "DerrubaEsquerda()", "Pcoleta()", "Pneutra()"};
const uint8_t COD_GARRA[5] = {GARRA_ABAIXA_COLETA, GARRA_DERRUBA_DIR, GARRA_DERRUBA_ESQ, GARRA_PCOLETA, GARRA_PNEUTRA};
const char *ROTULO_LADO[3] = {"ESQUERDA", "DIREITA", "AMBOS"};

Tela telaAtual = TELA_MENU;
SubSensor subSensor = SUB_COR;
int botaoAtivo = -1;
bool estavaTocando = false;
uint32_t ultimoEnvio = 0;
uint32_t ultimoIMU = 0;
uint32_t ultimoGiro = 0;
uint32_t ultimoUI = 0;

// ---------- Touch CST816D ----------
#define CST816_ADDR 0x15

bool readTouch(int16_t &x, int16_t &y) {
  Wire.beginTransmission(CST816_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(CST816_ADDR, 6);
  if (Wire.available() < 6) return false;
  uint8_t fingerNum = Wire.read();
  uint8_t xH = Wire.read();
  uint8_t xL = Wire.read();
  uint8_t yH = Wire.read();
  uint8_t yL = Wire.read();
  Wire.read();
  if (fingerNum == 0) return false;
  x = ((xH & 0x0F) << 8) | xL;
  y = ((yH & 0x0F) << 8) | yL;
  return true;
}

bool dentro(int16_t x, int16_t y, const Rect &r) {
  return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
}

// ---------- QMI8658 (IMU do controle) ----------
#define QMI8658_ADDR  0x6B
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL7 0x08
#define QMI8658_AX_L  0x35
#define GYRO_DEADBAND 0.55
#define ACCEL_WEIGHT  0.05

float accSens  = 1.0 / 8192.0;
float gyroSens = 1.0 / 64.0;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
float accOffX = 0, accOffY = 0;
float angleX = 0, angleY = 0, angleZ = 0;
unsigned long lastTime = 0;

void qmiWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void qmiReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)QMI8658_ADDR, (int)len);
  for (int i = 0; i < len; i++) buf[i] = Wire.read();
}

void qmiInit() {
  qmiWrite(QMI8658_CTRL1, 0x60);
  qmiWrite(QMI8658_CTRL2, 0x64);
  qmiWrite(QMI8658_CTRL3, 0x64);
  qmiWrite(QMI8658_CTRL7, 0x03);
  delay(100);
}

void qmiReadRaw() {
  uint8_t buf[12];
  qmiReadBytes(QMI8658_AX_L, buf, 12);
  int16_t rawAX = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t rawAY = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t rawAZ = (int16_t)(buf[5] << 8 | buf[4]);
  int16_t rawGX = (int16_t)(buf[7] << 8 | buf[6]);
  int16_t rawGY = (int16_t)(buf[9] << 8 | buf[8]);
  int16_t rawGZ = (int16_t)(buf[11] << 8 | buf[10]);
  accX = rawAX * accSens;
  accY = rawAY * accSens;
  accZ = rawAZ * accSens;
  gyroX = rawGX * gyroSens;
  gyroY = rawGY * gyroSens;
  gyroZ = rawGZ * gyroSens;
}

float applyDeadband(float v) {
  return (fabs(v) < GYRO_DEADBAND) ? 0.0 : v;
}

void updateAngles() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 515.0;
  lastTime = now;

  float gX = applyDeadband(gyroX - gyroOffX);
  float gY = applyDeadband(gyroY - gyroOffY);
  float gZ = applyDeadband(gyroZ - gyroOffZ);

  float aX = accX - accOffX;
  float aY = accY - accOffY;
  float aZ = accZ;

  float accAngleX = atan2(aY, sqrt(aX * aX + aZ * aZ)) * 180.0 / PI;
  float accAngleY = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180.0 / PI;

  angleX += gX * dt;
  angleY += gY * dt;
  angleZ += gZ * dt;

  angleX = (1.0 - ACCEL_WEIGHT) * angleX + ACCEL_WEIGHT * accAngleX;
  angleY = (1.0 - ACCEL_WEIGHT) * angleY + ACCEL_WEIGHT * accAngleY;
}

void calibraGiro() {
  gfx->fillScreen(COR_FUNDO);
  gfx->setTextColor(COR_TEXTO);
  gfx->setTextSize(2);
  gfx->setCursor(20, 140);
  gfx->print("CALIBRANDO...");
  gfx->setTextSize(1);
  gfx->setCursor(20, 170);
  gfx->print("Nao mova o controle");

  double sGX = 0, sGY = 0, sGZ = 0, sAX = 0, sAY = 0;
  int n = 0;
  unsigned long ini = millis();
  while (millis() - ini < 2000) {
    qmiReadRaw();
    sGX += gyroX; sGY += gyroY; sGZ += gyroZ;
    sAX += accX;  sAY += accY;
    n++;
    delay(20);
  }
  gyroOffX = sGX / n;
  gyroOffY = sGY / n;
  gyroOffZ = sGZ / n;
  accOffX  = sAX / n;
  accOffY  = sAY / n;
  angleX = angleY = angleZ = 0;
  lastTime = millis();
}

// ---------- Robos: lista e busca ----------
int indiceRobo(const uint8_t *mac) {
  for (int i = 0; i < MAX_ROBOS; i++) {
    if (robos[i].usado && memcmp(robos[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

// Lista cheia: substitui o mais antigo (o selecionado nunca e substituido)
int adicionaRobo(const uint8_t *mac, const char *nome) {
  int idx = -1;
  for (int i = 0; i < MAX_ROBOS && idx < 0; i++) {
    if (!robos[i].usado) idx = i;
  }
  if (idx < 0) {
    uint32_t maisAntigo = 0xFFFFFFFF;
    for (int i = 0; i < MAX_ROBOS; i++) {
      if (i == selected) continue;
      if (robos[i].lastSeen < maisAntigo) {
        maisAntigo = robos[i].lastSeen;
        idx = i;
      }
    }
  }
  if (idx < 0) return -1;
  memcpy(robos[idx].mac, mac, 6);
  strncpy(robos[idx].name, nome, 15);
  robos[idx].name[15] = 0;
  robos[idx].lastSeen = millis();
  robos[idx].usado = true;
  return idx;
}

bool ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  p.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&p) == ESP_OK;
}

// Roda no callback do WiFi: registra robos e copia a telemetria do robo selecionado
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  if (data[0] == MSG_ANNOUNCE && len == sizeof(Msg)) {
    Msg m;
    memcpy(&m, data, sizeof(Msg));
    int idx = indiceRobo(info->src_addr);
    if (idx < 0) {
      idx = adicionaRobo(info->src_addr, m.name);
      if (idx < 0) return;
      needRedraw = true;
    }
    robos[idx].lastSeen = millis();
  }
  else if (data[0] == MSG_TELEM && len == sizeof(Telem)) {
    if (selected >= 0 && memcmp(info->src_addr, robos[selected].mac, 6) == 0) {
      memcpy(&telem, data, sizeof(Telem));
      telemMs = millis();
    }
  }
  else if (data[0] == MSG_FRAME && len > CAM_CAB && len <= (int)sizeof(FrameChunk)) {
    recebeParte(data, len);
  }
}

void startScan() {
  Msg m = {};
  m.type = MSG_DISCOVER;
  strncpy(m.name, "CONTROLE", 15);
  esp_err_t r = esp_now_send(BROADCAST, (uint8_t *)&m, sizeof(m));
  statusText = (r == ESP_OK) ? "Buscando robos..." : "Erro na busca";
}

void enviaComando(uint8_t cmd) {
  if (selected < 0) return;
  Msg m = {};
  m.type = MSG_CMD;
  m.a = cmd;
  m.b = VEL_PADRAO;
  esp_now_send(robos[selected].mac, (uint8_t *)&m, sizeof(m));
}

void selectRobo(int i) {
  ensurePeer(robos[i].mac);
  selected = i;
  telemMs = 0;
  statusText = "Robo selecionado";
  enviaComando(CMD_STOP);
}

bool roboOnline() {
  return selected >= 0 && (millis() - robos[selected].lastSeen) < ONLINE_MS;
}

bool telemFresco() {
  return selected >= 0 && telemMs != 0 && (millis() - telemMs) < TELEM_MS;
}

bool pcaConectada() {
  return telemFresco() && (telem.okMask & 32);
}

// Converte RGB 8 bits para RGB565
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ---------- Desenho: utilitarios ----------
void textoCentro(const char *txt, Rect r, uint16_t cor, uint8_t tam) {
  int tw = strlen(txt) * 6 * tam;
  int th = 8 * tam;
  gfx->setTextSize(tam);
  gfx->setTextColor(cor);
  gfx->setCursor(r.x + (r.w - tw) / 2, r.y + (r.h - th) / 2);
  gfx->print(txt);
}

void desenhaRodape(const char *txt, uint16_t cor) {
  gfx->fillRect(0, 304, LCD_W, 16, COR_FUNDO);
  gfx->setTextSize(1);
  gfx->setTextColor(cor);
  gfx->setCursor(10, 308);
  gfx->print(txt);
}

// Texto que apaga so a propria area antes de escrever (sem piscar)
void desenhaValor(int x, int y, int w, const char *txt, uint16_t cor) {
  gfx->fillRect(x, y, w, 10, COR_FUNDO);
  gfx->setTextSize(1);
  gfx->setTextColor(cor);
  gfx->setCursor(x, y + 1);
  gfx->print(txt);
}

void desenhaStatus() {
  gfx->fillRect(146, 6, 94, 20, COR_FUNDO);
  gfx->setTextSize(1);
  gfx->setCursor(150, 12);
  if (selected < 0) {
    gfx->setTextColor(COR_CINZA);
    gfx->print("SEM ROBO");
  } else if (roboOnline()) {
    gfx->setTextColor(COR_VERDE);
    gfx->print("ROBO: ONLINE");
  } else {
    gfx->setTextColor(COR_VERMELHO);
    gfx->print("ROBO: OFFLINE");
  }
}

void desenhaCabecalho(const char *titulo) {
  gfx->fillRect(0, 0, LCD_W, 30, COR_FUNDO);
  gfx->fillRoundRect(AREA_VOLTAR.x, AREA_VOLTAR.y, AREA_VOLTAR.w, AREA_VOLTAR.h, 6, COR_CARD);
  textoCentro("< MENU", AREA_VOLTAR, COR_TEXTO, 1);
  gfx->setTextSize(1);
  gfx->setTextColor(COR_CINZA);
  gfx->setCursor(78, 12);
  gfx->print(titulo);
  desenhaStatus();
}

// ---------- Menu ----------
Rect areaMenu(int i) {
  int col = i % 2;
  int row = i / 2;
  Rect r = {6 + col * 118, 40 + row * 50, 110, 44};
  return r;
}

void desenhaMenu() {
  gfx->fillScreen(COR_FUNDO);
  gfx->setTextSize(2);
  gfx->setTextColor(COR_TEXTO);
  gfx->setCursor(10, 8);
  gfx->print("MENU");
  desenhaStatus();
  for (int i = 0; i < 9; i++) {
    Rect r = areaMenu(i);
    gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, COR_CARD);
    textoCentro(NOME_MENU[i], r, COR_TEXTO, 2);
  }
  desenhaRodape("Toque em um modo", COR_CINZA);
}

// ---------- Mover ----------
void desenhaBotaoMov(int b) {
  const Rect &r = AREA_MOV[b];
  bool ativo = (b == botaoAtivo);
  uint16_t bg = ativo ? COR_ACENTO : COR_CARD;
  uint16_t fg = ativo ? COR_FUNDO : COR_ACENTO;
  gfx->fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
  int cx = r.x + r.w / 2;
  int cy = r.y + r.h / 2;
  switch (b) {
    case 0: gfx->fillTriangle(cx, cy - 9, cx - 9, cy + 7, cx + 9, cy + 7, fg); break;
    case 1: gfx->fillTriangle(cx, cy + 9, cx - 9, cy - 7, cx + 9, cy - 7, fg); break;
    case 2: gfx->fillTriangle(cx - 9, cy, cx + 7, cy - 9, cx + 7, cy + 9, fg); break;
    case 3: gfx->fillTriangle(cx + 9, cy, cx - 7, cy - 9, cx - 7, cy + 9, fg); break;
    case 4: textoCentro("Curva E", r, fg, 1); break;
    case 5: textoCentro("Curva D", r, fg, 1); break;
  }
}

void desenhaMover() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("MOVER");
  for (int b = 0; b < 6; b++) desenhaBotaoMov(b);
  if (selected < 0) desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
  else              desenhaRodape("Segure para mover, solte para parar", COR_CINZA);
}

// ---------- Giro ----------
void linhaAngulo(int y, const char *nome, float v) {
  gfx->fillRect(10, y, 220, 30, COR_FUNDO);
  gfx->setTextSize(2);
  gfx->setTextColor(COR_CINZA);
  gfx->setCursor(10, y + 6);
  gfx->print(nome);
  gfx->setTextColor(COR_TEXTO);
  gfx->setCursor(80, y + 6);
  gfx->print(v, 1);
  gfx->print(char(247));
}

void linhaRaw(int y, const char *nome, float a, float b, float c, uint8_t casas) {
  gfx->fillRect(10, y, 220, 14, COR_FUNDO);
  gfx->setTextSize(1);
  gfx->setTextColor(COR_TEXTO);
  gfx->setCursor(10, y + 3);
  gfx->print(nome);
  gfx->print(" X:"); gfx->print(a, casas);
  gfx->print(" Y:"); gfx->print(b, casas);
  gfx->print(" Z:"); gfx->print(c, casas);
}

void desenhaGiro() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("GIRO");
  desenhaRodape("Giroscopio deste controle", COR_CINZA);
}

void desenhaGiroValores() {
  linhaAngulo(40, "Roll", angleX);
  linhaAngulo(80, "Pitch", angleY);
  linhaAngulo(120, "Yaw", angleZ);
  linhaRaw(170, "Acc(g)", accX, accY, accZ, 2);
  linhaRaw(186, "Gyro  ", gyroX - gyroOffX, gyroY - gyroOffY, gyroZ - gyroOffZ, 1);
}

// ---------- Sensores ----------
void desenhaSubTabs() {
  for (int i = 0; i < 5; i++) {
    Rect r = {6 + i * 46, 262, 44, 36};
    bool ativa = ((int)subSensor == i);
    gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, ativa ? COR_ACENTO : COR_CARD);
    textoCentro(ROTULO_SUB[i], r, ativa ? COR_FUNDO : COR_CINZA, 1);
  }
}

void desenhaSensorCor() {
  for (int k = 0; k < 2; k++) {
    int x = (k == 0) ? 6 : 124;
    bool ok = telem.okMask & (1 << k);
    char buf[24];
    desenhaValor(x, 40, 110, k == 0 ? "ESQ (TCA canal 0)" : "DIR (TCA canal 1)", COR_TEXTO);
    desenhaValor(x, 54, 110, ok ? "sensor OK" : "NAO ACHADO", ok ? COR_VERDE : COR_VERMELHO);
    snprintf(buf, sizeof(buf), "R: %u", (unsigned)telem.cor[k][0]); desenhaValor(x, 74, 110, buf, COR_TEXTO);
    snprintf(buf, sizeof(buf), "G: %u", (unsigned)telem.cor[k][1]); desenhaValor(x, 88, 110, buf, COR_TEXTO);
    snprintf(buf, sizeof(buf), "B: %u", (unsigned)telem.cor[k][2]); desenhaValor(x, 102, 110, buf, COR_TEXTO);
    snprintf(buf, sizeof(buf), "C: %u", (unsigned)telem.cor[k][3]); desenhaValor(x, 116, 110, buf, COR_TEXTO);

    uint16_t bg = COR_CARD;
    uint16_t fg = COR_TEXTO;
    const char *txt = "NADA";
    switch (telem.classe[k]) {
      case CLASSE_VERDE:    bg = COR_VERDE;    fg = COR_FUNDO; txt = "VERDE";    break;
      case CLASSE_VERMELHO: bg = COR_VERMELHO; fg = COR_FUNDO; txt = "VERMELHO"; break;
      case CLASSE_PRETO:    bg = COR_CINZA;    fg = COR_FUNDO; txt = "PRETO";    break;
      default: break;
    }
    gfx->fillRoundRect(x, 140, 110, 30, 8, bg);
    textoCentro(txt, Rect{x, 140, 110, 30}, fg, 1);
  }
  desenhaValor(6, 186, 228, "Verde: G>38%, R<33%, B<28% (C>=105)", COR_TEXTO);
  desenhaValor(6, 200, 228, "Vermelho: G<37%, R>48% (C>=80)", COR_TEXTO);
  desenhaValor(6, 214, 228, "Preto: C<190 (linha)", COR_TEXTO);
  desenhaValor(6, 228, 228, "L = R+G+B (mesmo filtro da OBR)", COR_CINZA);
}

void desenhaSensorRefl() {
  int ativos = 0;
  desenhaValor(6, 40, 228, "Ciano: >= 1800 (limiarD)", COR_ACENTO);
  desenhaValor(6, 52, 228, "Amarelo: >= 800 (limiar)", COR_AMARELO);
  for (int i = 0; i < 8; i++) {
    int x = 14 + i * 27;
    uint16_t v = telem.ir[i];
    if (v >= 800) ativos++;
    int h = (int)((uint32_t)v * 160 / 4095);
    if (h > 160) h = 160;
    uint16_t cor = (v >= 1800) ? COR_ACENTO : ((v >= 800) ? COR_AMARELO : COR_CARD);
    gfx->fillRect(x - 4, 66, 26, 170, COR_FUNDO);
    gfx->fillRect(x, 226 - h, 18, h, cor);
    gfx->setTextSize(1);
    gfx->setTextColor(COR_CINZA);
    gfx->setCursor(x + 5, 230);
    gfx->print(i + 1);
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "Sensores ativos (>=800): %d de 8", ativos);
  desenhaRodape(buf, COR_TEXTO);
}

void desenhaSensorUltra() {
  const char *nomes[3] = {"U1 frontal (T12/E11)", "U2 lado 1 (T21/E47)", "U3 lado 2 (T13/E14)"};
  for (int i = 0; i < 3; i++) {
    int y = 40 + i * 70;
    uint16_t mm = telem.ultra[i];
    desenhaValor(6, y, 228, nomes[i], COR_CINZA);

    char buf[16];
    gfx->fillRect(6, y + 14, 228, 20, COR_FUNDO);
    gfx->setTextSize(2);
    gfx->setTextColor(COR_TEXTO);
    gfx->setCursor(6, y + 16);
    if (mm == SEM_VALOR) {
      gfx->print("sem eco");
    } else {
      snprintf(buf, sizeof(buf), "%d.%d cm", mm / 10, mm % 10);
      gfx->print(buf);
    }

    int cm = (mm == SEM_VALOR) ? 0 : mm / 10;
    if (cm > 40) cm = 40;
    int larg = cm * 228 / 40;
    uint16_t cor = (mm != SEM_VALOR && mm / 10 < 10) ? COR_VERMELHO : COR_ACENTO;
    gfx->fillRect(6, y + 42, 228, 10, COR_CARD);
    gfx->fillRect(6, y + 42, larg, 10, cor);
  }
  desenhaRodape("Obstaculo frontal: U1 < 10 cm (vermelho)", COR_CINZA);
}

void desenhaSensorVlx() {
  const char *nomes[3] = {"VL frente (canal 2)", "VL direito (canal 5)", "VL esquerdo (canal 4)"};
  for (int i = 0; i < 3; i++) {
    int y = 40 + i * 70;
    uint16_t mm = telem.vlx[i];
    desenhaValor(6, y, 228, nomes[i], COR_CINZA);

    char buf[16];
    gfx->fillRect(6, y + 14, 228, 20, COR_FUNDO);
    gfx->setTextSize(2);
    gfx->setTextColor(COR_TEXTO);
    gfx->setCursor(6, y + 16);
    if (mm == SEM_VALOR) {
      gfx->setTextColor(COR_VERMELHO);
      gfx->print("ERRO");
    } else {
      snprintf(buf, sizeof(buf), "%u mm", (unsigned)mm);
      gfx->print(buf);
    }

    int mmBar = (mm == SEM_VALOR) ? 0 : mm;
    if (mmBar > 400) mmBar = 400;
    int larg = mmBar * 228 / 400;
    gfx->fillRect(6, y + 42, 228, 10, COR_CARD);
    gfx->fillRect(6, y + 42, larg, 10, COR_ACENTO);
  }
  desenhaRodape("Obstaculo frontal: VL frente < 170 mm", COR_CINZA);
}

// Fins de curso: pressionado = nivel LOW (como na OBR)
void desenhaSensorFim() {
  bool f1 = telem.fim & 1;
  bool f2 = telem.fim & 2;
  desenhaValor(6, 44, 228, "fimC1 (pino 9)", COR_CINZA);
  gfx->fillCircle(30, 86, 14, f1 ? COR_VERDE : COR_CARD);
  desenhaValor(60, 80, 170, f1 ? "PRESSIONADO" : "LIVRE", f1 ? COR_VERDE : COR_TEXTO);

  desenhaValor(6, 130, 228, "fimC2 (pino 10)", COR_CINZA);
  gfx->fillCircle(30, 172, 14, f2 ? COR_VERDE : COR_CARD);
  desenhaValor(60, 166, 170, f2 ? "PRESSIONADO" : "LIVRE", f2 ? COR_VERDE : COR_TEXTO);

  desenhaValor(6, 214, 228, "Pressionado = nivel LOW", COR_CINZA);
}

void desenhaSensorAtual() {
  if (!telemFresco()) {
    desenhaRodape(selected < 0 ? "Escolha um robo no modo Robos" : "Sem dados do robo", COR_VERMELHO);
    return;
  }
  switch (subSensor) {
    case SUB_COR:   desenhaSensorCor();   break;
    case SUB_REFL:  desenhaSensorRefl();  break;
    case SUB_ULTRA: desenhaSensorUltra(); break;
    case SUB_VLX:   desenhaSensorVlx();   break;
    case SUB_FIM:   desenhaSensorFim();   break;
  }
}

void desenhaSensores() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("SENSORES");
  desenhaSubTabs();
  desenhaSensorAtual();
}

// ---------- Motores por pulso ----------
void desenhaPulsoValor() {
  gfx->fillRect(6, 50, 228, 26, COR_FUNDO);
  char buf[12];
  snprintf(buf, sizeof(buf), "%u", (unsigned)pulsoSel);
  gfx->setTextSize(3);
  gfx->setTextColor(COR_TEXTO);
  gfx->setCursor(6, 52);
  gfx->print(buf);
}

void desenhaPasso() {
  for (int i = 0; i < 4; i++) {
    gfx->fillRoundRect(AREA_PASSO[i].x, AREA_PASSO[i].y, AREA_PASSO[i].w, AREA_PASSO[i].h, 8, COR_CARD);
    textoCentro(ROTULO_PASSO[i], AREA_PASSO[i], COR_TEXTO, 1);
  }
}

void desenhaDirecoes() {
  for (int i = 0; i < 4; i++) {
    bool ativa = (i == dirSel);
    gfx->fillRoundRect(AREA_DIR[i].x, AREA_DIR[i].y, AREA_DIR[i].w, AREA_DIR[i].h, 8,
                       ativa ? COR_ACENTO : COR_CARD);
    textoCentro(ROTULO_DIR[i], AREA_DIR[i], ativa ? COR_FUNDO : COR_TEXTO, 1);
  }
}

void desenhaPrevia() {
  char buf[40];
  snprintf(buf, sizeof(buf), "Pronto: %s %u pulsos", ROTULO_DIR[dirSel], (unsigned)pulsoSel);
  desenhaValor(6, 206, 228, buf, COR_TEXTO);
}

void desenhaBotoesConfirma() {
  gfx->fillRoundRect(AREA_CONFIRMA.x, AREA_CONFIRMA.y, AREA_CONFIRMA.w, AREA_CONFIRMA.h, 8, COR_VERDE);
  textoCentro("CONFIRMAR", AREA_CONFIRMA, COR_FUNDO, 1);
  gfx->fillRoundRect(AREA_PARAR.x, AREA_PARAR.y, AREA_PARAR.w, AREA_PARAR.h, 8, COR_VERMELHO);
  textoCentro("PARAR", AREA_PARAR, COR_FUNDO, 1);
}

void desenhaProgresso() {
  char buf[48];
  if (selected < 0) {
    desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
    return;
  }
  if (!telemFresco()) {
    desenhaRodape("Sem dados do robo", COR_VERMELHO);
    return;
  }
  switch (telem.estadoPulso) {
    case PULSO_EXECUTANDO:
      snprintf(buf, sizeof(buf), "Executando E %u D %u de %u",
               (unsigned)telem.pulsosE, (unsigned)telem.pulsosD, (unsigned)pulsoEnviado);
      desenhaRodape(buf, COR_ACENTO);
      break;
    case PULSO_CONCLUIDO:
      snprintf(buf, sizeof(buf), "Concluido: E %u D %u", (unsigned)telem.pulsosE, (unsigned)telem.pulsosD);
      desenhaRodape(buf, COR_VERDE);
      break;
    case PULSO_TIMEOUT:
      desenhaRodape("Tempo esgotado: verifique os encoders", COR_VERMELHO);
      break;
    case PULSO_PARADO:
      desenhaRodape("Parado pelo usuario", COR_AMARELO);
      break;
    default:
      desenhaRodape("Escolha e confirme os pulsos", COR_CINZA);
      break;
  }
}

void desenhaMotores() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("MOTORES");
  gfx->setTextSize(1);
  gfx->setTextColor(COR_CINZA);
  gfx->setCursor(6, 40);
  gfx->print("PULSOS");
  desenhaPulsoValor();
  desenhaPasso();
  desenhaDirecoes();
  desenhaPrevia();
  desenhaBotoesConfirma();
  desenhaProgresso();
}

// ---------- Garra ----------
void desenhaGarraStatus() {
  gfx->fillRect(6, 252, 228, 12, COR_FUNDO);
  gfx->setTextSize(1);
  if (!telemFresco()) {
    gfx->setTextColor(COR_CINZA);
    gfx->setCursor(6, 254);
    gfx->print("Sem dados do robo");
  } else if (telem.okMask & 32) {
    gfx->setTextColor(COR_VERDE);
    gfx->setCursor(6, 254);
    gfx->print("PCA9685 CONECTADA (canal 3, 0x40)");
  } else {
    gfx->setTextColor(COR_VERMELHO);
    gfx->setCursor(6, 254);
    gfx->print("PCA9685 NAO ENCONTRADA: garra off");
  }
}

void desenhaGarra() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("GARRA");
  bool ok = pcaConectada();
  for (int i = 0; i < 5; i++) {
    Rect r = {6, 40 + i * 42, 228, 36};
    gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, ok ? COR_ACENTO : COR_CARD);
    textoCentro(ROTULO_GARRA[i], r, ok ? COR_FUNDO : COR_CINZA, 1);
  }
  desenhaGarraStatus();
  desenhaRodape("Comandos da garra (robo parado)", COR_CINZA);
}

// ---------- RGB ----------
void desenhaRgb() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("RGB");
  for (int i = 0; i < 3; i++) {
    Rect r = {6 + i * 77, 40, 74, 28};
    bool ativo = (ladoRgb == i);
    gfx->fillRoundRect(r.x, r.y, r.w, r.h, 6, ativo ? COR_ACENTO : COR_CARD);
    textoCentro(ROTULO_LADO[i], r, ativo ? COR_FUNDO : COR_TEXTO, 1);
  }
  for (int c = 0; c < 16; c++) {
    int x = 6 + (c % 4) * 57;
    int y = 80 + (c / 4) * 40;
    uint16_t cor = rgb565(PALETA[c][0], PALETA[c][1], PALETA[c][2]);
    gfx->fillRoundRect(x, y, 51, 34, 6, cor);
    if (c == corRgbSel) gfx->drawRoundRect(x - 2, y - 2, 55, 38, 6, COR_TEXTO);
  }
  gfx->fillRoundRect(6, 246, 228, 16, 6, COR_CARD);
  textoCentro("APAGAR", Rect{6, 246, 228, 16}, COR_TEXTO, 1);
  desenhaRodape(corRgbSel >= 0 ? NOME_COR[corRgbSel] : "Toque em uma cor", COR_CINZA);
}

// ---------- OBR ----------
void desenhaObrStatus() {
  gfx->fillRect(6, 226, 228, 60, COR_FUNDO);
  gfx->setTextSize(2);
  if (!telemFresco()) {
    gfx->setTextColor(COR_CINZA);
    gfx->setCursor(6, 240);
    gfx->print("Sem dados do robo");
  } else if (telem.estado & 2) {
    gfx->setTextColor(COR_VERMELHO);
    gfx->setCursor(6, 240);
    gfx->print("LINHA VERMELHA");
  } else if (telem.estado & 1) {
    gfx->setTextColor(COR_VERDE);
    gfx->setCursor(6, 240);
    gfx->print("OBR RODANDO");
  } else {
    gfx->setTextColor(COR_CINZA);
    gfx->setCursor(6, 240);
    gfx->print("OBR PARADA");
  }
}

void desenhaObr() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("OBR");
  gfx->fillRoundRect(6, 50, 228, 70, 10, COR_VERDE);
  textoCentro("INICIAR OBR", Rect{6, 50, 228, 70}, COR_FUNDO, 2);
  gfx->fillRoundRect(6, 150, 228, 60, 10, COR_VERMELHO);
  textoCentro("PARAR OBR", Rect{6, 150, 228, 60}, COR_FUNDO, 2);
  desenhaObrStatus();
  desenhaRodape("Parar OBR reinicia o robo (~3 s)", COR_CINZA);
}

// ---------- Robos (tela) ----------
void desenhaListaRobos() {
  for (int i = 0; i < MAX_ROBOS; i++) {
    int y = 76 + i * 34;
    if (!robos[i].usado) {
      gfx->fillRoundRect(10, y, 220, 30, 8, COR_FUNDO);
      continue;
    }
    bool sel = (i == selected);
    bool on = (millis() - robos[i].lastSeen) < ONLINE_MS;
    uint16_t bg = sel ? COR_ACENTO : COR_CARD;
    uint16_t fg = sel ? COR_FUNDO : COR_TEXTO;
    gfx->fillRoundRect(10, y, 220, 30, 8, bg);
    gfx->setTextSize(1);
    gfx->setTextColor(fg);
    gfx->setCursor(18, y + 5);
    gfx->print(robos[i].name);

    char mac[12];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X", robos[i].mac[3], robos[i].mac[4], robos[i].mac[5]);
    gfx->setCursor(18, y + 17);
    gfx->print(mac);

    gfx->setTextColor(sel ? COR_FUNDO : (on ? COR_VERDE : COR_VERMELHO));
    gfx->setCursor(150, y + 17);
    gfx->print(on ? "ONLINE" : "sem sinal");
  }
}

void desenhaRobos() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("ROBOS");
  gfx->fillRoundRect(AREA_BUSCAR.x, AREA_BUSCAR.y, AREA_BUSCAR.w, AREA_BUSCAR.h, 8, COR_ACENTO);
  textoCentro("BUSCAR ROBOS", AREA_BUSCAR, COR_FUNDO, 1);
  desenhaListaRobos();
  desenhaRodape(statusText, COR_CINZA);
}

// ---------- Desenho do modo Mover (botao ativo) ----------
void desenhaGiroEstatico() {}

// ---------- Navegacao ----------
void trocaTela(Tela t) {
  Tela anterior = telaAtual;
  telaAtual = t;
  botaoAtivo = -1;
  enviaComando(CMD_STOP);   // o robo ignora este STOP quando a OBR esta ativa
  if (anterior == TELA_CAMERA) pedeCam(0);   // saiu da camera: CAM para
  switch (t) {
    case TELA_MENU:     desenhaMenu();     break;
    case TELA_MOVER:    desenhaMover();    break;
    case TELA_GIRO:     desenhaGiro();     break;
    case TELA_SENSORES: desenhaSensores(); break;
    case TELA_MOTORES:  desenhaMotores();  break;
    case TELA_GARRA:    desenhaGarra();    break;
    case TELA_RGB:      desenhaRgb();      break;
    case TELA_OBR:      desenhaObr();      break;
    case TELA_ROBOS:    desenhaRobos();    break;
    case TELA_CAMERA:   camPingMs = 0; desenhaCamera(); break;
  }
}

void tratarToque(int16_t x, int16_t y) {
  if (telaAtual == TELA_MENU) {
    for (int i = 0; i < 9; i++) {
      if (dentro(x, y, areaMenu(i))) {
        trocaTela((Tela)(i + 1));
        return;
      }
    }
    return;
  }

  if (dentro(x, y, AREA_VOLTAR)) {
    trocaTela(TELA_MENU);
    return;
  }

  switch (telaAtual) {
    case TELA_MOVER: {
      int b = -1;
      for (int i = 0; i < 6; i++) if (dentro(x, y, AREA_MOV[i])) b = i;
      if (b < 0) return;
      if (selected < 0) {
        desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
        return;
      }
      botaoAtivo = b;
      enviaComando(CMD_DO_BOTAO[b]);
      ultimoEnvio = millis();
      desenhaBotaoMov(b);
      break;
    }

    case TELA_SENSORES:
      for (int i = 0; i < 5; i++) {
        Rect r = {6 + i * 46, 262, 44, 36};
        if (dentro(x, y, r)) {
          subSensor = (SubSensor)i;
          desenhaSensores();
          return;
        }
      }
      break;

    case TELA_MOTORES:
      for (int i = 0; i < 4; i++) {
        if (dentro(x, y, AREA_PASSO[i])) {
          int v = (int)pulsoSel + VALOR_PASSO[i];
          if (v < PULSO_MIN) v = PULSO_MIN;
          if (v > PULSO_MAX) v = PULSO_MAX;
          pulsoSel = (uint16_t)v;
          desenhaPulsoValor();
          desenhaPrevia();
          return;
        }
      }
      for (int i = 0; i < 4; i++) {
        if (dentro(x, y, AREA_DIR[i])) {
          dirSel = i;
          desenhaDirecoes();
          desenhaPrevia();
          return;
        }
      }
      if (dentro(x, y, AREA_CONFIRMA)) {
        if (selected < 0) {
          desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
          return;
        }
        PacotePulsos p;
        p.type = MSG_PULSOS;
        p.dir = dirSel;
        p.pulsos = pulsoSel;
        p.vel = VEL_PULSO;
        esp_now_send(robos[selected].mac, (uint8_t *)&p, sizeof(p));
        pulsoEnviado = pulsoSel;
        desenhaRodape("Comando enviado ao robo", COR_ACENTO);
        return;
      }
      if (dentro(x, y, AREA_PARAR)) {
        enviaComando(CMD_STOP);
        return;
      }
      break;

    case TELA_GARRA:
      for (int i = 0; i < 5; i++) {
        Rect r = {6, 40 + i * 42, 228, 36};
        if (dentro(x, y, r)) {
          if (selected < 0) {
            desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
          } else if (!pcaConectada()) {
            desenhaRodape("PCA nao conectada: garra desativada", COR_VERMELHO);
          } else {
            Msg m = {};
            m.type = MSG_GARRA;
            m.a = COD_GARRA[i];
            esp_now_send(robos[selected].mac, (uint8_t *)&m, sizeof(m));
            desenhaRodape("Garra: comando enviado", COR_ACENTO);
          }
          return;
        }
      }
      break;

    case TELA_RGB:
      for (int i = 0; i < 3; i++) {
        Rect r = {6 + i * 77, 40, 74, 28};
        if (dentro(x, y, r)) {
          ladoRgb = i;
          desenhaRgb();
          return;
        }
      }
      for (int c = 0; c < 16; c++) {
        Rect r = {6 + (c % 4) * 57, 80 + (c / 4) * 40, 51, 34};
        if (dentro(x, y, r)) {
          corRgbSel = c;
          if (selected >= 0) {
            PacoteLed p = {MSG_LED, ladoRgb, PALETA[c][0], PALETA[c][1], PALETA[c][2]};
            esp_now_send(robos[selected].mac, (uint8_t *)&p, sizeof(p));
          }
          desenhaRgb();
          return;
        }
      }
      if (dentro(x, y, Rect{6, 246, 228, 16})) {
        corRgbSel = -1;
        if (selected >= 0) {
          PacoteLed p = {MSG_LED, ladoRgb, 0, 0, 0};
          esp_now_send(robos[selected].mac, (uint8_t *)&p, sizeof(p));
        }
        desenhaRgb();
      }
      break;

    case TELA_OBR:
      if (dentro(x, y, Rect{6, 50, 228, 70})) {
        if (selected < 0) {
          desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
        } else {
          Msg m = {};
          m.type = MSG_OBR;
          m.a = 1;
          esp_now_send(robos[selected].mac, (uint8_t *)&m, sizeof(m));
          desenhaRodape("Iniciando OBR...", COR_ACENTO);
        }
      } else if (dentro(x, y, Rect{6, 150, 228, 60})) {
        if (selected < 0) {
          desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
        } else {
          Msg m = {};
          m.type = MSG_OBR;
          m.a = 0;
          esp_now_send(robos[selected].mac, (uint8_t *)&m, sizeof(m));
          desenhaRodape("Parando OBR (robo reinicia)...", COR_AMARELO);
        }
      }
      break;

    case TELA_CAMERA:
      for (int b = 0; b < 4; b++) {
        if (dentro(x, y, AREA_CAM_MOV[b])) {
          if (selected < 0) {
            desenhaRodape("Escolha um robo no modo Robos", COR_VERMELHO);
            return;
          }
          botaoAtivo = b;
          enviaComando(CMD_DO_BOTAO[b]);
          ultimoEnvio = millis();
          desenhaBotaoCam(b);
          return;
        }
      }
      break;

    case TELA_ROBOS:
      if (dentro(x, y, AREA_BUSCAR)) {
        startScan();
      } else {
        for (int i = 0; i < MAX_ROBOS; i++) {
          Rect r = {10, 76 + i * 34, 220, 30};
          if (dentro(x, y, r) && robos[i].usado) {
            selectRobo(i);
            break;
          }
        }
      }
      desenhaRobos();
      break;

    default:
      break;
  }
}

// ---------- Camera (ESP-NOW) ----------
// Pede transmissao por broadcast (a=1 transmite, a=0 para)
void pedeCam(uint8_t liga) {
  Msg m = {};
  m.type = MSG_CAM_CTRL;
  m.a = liga;
  strncpy(m.name, "CONTROLE", 15);
  esp_now_send(BROADCAST, (uint8_t *)&m, sizeof(m));
}

// Monta o frame a partir dos pedacos (roda no callback do WiFi)
void recebeParte(const uint8_t *data, int len) {
  FrameChunk c = {};
  memcpy(&c, data, len);
  camUltimoMs = millis();
  if (c.len != len - CAM_CAB || c.total == 0 || c.total > CAM_MAX_PARTES ||
      c.seq >= c.total || c.tamanho > CAM_MAX_FRAME) return;

  if (c.frameId != camFrameRx) {          // novo frame: reinicia a montagem
    camFrameRx = c.frameId;
    camLenRx = c.tamanho;
    camTotalRx = c.total;
    camRecebidas = 0;
    memset(camMapa, 0, sizeof(camMapa));
  }
  if (c.total != camTotalRx || c.tamanho != camLenRx) return;

  if (!camMapa[c.seq]) {
    uint32_t off = (uint32_t)c.seq * CAM_CHUNK;
    if (off + c.len > CAM_MAX_FRAME) return;
    memcpy(camRx + off, c.dados, c.len);
    camMapa[c.seq] = 1;
    camRecebidas++;
  }
  if (camRecebidas == camTotalRx) {       // frame completo
    if (!camBusy) {
      memcpy(camShow, camRx, camLenRx);
      camLenShow = camLenRx;
      camPronto = true;
    }
    camFrameRx = 0xFFFF;
  }
}

void desenhaBotaoCam(int b) {
  const Rect &r = AREA_CAM_MOV[b];
  bool ativo = (b == botaoAtivo);
  uint16_t bg = ativo ? COR_ACENTO : COR_CARD;
  uint16_t fg = ativo ? COR_FUNDO : COR_ACENTO;
  gfx->fillRoundRect(r.x, r.y, r.w, r.h, 8, bg);
  int cx = r.x + r.w / 2;
  int cy = r.y + r.h / 2;
  switch (b) {
    case 0: gfx->fillTriangle(cx, cy - 8, cx - 8, cy + 6, cx + 8, cy + 6, fg); break;
    case 1: gfx->fillTriangle(cx, cy + 8, cx - 8, cy - 6, cx + 8, cy - 6, fg); break;
    case 2: gfx->fillTriangle(cx - 8, cy, cx + 6, cy - 8, cx + 6, cy + 8, fg); break;
    case 3: gfx->fillTriangle(cx + 8, cy, cx - 6, cy - 8, cx - 6, cy + 8, fg); break;
  }
}

void desenhaBotoesCam() {
  for (int b = 0; b < 4; b++) desenhaBotaoCam(b);
}

// Cada bloco decodificado vai para a area da imagem
bool camDesenhaBloco(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bmp) {
  gfx->draw16bitRGBBitmap(CAM_VIEW_X + x, CAM_VIEW_Y + y, bmp, w, h);
  return true;
}

void desenhaCamera() {
  gfx->fillScreen(COR_FUNDO);
  desenhaCabecalho("CAMERA");
  gfx->fillRect(CAM_VIEW_X, CAM_VIEW_Y, CAM_VIEW_W, CAM_VIEW_H, COR_FUNDO);
  desenhaBotoesCam();
  camUltimoMs = 0;
  camFpsMs = millis();
  camFpsCont = 0;
  camFps = 0;
}

// Decodifica e mostra o ultimo frame completo (a cada volta do loop)
void atualizaCameraImagem() {
  if (camPronto && !camBusy) {
    camPronto = false;
    camBusy = true;
    TJpgDec.drawJpg(0, 0, camShow, camLenShow);
    camBusy = false;
    camFpsCont++;
  }
}

// Rodape com status do video e do robo (a cada 200 ms)
void atualizaCameraRodape() {
  if (millis() - camFpsMs >= 1000) {
    camFps = camFpsCont;
    camFpsCont = 0;
    camFpsMs = millis();
  }
  bool sinal = camUltimoMs != 0 && (millis() - camUltimoMs) < CAM_TIMEOUT_MS;
  char buf[48];
  if (sinal) snprintf(buf, sizeof(buf), "Ao vivo: %u fps", (unsigned)camFps);
  else       snprintf(buf, sizeof(buf), "Procurando CAM...");
  if (selected < 0) strncat(buf, " | sem robo", sizeof(buf) - strlen(buf) - 1);
  desenhaRodape(buf, (sinal && selected >= 0) ? COR_VERDE : COR_VERMELHO);
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);
  memset(&telem, 0, sizeof(telem));

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  gfx->begin();
  gfx->fillScreen(COR_FUNDO);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  qmiInit();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Falha ao iniciar ESP-NOW");
  } else {
    esp_now_register_recv_cb(onRecv);
    ensurePeer(BROADCAST);
  }

  Serial.print("MAC do controle: ");
  Serial.println(WiFi.macAddress());

  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(camDesenhaBloco);

  calibraGiro();
  desenhaMenu();
}

void loop() {
  if (millis() - ultimoIMU >= 20) {
    ultimoIMU = millis();
    qmiReadRaw();
    updateAngles();
  }

  // Atualiza cabecalho e o conteudo dinamico da tela atual (200 ms)
  if (millis() - ultimoUI >= 200) {
    ultimoUI = millis();
    desenhaStatus();
    switch (telaAtual) {
      case TELA_SENSORES: desenhaSensorAtual(); break;
      case TELA_MOTORES:  desenhaProgresso();   break;
      case TELA_GARRA:    desenhaGarraStatus(); break;
      case TELA_OBR:      desenhaObrStatus();   break;
      case TELA_ROBOS:    desenhaListaRobos();  break;
      case TELA_CAMERA:   atualizaCameraRodape(); break;
      default: break;
    }
  }

  if (needRedraw) {
    needRedraw = false;
    if (telaAtual == TELA_ROBOS) desenhaRobos();
  }

  // Camera: decodifica a cada volta e pede transmissao a cada 500 ms
  if (telaAtual == TELA_CAMERA) {
    atualizaCameraImagem();
    if (millis() - camPingMs >= CAM_PING_MS) {
      camPingMs = millis();
      pedeCam(1);
    }
  }

  if (telaAtual == TELA_GIRO && millis() - ultimoGiro >= 100) {
    ultimoGiro = millis();
    desenhaGiroValores();
  }

  int16_t tx = 0, ty = 0;
  bool tocando = readTouch(tx, ty);
  if (tocando && !estavaTocando) {
    tratarToque(tx, ty);
  }
  if (!tocando && estavaTocando && botaoAtivo >= 0) {
    int antigo = botaoAtivo;
    botaoAtivo = -1;
    enviaComando(CMD_STOP);
    if (telaAtual == TELA_CAMERA) desenhaBotaoCam(antigo);
    else                          desenhaBotaoMov(antigo);
  }
  estavaTocando = tocando;

  // Camera: repete o comando enquanto o botao do robo estiver preso
  if (telaAtual == TELA_CAMERA && botaoAtivo >= 0 && millis() - ultimoEnvio >= INTERVALO_ENVIO_MS) {
    enviaComando(CMD_DO_BOTAO[botaoAtivo]);
    ultimoEnvio = millis();
  }

  // So o modo Mover envia comandos continuos
  if (telaAtual == TELA_MOVER && millis() - ultimoEnvio >= INTERVALO_ENVIO_MS) {
    enviaComando(botaoAtivo >= 0 ? CMD_DO_BOTAO[botaoAtivo] : CMD_STOP);
    ultimoEnvio = millis();
  }

  delay(10);
}