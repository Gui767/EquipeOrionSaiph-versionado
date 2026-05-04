// Array com os pinos ADC do ESP32-S3 escolhidos para a leitura
const int sensorPins[8] = {6, 8, 3, 9, 10, 7, 4, 5};
//5, 4, 7, 10, 9, 3, 8, 6 da direita pra esquerda do robô
// Array para armazenar as leituras analógicas (0 a 4095)
int sensorValues[8];



void setup() {
  Serial.begin(115200);

  // Não é estritamente obrigatório declarar pinMode para analogRead no ESP32, 
  // mas é uma boa prática para garantir que o pino não está em outro modo.
for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println("Iniciando leitura ANALOGICA do modulo de 8 canais...");
  delay(1000);

}

void loop() {
  Serial.print("Leituras: ");
  
  // Realiza a leitura analógica de cada pino
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    
    // Imprime o valor no monitor serial
    Serial.print(sensorValues[i]);
    Serial.print("\t"); // Adiciona um espaço de tabulação para manter as colunas alinhadas
  }
  
  Serial.println(); // Pula para a próxima linha
  delay(100); 
}