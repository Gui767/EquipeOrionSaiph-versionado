#include "IReight_model.hpp"

// Usando a Serial1 do ESP32-S3
#define IRMODELSerial Serial1 
#define Package_size 100

uint8_t IR_Data_number[IR_Num] = {0};
uint16_t IR_Data_Anglo[IR_Num] = {0};
uint8_t g_new_package_flag = 0;
uint8_t new_package[Package_size] = {0};

void serial_init(void) {
    Serial.begin(115200);
    // Configuração para ESP32-S3 (ajuste os pinos RX/TX se necessário)
    IRMODELSerial.begin(115200, SERIAL_8N1, 18, 17); 
}

void recv_data(void) {
    while (IRMODELSerial.available()) {
        Data_Deal(IRMODELSerial.read());
    }
}

void SET_Eight_Mode(uint8_t adjust, uint8_t ao, uint8_t num) {
    char buf[32];
    sprintf(buf, "$%d,%d,%d#", adjust, ao, num);
    IRMODELSerial.print(buf);
    delay(300);
}

void Data_Deal(char rxtemp) {
    static uint8_t step = 0;
    static bool started = false;
    if (rxtemp == '$') { 
        started = true; 
        step = 0; 
    }
    if (started) {
        new_package[step++] = rxtemp;
        if (rxtemp == '#') {
            g_new_package_flag = 1;
            started = false;
        }
        if (step >= Package_size) { 
            started = false; 
            step = 0; 
        }
    }
}

void Deal_Usart_AData(void) {
    if (new_package[1] != 'A') return;

    char str_copy[Package_size];
    memcpy(str_copy, (char*)new_package, Package_size);
    
    char *token = strtok(str_copy, ",#"); 
    int i = 0;
    
    while (token != NULL && i < IR_Num) {
        char *separator = strchr(token, ':');
        if (separator) {
            IR_Data_Anglo[i] = atoi(separator + 1);
            i++;
        }
        token = strtok(NULL, ",#");
    }

    // Print formatado para o Serial Monitor
    for(int j = 0; j < IR_Num; j++) {
        Serial.print("A"); Serial.print(j+1); 
        Serial.print(":"); Serial.print(IR_Data_Anglo[j]);
        Serial.print(" ");
    }
    Serial.println();
    
    memset(new_package, 0, Package_size);
}

void Deal_Usart_Data(void) {
    if (new_package[1] != 'D') return;
    // Processamento digital simplificado
    for(int i=0; i<IR_Num; i++) {
        IR_Data_number[i] = new_package[6 + i*5] - '0';
    }
    memset(new_package, 0, Package_size);
}