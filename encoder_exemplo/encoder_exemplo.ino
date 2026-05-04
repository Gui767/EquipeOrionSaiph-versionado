#include "MotorControl.h"

// Instanciando os motores
MotorControl m1(42, 2, 13, 14);
MotorControl m2(16, 17, 47, 21);

// "Pontes" para a interrupção
void IRAM_ATTR isr1() { m1.handleInterrupt(); }
void IRAM_ATTR isr2() { m2.handleInterrupt(); }

void setup() {
    Serial.begin(115200);
    
    m1.begin(isr1);
    m2.begin(isr2);

    Serial.println("Motores prontos!");
}

void loop() {
    // 1. RECEBIMENTO DE COMANDO (Igual ao anterior)
    if (Serial.available() > 0) {
        long alvo = Serial.parseInt();
        if (alvo != 0) {
            m1.move(alvo, 120);
            m2.move(alvo, 120);
        }
        while(Serial.available()) Serial.read();
    }

    // 2. LÓGICA DE PARADA COMPARTILHADA
    // Se o motor 1 OU o motor 2 atingirem o alvo...
    if (m1.update() || m2.update()) {
        // Forçamos a parada de ambos por segurança
        if (m1.isMoving()) m1.stop();
        if (m2.isMoving()) m2.stop();
        
        Serial.println(">> ALVO ATINGIDO (Um dos motores chegou)");
    }

    // 3. DEBUG (Opcional)
    static unsigned long t = 0;
    if (millis() - t > 300 && (m1.isMoving() || m2.isMoving())) {
        Serial.printf("M1: %ld | M2: %ld\n", m1.getCount(), m2.getCount());
        t = millis();
    }
}