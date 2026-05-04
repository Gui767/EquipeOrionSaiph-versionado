#include "MotorControl.h"

// =================================================================================
// CONSTRUTOR
// =================================================================================
MotorControl::MotorControl(int encA, int encB, int pwm1, int pwm2) {
    _pinA = encA;
    _pinB = encB;
    _pwm1 = pwm1;
    _pwm2 = pwm2;
    _counter = 0;
    _target = 0;
    _isMoving = false;
    _speed = 0;
}

// =================================================================================
// INICIALIZAÇÃO
// =================================================================================
void MotorControl::begin(void (*isr)()) {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    
    // Configura a interrupção no pino A
    attachInterrupt(digitalPinToInterrupt(_pinA), isr, RISING);

    // Configuração do PWM (específico para ESP32)
    ledcAttach(_pwm1, 10000, 8); // 10kHz, 8 bits de resolução
    ledcAttach(_pwm2, 10000, 8);
}

// =================================================================================
// MOVIMENTAÇÃO
// =================================================================================
void MotorControl::move(long steps, int speed) {
    _counter = 0;
    _target = steps;
    _speed = speed;
    _isMoving = true;

    // Determina a direção baseada no sinal de 'steps'
    bool frente = (steps > 0);

    if (frente) {
        ledcWrite(_pwm1, _speed);
        ledcWrite(_pwm2, 0);
    } else {
        ledcWrite(_pwm1, 0);
        ledcWrite(_pwm2, _speed);
    }
}

void MotorControl::stop() {
    ledcWrite(_pwm1, 0);
    ledcWrite(_pwm2, 0);
    _isMoving = false;
}

// =================================================================================
// ATUALIZAÇÃO (Lógica de parada)
// =================================================================================
bool MotorControl::update() {
    // Se estiver movendo e a contagem atingir ou ultrapassar o alvo
    if (_isMoving && abs(_counter) >= abs(_target)) {
        stop();
        return true; // Retorna true informando que CHEGOU ao alvo
    }
    return false; // Retorna false se ainda estiver no caminho ou parado
}

// =================================================================================
// TRATAMENTO DE INTERRUPÇÃO E GETTERS
// =================================================================================
void MotorControl::handleInterrupt() {
    // Lógica de quadratura simples
    if (digitalRead(_pinB) == LOW) {
        _counter++;
    } else {
        _counter--;
    }
}

long MotorControl::getCount() {
    return abs(_counter);
}

bool MotorControl::isMoving() {
    return _isMoving;
}