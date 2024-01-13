#include <Arduino.h>
#include <PID.h>

// Pinos de entrada e saída.
const int INPUT_PIN = 10;
const int OUTPUT_PIN = 20;

// Setpoint.
const float SETPOINT = 512;

// Parâmetros básicos do PID.
const float K_P = 1;             // [1]
const float K_I = 1;             // [1/s]
const float K_D = 1;             // [s]
const float SAMPLE_TIME = 20e-3; // [s]

// Cria um objeto do tipo PID com os parâmetros definidos.
PID myPid(K_P, K_I, K_D, SAMPLE_TIME);

// Parâmetros extra do PID.
const PID::Direction direction = PID::Direction::DIRECT; // Direção.
const float lo = 0;                                      // Saída mínima.
const float hi = 255;                                    // Saída máxima.

void setup()
{
    // Configura o PID com os parâmetros extra.
    myPid.setDirection(direction);
    myPid.setBounds(lo, hi);

    // Configura os pinos.
    pinMode(INPUT_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
}

void loop()
{
    // Executa o loop a cada SAMPLE_TIME.
    const static long ST_MS = SAMPLE_TIME * 1000; // [ms]

    static long lastTime = millis();
    long time;
    while ((time = millis()) - lastTime < ST_MS)
        ;
    lastTime = time;

    // Computa o novo valor de saída.
    float input = analogRead(INPUT_PIN);
    float output = myPid.compute(input, SETPOINT);
    analogWrite(OUTPUT_PIN, (int)output);
}
