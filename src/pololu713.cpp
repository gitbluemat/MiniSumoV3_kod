#include <Arduino.h>
#include <avr/io.h>

#define AIN1  8
#define AIN2  9

#define BIN1 PD6
#define BIN2 PD7

#define PWMA PD3
#define PWMB PD5

class Motor {
    public:
    void forward(int speed) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN2, LOW);
        digitalWrite(BIN1, HIGH);
        analogWrite(PWMA, speed);
        analogWrite(PWMB, speed);
    }
    void backward(int speed) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN2, HIGH);
        digitalWrite(BIN1, LOW);
        analogWrite(PWMA, speed);
        analogWrite(PWMB, speed);
    }
    void left(int speed){
        digitalWrite(BIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, speed);
        analogWrite(PWMB, speed);
    }
    void right(int speed){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN2, LOW);
        digitalWrite(BIN1, HIGH);
        analogWrite(PWMA, speed);
        analogWrite(PWMB, speed);
    }
};