#include <Arduino.h>
#include "MotorControl_L293D.hpp"


#define Rf 13
#define Rb 12
#define Lf 8
#define Lb 7
#define enR 11
#define enL 10

Bot bot(Rf, Rb, Lf, Lb, enR, enL);

uint8_t fast = 255;
uint8_t mid = (255/2) + 10;
uint8_t slow = 10;

void setup() {
    bot.init();
    bot.stop(0);
}

void loop() {
    bot.forward(1, fast);
    bot.reverse(1, fast);
    bot.left(1, fast);
    bot.right(1, fast);

    bot.forward(1, mid);
    bot.reverse(1, mid);
    bot.left(1, mid);
    bot.right(1, mid);

    bot.forward(1, slow);
    bot.reverse(1, slow);
    bot.left(1, slow);
    bot.right(1, slow);
}