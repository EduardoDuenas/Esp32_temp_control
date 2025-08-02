#ifndef MYLED_H
#define MYLED_H
#include "driver/gpio.h"
#include <cstdint>

class My_Led{
    private:
        int16_t led_pin=-1;     //led io pin set initially as -1 indicationg unvalid number
    public:
        My_Led();
        My_Led(int16_t led_gpio);
        void config();
        void set();
        void clr();
};

#endif