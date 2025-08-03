#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include "Command.h"
#include "My_Led.h"

class Set_Led : public Command{
    private:
        My_Led *my_led;
    public:
        Set_Led(My_Led *led) : my_led(led){}
        void run() override;
};

class Clr_Led : public Command {
    private:
        My_Led *my_led;
    public:
        Clr_Led(My_Led *led) : my_led(led){}
        void run() override;
};

class Config_Led : public Command{
    private:
        My_Led *my_led;
    public:
        Config_Led(My_Led *led) : my_led(led){}
        void run() override;
};

#endif