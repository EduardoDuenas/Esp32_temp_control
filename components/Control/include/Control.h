#ifndef CONTROL_H
#define CONTROL_H
#include "Command.h"
#include "driver/gpio.h"

int16_t INPUT_LOW  = 20;
int16_t INPUT_HIGH = 30;

class Control{
    private:
        Command *config_my_high_action;
        Command *config_my_low_action;
        Command *set_my_high_action;
        Command *set_my_low_action;
        Command *clr_my_high_action;
        Command *clr_my_low_action;
    public:
        ~Control();
        void define_config_my_high_action(Command *command);
        void define_config_my_low_action(Command *command);
        void define_set_my_high_action(Command *command);
        void define_set_my_low_action(Command *command);
        void define_clr_my_high_action(Command *command);
        void define_clr_my_low_action(Command *command);
        void config_my_actions();
        void my_input(float input);
};

#endif