#ifndef CONTROL_H
#define CONTROL_H
#include "Command.h"
#include "driver/gpio.h"
#include "esp_log.h"


class Control{
    int16_t input_low  = 20;
    int16_t input_high = 30;
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
        void change_param(int16_t low_param, int16_t high_param);
        void my_input(float input);
};

#endif