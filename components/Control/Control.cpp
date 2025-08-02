#include <stdio.h>
#include "Control.h"


Control::~Control(){
    delete config_my_high_action;
    delete config_my_low_action;
    delete set_my_high_action;
    delete set_my_low_action;
    delete clr_my_high_action;
    delete clr_my_low_action;
}
void Control::define_config_my_high_action(Command *command){
    this->config_my_high_action = command;
}
void Control::define_config_my_low_action(Command *command){
    this->config_my_low_action = command;
}
void Control::define_set_my_high_action(Command *command){
    this->set_my_high_action = command;
}
void Control::define_set_my_low_action(Command *command){
    this->set_my_low_action = command;
}
void Control::define_clr_my_high_action(Command *command){
    this->clr_my_high_action = command;
}
void Control::define_clr_my_low_action(Command *command){
    this->clr_my_low_action = command;
}
void Control::config_my_actions(){
    if(this->config_my_high_action != NULL && this->config_my_low_action != NULL){
        this->config_my_high_action->run();
        this->config_my_low_action->run();
    }
}
void Control::my_input(float input){
    if(this->set_my_high_action != NULL && this->set_my_low_action != NULL && this->clr_my_high_action != NULL && this->clr_my_low_action != NULL){
            if(input > INPUT_HIGH){
                this->set_my_low_action->run();
                this->set_my_high_action->run();
            }else if(input < INPUT_LOW) {
                this->clr_my_low_action->run();
                this->clr_my_high_action->run();
            }else{
                this->set_my_low_action->run();
                this->clr_my_high_action->run();
            }
    }
}

