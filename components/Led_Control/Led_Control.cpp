#include <stdio.h>
#include "Led_Control.h"
       
void Set_Led::run(){
    my_led->set();
}

void Clr_Led::run(){
    my_led->clr();
}

void Config_Led::run(){
    my_led->config();
}
