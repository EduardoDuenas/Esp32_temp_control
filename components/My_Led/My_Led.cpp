#include <stdio.h>
#include "My_Led.h"

My_Led::My_Led(){}
My_Led::My_Led(int16_t led_gpio){
    led_pin = led_gpio;
}
void My_Led::config(){
    if (led_pin == -1) return;
    esp_rom_gpio_pad_select_gpio(led_pin);
    gpio_set_direction((gpio_num_t) led_pin, GPIO_MODE_OUTPUT); //set io pin as output
}
void My_Led::set(){
    if (led_pin == -1) return;
    gpio_set_level((gpio_num_t)led_pin, true);
}
void My_Led::clr(){
    if (led_pin == -1) return;
    gpio_set_level((gpio_num_t)led_pin, false);
}