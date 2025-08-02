//#define TEST
#ifndef TEST
#define PATTERN
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "math.h"
#include "soc/gpio_num.h"

#ifdef __cplusplus
}
#endif

#include "Control.h"
#include "My_Led.h"
#include "Led_Control.h"



#define I2C_MASTER_TIMEOUT_MS       	1000

#define BMP280_I2C_ADDR                 0x77
#define I2C_CLK_Frequency               100000

#define START_READ_CALC_A1          	0x88  // Calibration data (16 bits)
#define CALIBR_LEN						26

#define BMP280_CONTROL_REGISTER 		0xF4  // Control register
#define BMP280_RESULTS			        0xF6  // Read results here
#define BMP280_REQUEST_TEMP_CMD         0x2E  // Request temperature measurement
#define BMP280_REQUEST_PRESSURE_CMD     0x34  // Request pressure measurement

#define LED_LOW							GPIO_NUM_4
#define LED_HIGH						GPIO_NUM_5


TaskHandle_t read_bmp_280_task_handle = NULL;

#ifdef TEST

void config(int16_t led_pin){
    esp_rom_gpio_pad_select_gpio(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
}
void set(int16_t led_pin){
    gpio_set_level(led_pin, true);
}
void clr(int16_t led_pin){
    gpio_set_level(led_pin, false);
}

#else
/* class Command{  //parent Class for commands
    public:
        virtual ~Command(){}
        virtual void run() = 0; //command place holder
};

class My_Led{
    private:
        int16_t led_pin=-1;     //led io pin set initially as -1 indicationg unvalid number
    public:
        My_Led(){}
        My_Led(int16_t led_gpio){
            led_pin = led_gpio;
        }
        void config(){
            if (led_pin == -1) return;
            esp_rom_gpio_pad_select_gpio(led_pin);
            gpio_set_direction((gpio_num_t) led_pin, GPIO_MODE_OUTPUT); //set io pin as output
        }
        void set(){
            if (led_pin == -1) return;
            gpio_set_level((gpio_num_t)led_pin, true);
        }
        void clr(){
            if (led_pin == -1) return;
            gpio_set_level((gpio_num_t)led_pin, false);
        }
};

class Set_Led : public Command{
    private:
        My_Led *my_led;
    public:
        Set_Led(My_Led *led) : my_led(led){}
        void run() override{
            my_led->set();
        }
};

class Clr_Led : public Command{
    private:
        My_Led *my_led;
    public:
        Clr_Led(My_Led *led) : my_led(led){}
        void run() override{
            my_led->clr();
        }
};

class Config_Led : public Command{
    private:
        My_Led *my_led;
    public:
        Config_Led(My_Led *led) : my_led(led){}
        void run() override{
            my_led->config();
        }
};

class Control{
    private:
        Command *config_my_high_led;
        Command *config_my_low_led;
        Command *set_my_high_led;
        Command *set_my_low_led;
        Command *clr_my_high_led;
        Command *clr_my_low_led;
    public:
        ~Control(){
            delete config_my_high_led;
            delete config_my_low_led;
            delete set_my_high_led;
            delete set_my_low_led;
            delete clr_my_high_led;
            delete clr_my_low_led;
        }
        void define_config_my_high_led(Command *command){
            this->config_my_high_led = command;
        }
        void define_config_my_low_led(Command *command){
            this->config_my_low_led = command;
        }
        void define_set_my_high_led(Command *command){
            this->set_my_high_led = command;
        }
        void define_set_my_low_led(Command *command){
            this->set_my_low_led = command;
        }
        void define_clr_my_high_led(Command *command){
            this->clr_my_high_led = command;
        }
        void define_clr_my_low_led(Command *command){
            this->clr_my_low_led = command;
        }
        void config_my_leds(){
            if(this->config_my_high_led != NULL && this->config_my_low_led != NULL){
                this->config_my_high_led->run();
                this->config_my_low_led->run();
            }
        }
        void my_temp(float temp){
            if(this->set_my_high_led != NULL && this->set_my_low_led != NULL && this->clr_my_high_led != NULL && this->clr_my_low_led != NULL){
                	if(temp > TEMP_HIGH){
                        this->set_my_low_led->run();
                        this->set_my_high_led->run();
                    }else if(temp < TEMP_LOW) {
                        this->clr_my_low_led->run();
                        this->clr_my_high_led->run();
                    }else{
                        this->set_my_low_led->run();
                        this->clr_my_high_led->run();
                    }
            }
        }
};
 */
#endif

My_Led led_low_temp(LED_LOW);
My_Led led_high_temp(LED_HIGH);

#ifndef PATTERN
static void config_leds(){
#ifdef TEST //c
    config(LED_LOW);
    config(LED_HIGH);
#else //c++
    led_low_temp.config();
    led_high_temp.config();
#endif
}

static void set_leds(float temp){
#ifdef TEST //c
	if(temp > TEMP_HIGH){
		set(LED_LOW);
		set(LED_HIGH);
	}else if(temp < TEMP_LOW) {
		clr(LED_LOW);
		clr(LED_HIGH);
	}else{
		set(LED_LOW);
		clr(LED_HIGH);
	}
#else //c++
	if(temp > TEMP_HIGH){
		led_low_temp.set();
		led_high_temp.set();
	}else if(temp < TEMP_LOW) {
		led_low_temp.clr();
		led_high_temp.clr();
	}else{
		led_low_temp.set();
		led_high_temp.clr();
	}
#endif  
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
static void i2c_master_init_bus(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

}
#ifdef __cplusplus
}
#endif
static void i2c_master_init_handle(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t address){
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_CLK_Frequency,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

// Read bytes from I2C device but you need to specify the register address to read from and len or how many bytes you will read. The results is parsed into the "data" array 
static esp_err_t read_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len){
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Write a byte to the I2C register address of the device 
static esp_err_t write_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data){
    uint8_t write_buf[2] = {reg_addr, data}; // The packet must have an address followed by the data
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void read_bmp_280_task(void *arg){

#ifndef PATTERN
    config_leds();
    set_leds(50);
#else
    Control My_Control;
    My_Control.define_config_my_high_action(new Config_Led(&led_high_temp));
    My_Control.define_config_my_low_action(new Config_Led(&led_low_temp));
    My_Control.define_set_my_high_action(new Set_Led(&led_high_temp));
    My_Control.define_set_my_low_action(new Set_Led(&led_low_temp));
    My_Control.define_clr_my_high_action(new Clr_Led(&led_high_temp));
    My_Control.define_clr_my_low_action(new Clr_Led(&led_low_temp));

    My_Control.config_my_actions();
    My_Control.my_input(50);
#endif

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init_bus(&bus_handle);

    i2c_master_init_handle(&bus_handle, &dev_handle, BMP280_I2C_ADDR);

    uint8_t data_cali[CALIBR_LEN] = {0}; // 26 bytes of calibration data
    uint8_t data_temp[2] = {0};  // 2 bytes of uncompensated temperature data

    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

    for(;;){
        // Start reading calibration data
        read_byte_i2c(dev_handle, START_READ_CALC_A1, data_cali, CALIBR_LEN);
        ac1 = (int16_t) (data_cali[0] << 8) | data_cali[1];
        ac2 = (int16_t) (data_cali[2] << 8) | data_cali[3];
        ac3 = (int16_t) (data_cali[4] << 8) | data_cali[5];
        ac4 = (uint16_t) (data_cali[6] << 8) | data_cali[7];
        ac5 = (uint16_t) (data_cali[8] << 8) | data_cali[9];
        ac6 = (uint16_t) (data_cali[10] << 8) | data_cali[11];
        b1 = (int16_t) (data_cali[12] << 8) | data_cali[13];
        b2 = (int16_t) (data_cali[14] << 8) | data_cali[15];
        mb = (int16_t) (data_cali[16] << 8) | data_cali[17];
        mc = (int16_t) (data_cali[18] << 8) | data_cali[19];
        md = (int16_t) (data_cali[20] << 8) | data_cali[21];
        
        // Write a request command (BMP280_REQUEST_TEMP_CMD) to the control register (0x4F)
        write_byte_i2c(dev_handle, BMP280_CONTROL_REGISTER, BMP280_REQUEST_TEMP_CMD);

        // Wait for 5 ms then read from the register 0xF6 which gives us raw data
        vTaskDelay(5 / portTICK_PERIOD_MS);
        read_byte_i2c(dev_handle, BMP280_RESULTS, data_temp, 2);

        uint16_t raw_temp = (uint16_t) (data_temp[0] << 8) | data_temp[1];

        // Perform calculation according to the datasheet
        float x1 = ((raw_temp - ac6) * (ac5/(pow(2,15))));
        float x2 = ((mc*(pow(2,11))) / (x1+md));
        float b5 = x1 + x2;
        float true_temp = (b5+8)/(pow(2,4));
        true_temp /= 10.0;
        
#ifndef PATTERN
        set_leds(true_temp);
#else
        My_Control.my_input(true_temp);
#endif

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

#ifdef __cplusplus
extern "C" {
#endif
void app_main(void){
    xTaskCreatePinnedToCore(read_bmp_280_task, "read_bmp_280_task", 4098, NULL, 10, &read_bmp_280_task_handle, 1);  // Core 1
}
#ifdef __cplusplus
}
#endif