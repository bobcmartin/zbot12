/*
    project : zbot11
    version 1.2 with custom yaml binding to fix tcc0
    IR bounded roaming / no balance
    
    aug 2026


*/

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <math.h>
#include <stdio.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>

#define RAD2DEG 57.29577f
double accel_fx, accel_fy, accel_fz;
double gyro_fx, gyro_fy, gyro_fz;

struct sensor_value accel_data[3];
struct sensor_value gyro_data[3];

uint64_t  now,last_scan,last_update;

uint64_t  UPDATE_INTYERVAL = 1000; // display update rate
uint64_t  SCAN_INTERVAL = 500;   // 250ms  IMU scan update


// PID loop gains
float Kp = 12.0;
float Ki = 0.5;
float Kd = 2.5;

// global PID loop data
float pid_output;
float setpoint_angle,current_angle;


char oled_string[16];
uint32_t scan_count;
double imu_angle;
double bat_volt = 6.2;
int16_t ir_val;


void gpio_init(void);
void update_oled(void);
void set_oled_font(void);
void blinky(void);
void serial_debug(void);
void pwm_init(void);
void set_motor_left(int power);
void set_motor_right(int power);
void roam(void);
void adc_init(void);
void scan_adc(void);
uint16_t read_ir_sensor(void);


// all of these global for now
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec blip =
    GPIO_DT_SPEC_GET(DT_ALIAS(blip0), gpios);

const struct device *const imu_dev = DEVICE_DT_GET_ANY(invensense_mpu6050);
const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/* Define the structures for the Left Motor */
static const struct pwm_dt_spec pwm_l = PWM_DT_SPEC_GET(DT_ALIAS(motorleft));
static const struct gpio_dt_spec dir_l =
    GPIO_DT_SPEC_GET(DT_ALIAS(motorleft), dir_gpios);

/* Define the structures for the Right Motor */
static const struct pwm_dt_spec pwm_r = PWM_DT_SPEC_GET(DT_ALIAS(motorright));
static const struct gpio_dt_spec dir_r =
    GPIO_DT_SPEC_GET(DT_ALIAS(motorright), dir_gpios);


// adsc channels
static const struct adc_dt_spec adc_channel_1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_channel_2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);

static uint16_t sample_buffer;

static struct adc_sequence sequence = {
	.buffer = &sample_buffer,
	.buffer_size = sizeof(sample_buffer),
};



static inline void blip_on(void) {gpio_pin_set_dt(&blip, 1);}
static inline void blip_off(void) {gpio_pin_set_dt(&blip, 0);}


#define NULL_TASK 0
#define BALANCE_WAIT 1
#define BALANCE_ACTIVE 2



void main(void) 
{

  last_scan = 0;

  gpio_init();
  pwm_init();
  adc_init();

  cfb_framebuffer_init(display_dev);

  /*
  if (!device_is_ready(display_dev)) 
  {

    printf("Device %s is not ready\n", display_dev->name);
    // error_blink();
    return;
  }

  if (!device_is_ready(imu_dev)) {
    printf(" %s is not ready\n", imu_dev->name);
    
    // error_blink();
    
    return;
  }
*/
  

  // clear framebuffer
  // 5X8 font not being compiled in, no reason found yet 2026.06.29
  // set_oled_font();
  
  cfb_framebuffer_clear(display_dev, true);
  cfb_framebuffer_finalize(display_dev);

  cfb_print(display_dev, "Hello", 1, 1);
  cfb_framebuffer_finalize(display_dev);

  printf("hello\r\n");


  while(1)
    {
      
      now = k_uptime_get();

      if((now - last_scan) > SCAN_INTERVAL)
        {
          scan_count++;
          last_scan = now;
          /*
          cfb_print(display_dev, "step 1", 1, 16);
          cfb_framebuffer_finalize(display_dev);
          printf("step 1 ");
          */
          scan_adc();
          /*
          cfb_print(display_dev, "step 2", 1, 32);
          cfb_framebuffer_finalize(display_dev);
          printf("step 2 ");
          */

          update_oled();
          blinky();
        }  


    }  // end while 1
  
  
  // main task
  // roam();

  
} // end main

#define ROAM_START          1
#define ROAM_FORWARD        2
#define ROAM_WALL_DETECT    3
#define ROAM_RECOVER        4
#define ROAM_STOP           5


uint8_t roam_state = ROAM_START;
#define PROX_LIMIT  200     // prox detect adc count for 60mm target distance

void roam(void)
{

  switch(roam_state)
    {

        case ROAM_START:    // check forward path, go into RECOVER mode if needed
          if(read_ir_sensor() > PROX_LIMIT)
            roam_state = ROAM_RECOVER;
          else 
            {
              // start moving forward
              
              roam_state = ROAM_FORWARD;
            }

        break;
        
        

      case ROAM_FORWARD:    // move forward intil we detetc something

        break;



      case ROAM_WALL_DETECT:

         break;


      case ROAM_RECOVER:
              break;



    } // end switch (roam_state)


} // end roam


void adc_init(void)
{
  int ret;
  
  ret = adc_channel_setup_dt(&adc_channel_1);
  printf("adc_init ret %d\r\n",ret);

}

void scan_adc(void)
{
    
    ir_val = read_ir_sensor();

}

uint16_t read_ir_sensor(void)
{

  int ret;
  
  adc_sequence_init_dt(&adc_channel_1, &sequence);
  ret = adc_read(adc_channel_1.dev, &sequence);
  printf("adc_read_dt %d  sample_buffer %d\r\n",ret,sample_buffer);

  if(ret != 0)
    printf("adc read error\r\n");
  return(sample_buffer);

}

void blinky(void)
{
  
  static unsigned char led_state = false;
 //  update blinky
      if (led_state) {
        gpio_pin_set_dt(&led, 0);
        led_state = false;
      } else {
        gpio_pin_set_dt(&led, 1);
        led_state = true;
      }

}

void serial_debug(void)
{
   
      printf("scan %05u Accel: %5.3f %5.3f %5.3f m/s^2\n", scan_count, accel_fx,
            accel_fy, accel_fz);
      printf("IMU angle %5.3f\r\n", imu_angle);
   
}


void gpio_init(void) {
  if (!gpio_is_ready_dt(&led)) {
    return;
  }
  /* Configure the pin as output and initialize it to 'off' */
  gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

  if (!gpio_is_ready_dt(&blip)) {
    return;
  }
  /* Configure the pin as output and initialize it to 'off' */
  gpio_pin_configure_dt(&blip, GPIO_OUTPUT_INACTIVE);
}

void update_oled(void) {
  
  cfb_framebuffer_clear(display_dev, true);
  cfb_framebuffer_finalize(display_dev);

  sprintf(oled_string, "scan %05u", scan_count);
  cfb_print(display_dev, oled_string, 1, 1);
    
  sprintf(oled_string, "adc %d", ir_val);
  cfb_print(display_dev, oled_string, 1, 16);
  
  // display data
  cfb_framebuffer_finalize(display_dev);

}

void set_oled_font(void)
{

    uint16_t rows;
    uint8_t font_width, font_height;
    int font_idx = -1;
    int num_fonts = cfb_get_numof_fonts(display_dev);

    cfb_framebuffer_clear(display_dev, true);

    /* * Select the highly space-efficient 5x8 font.
     * CFB numbers fonts by index. We iterate to find the 5x8 dimension.
     */
    

    for (int i = 0; i < num_fonts; i++) {
        cfb_get_font_size(display_dev, i, &font_width, &font_height);
        printf("f %d w %d h %d\r\n",i,font_width,font_height);
        if (font_width == 5 && font_height == 8) {
            font_idx = i;
            break;
        }
    }

   
    printf("font index %d\n\r",font_idx);


        // Set the active font to our 5x8 choice
    cfb_framebuffer_set_font(display_dev, font_idx);

}

void pwm_init(void) {
  // Initialize PWMs
  if (!pwm_is_ready_dt(&pwm_l) || !pwm_is_ready_dt(&pwm_r)) {

    return;
  }

  // Initialize Directions
  gpio_pin_configure_dt(&dir_l, GPIO_OUTPUT_INACTIVE);
  gpio_pin_configure_dt(&dir_r, GPIO_OUTPUT_INACTIVE);
}

void set_motor_left(int power) {

  if (power > 100)
    power = 100; // clip to 100%

  if (power < -100)
    power = -100; // clip to -100%

  if (power == 0) {
    gpio_pin_set_dt(&dir_l, 1);
    pwm_set_pulse_dt(&pwm_l, 0);
  }

  if (power > 0) {
    gpio_pin_set_dt(&dir_l, 1);
    uint32_t pulse = (pwm_l.period / 100) * (uint32_t)power;
    pwm_set_pulse_dt(&pwm_l, pulse);

  } else {
    gpio_pin_set_dt(&dir_l, 0);
    power = -power; // Absolute value for PWM
    uint32_t pulse = (pwm_l.period / 100) * (uint32_t)power;
    pwm_set_pulse_dt(&pwm_l, pulse);
  }
}

void set_motor_right(int power) {
  if (power > 100)
    power = 100; // clip to 100%

  if (power < -100)
    power = -100; // clip to -100%

  if (power == 0) {
    gpio_pin_set_dt(&dir_r, 1);
    pwm_set_pulse_dt(&pwm_r, 0);
  }

  if (power > 0) {
    gpio_pin_set_dt(&dir_r, 1);
    uint32_t pulse = (pwm_r.period / 100) * (uint32_t)power;
    pwm_set_pulse_dt(&pwm_r, pulse);

  } else {
    gpio_pin_set_dt(&dir_r, 0);
    power = -power; // Absolute value for PWM
    uint32_t pulse = (pwm_r.period / 100) * (uint32_t)power;
    pwm_set_pulse_dt(&pwm_r, pulse);
  }
}


void square_wave(void)
{

    // squarewave blip
    /*
    gpio_pin_set_dt(&blip, 1);
    gpio_pin_set_dt(&blip, 0);
    gpio_pin_set_dt(&blip, 1);
    gpio_pin_set_dt(&blip, 0);
    */

}
