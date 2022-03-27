#include "epd_board.h"
/**
 * @file epd_board_v6_s2.c
 * Do not forget that this is a different SOC, so do a:
 * idf.py fullclean             (In case you build it before on other target)
 * idf.py set-target esp32s2
 * 
 * If it get's stuck and won't change, as latest: delete the sdkconfig.default
 * Before building or it will fail
 */
#include "esp_log.h"
#include "../display_ops.h"
#include "../i2s_data_bus.h"
#include "../rmt_pulse.h"
#include "../tps65185.h"
#include <driver/gpio.h>
#include <driver/i2c.h>

static int v6_wait_for_interrupt(int timeout) __attribute__((unused));

// ESP32-S2 configuration (proposed)
#define CFG_SCL             GPIO_NUM_8
#define CFG_SDA             GPIO_NUM_9

#define EPDIY_I2C_PORT      I2C_NUM_0
#define D7 GPIO_NUM_40
#define D6 GPIO_NUM_39
#define D5 GPIO_NUM_38
#define D4 GPIO_NUM_37
#define D3 GPIO_NUM_36
#define D2 GPIO_NUM_35
#define D1 GPIO_NUM_34
#define D0 GPIO_NUM_33

/* TPS65185 5 control lines */
#define TPS_PWRGOOD       GPIO_NUM_46
#define TPS_WAKEUP        GPIO_NUM_18
#define TPS_PWRUP         GPIO_NUM_16
#define TPS_INTERRUPT     GPIO_NUM_11 // old  CFG_INTR
#define TPS_VCOM_CTRL     GPIO_NUM_10

/* EPD Control Lines */
#define CKV GPIO_NUM_41
#define STH GPIO_NUM_42
#define EPD_STV  GPIO_NUM_15
#define EPD_MODE GPIO_NUM_14
#define EPD_OE   GPIO_NUM_13
#define V4_LATCH_ENABLE GPIO_NUM_12

/* Edges */
#define CKH GPIO_NUM_21

typedef struct {
    i2c_port_t port;
    bool pwrup;
    bool vcom_ctrl;
    bool wakeup;
    bool others[8];
} epd_config_register_t;

static i2s_bus_config i2s_config = {
  .clock = CKH,
  .start_pulse = STH,
  .data_0 = D0,
  .data_1 = D1,
  .data_2 = D2,
  .data_3 = D3,
  .data_4 = D4,
  .data_5 = D5,
  .data_6 = D6,
  .data_7 = D7,
};

static bool interrupt_done = false;

static void IRAM_ATTR interrupt_handler(void* arg) {
    interrupt_done = true;
}

static int v6_wait_for_interrupt(int timeout) {
  int tries = 0;
  while (!interrupt_done && gpio_get_level(TPS_INTERRUPT) == 1) {
    if (tries >= 500) {
        return -1;
    }
    tries++;
    vTaskDelay(1);
  }
  int int1 = 0;
  int int2 = 0;
  interrupt_done = false;

  int1 = tps_read_register(EPDIY_I2C_PORT, TPS_REG_INT1);
  int2 = tps_read_register(EPDIY_I2C_PORT, TPS_REG_INT2);
  // Note: Would be nice to make an human translation of the results of this
  //       so you can get right away what's the issue without browsing datasheet
  printf("INT1: %d\n", int1);
  printf("INT2: %d\n", int2);
	int1 |= int2 << 8;

  while (!gpio_get_level(TPS_INTERRUPT)) { vTaskDelay(1); }
  return int1;
}

static epd_config_register_t config_reg;

static void epd_board_init(uint32_t epd_row_width) {
  gpio_hold_dis(CKH); // free CKH after wakeup

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = CFG_SDA;
  conf.scl_io_num = CFG_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
  ESP_ERROR_CHECK(i2c_param_config(EPDIY_I2C_PORT, &conf));

  ESP_ERROR_CHECK(i2c_driver_install(EPDIY_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

  config_reg.port = EPDIY_I2C_PORT;
  config_reg.pwrup = false;
  config_reg.vcom_ctrl = false;
  config_reg.wakeup = false;
  for (int i=0; i<8; i++) {
      config_reg.others[i] = false;
  }

    /*  
     TPS_VCOM_CTRL     Output 
     TPS_WAKEUP        Output
     TPS_PWRUP         Output
     TPS_INTERRUPT     Input now CFG_INTR
     TPS_PWRGOOD       Input
     */
    // Control lines are now controlled directly  (1->4) by S2
    // Also TPS65185 control pins are set up here (4->8)
    gpio_num_t EP_CONTROL[] = {STH, EPD_STV, EPD_MODE, EPD_OE, V4_LATCH_ENABLE, TPS_WAKEUP, TPS_PWRUP, TPS_VCOM_CTRL};
   
    for (int x = 0; x < 8; x++) {
        printf("IO %d to LOW (loop:%d)\n", (int)EP_CONTROL[x], x);
        // Added because certain Pins like IO42 would not turn HI withouth resetting them first
        gpio_reset_pin(EP_CONTROL[x]);
        gpio_set_direction(EP_CONTROL[x], GPIO_MODE_OUTPUT);
        gpio_set_level(EP_CONTROL[x], 0);
    }
    gpio_set_direction(TPS_PWRGOOD, GPIO_MODE_INPUT);
    // TODO: Interrupts should be added for incoming signals from TPS65185
    gpio_set_direction(TPS_INTERRUPT, GPIO_MODE_INPUT);
    gpio_set_intr_type(TPS_INTERRUPT, GPIO_INTR_NEGEDGE);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(TPS_INTERRUPT, interrupt_handler, (void *) TPS_INTERRUPT));

  // Setup I2S
  // add an offset off dummy bytes to allow for enough timing headroom
  i2s_bus_init( &i2s_config , epd_row_width + 32);

  rmt_pulse_init(CKV);
}

static void epd_board_deinit() {
  //gpio_reset_pin(TPS_INTERRUPT);
  //rtc_gpio_isolate(TPS_INTERRUPT);
  gpio_set_level(TPS_VCOM_CTRL, 0);
  gpio_set_level(TPS_PWRUP, 0);

  int tries = 0;
  while (gpio_get_level(TPS_PWRGOOD) == 1) {
    if (tries >= 500) {
      ESP_LOGE("epdiy", "failed to shut down TPS65185!");
      break;
    }
    tries++;
    vTaskDelay(1);
  }
  // Not sure why we need this delay, but the TPS65185 seems to generate an interrupt after some time that needs to be cleared.
  vTaskDelay(500);
  ESP_LOGI("epdiy", "going to sleep.");
  i2c_driver_delete(EPDIY_I2C_PORT);
}

static void epd_board_set_ctrl(epd_ctrl_state_t *state, const epd_ctrl_state_t * const mask) {
  uint8_t value = 0x00;
  if (state->ep_sth) {
    fast_gpio_set_hi(STH);
  } else {
    fast_gpio_set_lo(STH);
  }

  if (mask->ep_output_enable || mask->ep_mode || mask->ep_stv) {
    if (state->ep_output_enable) gpio_set_level(EPD_OE, 1);
    if (state->ep_mode)          gpio_set_level(EPD_MODE, 1);
    if (state->ep_stv)           gpio_set_level(EPD_STV, 1);
    if (config_reg.pwrup)        gpio_set_level(TPS_PWRUP, 1);
    if (config_reg.vcom_ctrl)    gpio_set_level(TPS_VCOM_CTRL, 1);
    if (config_reg.wakeup)       gpio_set_level(TPS_WAKEUP, 1);
  }

  if (state->ep_latch_enable) {
    fast_gpio_set_hi(V4_LATCH_ENABLE);
  } else {
    fast_gpio_set_lo(V4_LATCH_ENABLE);
  }
}

static void epd_board_poweron(epd_ctrl_state_t *state) {
  i2s_gpio_attach(&i2s_config);

  epd_ctrl_state_t mask = {
    .ep_stv = true,
  };
  state->ep_stv = true;
  config_reg.wakeup = true;
  epd_board_set_ctrl(state, &mask);
  config_reg.pwrup = true;
  epd_board_set_ctrl(state, &mask);
  config_reg.vcom_ctrl = true;
  epd_board_set_ctrl(state, &mask);

  // give the IC time to powerup and set lines
  vTaskDelay(1);

  printf("\nThermistor °C: %d\n", tps_read_thermistor(EPDIY_I2C_PORT));
  printf("Waiting for PWRGOOD...\n");
  // while (!(pca9555_read_input(reg->port, 1) & CFG_PIN_PWRGOOD)) {
  while (gpio_get_level(TPS_PWRGOOD) == 0) {
      vTaskDelay(5);
  }
  printf("TPS_PWRGOOD ok\n");

  ESP_ERROR_CHECK(tps_write_register(config_reg.port, TPS_REG_ENABLE, 0x3F));

#ifdef CONFIG_EPD_DRIVER_S2_VCOM
  tps_set_vcom(config_reg.port, CONFIG_EPD_DRIVER_S2_VCOM);
// Arduino IDE...
#else
  extern int epd_driver_v6_vcom;
  tps_set_vcom(config_reg.port, epd_driver_v6_vcom);
#endif

  state->ep_sth = true;
  mask = (const epd_ctrl_state_t){
    .ep_sth = true,
  };
  epd_board_set_ctrl(state, &mask);

  int tries = 0;
  while (!((tps_read_register(config_reg.port, TPS_REG_PG) & 0xFA) == 0xFA)) {
    if (tries >= 500) {
      ESP_LOGE("epdiy", "Power enable failed! PG status: %X", tps_read_register(config_reg.port, TPS_REG_PG));
      return;
    }
    tries++;
    vTaskDelay(1);
  }
}

static void epd_board_poweroff(epd_ctrl_state_t *state) {
  epd_ctrl_state_t mask = {
    .ep_stv = true,
    .ep_output_enable = true,
    .ep_mode = true,
  };
  config_reg.vcom_ctrl = false;
  config_reg.pwrup = false;
  state->ep_stv = false;
  state->ep_output_enable = false;
  state->ep_mode = false;
  epd_board_set_ctrl(state, &mask);
  vTaskDelay(1);
  config_reg.wakeup = false;
  epd_board_set_ctrl(state, &mask);

  i2s_gpio_detach(&i2s_config);
}

static float epd_board_ambient_temperature() {
  return tps_read_thermistor(EPDIY_I2C_PORT);
}

// Nicely made
const EpdBoardDefinition epd_board_s2 = {
  .init = epd_board_init,
  .deinit = epd_board_deinit,
  .set_ctrl = epd_board_set_ctrl,
  .poweron = epd_board_poweron,
  .poweroff = epd_board_poweroff,

  .temperature_init = NULL,
  .ambient_temperature = epd_board_ambient_temperature,
};
