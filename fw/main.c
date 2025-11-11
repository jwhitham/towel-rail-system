/*
 * 
 * Copyright (c) 2025 Jack Whitham
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * towel-rail-system main
 *
 * See README.md: this is a sample project for the wifi_settings library at
 * https://github.com/jwhitham/pico-wifi-settings
 * 
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/cyw43_driver.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"

#include "wifi_settings.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"

#include "lwip/udp.h"

#include "temperature.h"
#include "settings.h"

#if PICO_CYW43_ARCH_POLL
#error "Expected interrupt settings"
#endif

#define MAX_REPORT_SIZE     100
#define ID_GET_STATUS_HANDLER (ID_FIRST_USER_HANDLER + 0)


typedef enum {
    TEMP_COLD = 0,      // Cold
    TEMP_MILD,          // Normal
    TEMP_HOT,           // Too hot in the house already
    TEMP_ERROR,         // Controller temperature is too high or low
} temperature_t;

typedef enum {
    MODE_OFF = 0,
    MODE_ON_WHEN_COLD,
    MODE_ON_WHEN_MILD,
    MODE_ON_WHEN_HOT,
    MODE_ON_BOOST,
} on_mode_t;

typedef struct config_t {
    float                       cold_threshold;     // largest numerical value
    float                       not_cold_threshold;
    float                       not_hot_threshold;
    float                       hot_threshold;      // smallest numerical value
    ip_addr_t                   report_addr;
    int                         report_port;
    int                         report_interval_s;
    int                         max_on_time_m;
    int                         control_port;
} config_t;

typedef struct control_status_t {
    config_t                    config;
    int                         heartbeat_counter;
    temperature_t               temperature_band;
    on_mode_t                   on_mode;
    bool                        output_value;
    bool                        last_output_value;
    absolute_time_t             report_update_time;
    absolute_time_t             on_mode_end_time;
    float                       bathroom_temperature_value;
    float                       internal_temperature_value;
    struct temperature_t*       temperature_handle;
    struct udp_pcb*             comms_pcb;
} control_status_t;

static void make_report(control_status_t* cs, char* message, size_t size) {
    const char* mode_text = "OFF";
    switch (cs->on_mode) {
        case MODE_ON_WHEN_HOT:
            mode_text = "ON_WHEN_HOT";
            break;
        case MODE_ON_BOOST:
            mode_text = "ON_BOOST";
            break;
        case MODE_ON_WHEN_MILD:
            mode_text = "ON_WHEN_MILD";
            break;
        case MODE_ON_WHEN_COLD:
            mode_text = "ON_WHEN_COLD";
            break;
        default:
            break;
    }

    const char* temp_text = "MILD";
    switch (cs->temperature_band) {
        case TEMP_COLD:
            temp_text = "COLD";
            break;
        case TEMP_HOT:
            temp_text = "HOT";
            break;
        case TEMP_ERROR:
            temp_text = "ERROR";
            break;
        default:
            break;
    }

    const int64_t second_us = 1000000LL;
    const int64_t minute_us = 60LL * second_us;
    // current monotonic time in microseconds
    const int64_t now_us = (int64_t) to_us_since_boot(get_absolute_time());
    // uptime: round down to a second
    const int64_t uptime_s = now_us / second_us;
    // on time left: round to nearest minute
    const int64_t on_time_left_m =
        (((int64_t) to_us_since_boot(cs->on_mode_end_time) - now_us) + (minute_us / 2LL)) / minute_us;

    snprintf(message, size,
        "bath %1.2f bed %1.2f int %1.2f mode %s out %s temp %s left %u up %u\n",
        (double) cs->bathroom_temperature_value,
        (double) temperature_bedroom(cs->temperature_handle),
        (double) cs->internal_temperature_value,
        mode_text,
        cs->output_value ? "TRUE" : "FALSE",
        temp_text,
        (on_time_left_m > 0LL) ? (unsigned) on_time_left_m : 0U,
        (unsigned) uptime_s);
}

static void make_report_and_send_by_udp(control_status_t* cs) {
    if ((!cs->config.report_port) || (!cs->comms_pcb)) {
        // reporting is disabled
        return;
    }
    char message[MAX_REPORT_SIZE];
    make_report(cs, message, sizeof(message));
    size_t size = strlen(message);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, size, PBUF_RAM);
    if (!p) {
        return;
    }

    memcpy(p->payload, message, size);
    (void) udp_sendto(cs->comms_pcb, p, &cs->config.report_addr, cs->config.report_port);
    pbuf_free(p);
}


static void periodic_task(control_status_t* cs) {
    // Read temperature sensors
    temperature_update(cs->temperature_handle);

    // Check internal temperature
    cs->internal_temperature_value = temperature_internal(cs->temperature_handle);
    bool internal_out_of_range = ((cs->internal_temperature_value < MIN_INTERNAL_TEMP)
                || (cs->internal_temperature_value > MAX_INTERNAL_TEMP));

    // Determine temperature range
    cs->bathroom_temperature_value = temperature_bathroom(cs->temperature_handle);
    bool state_changed = false;
    switch (cs->temperature_band) {
        case TEMP_COLD:
            if (cs->bathroom_temperature_value > cs->config.not_cold_threshold) {
                cs->temperature_band = TEMP_MILD;
                state_changed = true;
            }
            break;
        case TEMP_HOT:
            if (cs->bathroom_temperature_value < cs->config.not_hot_threshold) {
                cs->temperature_band = TEMP_MILD;
                state_changed = true;
            }
            break;
        case TEMP_MILD:
            if (cs->bathroom_temperature_value > cs->config.hot_threshold) {
                cs->temperature_band = TEMP_HOT;
                state_changed = true;
            } else if (cs->bathroom_temperature_value < cs->config.cold_threshold) {
                cs->temperature_band = TEMP_COLD;
                state_changed = true;
            }
            break;
        default:
            // stay in error state while out of range
            if (internal_out_of_range) {
                cs->temperature_band = TEMP_ERROR;
            } else {
                cs->temperature_band = TEMP_HOT;
                state_changed = true;
            }
            break;
    }

    // Turn off if the timeout is reached
    if (time_reached(cs->on_mode_end_time) && (cs->on_mode != MODE_OFF)) {
        cs->on_mode = MODE_OFF;
        state_changed = true;
    }

    // React to internal out of range condition by turning off
    if (internal_out_of_range) {
        if (cs->temperature_band != TEMP_ERROR) {
            state_changed = true;
        }
        cs->temperature_band = TEMP_ERROR;
        cs->on_mode_end_time = nil_time;
        cs->on_mode = MODE_OFF;
    }

    // Determine output
    switch (cs->on_mode) {
        case MODE_ON_WHEN_COLD:
            cs->output_value = (cs->temperature_band == TEMP_COLD);
            break;
        case MODE_ON_WHEN_MILD:
            cs->output_value = (cs->temperature_band == TEMP_COLD)
                            || (cs->temperature_band == TEMP_MILD);
            break;
        case MODE_ON_WHEN_HOT:
        case MODE_ON_BOOST:
            cs->output_value = (cs->temperature_band == TEMP_COLD)
                            || (cs->temperature_band == TEMP_MILD)
                            || (cs->temperature_band == TEMP_HOT);
            break;
        default:
            cs->output_value = false;
            break;
    }

    // Set output
    if (cs->output_value != cs->last_output_value) {
        cs->last_output_value = cs->output_value;
        gpio_put(SSR_GPIO, (int) cs->output_value);
        state_changed = true;
    }

    // Heartbeat LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, (cs->heartbeat_counter == 0) ? 1 : 0);

    // Heartbeat update: faster when WiFi is not working
    if ((cs->heartbeat_counter > 0) && !wifi_settings_is_connected()) {
        cs->heartbeat_counter = 0;
    } else if (cs->heartbeat_counter >= 9) {
        cs->heartbeat_counter = 0;
    } else {
        cs->heartbeat_counter++;
    }

    // UDP report: send periodic update
    if (time_reached(cs->report_update_time)) {
        state_changed = true;
        cs->report_update_time = delayed_by_ms(cs->report_update_time, cs->config.report_interval_s * 1000);
    }
    if (state_changed) {
        make_report_and_send_by_udp(cs);
    }
}

static int32_t remote_handler_get_status(
        uint8_t msg_type,
        uint8_t* data_buffer,
        uint32_t input_data_size,
        int32_t input_parameter,
        uint32_t* output_data_size,
        void* arg) {

    control_status_t* cs = (control_status_t *) arg;
    switch (input_parameter) {
        case 0:
            // Text report
            make_report(cs, (char*) data_buffer, (size_t) *output_data_size);
            *output_data_size = strlen((char*) data_buffer);
            break;
        case 1:
            // Data structure dump
            if (*output_data_size > sizeof(control_status_t)) {
                *output_data_size = sizeof(control_status_t);
            }
            memcpy(data_buffer, cs, (size_t) *output_data_size);
            break;
        default:
            *output_data_size = 0;
            break;
    }
    return 0;
}

static bool mode_setting(control_status_t* cs, const char* recv_buffer, size_t recv_size) {
    char copy_buffer[40];
    if (recv_size >= sizeof(copy_buffer)) {
        recv_size = sizeof(copy_buffer) - 1;
    }
    memcpy(copy_buffer, recv_buffer, recv_size);
    copy_buffer[recv_size] = '\0';
    char command[sizeof(copy_buffer)] = {0};
    int minutes = -1;
    if (2 != sscanf(copy_buffer, "%s %d", command, &minutes)) {
        return false;
    }
    if ((minutes < 0) || (minutes > cs->config.max_on_time_m)) {
        return false;
    }
    if (strcasecmp(command, "ON_BOOST") == 0) {
        cs->on_mode = MODE_ON_BOOST;
    } else if (strcasecmp(command, "OFF") == 0) {
        cs->on_mode = MODE_OFF;
    } else if (cs->on_mode == MODE_ON_BOOST) {
        // ignore all other commands
        return false;
    } else if (strcasecmp(command, "ON_WHEN_COLD") == 0) {
        cs->on_mode = MODE_ON_WHEN_COLD;
    } else if (strcasecmp(command, "ON_WHEN_MILD") == 0) {
        cs->on_mode = MODE_ON_WHEN_MILD;
    } else if (strcasecmp(command, "ON_WHEN_HOT") == 0) {
        cs->on_mode = MODE_ON_WHEN_HOT;
    } else {
        // unknown command
        return false;
    }
    // update the turn-off time
    if (cs->on_mode != MODE_OFF) {
        cs->on_mode_end_time = make_timeout_time_ms(minutes * 60 * 1000);
    } else {
        cs->on_mode_end_time = nil_time;
    }
    // send report at the next periodic update
    cs->report_update_time = get_absolute_time();
    return true;
}

static void comms_recv_callback(void *arg, struct udp_pcb *pcb,
        struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    control_status_t* cs = (control_status_t *) arg;
    (void) mode_setting(cs, (const char*) p->payload, (size_t) p->len);
    pbuf_free(p);
}

static int config_get_float(const char* key, float default_value) {
    char tmp[16];
    uint size = sizeof(tmp) - 1;
    if (!wifi_settings_get_value_for_key(key, tmp, &size)) {
        return default_value;
    }
    tmp[size] = '\0';
    char* end = NULL;
    float value = strtof(tmp, &end);
    if ((end[0] == '\0') && (end != tmp)) {
        // read at least one digit and reached the end of the string
        return value;
    }
    return default_value;
}

static int config_get_int(const char* key, int min_value, int default_value, int max_value) {
    char tmp[16];
    uint size = sizeof(tmp) - 1;
    if (!wifi_settings_get_value_for_key(key, tmp, &size)) {
        return default_value;
    }
    tmp[size] = '\0';
    char* end = NULL;
    long value = strtol(tmp, &end, 0);
    if ((end[0] == '\0') && (end != tmp)) {
        // read at least one digit and reached the end of the string
        if ((value >= (long) min_value) && (value <= (long) max_value)) {
            // value is within the allowed range
            return value;
        }
    }
    return default_value;
}

static void config_init(control_status_t* cs) {
    // min/max values for ADC readings
    cs->config.cold_threshold = config_get_float("cold_threshold", 10.0f);
    cs->config.not_cold_threshold = config_get_float("not_cold_threshold", 11.0f);
    cs->config.not_hot_threshold = config_get_float("not_hot_threshold", 24.0f);
    cs->config.hot_threshold = config_get_float("hot_threshold", 25.0f);

    // where to send messages
    char address[32];
    uint size = sizeof(address) - 1;
    if (wifi_settings_get_value_for_key("report_address", address, &size)) {
        address[size] = '\0';
        if (ipaddr_aton(address, &cs->config.report_addr)) {
            // Address is valid
            cs->config.report_port = config_get_int("report_port", 0, 0, UINT16_MAX);
            cs->config.report_interval_s = config_get_int("report_interval_s", 1, 30, INT_MAX);
        }
    }

    // where to listen for commands
    cs->config.control_port = config_get_int("control_port", 0, 0, UINT16_MAX);

    // max time for any "on" command
    cs->config.max_on_time_m = config_get_int("max_on_time_m", 1, 45, 180);
}

static void state_init(control_status_t* cs) {
    // initial setup
    memset(cs, 0, sizeof(control_status_t));
    cs->temperature_band = TEMP_MILD;
    cs->on_mode = MODE_OFF;

    // read config
    config_init(cs);

    // WiFi setup
    cs->comms_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (cs->comms_pcb && (udp_bind(cs->comms_pcb, NULL, cs->config.control_port) == 0)) {
        udp_recv(cs->comms_pcb, comms_recv_callback, cs);
    }

    // Temperature ADC setup
    cs->temperature_handle = temperature_init();
    while (!cs->temperature_handle) {
        printf("unable to allocate temperature_handle\n");
        sleep_ms(1000);
    }

    // generate timeouts
    cs->on_mode_end_time = nil_time;
    cs->report_update_time = make_timeout_time_ms(cs->config.report_interval_s * 1000);
}

int main(void) {
    set_sys_clock_khz(48000, true);     // minimum frequency needed for USB
    cyw43_set_pio_clock_divisor(1, 0);  // needed so that cyw43 still works

    stdio_init_all();

    gpio_init(SSR_GPIO);
    gpio_set_dir(SSR_GPIO, GPIO_OUT);
    gpio_set_drive_strength(SSR_GPIO, GPIO_DRIVE_STRENGTH_12MA);

    int rc = wifi_settings_init();
    while (rc != PICO_OK) {
        printf("wifi_settings_init returned %d\n", rc);
        sleep_ms(1000);
    }
    
    // Load configuration and initial state
    control_status_t* cs = malloc(sizeof(control_status_t));
    while (!cs) {
        printf("unable to allocate cs\n");
        sleep_ms(1000);
    }
    state_init(cs);
    wifi_settings_remote_set_handler(ID_GET_STATUS_HANDLER, remote_handler_get_status, cs);
    wifi_settings_connect();

    // main loop
    absolute_time_t update_time = get_absolute_time();
    while (true) {
        cyw43_arch_lwip_begin();
        periodic_task(cs);
        cyw43_arch_lwip_end();

        sleep_until(update_time);
        update_time = delayed_by_ms(update_time, 100);
    }
}
