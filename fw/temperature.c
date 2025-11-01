/*
 * 
 * Copyright (c) 2025 Jack Whitham
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * towel-rail-system temperature ADC driver
 *
 * This component reads the temperature using the ADCs, applies filtering
 * to reduce noise, and returns values in degrees Celsius.
 *
 * External thermistors are Vishay 640-15K types, see NTCLE100E3.pdf
 * 
 */

#include "settings.h"
#include "temperature.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/adc.h"

#define ADC_FULL_SCALE      (1 << 12)
#define HISTORY_SIZE        100     // Temperatures averaged over 100 samples
#define ADC_REF_VOLTAGE     2.506f  // Using a TL431A reference
#define FIXED_RESISTANCE    15000.0f

typedef struct sensor_history_t {
    int         index;
    int         total;
    int16_t     data[HISTORY_SIZE];
} sensor_history_t;

typedef struct temperature_t {
    sensor_history_t    bathroom_sensor_history;
    sensor_history_t    bedroom_sensor_history;
    sensor_history_t    internal_sensor_history;
    float               vishay_lookup_table[ADC_FULL_SCALE];
    float               internal_lookup_table[ADC_FULL_SCALE];
} temperature_t;


static void update_history(sensor_history_t* sh, int16_t new_value) {
    int16_t old_value = sh->data[sh->index];
    sh->data[sh->index] = new_value;
    sh->total += (int) new_value - (int) old_value;
    sh->index++;
    if (sh->index >= HISTORY_SIZE) {
        sh->index = 0;
    }
}

void temperature_update(struct temperature_t* t) {
    adc_select_input(ADC_BATHROOM);
    update_history(&t->bathroom_sensor_history, adc_read());
    adc_select_input(ADC_BEDROOM);
    update_history(&t->bedroom_sensor_history, adc_read());
    adc_select_input(ADC_INT_TEMP);
    update_history(&t->internal_sensor_history, adc_read());
}

static float vishay_temperature_calc(const float fraction) {
    // determine resistance
    const float thermistor_resistance = FIXED_RESISTANCE / ((1.0 / fraction) - 1.0);
    const float A1 = 3.354016e-3f;
    const float B1 = 2.744032e-4f;
    const float C1 = 3.666944e-6f;
    const float D1 = 1.375492e-7f;
    const float CtoK = 273.15f;
    const float ratio = logf(thermistor_resistance / FIXED_RESISTANCE);
    return (1.0f / (A1 + (B1 * ratio) + (C1 * powf(ratio, 2.0f)) + (D1 * powf(ratio, 3.0f)))) - CtoK;
}

static float internal_temperature_calc(const float fraction) {
    const float voltage = ADC_REF_VOLTAGE * fraction;
    // Apply the equation from the RP2040 data sheet (section 4.9.5, "Temperature Sensor")
    // Return value in Celsius
    return 27.0f - ((voltage - 0.706f) / 0.001721f);
}

static void pin_init(int pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_disable_pulls(pin);
    gpio_set_input_enabled(pin, false);
}

struct temperature_t* temperature_init() {
    temperature_t* t = calloc(1, sizeof(temperature_t));
    if (!t) {
        return NULL;
    }

    adc_init();
    adc_set_temp_sensor_enabled(true);
    pin_init(ADC_BEDROOM_PIN);
    pin_init(ADC_BATHROOM_PIN);
    pin_init(ADC_UNUSED_PIN);
    
    // compute lookup table: values at 0 and ADC_FULL_SCALE - 1 are not filled
    // as these represent fraction 0.0 and 1.0
    for (int i = 1; i < (ADC_FULL_SCALE - 1); i++) {
        float fraction = ((float) i) / (float) (ADC_FULL_SCALE - 1);
        t->vishay_lookup_table[i] = vishay_temperature_calc(fraction);
        t->internal_lookup_table[i] = internal_temperature_calc(fraction);
    }

    // fill the history with the current temperature
    for (int i = 0; i < HISTORY_SIZE; i++) {
        temperature_update(t);
    }
    return t;
}

static float temperature_lookup(const float* lookup_table, const struct sensor_history_t* history) {
    float fractional_index = ((float) history->total) / (float) HISTORY_SIZE;
    int integer_index = (int) floorf(fractional_index);

    // bound to valid region of lookup table
    if (integer_index <= 0) {
        integer_index = 1;
    }
    if (integer_index >= (ADC_FULL_SCALE - 3)) {
        integer_index = ADC_FULL_SCALE - 3;
    }
    // linear interpolation
    float below_value = lookup_table[integer_index];
    float above_value = lookup_table[integer_index + 1];
    return ((fractional_index - (float) integer_index) * (above_value - below_value)) + below_value;
}

float temperature_bedroom(const struct temperature_t* t) {
    return temperature_lookup(t->vishay_lookup_table, &t->bedroom_sensor_history);
}

float temperature_bathroom(const struct temperature_t* t) {
    return temperature_lookup(t->vishay_lookup_table, &t->bathroom_sensor_history);
}

float temperature_internal(const struct temperature_t* t) {
    return temperature_lookup(t->internal_lookup_table, &t->internal_sensor_history);
}
