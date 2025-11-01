/*
 * 
 * Copyright (c) 2025 Jack Whitham
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * towel-rail-system settings.h
 *
 * These settings define the GPIO pin numbers used by the project.
 * 
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#define SSR_GPIO            7       // pin 10
#define ADC_BATHROOM_PIN    26      // pin 31 - bathroom
#define ADC_BATHROOM        0       //
#define ADC_BEDROOM_PIN     27      // pin 32 - bedroom
#define ADC_BEDROOM         1       //
#define ADC_UNUSED_PIN      28      // pin 34 - unused
#define ADC_UNUSED          2       //
#define ADC_VSYS            3       // not easily usable due to sharing
#define ADC_INT_TEMP        4       // RP2040 internal temp sensor
#define MIN_INTERNAL_TEMP   -10.0f
#define MAX_INTERNAL_TEMP   37.9f


#endif
