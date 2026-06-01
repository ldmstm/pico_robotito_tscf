/**
 * @file ESP32Encoder.h
 *
 * @brief C migration from ESP32Encoder.cpp
 *
 * @date May 31, 2026
 * @author sopapo
 */
#ifndef __ESP32ENCODER_H_
#define __ESP32ENCODER_H_

#include <driver/gpio.h>
#include <driver/pcnt.h>

#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portable.h>
#include <freertos/semphr.h>
#endif

#include <stdbool.h>
#include <stdint.h>

#define MAX_ESP32_ENCODERS   PCNT_UNIT_MAX
#define _INT16_MAX           32766
#define _INT16_MIN           -32766
#define ISR_CORE_USE_DEFAULT (0xffffffff)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	single = 0,
	half,
	full,
} encType;

typedef enum
{
	down = 0, // cambio el orden asi arranca por default en down
	up,
	none,
} puType;

typedef void (*enc_isr_cb_t)(void*);

typedef struct 
{
    bool             always_interrupt;  // 1B
    bool             attached;          // 1B, private
    bool             direction;         // 1B, private
    bool             working;           // 1B, private

    gpio_num_t       aPinNumber;                   // 4B
    gpio_num_t       bPinNumber;                   // 4B
    uint32_t         countsMode;                   // 4B, default value = 2
    enc_isr_cb_t     _enc_isr_cb;                  // 4B
    void             *_enc_isr_cb_data;            // 4B
                                                   // 4B padding?

    volatile int64_t count; // 8B, defualt value = 0

    pcnt_config_t    r_enc_config; // Struct masivo
    pcnt_unit_t      unit;         // Struct masivo
} ESP32Encoder;

// ESP32Encoder(...) creator
void ESP32Encoder_init(ESP32Encoder *encoder, bool always_interrput_, enc_isr_cb_t enc_isr_cb, void *enc_isr_cb_data);

// ~ESP32Encoder() destructor
void ESP32Encoder_destroy(ESP32Encoder *encoder);

// public methods
void    ESP32Encoder_attachHalfQuad(ESP32Encoder *encoder, int aPintNumber, int bPinNumber);
void    ESP32Encoder_attachFullQuad(ESP32Encoder *encoder, int aPintNumber, int bPinNumber);
void    ESP32Encoder_attachSingleEdge(ESP32Encoder *encoder, int aPintNumber, int bPinNumber);
void    ESP32Encoder_setCount(ESP32Encoder *encoder, int64_t value);
void    ESP32Encoder_setFilter(ESP32Encoder *encoder, uint16_t value);
void    ESP32Encoder_detach(ESP32Encoder *encoder);
int64_t ESP32Encoder_getCount(ESP32Encoder *encoder);
int64_t ESP32Encoder_clearCount(ESP32Encoder *encoder);
int64_t ESP32Encoder_pauseCount(ESP32Encoder *encoder);
int64_t ESP32Encoder_resumeCount(ESP32Encoder *encoder);
bool    ESP32Encoder_isAttached(ESP32Encoder *encoder);

// deprecado, no se como se maneja esto en gcc es __atrubute__((deprecated)) supongo
void ESP32Encoder_detatch(ESP32Encoder *encoder);

// private methods
// void attach(int aPintNumber, int bPinNumber, encType et);
// int64_t getCountRaw();

#ifdef __cplusplus
}
#endif

#endif
