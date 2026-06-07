#include <ESP32Encoder.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <rom/gpio.h>
#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#endif

#include <soc/soc_caps.h>
#if SOC_PCNT_SUPPORTED
// Not all esp32 chips support the pcnt (notably the esp32c3 does not)
#include <soc/pcnt_struct.h>
#include "esp_log.h"
#include "esp_ipc.h"
#if ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3) )
#include <freertos/FreeRTOS.h>
#include <rom/gpio.h>
#endif

static const char* TAG_ENCODER = "ESP32Encoder";

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

/*********************************
 *********** PORT A C ************
 *********************************/

static volatile puType       useInternalWeakPullResistors = down;
static volatile uint32_t     isrServiceCpuCore = ISR_CORE_USE_DEFAULT;
static volatile ESP32Encoder *encoders[MAX_ESP32_ENCODERS] = { NULL, }; // en gcc { 0 } aca ni idea

static volatile bool         attachedInterrupt = false;

void
ESP32Encoder_init(ESP32Encoder *encoder,
                  bool always_interrupt_,
                  enc_isr_cb_t enc_isr_cb,
                  void *enc_isr_cb_data)
{
    if (!encoder)
    {
		ESP_LOGE(TAG_ENCODER, "ESP32Encoder_init error: `encoder` is NULL");
        return;
    }

    *encoder = (ESP32Encoder){
        .always_interrupt = always_interrupt_,
        .attached         = false,
        .working          = false,
        /*.direction        = false, podria inicializar explicitamente*/

        .aPinNumber       = (gpio_num_t)0,
        .bPinNumber       = (gpio_num_t)0,
        .countsMode       = 2,
        ._enc_isr_cb      = enc_isr_cb,
        ._enc_isr_cb_data = enc_isr_cb_data,

        .count            = 0,

        .r_enc_config     = (pcnt_config_t){ 0 },
        .unit             = (pcnt_unit_t)-1,
    };

    if (!enc_isr_cb_data)
        encoder->_enc_isr_cb_data = encoder;
}

// no cambiar ownership
void
ESP32Encoder_destroy(ESP32Encoder *encoder)
{
    if (!encoder)
    {
		ESP_LOGE(TAG_ENCODER, "ESP32Encoder_destroy error: `encoder` is NULL");
        return;
    }

    if (encoder->attached)
    {
        ESP32Encoder_detach(encoder);
    }
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
#ifdef CONFIG_IDF_TARGET_ESP32S2
	#define COUNTER_H_LIM cnt_thr_h_lim_lat_un
	#define COUNTER_L_LIM cnt_thr_l_lim_lat_un
	#define thres0_lat cnt_thr_thres0_lat_un
	#define thres1_lat cnt_thr_thres1_lat_un

#elif CONFIG_IDF_TARGET_ESP32S3
	#define COUNTER_H_LIM cnt_thr_h_lim_lat_un
	#define COUNTER_L_LIM cnt_thr_l_lim_lat_un
	#define thres0_lat cnt_thr_thres0_lat_un
	#define thres1_lat cnt_thr_thres1_lat_un
#else
	#define COUNTER_H_LIM h_lim_lat
	#define COUNTER_L_LIM l_lim_lat
#endif

static void
esp32encoder_pcnt_intr_handler(void *arg)
{
	ESP32Encoder * esp32enc = (ESP32Encoder *)(arg); // <- unico cambio
	pcnt_unit_t unit = esp32enc->r_enc_config.unit;

	_ENTER_CRITICAL();
	if(PCNT.status_unit[unit].COUNTER_H_LIM){
		esp32enc->count = esp32enc->count + esp32enc->r_enc_config.counter_h_lim;
		pcnt_counter_clear(unit);
	} else if(PCNT.status_unit[unit].COUNTER_L_LIM){
		esp32enc->count = esp32enc->count + esp32enc->r_enc_config.counter_l_lim;
		pcnt_counter_clear(unit);
	} else if(esp32enc->always_interrupt && (PCNT.status_unit[unit].thres0_lat || PCNT.status_unit[unit].thres1_lat)) {
		int16_t c;
		pcnt_get_counter_value(unit, &c);
		esp32enc->count = esp32enc->count + c;
		pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
		pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
		pcnt_event_enable(unit, PCNT_EVT_THRES_0);
		pcnt_event_enable(unit, PCNT_EVT_THRES_1);
		pcnt_counter_clear(unit);
		if (esp32enc->_enc_isr_cb) {
			esp32enc->_enc_isr_cb(esp32enc->_enc_isr_cb_data);
		}
	}
	_EXIT_CRITICAL();
}

static IRAM_ATTR void
ipc_install_isr_on_core(void *arg)
{
    esp_err_t *result = (esp_err_t*) arg;
    *result = pcnt_isr_service_install(0);
}

static void
ESP32Encoder_attach(ESP32Encoder *encoder, int a, int b, encType et) {
    if (!encoder) {
		ESP_LOGE(TAG_ENCODER, "ESP32Encoder_attach error: `encoder` is NULL");
        return;
    }

	if (encoder->attached) {
		ESP_LOGE(TAG_ENCODER, "attach: already attached");
		return;
	}
	int index = 0;
	_ENTER_CRITICAL();
	for (; index < MAX_ESP32_ENCODERS; index++) {
		if (encoders[index] == NULL) {
			encoders[index] = encoder;
			break;
		}
	}
	_EXIT_CRITICAL();
	if (index == MAX_ESP32_ENCODERS) {
		while(1) {
			ESP_LOGE(TAG_ENCODER, "Too many encoders, FAIL!");
			delay(100);
		}
		
	}

	// Set data now that pin attach checks are done
	encoder->unit = (pcnt_unit_t) index;
	encoder->aPinNumber = (gpio_num_t) a;
	encoder->bPinNumber = (gpio_num_t) b;

	//Set up the IO state of hte pin
	gpio_pad_select_gpio(encoder->aPinNumber);
	gpio_pad_select_gpio(encoder->bPinNumber);
	gpio_set_direction(encoder->aPinNumber, GPIO_MODE_INPUT);
	gpio_set_direction(encoder->bPinNumber, GPIO_MODE_INPUT);
	if(useInternalWeakPullResistors == down){
		gpio_pulldown_en(encoder->aPinNumber);
		gpio_pulldown_en(encoder->bPinNumber);
	}
	if(useInternalWeakPullResistors == up){
		gpio_pullup_en(encoder->aPinNumber);
		gpio_pullup_en(encoder->bPinNumber);
	}
	// Set up encoder PCNT configuration
	// Configure channel 0
	encoder->r_enc_config.pulse_gpio_num = encoder->aPinNumber; //Rotary Encoder Chan A
	encoder->r_enc_config.ctrl_gpio_num = encoder->bPinNumber;    //Rotary Encoder Chan B

	encoder->r_enc_config.unit = encoder->unit;
	encoder->r_enc_config.channel = PCNT_CHANNEL_0;

	encoder->r_enc_config.pos_mode = et != single ? PCNT_COUNT_DEC : PCNT_COUNT_DIS; //Count Only On Rising-Edges
	encoder->r_enc_config.neg_mode = PCNT_COUNT_INC;   // Discard Falling-Edge

	encoder->r_enc_config.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
	encoder->r_enc_config.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

	encoder->r_enc_config.counter_h_lim = _INT16_MAX;
	encoder->r_enc_config.counter_l_lim = _INT16_MIN ;

	pcnt_unit_config(&encoder->r_enc_config);

	// Configure channel 0
	encoder->r_enc_config.pulse_gpio_num = encoder->bPinNumber; //make prior control into signal
	encoder->r_enc_config.ctrl_gpio_num = encoder->aPinNumber;    //and prior signal into control

	encoder->r_enc_config.channel = PCNT_CHANNEL_1; // channel 1

	encoder->r_enc_config.pos_mode = PCNT_COUNT_DIS; //disabling channel 1
	encoder->r_enc_config.neg_mode = PCNT_COUNT_DIS;   // disabling channel 1

	encoder->r_enc_config.lctrl_mode = PCNT_MODE_DISABLE;    // disabling channel 1
	encoder->r_enc_config.hctrl_mode = PCNT_MODE_DISABLE; // disabling channel 1

	if (et == full) {
		// set up second channel for full quad

		encoder->r_enc_config.pos_mode = PCNT_COUNT_DEC; //Count Only On Rising-Edges
		encoder->r_enc_config.neg_mode = PCNT_COUNT_INC;   // Discard Falling-Edge

		encoder->r_enc_config.lctrl_mode = PCNT_MODE_REVERSE;    // prior high mode is now low
		encoder->r_enc_config.hctrl_mode = PCNT_MODE_KEEP; // prior low mode is now high
	}
	pcnt_unit_config(&encoder->r_enc_config);

	// Filter out bounces and noise
	ESP32Encoder_setFilter(encoder, 250); // Filter Runt Pulses

	/* Enable events on maximum and minimum limit values */
	pcnt_event_enable(encoder->unit, PCNT_EVT_H_LIM);
	pcnt_event_enable(encoder->unit, PCNT_EVT_L_LIM);
	pcnt_counter_pause(encoder->unit); // Initial PCNT init
	/* Register ISR service and enable interrupts for PCNT unit */
	_ENTER_CRITICAL();
	if(! attachedInterrupt){
#ifdef CONFIG_IDF_TARGET_ESP32S2 // esp32-s2 is single core, no ipc call
			esp_err_t er = pcnt_isr_service_install(0);
			if (er != ESP_OK){
				ESP_LOGE(TAG_ENCODER, "Encoder install isr service failed");
			}
#else
		if (isrServiceCpuCore == ISR_CORE_USE_DEFAULT || isrServiceCpuCore == xPortGetCoreID()) {
			esp_err_t er = pcnt_isr_service_install(0);
			if (er != ESP_OK){
				ESP_LOGE(TAG_ENCODER, "Encoder install isr service on same core failed");
			}
		} else {
			esp_err_t ipc_ret_code = ESP_FAIL;
			esp_err_t er = esp_ipc_call_blocking(isrServiceCpuCore, ipc_install_isr_on_core, &ipc_ret_code);
			if (er != ESP_OK){
				ESP_LOGE(TAG_ENCODER, "IPC call to install isr service on core %ld failed", isrServiceCpuCore);
			}
			if (ipc_ret_code != ESP_OK){
				ESP_LOGE(TAG_ENCODER, "Encoder install isr service on core %ld failed", isrServiceCpuCore);
			}
		}
#endif

		attachedInterrupt=true;
	}
	_EXIT_CRITICAL();

	// Add ISR handler for this unit
	if (pcnt_isr_handler_add(encoder->unit, esp32encoder_pcnt_intr_handler, encoder) != ESP_OK) {
		ESP_LOGE(TAG_ENCODER, "Encoder install interrupt handler for unit %d failed", encoder->unit);
	}

	if (encoder->always_interrupt){
		pcnt_set_event_value(encoder->unit, PCNT_EVT_THRES_0, -1);
		pcnt_set_event_value(encoder->unit, PCNT_EVT_THRES_1, 1);
		pcnt_event_enable(encoder->unit, PCNT_EVT_THRES_0);
		pcnt_event_enable(encoder->unit, PCNT_EVT_THRES_1);
	}
	pcnt_counter_clear(encoder->unit);
	pcnt_intr_enable(encoder->unit);
	pcnt_counter_resume(encoder->unit);

	encoder->attached = true;
}

void
ESP32Encoder_useInternalWeakPullResistors(puType type)
{
    useInternalWeakPullResistors = type;
}

void
ESP32Encoder_attachHalfQuad(ESP32Encoder *encoder, int aPinNumber, int bPinNumber)
{
	ESP32Encoder_attach(encoder, aPinNumber, bPinNumber, half);
}

void 
ESP32Encoder_attachSingleEdge(ESP32Encoder *encoder, int aPinNumber, int bPinNumber)
{
	ESP32Encoder_attach(encoder, aPinNumber, bPinNumber, single);
}

void
ESP32Encoder_attachFullQuad(ESP32Encoder *encoder, int aPinNumber, int bPinNumber)
{
	ESP32Encoder_attach(encoder, aPinNumber, bPinNumber, full);
}

void
ESP32Encoder_detach(ESP32Encoder *encoder)
{
	if (!encoder || !encoder->attached || encoder->unit < 0 || encoder->unit >= MAX_ESP32_ENCODERS) {
		return;
	}
	pcnt_counter_pause(encoder->unit);
	pcnt_isr_handler_remove(encoder->unit);
	encoders[encoder->unit] = NULL;
	encoder->attached = false;
}

__attribute__ ((deprecated("Use ESP32Encoder_detach instead")))
void
ESP32Encoder_detatch(ESP32Encoder *encoder)
{
	ESP32Encoder_detach(encoder);
}

static int64_t
ESP32Encoder_getCountRaw(ESP32Encoder *encoder)
{
	int16_t c;
	int64_t compensate = 0;

	pcnt_get_counter_value(encoder->unit, &c);
	// check if counter overflowed, if so re-read and compensate
	// see https://github.com/espressif/esp-idf/blob/v4.4.1/tools/unit-test-app/components/test_utils/ref_clock_impl_rmt_pcnt.c#L168-L172
	if (PCNT.int_st.val & BIT(encoder->unit)) {
        pcnt_get_counter_value(encoder->unit, &c);
		if(PCNT.status_unit[encoder->unit].COUNTER_H_LIM){
			compensate = encoder->r_enc_config.counter_h_lim;
		} else if (PCNT.status_unit[encoder->unit].COUNTER_L_LIM) {
			compensate = encoder->r_enc_config.counter_l_lim;
		}
	}

	return compensate + c;
}

void
ESP32Encoder_setCount(ESP32Encoder *encoder, int64_t value)
{
	if (!encoder->attached) return;
	_ENTER_CRITICAL();
	encoder->count = value - ESP32Encoder_getCountRaw(encoder);
	_EXIT_CRITICAL();
}

int64_t
ESP32Encoder_getCount(ESP32Encoder *encoder)
{
	if (!encoder->attached) return 0;
	_ENTER_CRITICAL();
	int64_t result = encoder->count + ESP32Encoder_getCountRaw(encoder);
	_EXIT_CRITICAL();

	return result;
}

int64_t
ESP32Encoder_clearCount(ESP32Encoder *encoder)
{
	if (!encoder->attached) return ESP_FAIL;
	_ENTER_CRITICAL();
	encoder->count = 0;
	_EXIT_CRITICAL();

	return pcnt_counter_clear(encoder->unit);
}

int64_t
ESP32Encoder_pauseCount(ESP32Encoder *encoder)
{
	if (!encoder->attached) return ESP_FAIL;
	return pcnt_counter_pause(encoder->unit);
}

int64_t
ESP32Encoder_resumeCount(ESP32Encoder *encoder)
{
	if (!encoder->attached) return ESP_FAIL;
	return pcnt_counter_resume(encoder->unit);
}

void
ESP32Encoder_setFilter(ESP32Encoder *encoder, uint16_t value)
{
	if (!encoder->attached) return;
	if(value>1023)value=1023;
	if(value==0) {
		pcnt_filter_disable(encoder->unit);
	} else {
		pcnt_set_filter_value(encoder->unit, value);
		pcnt_filter_enable(encoder->unit);
	}

}

bool 
ESP32Encoder_isAttached(ESP32Encoder *encoder)
{
    return encoder->attached;
}

#else
#warning PCNT not supported on this SoC, this will likely lead to linker errors when using ESP32Encoder
#endif // SOC_PCNT_SUPPORTED