/************************************************************************
 * ARM V Control
 * Copyright (C) 2022-2022 Limited, All Rights Reserved
 * Written by Davi Reis and Guilherme Rosada
 *
 *
 */

#include "conf_board.h"
#include "ili9341.h"
#include "lvgl.h"
#include "spotify_logo.h"
#include "touch/touch.h"
#include <asf.h>
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_IDX 8
#define LED_IDX_MASK (1 << LED_IDX)

// Botão
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_IDX 11
#define BUT_IDX_MASK (1 << BUT_IDX)

// Play / Pause
#define PLAY_PAUSE_PIO PIOA
#define PLAY_PAUSE_PIO_ID ID_PIOA
#define PLAY_PAUSE_IDX 13
#define PLAY_PAUSE_IDX_MASK (1 << PLAY_PAUSE_IDX)

// LED Play / Pause
#define LED_PLAY_PAUSE_PIO PIOA
#define LED_PLAY_PAUSE_PIO_ID ID_PIOA
#define LED_PLAY_PAUSE_IDX 0
#define LED_PLAY_PAUSE_IDX_MASK (1 << LED_PLAY_PAUSE_IDX)

// Next
#define NEXT_PIO PIOC
#define NEXT_PIO_ID ID_PIOC
#define NEXT_PIO_IDX 30
#define NEXT_IDX_MASK (1 << NEXT_PIO_IDX)

// Previous
#define PREV_PIO PIOC
#define PREV_PIO_ID ID_PIOC
#define PREV_PIO_IDX 31
#define PREV_IDX_MASK (1 << PREV_PIO_IDX)

// Shuffle
#define SHUFFLE_PIO PIOC
#define SHUFFLE_PIO_ID ID_PIOC
#define SHUFFLE_PIO_IDX 17
#define SHUFFLE_IDX_MASK (1 << SHUFFLE_PIO_IDX)

// LED Shuffle
#define LED_SHUFFLE_PIO PIOD
#define LED_SHUFFLE_PIO_ID ID_PIOD
#define LED_SHUFFLE_PIO_IDX 28
#define LED_SHUFFLE_IDX_MASK (1 << LED_SHUFFLE_PIO_IDX)

// Loop
#define LOOP_PIO PIOA
#define LOOP_PIO_ID ID_PIOA
#define LOOP_PIO_IDX 19
#define LOOP_IDX_MASK (1 << LOOP_PIO_IDX)

// // LED Loop 1
#define LED_LOOP_1_PIO PIOA
#define LED_LOOP_1_PIO_ID ID_PIOA
#define LED_LOOP_1_PIO_IDX 24
#define LED_LOOP_1_IDX_MASK (1 << LED_LOOP_1_PIO_IDX)

// LED Loop 2
#define LED_LOOP_2_PIO PIOD
#define LED_LOOP_2_PIO_ID ID_PIOD
#define LED_LOOP_2_PIO_IDX 31
#define LED_LOOP_2_IDX_MASK (1 << LED_LOOP_2_PIO_IDX)

// Power On/Off
#define POWER_PIO PIOB
#define POWER_PIO_ID ID_PIOB
#define POWER_PIO_IDX 3
#define POWER_IDX_MASK (1 << POWER_PIO_IDX)

// LED Power On/Off
#define LED_POWER_PIO PIOD
#define LED_POWER_PIO_ID ID_PIOD
#define LED_POWER_PIO_IDX 25
#define LED_POWER_IDX_MASK (1 << LED_POWER_PIO_IDX)

// Botão analógico de Volume (PD30)
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 5 // Canal do pino PD30

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX (320)
#define LV_VER_RES_MAX (240)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv; /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BLUETOOTH_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_ANALOGIC_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_ANALOGIC_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_RXDATA_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_RXDATA_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_SLEEP_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_SLEEP_STACK_PRIORITY (tskIDLE_PRIORITY)

/** Queue for msg log send data */
QueueHandle_t xQueueButtonModified;
QueueHandle_t xQueueAnalogicData;
QueueHandle_t xQueueRxData;
QueueHandle_t xQueueLcdData;

TaskHandle_t xTaskBluetooth;
TaskHandle_t xTaskReceive;
TaskHandle_t xTaskAnalogic;

SemaphoreHandle_t xSemaphoreSleep;

typedef struct
{
    char id;
    char data1[2];
    char data0[2];
    char eop;
} button;

typedef struct
{
    char id;
    char is_playing;
    char is_shuffle;
    char repeat_mode;
    int volume;
    char music_name[33];
    char artist[17];
    int duration;
    int current_time;
    char eop[5];
} music_info;

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

static void task_receive(void *pvParameters);
static void task_sleep(void *pvParameters);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void pin_toggle(Pio *pio, uint32_t mask);
static void USART1_init(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
static void configure_console(void);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile char playing = 0;
volatile char power_on = 0;
volatile char handshake_completed = 0;
volatile music_info current_music = {0, 0, 0, 0, 0, "", "", 0, 0, ""};
volatile int last_music_duration = 0;
/************************************************************************/
/* RTOS application HOOK */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
    printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
    /* If the parameters have been corrupted then inspect pxCurrentTCB to
     * identify which task has overflowed its stack.
     */
    for (;;) {
    }
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
    pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Clicked");
    } else if (code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("Toggled");
    }
}

void initial_screen_lcd(void) {
    lv_obj_t *label;

    // create label with "ARM V Control - Spotify"
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "ARM V Control - Spotify");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -80);

    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &spotify_logo);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Conforto na hora de tocar musica.");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 80);

    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "ARM V Control Copyright 2022");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 105);
}

// update screen with music_info
void update_screen_lcd(music_info *music) {
    lv_obj_clean(lv_scr_act());

    lv_obj_t *label;

    // create label with "ARM V Control - Spotify"
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "ARM V Control - Spotify");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -80);

    // create label with music name
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, music->music_name);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -40);

    // create label with artist
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, music->artist);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // create label with duration
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Duration:");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 40);

    int current_time = music->current_time; // current_time in seconds
    int current_time_sec = current_time % 60;
    int current_time_min = current_time / 60;

    //<> DEBUG timer
    printf("timer: %d\r\n", current_time);

    // create bar with duration
    lv_obj_t *bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(bar, 200, 15);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 60);
    lv_bar_set_range(bar, 0, music->duration);
    lv_bar_set_value(bar, current_time, LV_ANIM_OFF);
    // convert music duration to minutes and seconds
    int minutes = music->duration / 60;
    int seconds = music->duration % 60;
    char duration[16];
    sprintf(duration, "%d:%02d/%d:%02d", current_time_min, current_time_sec, minutes, seconds);

    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, duration);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 85);

    // create bar with volume
    lv_obj_t *bar1 = lv_bar_create(lv_scr_act());
    lv_obj_set_size(bar1, 15, 200);
    lv_obj_align(bar1, LV_ALIGN_BOTTOM_LEFT, 5, -10);
    lv_bar_set_range(bar1, 0, 100);
    lv_bar_set_value(bar1, music->volume, LV_ANIM_OFF);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
void TC0_Handler(void) {

    volatile uint32_t status = tc_get_status(TC0, 0);

    pin_toggle(LED_POWER_PIO, LED_POWER_IDX_MASK);
}

void TC1_Handler(void) {
    volatile uint32_t ul_dummy;

    ul_dummy = tc_get_status(TC0, 1);

    /* Avoid compiler warning */
    UNUSED(ul_dummy);

    /* Selecina canal e inicializa conversão */
    afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
    afec_start_software_conversion(AFEC_POT);
}

void TC3_Handler(void) {
    volatile uint32_t status = tc_get_status(TC1, 0);
    // increase current_time from song if playing
    printf("Status %c\r\n", current_music.is_playing);
    if (current_music.is_playing == '1') {
        current_music.current_time++;
    }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask) {
    if (pio_get_output_data_status(pio, mask))
        pio_clear(pio, mask);
    else
        pio_set(pio, mask);
}

// play pause led toggle with music_info_struct is_playing status
void play_pause_led_toggle(music_info *music_info_struct) {
    if (music_info_struct->is_playing == '1') {
        pio_set(LED_PLAY_PAUSE_PIO, LED_PLAY_PAUSE_IDX_MASK);
    } else {
        pio_clear(LED_PLAY_PAUSE_PIO, LED_PLAY_PAUSE_IDX_MASK);
    }
}

// Shuffle led toggle with music_info_struct is_shuffle status
void shuffle_led_toggle(music_info *music_info_struct) {
    if (music_info_struct->is_shuffle == '1') {
        pio_set(LED_SHUFFLE_PIO, LED_SHUFFLE_IDX_MASK);
    } else {
        pio_clear(LED_SHUFFLE_PIO, LED_SHUFFLE_IDX_MASK);
    }
}

// Loop led toggle with music_info_struct repeat_mode status
void loop_led_toggle(music_info *music_info_struct) {
    if (music_info_struct->repeat_mode == '1') {
        pio_set(LED_LOOP_1_PIO, LED_LOOP_1_IDX_MASK);
        pio_clear(LED_LOOP_2_PIO, LED_LOOP_2_IDX_MASK);
    } else if (music_info_struct->repeat_mode == '2') {
        pio_set(LED_LOOP_1_PIO, LED_LOOP_1_IDX_MASK);
        pio_set(LED_LOOP_2_PIO, LED_LOOP_2_IDX_MASK);
    } else {
        pio_clear(LED_LOOP_1_PIO, LED_LOOP_1_IDX_MASK);
        pio_clear(LED_LOOP_2_PIO, LED_LOOP_2_IDX_MASK);
    }
}

// delete big white spaces from string
void delete_big_white_spaces(char *str) {
    int i, j;
    for (i = 0, j = 0; str[i] != '\0'; i++) {
        if (str[i] != ' ' || str[i + 1] != ' ') {
            str[j] = str[i];
            j++;
        }
    }
    str[j] = '\0';
}

// receive buffer
void decode_music_info(char *buffer, music_info *current_music, int *last_music_duration) {
    // delete 3 first characters from buffer
    char *new_buffer = &buffer[3];

    // create struct music_info from buffer
    (*current_music).id = new_buffer[0];
    (*current_music).is_playing = new_buffer[1];

    (*current_music).is_shuffle = new_buffer[2];
    (*current_music).repeat_mode = new_buffer[3];

    char volume[4];
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            volume[i] = '\0';
        } else {
            volume[i] = new_buffer[4 + i];
        }
    }
    (*current_music).volume = atoi(volume);

    for (int i = 0; i < 33; i++) {
        if (i == 32) {
            (*current_music).music_name[i] = '\0';
        } else {
            (*current_music).music_name[i] = new_buffer[7 + i];
        }
    }

    for (int i = 0; i < 17; i++) {
        if (i == 16) {
            (*current_music).artist[i] = '\0';
        } else {
            (*current_music).artist[i] = new_buffer[39 + i];
        }
    }

    char duration[4];

    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            duration[i] = '\0';
        } else {
            duration[i] = new_buffer[55 + i];
        }
    }
    (*current_music).duration = atoi(duration);

    for (int i = 0; i < 5; i++) {
        if (i == 4) {
            (*current_music).eop[i] = '\0';
        } else {
            (*current_music).eop[i] = new_buffer[i + 58];
        }
    }

    // remove all '~' from music_name
    for (int i = 0; i < 33; i++) {
        if ((*current_music).music_name[i] == '~') {
            (*current_music).music_name[i] = ' ';
        }
    }

    // remove all '~' from artist
    for (int i = 0; i < 17; i++) {
        if ((*current_music).artist[i] == '~') {
            (*current_music).artist[i] = ' ';
        }
    }

    delete_big_white_spaces((*current_music).music_name);
    delete_big_white_spaces((*current_music).artist);

    // print (*current_music)
    printf("\n\n");
    printf("id: %c\n", (*current_music).id);
    printf("is_playing: %c\n", (*current_music).is_playing);
    printf("is_shuffle: %c\n", (*current_music).is_shuffle);
    printf("repeat_mode: %c\n", (*current_music).repeat_mode);
    printf("volume: %d\n", (*current_music).volume);
    printf("music_name: %s\n", (*current_music).music_name);
    printf("artist: %s\n", (*current_music).artist);
    printf("duration: %d\n", (*current_music).duration);
    printf("eop: %s\n", (*current_music).eop);

    play_pause_led_toggle(current_music);
    shuffle_led_toggle(current_music);
    loop_led_toggle(current_music);

    // check if last_music_duration has changed
    if (*last_music_duration != (*current_music).duration) {

        printf("Music changed! - l:%d -> n:%d \n", *last_music_duration, (*current_music).duration);
        *last_music_duration = (*current_music).duration;
        (*current_music).current_time = 0;
    }
    // update lcd with music_info_struct
    xQueueSend(xQueueLcdData, current_music, 10);
}

void LED_init(Pio *p_pio, uint32_t ul_id, const uint32_t mask, int estado) {
    pmc_enable_periph_clk(ul_id);
    pio_set_output(p_pio, mask, estado, 0, 0);
};

void BTN_interrupt_init(Pio *pio, uint32_t btn_id, uint32_t btn_mask, void (*callback)(void), uint32_t IRQ_type, int prioridade) {
    pio_set_input(pio, btn_mask, PIO_PULLUP | PIO_DEBOUNCE);
    pio_enable_interrupt(pio, btn_mask);
    NVIC_EnableIRQ(btn_id);
    NVIC_SetPriority(btn_id, prioridade);
    pio_set_debounce_filter(pio, btn_mask, 60);
    pio_handler_set(
        pio,
        btn_id,
        btn_mask,
        IRQ_type,
        *callback);
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();

    pmc_enable_periph_clk(ID_TC);

    tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

    NVIC_SetPriority((IRQn_Type)ID_TC, 4);
    NVIC_EnableIRQ((IRQn_Type)ID_TC);
    tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void configure_console(void) {
    const usart_serial_options_t uart_serial_options = {
        .baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
        .charlength = CONF_UART_CHAR_LENGTH,
#endif
        .paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
        .stopbits = CONF_UART_STOP_BITS,
#endif
    };

    /* Configure console UART. */
    stdio_serial_init(CONF_UART, &uart_serial_options);

/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
    setbuf(stdout, NULL);
#else
/* Already the case in IAR's Normal DLIB default configuration: printf()
 * emits one character at a time.
 */
#endif
};

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
    /*************************************
     * Ativa e configura AFEC
     *************************************/
    /* Ativa AFEC - 0 */
    afec_enable(afec);

    /* struct de configuracao do AFEC */
    struct afec_config afec_cfg;

    /* Carrega parametros padrao */
    afec_get_config_defaults(&afec_cfg);

    /* Configura AFEC */
    afec_init(afec, &afec_cfg);

    /* Configura trigger por software */
    afec_set_trigger(afec, AFEC_TRIG_SW);

    /*** Configuracao específica do canal AFEC ***/
    struct afec_ch_config afec_ch_cfg;
    afec_ch_get_config_defaults(&afec_ch_cfg);
    afec_ch_cfg.gain = AFEC_GAINVALUE_0;
    afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

    /*
    * Calibracao:
    * Because the internal ADC offset is 0x200, it should cancel it and shift
    down to 0.
    */
    afec_channel_set_analog_offset(afec, afec_channel, 0x200);

    /***  Configura sensor de temperatura ***/
    struct afec_temp_sensor_config afec_temp_sensor_cfg;

    afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
    afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

    /* configura IRQ */
    afec_set_callback(afec, afec_channel, callback, 1);
    NVIC_SetPriority(afec_id, 4);
    NVIC_EnableIRQ(afec_id);
}

uint32_t usart_puts(uint8_t *pstring) {
    uint32_t i;

    while (*(pstring + i))
        if (uart_is_tx_empty(USART_COM))
            usart_serial_putchar(USART_COM, *(pstring + i++));
}

void usart_put_string(Usart *usart, char str[]) {
    usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
    uint timecounter = timeout_ms;
    uint32_t rx;
    uint32_t counter = 0;

    while ((timecounter > 0) && (counter < bufferlen)) {
        if (usart_read(usart, &rx) == 0) {
            buffer[counter++] = rx;
        } else {
            timecounter--;
            vTaskDelay(1);
        }
    }
    buffer[counter] = 0x00;
    return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
                        char buffer_tx[], int timeout) {
    usart_put_string(usart, buffer_tx);
    usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

static void configure_lcd(void) {
    /**LCD pin configure on SPI*/
    pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS); //
    pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
    pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
    pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
    pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
    pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);

    ili9341_init();
    ili9341_backlight_on();
}

void config_usart0(void) {
    sysclk_enable_peripheral_clock(ID_USART0);
    usart_serial_options_t config;
    config.baudrate = 9600;
    config.charlength = US_MR_CHRL_8_BIT;
    config.paritytype = US_MR_PAR_NO;
    config.stopbits = false;
    usart_serial_init(USART0, &config);
    usart_enable_tx(USART0);
    usart_enable_rx(USART0);

    // RX - PB0  TX - PB1
    pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
    pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);

    /* ativando interrupcao */
    // usart_enable_interrupt(USART0, US_IER_RXRDY);
    // NVIC_SetPriority(ID_USART0, 4);
    // NVIC_EnableIRQ(ID_USART0);
}

int hc05_init(void) {
    char buffer_rx[128];
    usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEARM V Control D", 100);
    // usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEARM V Control G", 100);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN1234", 100);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

// transform int to hex
void int_to_hex(char *analogic_data, uint32_t num) {
    sprintf(analogic_data, "%04X", num);
}
/************************************************************************/
/* CALLBACKS                                                                */
/************************************************************************/
void btn_play_callback(void) {
    // Apertou o botão play -> Altera o status de playing
    playing = !playing;
    button btn = {'P', "00", "01", 'X'};
    // printf("Apertou o botao de play");
    xQueueSendFromISR(xQueueButtonModified, (void *)&btn, 5);
}

void btn_next_callback(void) {
    button btn = {'N', "00", "01", 'X'};
    printf("Apertou o botao de next");
    xQueueSendFromISR(xQueueButtonModified, (void *)&btn, 5);
}

void btn_prev_callback(void) {
    button btn = {'B', "00", "01", 'X'};
    // printf("Apertou o botao de prev");
    xQueueSendFromISR(xQueueButtonModified, (void *)&btn, 5);
}

void btn_shuffle_callback(void) {
    button btn = {'S', "00", "01", 'X'};
    // printf("Apertou o botao de shuffle");
    xQueueSendFromISR(xQueueButtonModified, (void *)&btn, 5);
}

void btn_loop_callback(void) {
    button btn = {'L', "00", "01", 'X'};
    // printf("Apertou o botao de loop");
    xQueueSendFromISR(xQueueButtonModified, (void *)&btn, 5);
}

void btn_power_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    xSemaphoreGiveFromISR(xSemaphoreSleep, &xHigherPriorityTaskWoken); // libera o semaforo para por controle em sleep
    pin_toggle(LED_POWER_PIO, LED_POWER_IDX_MASK);
}

static void AFEC_pot_Callback(void) {
    uint32_t analogic_read;
    analogic_read = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    xQueueSendFromISR(xQueueAnalogicData, &analogic_read, &xHigherPriorityTaskWoken);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_lcd(void *pvParameters) {

    initial_screen_lcd();

    music_info music_info_struct;

    for (;;) {
        lv_tick_inc(50);
        lv_task_handler();
        vTaskDelay(100);

        if (xQueueReceive(xQueueLcdData, (void *)&music_info_struct, 100)) {

            update_screen_lcd(&music_info_struct);
        }
    }
}

void task_analogic(void) {
    // Init para leitura do ADC
    config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);

    // Init do TC
    TC_init(TC0, ID_TC1, 1, 10);
    tc_start(TC0, 1);

    // Variáveis para armazenamento
    uint32_t analogic_read;
    uint32_t amostra[10];
    int count = 0;
    uint32_t media;
    char analogic_data[5];

    for (;;) {
        if (xQueueReceive(xQueueAnalogicData, &analogic_read, 1000)) {
            // Guarda o dado lido no vetor amostra
            // acrescenta o contador
            amostra[count] = analogic_read;
            count++;
        }

        // Calcula média, se o contador for igual a 10
        if (count == 10) {
            media = 0;
            for (int i = 0; i < 10; i++) {
                media += amostra[i];
            }
            media /= 10;
            count = 0;

            // Converte para hexadecimal e salva em analogic_data
            int_to_hex(&analogic_data, media);

            // Empacota o dado no struct button
            button btn = {'V', {analogic_data[0], analogic_data[1]}, {analogic_data[2], analogic_data[3]}, 'X'};

            // Envia média para a fila
            xQueueSend(xQueueButtonModified, &btn, 10);
        }
    }
}

void task_bluetooth(void) {
    // Init
    config_usart0();
    hc05_init();

    power_on = 1;
    TC_init(TC0, ID_TC0, 0, 10);
    tc_start(TC0, 0);

    // only create task if receive handshake
    while (!handshake_completed) {
        char rx;
        if (usart_read(USART_COM, &rx) == 0) {
            if (rx == 'H') {
                printf("Handshake received\n");

                while (!usart_is_tx_ready(USART_COM)) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                usart_put_string(USART_COM, "H");

                tc_stop(TC0, 0);
                pio_set(LED_POWER_PIO, LED_POWER_IDX_MASK);

                handshake_completed = 1;
            }
        }
    }

    if (handshake_completed) {
        xTaskCreate(task_receive, "RXDATA", TASK_RXDATA_STACK_SIZE, NULL, TASK_RXDATA_STACK_PRIORITY, &xTaskReceive);
        xTaskCreate(task_analogic, "ANLG", TASK_ANALOGIC_STACK_SIZE, NULL, TASK_ANALOGIC_STACK_PRIORITY, &xTaskAnalogic);
    }

    char command_test = '1';
    char eof = 'X';

    for (;;) {
        // Checa se recebeu alguma informação na fila de Botões
        // Traduz a informação (struct) para o protocolo de comunicação (string)
        // {id} {v0} {v1} {eop} -- 1 byte cada campo
        // Espera USART ficar livre para enviar pacote.

        button btn_status;

        if (xQueueReceive(xQueueButtonModified, (void *)&btn_status, 500)) {
            // transforma struct em string
            char package[7];
            package[0] = btn_status.id;
            package[1] = btn_status.data1[0];
            package[2] = btn_status.data1[1];
            package[3] = btn_status.data0[0];
            package[4] = btn_status.data0[1];
            package[5] = btn_status.eop;
            package[6] = '\0';

            // espera usart ficar livre para enviar string
            while (!usart_is_tx_ready(USART_COM)) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            usart_put_string(USART_COM, package);

        } // end if Queue

        // dorme por 500 ms
        // vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void task_sleep(void *pvParameters) {

    for (;;) {
        // Espera o semaforo para poder dormir
        if (xSemaphoreTake(xSemaphoreSleep, portMAX_DELAY) == pdTRUE) {
            if (power_on) {
                printf("\nDesativando Controle... \n");
                vTaskDelete(xTaskBluetooth);
                xQueueReset(xQueueButtonModified);
                vTaskDelete(xTaskAnalogic);
                xQueueReset(xQueueAnalogicData);
                vTaskDelete(xTaskReceive);
                xQueueReset(xQueueRxData);
                xQueueReset(xQueueLcdData);

                power_on = 0;
                handshake_completed = 0;
                printf("Entrando em sleep mode...\n");
                pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
            } else {
                printf("Ativando controle novamente... \n");
                xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTaskBluetooth);
                power_on = 1;
            }
        }
    }
}
static void task_receive(void *pvParameters) {

    // init TC
    TC_init(TC1, ID_TC3, 0, 1);
    tc_start(TC1, 0);
    char buffer[66];
    char record_buffer = 0;
    int count = 0;

    for (;;) {
        char rx;
        if (usart_read(USART_COM, &rx) == 0) {
            if (rx == 'P' && record_buffer == 0 && count == 0) {
                record_buffer = 1;
            }

            if (count == 65) {
                buffer[count] = '\0';
                count = 0;
                record_buffer = 0;

                // check if eop is correct
                if (buffer[61] == 'A' && buffer[62] == 'R' && buffer[63] == 'M' && buffer[64] == 'C') {

                    decode_music_info(buffer, &current_music, &last_music_duration);
                } else {
                    printf("EOP incorreto\n");
                }
            }

            if (record_buffer == 1 && count < 65) {
                buffer[count] = rx;
                // print buffer
                // printf("%c", buffer[count]);
                count++;
            }
        }
    }
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    ili9341_set_top_left_limit(area->x1, area->y1);
    ili9341_set_bottom_right_limit(area->x2, area->y2);
    ili9341_copy_pixels_to_screen(color_p, (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    int px, py, pressed;

    if (readPoint(&px, &py))
        data->state = LV_INDEV_STATE_PRESSED;
    else
        data->state = LV_INDEV_STATE_RELEASED;

    data->point.x = px;
    data->point.y = py;
}

void configure_lvgl(void) {
    lv_init();
    lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);

    lv_disp_drv_init(&disp_drv);       /*Basic initialization*/
    disp_drv.draw_buf = &disp_buf;     /*Set an initialized buffer*/
    disp_drv.flush_cb = my_flush_cb;   /*Set a flush callback to draw to the display*/
    disp_drv.hor_res = LV_HOR_RES_MAX; /*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = LV_VER_RES_MAX; /*Set the vertical resolution in pixels*/

    lv_disp_t *disp;
    disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

    /* Init input on LVGL */
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_input_read;
    lv_indev_t *my_indev = lv_indev_drv_register(&indev_drv);
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
    /* Initialize the SAM system */
    sysclk_init();
    board_init();
    configure_console();

    /* LCd, touch and lvgl init*/
    configure_lcd();
    configure_touch();
    configure_lvgl();

    // Inicializa todos os botões
    // -- play / pause
    BTN_interrupt_init(PLAY_PAUSE_PIO, PLAY_PAUSE_PIO_ID, PLAY_PAUSE_IDX_MASK, btn_play_callback, PIO_IT_FALL_EDGE, 5);
    // -- next / foward
    BTN_interrupt_init(NEXT_PIO, NEXT_PIO_ID, NEXT_IDX_MASK, btn_next_callback, PIO_IT_FALL_EDGE, 5);
    // -- prev / backward
    BTN_interrupt_init(PREV_PIO, PREV_PIO_ID, PREV_IDX_MASK, btn_prev_callback, PIO_IT_FALL_EDGE, 5);
    // -- shuffle
    BTN_interrupt_init(SHUFFLE_PIO, SHUFFLE_PIO_ID, SHUFFLE_IDX_MASK, btn_shuffle_callback, PIO_IT_FALL_EDGE, 5);
    // -- loop
    BTN_interrupt_init(LOOP_PIO, LOOP_PIO_ID, LOOP_IDX_MASK, btn_loop_callback, PIO_IT_FALL_EDGE, 5);
    // -- power on/off
    BTN_interrupt_init(POWER_PIO, POWER_PIO_ID, POWER_IDX_MASK, btn_power_callback, PIO_IT_FALL_EDGE, 5);

    // -- Power on/off led
    LED_init(LED_POWER_PIO, LED_POWER_PIO_ID, LED_POWER_IDX_MASK, 0);
    // -- Play / pause led LED_PLAY_PAUSE_PIO
    LED_init(LED_PLAY_PAUSE_PIO, LED_PLAY_PAUSE_PIO_ID, LED_PLAY_PAUSE_IDX_MASK, 1);
    // -- Loop led 1
    LED_init(LED_LOOP_1_PIO, LED_LOOP_1_PIO_ID, LED_LOOP_1_IDX_MASK, 1);
    // -- Loop led 2
    LED_init(LED_LOOP_2_PIO, LED_LOOP_2_PIO_ID, LED_LOOP_2_IDX_MASK, 1);
    // Shuffle 1 led
    LED_init(LED_SHUFFLE_PIO, LED_SHUFFLE_PIO_ID, LED_SHUFFLE_IDX_MASK, 1);

    /*
     Cria as Tasks necessárias para o funcionamento do controle
         - task_bluetooth -> Recebe uma struct na fila e envia o dado via Serial
         - task_analogic -> Trata a leitura de componentes analógicos

  */
    xSemaphoreSleep = xSemaphoreCreateBinary();
    if (xSemaphoreSleep == NULL)
        printf("falha em criar o semaforo \n");

    xTaskCreate(task_sleep, "SLP", TASK_SLEEP_STACK_SIZE, NULL, TASK_SLEEP_STACK_PRIORITY, NULL);
    xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTaskBluetooth);

    if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Failed to create lcd task\r\n");
    }

    xQueueButtonModified = xQueueCreate(10, sizeof(button));
    xQueueAnalogicData = xQueueCreate(100, sizeof(uint32_t));
    xQueueRxData = xQueueCreate(100, sizeof(char));
    xQueueLcdData = xQueueCreate(100, sizeof(music_info));

    /* Start the scheduler. */
    vTaskStartScheduler();

    while (1) {
    }

    /* Will only get here if there was insufficient memory to create the idle task. */
    return 0;
}
