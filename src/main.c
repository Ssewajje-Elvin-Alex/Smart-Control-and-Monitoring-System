// CoCIS Smart Control and Monitoring System - under-400-line version

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "driver/uart.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"
#define SPI_CLK       GPIO_NUM_18
#define SPI_MOSI      GPIO_NUM_23
#define SPI_MISO      GPIO_NUM_19
#define RFID_RESET    GPIO_NUM_4
#define ENTRY_CS      GPIO_NUM_5
#define EXIT_CS       GPIO_NUM_16
#define SD_CS         GPIO_NUM_15
#define SHARED_HOST   SPI2_HOST
#define SHIFT_PL      GPIO_NUM_17
#define SHIFT_CP      GPIO_NUM_2
#define SHIFT_Q7      GPIO_NUM_36
#define PIN_ENTRY     GPIO_NUM_25
#define PIN_EXIT      GPIO_NUM_26
#define PIN_ROOM1     GPIO_NUM_27
#define PIN_ROOM2     GPIO_NUM_14
#define PIR1          GPIO_NUM_34
#define PIR2          GPIO_NUM_35
#define LIGHT1        GPIO_NUM_32
#define LIGHT2        GPIO_NUM_33
#define RED_LED       GPIO_NUM_22
#define BLUE_LED      GPIO_NUM_21
#define BUZZER        GPIO_NUM_13
#define R1_ENTER      0
#define R1_EXIT       1
#define R2_ENTER      2
#define R2_EXIT       3
#define EMERGENCY_BTN 4
#define MAIN_DOOR_TIME 3000000LL
#define ROOM_DOOR_TIME 2000000LL
#define LIGHT_DELAY    2000000LL
#define FLASH_DELAY     200000LL
#define CARD_RELEASE    800000LL
#define PWM_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_TIMER   LEDC_TIMER_1
#define BUZZER_TIMER  LEDC_TIMER_0
#define CLOSED        0
#define OPEN          90
#define SD_MOUNT      "/sdcard"
#define LOG_FILE      "/sdcard/events.csv"
#define RESET_COMMAND "RESET"
#define HOURS_FILE "/sdcard/hours.cfg"
enum { MAIN_ENTRY = 0, MAIN_EXIT, ROOM1_DOOR, ROOM2_DOOR, DOOR_COUNT }; static const gpio_num_t servo_pins[DOOR_COUNT] = { PIN_ENTRY, PIN_EXIT, PIN_ROOM1, PIN_ROOM2 }; static const char *door_names[DOOR_COUNT] = { "MAIN ENTRANCE", "MAIN EXIT", "LECTURE ROOM 1", "LECTURE ROOM 2" };
static bool door_open[DOOR_COUNT]; static int64_t door_deadline[DOOR_COUNT]; typedef struct { const char *name; gpio_num_t pir; gpio_num_t light; int occupancy; bool light_on; bool last_motion; int64_t light_deadline; } room_t; static room_t rooms[2] = { { .name = "ROOM 1", .pir = PIR1, .light = LIGHT1 }, {
        .name = "ROOM 2", .pir = PIR2, .light = LIGHT2 } }; typedef struct { uint8_t uid[4]; const char *name; } student_t; static student_t students[] = { { .uid = {0x01, 0x02, 0x03, 0x04}, .name = "STUDENT A - BLUE" }, { .uid = {0x11, 0x22, 0x33, 0x44}, .name = "STUDENT B - GREEN" } };
#define STUDENT_COUNT \
(sizeof(students) / sizeof(students[0]))
static int building_count = 0; typedef struct { bool entry; uint8_t uid[4]; } rfid_event_t; typedef struct { spi_device_handle_t spi; bool entry; bool latched; uint8_t previous_uid[4]; int64_t last_seen; } reader_t; static reader_t entry_reader = {.entry = true}; static reader_t exit_reader = {.entry = false};
static QueueHandle_t rfid_queue; static bool emergency_active = false; static bool red_phase = true; static int64_t next_flash = 0; static uint8_t previous_raw; static uint8_t stable_buttons; static TickType_t last_change; static char serial_buffer[48]; static size_t serial_length = 0; typedef struct {
    uint64_t uptime_ms; char eat_time[24]; char source[24]; char event[32]; char detail[96]; } log_record_t; static QueueHandle_t log_queue; static SemaphoreHandle_t file_mutex; static bool sd_ready = false; static sdmmc_card_t *sd_card = NULL; static volatile uint32_t records_saved = 0; static volatile uint32_t records_failed = 0;
static volatile uint32_t records_dropped = 0; typedef enum { TERM_MAIN = 0, TERM_ROOM1 = 1, TERM_ROOM2 = 2, TERM_ALL = 3 } terminal_t; static terminal_t active_view = TERM_ALL; static const char *terminal_names[3] = { "TERMINAL 1 - MAIN ENTRANCE/EXIT", "TERMINAL 2 - LECTURE ROOM 1", "TERMINAL 3 - LECTURE ROOM 2" };

/* Terminal views and occupancy reporting */
static void terminal_event(terminal_t term, const char *format, ...) {
    if (active_view != TERM_ALL && active_view != term) { return;
    } char message[192]; va_list args; va_start(args, format); vsnprintf(message, sizeof(message), format, args); va_end(args); printf("[%s] %s\n", terminal_names[term], message); }
static terminal_t terminal_for_door(int door) {
    if (door == ROOM1_DOOR) return TERM_ROOM1;
    if (door == ROOM2_DOOR) return TERM_ROOM2;
    return TERM_MAIN;
}
static int rooms_total(void) { return rooms[0].occupancy + rooms[1].occupancy;
}
static void print_counts(void) { printf("[LIVE OCCUPANCY] BUILDING=%d | LECTURE ROOM 1=%d | " "LECTURE ROOM 2=%d | OTHER AREAS=%d\n", building_count, rooms[0].occupancy, rooms[1].occupancy, building_count - rooms_total()); }
static void print_terminal_status(terminal_t term) {
    if (term == TERM_MAIN) { printf("[TERMINAL 1 - MAIN] BUILDING=%d | ROOM1=%d | ROOM2=%d | " "OTHER AREAS=%d | ENTRY DOOR=%s | EXIT DOOR=%s\n", building_count, rooms[0].occupancy, rooms[1].occupancy, building_count - rooms_total(), door_open[MAIN_ENTRY] ? "OPEN" : "CLOSED", door_open[MAIN_EXIT] ? "OPEN" : "CLOSED"); return;
    } int room = (term == TERM_ROOM1) ? 0 : 1; printf("[TERMINAL %d - ROOM %d] BUILDING=%d | ROOM OCCUPANCY=%d | " "DOOR=%s | LIGHT=%s | PIR=%s\n", room + 2, room + 1, building_count, rooms[room].occupancy,
    door_open[ROOM1_DOOR + room] ? "OPEN" : "CLOSED", rooms[room].light_on ? "ON" : "OFF", rooms[room].last_motion ? "MOTION" : "NO MOTION"); }
static volatile bool wifi_connected = false; static volatile bool ntp_received = false; static bool clock_manually_set = false;
static bool sntp_started = false; static bool admin_session = false; static int open_minute = 7 * 60; static int close_minute = 19 * 60;

/* Software clock, schedules and administrator settings */
static bool clock_valid(void) { time_t current = time(NULL); return (ntp_received || clock_manually_set) && current >= 1704067200;
}
static void clock_stamp(char out[24]) {
    if (!clock_valid()) { snprintf(out, 24, "UNSYNCED"); return; } time_t now = time(NULL); struct tm tm_eat; localtime_r(&now, &tm_eat);
    if (strftime(out, 24, "%Y-%m-%d %H:%M:%S", &tm_eat) == 0) { snprintf(out, 24, "UNSYNCED"); } }
static bool access_open(void) {
    if (!clock_valid()) return false;
    time_t now = time(NULL); struct tm tm_eat; localtime_r(&now, &tm_eat); int minutes = tm_eat.tm_hour * 60 + tm_eat.tm_min;
    if (open_minute == close_minute) return false;
    if (open_minute < close_minute) { return minutes >= open_minute && minutes < close_minute;
    } return minutes >= open_minute || minutes < close_minute;
}
static bool parse_hours(const char *s, int *minute) { int h, m; char extra;
    if (sscanf(s, "%d:%d%c", &h, &m, &extra) != 2 || h < 0 || h > 23 || m < 0 || m > 59) { return false;
    } *minute = h * 60 + m; return true;
}
static void show_time(void) { char eat[24]; clock_stamp(eat); printf("TIME (EAT): %s | Wi-Fi: %s | SNTP: %s\n", eat, wifi_connected ? "CONNECTED" : "OFFLINE", ntp_received ? "SYNCHRONIZED" : "NOT YET"); }
static void show_schedule(void) { printf("SCHEDULE (EAT): OPEN %02d:%02d | CLOSE %02d:%02d | ENTRY %s\n", open_minute / 60, open_minute % 60, close_minute / 60, close_minute % 60, access_open() ? "ALLOWED" : "CLOSED / CLOCK UNSYNCED"); printf("Exits and emergency evacuation are never restricted by schedule.\n"); }
static void schedule_load(void) { FILE *f = fopen(HOURS_FILE, "r");
    if (!f) return;
    int a, b;
    if (fscanf(f, "%d %d", &a, &b) == 2 && a >= 0 && a < 1440 && b >= 0 && b < 1440) { open_minute = a; close_minute = b; } fclose(f); }
static bool schedule_save(void) {
    if (!sd_ready) return false;
    if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(300)) != pdTRUE) { return false;
    } FILE *f = fopen(HOURS_FILE, "w"); bool ok = false;
    if (f) { ok = fprintf(f, "%d %d\n", open_minute, close_minute) > 0;
        if (fclose(f) != 0) ok = false;
    } xSemaphoreGive(file_mutex); return ok;
}
static bool set_manual_time(const char *value) { int y, mon, d, h, min, sec; char extra;
    if (sscanf(value, "%d-%d-%d %d:%d:%d%c", &y, &mon, &d, &h, &min, &sec, &extra) != 6 || y < 2024 || y > 2100 || mon < 1 || mon > 12 || d < 1 || d > 31 || h < 0 || h > 23 || min < 0 || min > 59 || sec < 0 || sec > 59) { return false;
    } struct tm entered = {0}; entered.tm_year = y - 1900; entered.tm_mon = mon - 1; entered.tm_mday = d; entered.tm_hour = h; entered.tm_min = min; entered.tm_sec = sec; entered.tm_isdst = -1; time_t stamp = mktime(&entered);
    if (stamp == (time_t)-1) return false;
    struct tm check; localtime_r(&stamp, &check);
    if (check.tm_year != y - 1900 || check.tm_mon != mon - 1 || check.tm_mday != d || check.tm_hour != h || check.tm_min != min || check.tm_sec != sec) return false;
    struct timeval tv = {.tv_sec = stamp, .tv_usec = 0};
    if (settimeofday(&tv, NULL) != 0) return false;
    clock_manually_set = true; return true;
}
static void log_event( const char *source, const char *event, const char *format, ... ) {
    if (!sd_ready || log_queue == NULL) { return;
    } log_record_t record = {0}; record.uptime_ms = (uint64_t)(esp_timer_get_time() / 1000); clock_stamp(record.eat_time); snprintf( record.source, sizeof(record.source), "%s", source ); snprintf( record.event, sizeof(record.event), "%s", event ); va_list args; va_start(args, format);
    vsnprintf( record.detail, sizeof(record.detail), format, args ); va_end(args);
    if (xQueueSend(log_queue, &record, 0) != pdTRUE) { records_dropped++; printf("WARNING: SD logging queue full\n"); } }

/* GPIO, PWM servo doors, room lights and buttons */
static void gpio_initialize(void) { gpio_config_t outputs = { .pin_bit_mask = (1ULL << SHIFT_PL) | (1ULL << SHIFT_CP) | (1ULL << RFID_RESET) | (1ULL << ENTRY_CS) | (1ULL << EXIT_CS) | (1ULL << SD_CS) | (1ULL << LIGHT1) | (1ULL << LIGHT2) |
        (1ULL << RED_LED) | (1ULL << BLUE_LED), .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE }; ESP_ERROR_CHECK(gpio_config(&outputs)); gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << SHIFT_Q7) | (1ULL << PIR1) | (1ULL << PIR2), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE }; ESP_ERROR_CHECK(gpio_config(&inputs)); gpio_set_level(ENTRY_CS, 1); gpio_set_level(EXIT_CS, 1); gpio_set_level(SD_CS, 1);
    gpio_set_level(RFID_RESET, 0); gpio_set_level(SHIFT_CP, 0); gpio_set_level(SHIFT_PL, 1); gpio_set_level(LIGHT1, 0); gpio_set_level(LIGHT2, 0); gpio_set_level(RED_LED, 0); gpio_set_level(BLUE_LED, 0); }
static void servo_initialize(void) { ledc_timer_config_t timer = { .speed_mode = PWM_MODE, .timer_num = SERVO_TIMER, .duty_resolution = LEDC_TIMER_16_BIT, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK }; ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (int i = 0; i < DOOR_COUNT; i++) { ledc_channel_config_t channel = { .gpio_num = servo_pins[i], .speed_mode = PWM_MODE, .channel = (ledc_channel_t)(i + 1), .timer_sel = SERVO_TIMER, .duty = 0, .hpoint = 0 }; ESP_ERROR_CHECK( ledc_channel_config(&channel) ); } }
static void servo_move(int door, int angle) {
    if (door < 0 || door >= DOOR_COUNT) { return;
    } uint32_t pulse_us = 500U + ((uint32_t)angle * 2000U / 180U); uint32_t duty = (pulse_us * 65536U) / 20000U; ledc_channel_t channel = (ledc_channel_t)(door + 1); ESP_ERROR_CHECK( ledc_set_duty(PWM_MODE, channel, duty) ); ESP_ERROR_CHECK( ledc_update_duty(PWM_MODE, channel) ); }
static void open_door(int door, int64_t duration) {
    if (emergency_active) return;
    servo_move(door, OPEN); door_open[door] = true; door_deadline[door] = esp_timer_get_time() + duration; terminal_event(terminal_for_door(door), "DOOR OPENED: %s", door_names[door]); log_event("DOOR", "OPEN", "%s", door_names[door]); }
static void update_doors(void) {
    if (emergency_active) return;
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < DOOR_COUNT; i++) {
        if (door_open[i] && now >= door_deadline[i]) { servo_move(i, CLOSED); door_open[i] = false; door_deadline[i] = 0; terminal_event(terminal_for_door(i), "DOOR CLOSED: %s", door_names[i]); log_event("DOOR", "CLOSE", "%s", door_names[i]); } } }
static void buzzer_initialize(void) { ledc_timer_config_t timer = { .speed_mode = PWM_MODE, .timer_num = BUZZER_TIMER, .duty_resolution = LEDC_TIMER_10_BIT, .freq_hz = 1000, .clk_cfg = LEDC_AUTO_CLK }; ESP_ERROR_CHECK(ledc_timer_config(&timer)); ledc_channel_config_t channel = {
        .gpio_num = BUZZER, .speed_mode = PWM_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = BUZZER_TIMER, .duty = 0, .hpoint = 0 }; ESP_ERROR_CHECK(ledc_channel_config(&channel)); }
static void buzzer_set(bool on) { ESP_ERROR_CHECK( ledc_set_duty( PWM_MODE, LEDC_CHANNEL_0, on ? 512 : 0 ) ); ESP_ERROR_CHECK( ledc_update_duty( PWM_MODE, LEDC_CHANNEL_0 ) ); }
static void set_room_light(int room, bool on) { room_t *r = &rooms[room];
    if (r->light_on == on) return;
    gpio_set_level(r->light, on ? 1 : 0); r->light_on = on; terminal_event(room == 0 ? TERM_ROOM1 : TERM_ROOM2, "ROOM %d LIGHT: %s | OCCUPANCY=%d", room + 1, on ? "ON" : "OFF", r->occupancy); log_event(r->name, on ? "LIGHT_ON" : "LIGHT_OFF", "Occupancy %d", r->occupancy); }

static void room_entry(int room) {
    if (emergency_active) return;
    room_t *r = &rooms[room]; terminal_t term = room == 0 ? TERM_ROOM1 : TERM_ROOM2;
    if (rooms_total() >= building_count) { terminal_event(term, "ROOM %d ENTRY DENIED: ALL %d BUILDING OCCUPANTS ALREADY ALLOCATED", room + 1, building_count); print_counts(); log_event(r->name, "ENTRY_DENIED", "Building %d Room1 %d Room2 %d", building_count, rooms[0].occupancy, rooms[1].occupancy); return; }
    if (!access_open()) { terminal_event(term, "ROOM %d ENTRY DENIED: BUILDING CLOSED OR TIME UNSYNCED", room + 1); print_counts(); log_event(r->name, "ENTRY_DENIED", "Schedule/clock restriction"); return; } open_door(ROOM1_DOOR + room, ROOM_DOOR_TIME); r->occupancy++; r->light_deadline = 0; set_room_light(room, true);
    terminal_event(term, "ROOM %d ENTRY GRANTED | OCCUPANCY=%d | BUILDING=%d", room + 1, r->occupancy, building_count); log_event(r->name, "ENTRY", "Occupancy %d Building %d", r->occupancy, building_count); print_counts(); }
static void room_exit(int room) {
    if (emergency_active) return;
    room_t *r = &rooms[room]; terminal_t term = room == 0 ? TERM_ROOM1 : TERM_ROOM2; open_door(ROOM1_DOOR + room, ROOM_DOOR_TIME);
    if (r->occupancy == 0) { terminal_event(term, "ROOM %d EXIT NOT COUNTED: ROOM ALREADY EMPTY", room + 1); log_event(r->name, "EXIT_EMPTY", "Occupancy 0"); print_counts(); return; } r->occupancy--; terminal_event(term, "ROOM %d EXIT RECORDED | OCCUPANCY=%d | BUILDING=%d", room + 1, r->occupancy, building_count);
    log_event(r->name, "EXIT", "Occupancy %d Building %d", r->occupancy, building_count);
    if (r->occupancy == 0) { r->light_deadline = esp_timer_get_time() + LIGHT_DELAY; terminal_event(term, "ROOM %d EMPTY: LIGHT OFF AFTER 2 SECONDS", room + 1); } else { r->light_deadline = 0; set_room_light(room, true); } print_counts(); }
static void update_room_lighting(void) { int64_t now = esp_timer_get_time();
    for (int i = 0; i < 2; i++) { room_t *r = &rooms[i]; bool motion = gpio_get_level(r->pir);
        if (motion != r->last_motion) { r->last_motion = motion; terminal_event(i == 0 ? TERM_ROOM1 : TERM_ROOM2, "ROOM %d PIR: %s | OCCUPANCY=%d", i + 1, motion ? "MOTION" : "CLEAR", r->occupancy); log_event(r->name, motion ? "PIR_MOTION" : "PIR_CLEAR", "Occupancy %d", r->occupancy); }
        if (emergency_active) { set_room_light(i, true); } else if (r->occupancy > 0) { r->light_deadline = 0; set_room_light(i, true); } else if (r->light_deadline != 0 && now >= r->light_deadline) { r->light_deadline = 0; set_room_light(i, false); } } }
static uint8_t shift_read(void) { uint8_t value = 0; gpio_set_level(SHIFT_CP, 0); gpio_set_level(SHIFT_PL, 0); esp_rom_delay_us(5); gpio_set_level(SHIFT_PL, 1); esp_rom_delay_us(5);
    for (int bit = 7; bit >= 0; bit--) {
        if (gpio_get_level(SHIFT_Q7)) { value |= (1U << bit); } gpio_set_level(SHIFT_CP, 1); esp_rom_delay_us(5); gpio_set_level(SHIFT_CP, 0); esp_rom_delay_us(5); } return value;
}

/* Emergency handling: evacuation, reset and warning lights */

static void activate_emergency(void) {
    if (emergency_active) return;
    emergency_active = true; int prev_building = building_count; int prev_r1 = rooms[0].occupancy; int prev_r2 = rooms[1].occupancy; building_count = 0; rooms[0].occupancy = 0; rooms[1].occupancy = 0;
    if (rfid_queue != NULL) xQueueReset(rfid_queue);
    log_event("EMERGENCY", "ACTIVATED", "Evacuate; zero counts; before B%d R1%d R2%d", prev_building, prev_r1, prev_r2); printf("\n!!! EMERGENCY EVACUATION ACTIVE !!!\n");
    for (int i = 0; i < DOOR_COUNT; i++) { servo_move(i, OPEN); door_open[i] = true; door_deadline[i] = 0; }
    for (int i = 0; i < 2; i++) { rooms[i].light_deadline = 0; set_room_light(i, true); } buzzer_set(true); red_phase = true; gpio_set_level(RED_LED, 1); gpio_set_level(BLUE_LED, 0); next_flash = esp_timer_get_time() + FLASH_DELAY; printf("ALL FOUR DOORS HELD OPEN | BUZZER CONTINUOUS | ALARM LEDs ACTIVE\n");
    printf("BUILDING, ROOM 1 AND ROOM 2 COUNTS RESET TO ZERO\n"); print_counts(); print_terminal_status(TERM_MAIN); print_terminal_status(TERM_ROOM1); print_terminal_status(TERM_ROOM2); }
static void update_alarm(void) {
    if (!emergency_active) { return;
    } int64_t now = esp_timer_get_time();
    if (now >= next_flash) { red_phase = !red_phase; gpio_set_level( RED_LED, red_phase ? 1 : 0 ); gpio_set_level( BLUE_LED, red_phase ? 0 : 1 ); next_flash = now + FLASH_DELAY; } }
static void reset_emergency(void) {
    if (!emergency_active) { printf("NO ACTIVE EMERGENCY\n"); return; }
    if (!(stable_buttons & (1U << EMERGENCY_BTN))) { printf("RELEASE EMERGENCY BUTTON FIRST\n"); return; } buzzer_set(false); gpio_set_level(RED_LED, 0); gpio_set_level(BLUE_LED, 0);
    for (int i = 0; i < DOOR_COUNT; i++) { servo_move(i, CLOSED); door_open[i] = false; door_deadline[i] = 0; } emergency_active = false;
    if (rfid_queue != NULL) xQueueReset(rfid_queue);
    building_count = 0; rooms[0].occupancy = 0; rooms[1].occupancy = 0;
    for (int i = 0; i < 2; i++) { rooms[i].light_deadline = esp_timer_get_time() + LIGHT_DELAY; } printf("EMERGENCY RESET ACCEPTED | BUZZER OFF | FOUR DOORS CLOSED\n"); printf("ALL OCCUPANCY COUNTS REMAIN ZERO\n"); log_event("EMERGENCY", "RESET", "RESET accepted; occupancy remains zero"); print_counts(); }
static void update_buttons(void) { uint8_t current = shift_read(); TickType_t now = xTaskGetTickCount();
    if (current != previous_raw) { previous_raw = current; last_change = now; }
    if ( current != stable_buttons && (now - last_change) >= pdMS_TO_TICKS(40) ) { uint8_t pressed = stable_buttons & (uint8_t)(~current); stable_buttons = current;
        if (pressed & (1U << EMERGENCY_BTN)) { activate_emergency(); }
        if (emergency_active) { return;
        }
        if (pressed & (1U << R1_ENTER)) { room_entry(0); }
        if (pressed & (1U << R1_EXIT)) { room_exit(0); }
        if (pressed & (1U << R2_ENTER)) { room_entry(1); }
        if (pressed & (1U << R2_EXIT)) { room_exit(1); } } }
#define REG_COMMAND     0x01
#define REG_COM_IRQ     0x04
#define REG_ERROR       0x06
#define REG_FIFO_DATA   0x09
#define REG_FIFO_LEVEL  0x0A
#define REG_BIT_FRAMING 0x0D
#define REG_COLL        0x0E
#define REG_MODE        0x11
#define REG_TX_CONTROL  0x14
#define REG_TX_ASK      0x15
#define REG_T_MODE      0x2A
#define REG_T_PRESCALER 0x2B
#define REG_T_RELOAD_H  0x2C
#define REG_T_RELOAD_L  0x2D
#define REG_VERSION     0x37
#define CMD_IDLE        0x00
#define CMD_TRANSCEIVE  0x0C

/* MFRC522 RFID driver and access decisions */
static void rfid_write( spi_device_handle_t dev, uint8_t reg, uint8_t value ) { uint8_t tx[2] = { (uint8_t)((reg << 1) & 0x7E), value }; spi_transaction_t t = { .length = 16, .tx_buffer = tx }; ESP_ERROR_CHECK(spi_device_transmit(dev, &t)); }
static uint8_t rfid_read( spi_device_handle_t dev, uint8_t reg ) { uint8_t tx[2] = { (uint8_t)(0x80 | ((reg << 1) & 0x7E)), 0 }; uint8_t rx[2] = {0}; spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx }; ESP_ERROR_CHECK(spi_device_transmit(dev, &t)); return rx[1];
}
static void fifo_write( spi_device_handle_t dev, const uint8_t *data, int length ) { uint8_t tx[65] = {0}; tx[0] = (REG_FIFO_DATA << 1) & 0x7E; memcpy(&tx[1], data, length); spi_transaction_t t = { .length = (length + 1) * 8, .tx_buffer = tx }; ESP_ERROR_CHECK(spi_device_transmit(dev, &t)); }
static void fifo_read( spi_device_handle_t dev, uint8_t *data, int length ) { uint8_t tx[65] = {0}; uint8_t rx[65] = {0}; uint8_t addr = 0x80 | ((REG_FIFO_DATA << 1) & 0x7E);
    for (int i = 0; i < length; i++) { tx[i] = addr; } tx[length] = 0; spi_transaction_t t = { .length = (length + 1) * 8, .tx_buffer = tx, .rx_buffer = rx }; ESP_ERROR_CHECK(spi_device_transmit(dev, &t)); memcpy(data, &rx[1], length); }
static void rfid_setup(spi_device_handle_t dev) { rfid_write(dev, REG_T_MODE, 0x8D); rfid_write(dev, REG_T_PRESCALER, 0x3E); rfid_write(dev, REG_T_RELOAD_H, 0); rfid_write(dev, REG_T_RELOAD_L, 30); rfid_write(dev, REG_TX_ASK, 0x40); rfid_write(dev, REG_MODE, 0x3D); uint8_t control = rfid_read(dev, REG_TX_CONTROL);
    rfid_write( dev, REG_TX_CONTROL, control | 0x03 ); }
static int rfid_exchange( spi_device_handle_t dev, const uint8_t *data, int length, uint8_t last_bits, uint8_t *response, int max_response ) { rfid_write(dev, REG_COMMAND, CMD_IDLE); rfid_write(dev, REG_COM_IRQ, 0x7F); rfid_write(dev, REG_FIFO_LEVEL, 0x80);
    fifo_write(dev, data, length); rfid_write(dev, REG_BIT_FRAMING, last_bits); rfid_write(dev, REG_COMMAND, CMD_TRANSCEIVE); rfid_write( dev, REG_BIT_FRAMING, last_bits | 0x80 ); bool completed = false;
    for (int i = 0; i < 100; i++) { uint8_t irq = rfid_read(dev, REG_COM_IRQ);
        if (irq & 0x30) { completed = true; break;
        }
        if (irq & 0x01) { break;
        } vTaskDelay(pdMS_TO_TICKS(1)); } rfid_write(dev, REG_BIT_FRAMING, last_bits);
    if (!completed) { return -1;
    }
    if (rfid_read(dev, REG_ERROR) & 0x1B) { return -2;
    } int count = rfid_read(dev, REG_FIFO_LEVEL);
    if (count <= 0 || count > max_response) { return -3;
    } fifo_read(dev, response, count); return count;
}
static bool rfid_get_uid( spi_device_handle_t dev, uint8_t uid[4] ) { uint8_t request = 0x26; uint8_t atqa[2] = {0}; int count = rfid_exchange( dev, &request, 1, 7, atqa, sizeof(atqa) );
    if (count != 2) { return false;
    } vTaskDelay(pdMS_TO_TICKS(5)); uint8_t anticollision[2] = {0x93, 0x20}; uint8_t result[5] = {0}; uint8_t coll = rfid_read(dev, REG_COLL); rfid_write( dev, REG_COLL, coll & (uint8_t)~0x80 ); count = rfid_exchange( dev, anticollision, 2, 0, result, sizeof(result) );
    if (count != 5) { return false;
    } uint8_t bcc = result[0] ^ result[1] ^ result[2] ^ result[3];
    if (bcc != result[4] || result[0] == 0x88) { return false;
    } memcpy(uid, result, 4); return true;
}
static void scan_reader(reader_t *r) { uint8_t uid[4]; bool detected = rfid_get_uid(r->spi, uid); int64_t now = esp_timer_get_time();
    if (detected) { bool fresh = !r->latched || memcmp(r->previous_uid, uid, 4) != 0; r->last_seen = now;
        if (fresh) { memcpy(r->previous_uid, uid, 4); r->latched = true;
            if (emergency_active) return;
            rfid_event_t event = { .entry = r->entry }; memcpy(event.uid, uid, 4);
            if ( xQueueSend( rfid_queue, &event, 0 ) != pdTRUE ) { printf("RFID QUEUE FULL\n"); } } } else if ( r->latched && now - r->last_seen >= CARD_RELEASE ) { r->latched = false; } }
static void rfid_task(void *arg) { (void)arg;
    while (1) { scan_reader(&entry_reader); scan_reader(&exit_reader); vTaskDelay(pdMS_TO_TICKS(30)); } }
static int find_student(const uint8_t uid[4]) {
    for (size_t i = 0; i < STUDENT_COUNT; i++) {
        if (memcmp(uid, students[i].uid, 4) == 0) { return (int)i;
        } } return -1;
}

static void process_rfid(const rfid_event_t *event) { char uid_text[20]; snprintf(uid_text, sizeof(uid_text), "%02X:%02X:%02X:%02X", event->uid[0], event->uid[1], event->uid[2], event->uid[3]);
    if (emergency_active) return;
    int index = find_student(event->uid); const char *direction = event->entry ? "ENTRY" : "EXIT"; terminal_event(TERM_MAIN, "RFID %s SCAN: %s", direction, uid_text);
    if (index < 0) { terminal_event(TERM_MAIN, "ACCESS DENIED: CARD IS NOT AUTHORIZED"); log_event("RFID", "ACCESS_DENIED", "%s UID %s", direction, uid_text); print_counts(); return; }
    if (event->entry) {
        if (!access_open()) { terminal_event(TERM_MAIN, "ENTRY DENIED: CLOSED HOURS OR CLOCK UNSYNCED"); log_event("RFID", "ENTRY_DENIED", "UID %s schedule/clock", uid_text); print_counts(); return; } building_count++; open_door(MAIN_ENTRY, MAIN_DOOR_TIME);
        terminal_event(TERM_MAIN, "ENTRY GRANTED: %s | BUILDING=%d", students[index].name, building_count); log_event("RFID", "ENTRY_GRANTED", "UID %s Building %d", uid_text, building_count); print_counts(); } else { open_door(MAIN_EXIT, MAIN_DOOR_TIME);
        if (building_count <= 0) { terminal_event(TERM_MAIN, "EXIT NOT COUNTED: BUILDING ALREADY EMPTY"); log_event("RFID", "EXIT_EMPTY", "UID %s Building 0", uid_text); print_counts(); return; }
        if (building_count - 1 < rooms_total()) { terminal_event(TERM_MAIN, "EXIT NOT COUNTED: RECORD ROOM EXIT FIRST (ROOMS=%d)", rooms_total()); log_event("RFID", "EXIT_RECONCILE", "UID %s Building %d Rooms %d", uid_text, building_count, rooms_total()); print_counts(); return; }
        building_count--; terminal_event(TERM_MAIN, "EXIT RECORDED: %s | BUILDING=%d", students[index].name, building_count); log_event("RFID", "EXIT_GRANTED", "UID %s Building %d", uid_text, building_count); print_counts(); } }
static void process_rfid_queue(void) { rfid_event_t event;
    for (int i = 0; i < 8; i++) {
        if ( xQueueReceive( rfid_queue, &event, 0 ) != pdTRUE ) { break;
        } process_rfid(&event); } }

/* Shared SPI, microSD files and logging task */
static bool spi_initialize(void) { spi_bus_config_t bus = { .mosi_io_num = SPI_MOSI, .miso_io_num = SPI_MISO, .sclk_io_num = SPI_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096 }; esp_err_t err = spi_bus_initialize( SHARED_HOST, &bus, SPI_DMA_CH_AUTO );
    if (err != ESP_OK) { printf( "SPI BUS ERROR: %s\n", esp_err_to_name(err) ); return false;
    } return true;
}
static bool sd_initialize(void) { sdmmc_host_t host = SDSPI_HOST_DEFAULT(); host.slot = SHARED_HOST; host.max_freq_khz = 4000; sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT(); slot.gpio_cs = SD_CS; slot.host_id = SHARED_HOST; esp_vfs_fat_mount_config_t mount = {
        .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024 }; printf("MOUNTING MICROSD CARD...\n"); esp_err_t err = esp_vfs_fat_sdspi_mount( SD_MOUNT, &host, &slot, &mount, &sd_card );
    if (err != ESP_OK) { printf( "MICROSD MOUNT FAILED: %s\n", esp_err_to_name(err) ); return false;
    } printf("MICROSD CARD MOUNTED SUCCESSFULLY\n"); FILE *file = fopen(LOG_FILE, "a+");
    if (file == NULL) { printf("ERROR: CANNOT OPEN EVENTS.CSV\n"); return false;
    } fseek(file, 0, SEEK_END); long size = ftell(file);
    if (size == 0) { fprintf( file, "uptime_ms,source,event,details\n" ); }
    if (fclose(file) != 0) { printf("ERROR: CANNOT CLOSE LOG FILE\n"); return false;
    } sd_ready = true; schedule_load(); printf("LOG FILE READY: %s\n", LOG_FILE); return true;
}
static bool sd_write_record(const log_record_t *r) { FILE *file = fopen(LOG_FILE, "a");
    if (file == NULL) { return false;
    } int result = fprintf( file, "%llu,%s,%s,EAT=%s %s\n", (unsigned long long)r->uptime_ms, r->source, r->event, r->eat_time, r->detail ); int close_result = fclose(file); return result > 0 && close_result == 0;
}
static void sd_logger_task(void *arg) { (void)arg; log_record_t record;
    while (1) {
        if ( xQueueReceive( log_queue, &record, portMAX_DELAY ) == pdTRUE ) {
            if ( xSemaphoreTake( file_mutex, portMAX_DELAY ) == pdTRUE ) { bool success = sd_write_record(&record);
                if (success) { records_saved++; } else { records_failed++; printf("MICROSD WRITE FAILED\n"); } xSemaphoreGive(file_mutex); } } } }
static void show_logs(void) {
    if (!sd_ready) { printf("MICROSD IS NOT AVAILABLE\n"); return; }
    if ( xSemaphoreTake( file_mutex, pdMS_TO_TICKS(100) ) != pdTRUE ) { printf("SD BUSY - TRY LOGS AGAIN\n"); return; } FILE *file = fopen(LOG_FILE, "r");
    if (file == NULL) { printf("CANNOT READ EVENTS.CSV\n"); xSemaphoreGive(file_mutex); return; } char lines[8][160] = {{0}}; char buffer[160]; int total = 0;
    while (fgets(buffer, sizeof(buffer), file)) { snprintf( lines[total % 8], sizeof(lines[0]), "%s", buffer ); total++; } fclose(file); xSemaphoreGive(file_mutex); printf("\n===== LAST SAVED SD RECORDS =====\n"); int start = total > 8 ? total - 8 : 0;
    for (int i = start; i < total; i++) { printf("%s", lines[i % 8]); } printf("=================================\n"); }
static void show_sd_status(void) { printf("\n========== MICROSD STATUS ==========\n"); printf( "CARD: %s\n", sd_ready ? "MOUNTED" : "NOT AVAILABLE" ); printf("FILE: %s\n", LOG_FILE); printf( "RECORDS SAVED: %lu\n", (unsigned long)records_saved ); printf( "WRITE FAILURES: %lu\n", (unsigned long)records_failed );
    printf( "QUEUE DROPS: %lu\n", (unsigned long)records_dropped );
    if (log_queue != NULL) { printf( "PENDING RECORDS: %u\n", (unsigned)uxQueueMessagesWaiting(log_queue) ); } printf("====================================\n"); }
static void rfid_initialize(void) { gpio_set_level(RFID_RESET, 0); vTaskDelay(pdMS_TO_TICKS(20)); gpio_set_level(RFID_RESET, 1); vTaskDelay(pdMS_TO_TICKS(50)); spi_device_interface_config_t entry = { .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = ENTRY_CS, .queue_size = 1 };
    ESP_ERROR_CHECK( spi_bus_add_device( SHARED_HOST, &entry, &entry_reader.spi ) ); spi_device_interface_config_t exit = { .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = EXIT_CS, .queue_size = 1 }; ESP_ERROR_CHECK( spi_bus_add_device( SHARED_HOST, &exit, &exit_reader.spi )
    ); rfid_setup(entry_reader.spi); rfid_setup(exit_reader.spi); printf( "ENTRY RFID VERSION: 0x%02X\n", rfid_read(entry_reader.spi, REG_VERSION) ); printf( "EXIT RFID VERSION: 0x%02X\n", rfid_read(exit_reader.spi, REG_VERSION) ); }
static void print_status(void) { printf("\n========== COCIS SYSTEM STATUS ==========\n"); show_time(); show_schedule(); printf("EMERGENCY: %s\n", emergency_active ? "ACTIVE" : "INACTIVE"); print_counts(); print_terminal_status(TERM_MAIN); print_terminal_status(TERM_ROOM1); print_terminal_status(TERM_ROOM2);
    printf("MICROSD: %s | FILE: %s\n", sd_ready ? "READY" : "NOT READY", LOG_FILE); printf("=========================================\n"); }

/* Wi-Fi connectivity, SNTP clock and serial control */
static void wifi_handler(void *arg, esp_event_base_t base, int32_t id, void *data) { (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) { esp_wifi_connect(); } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) { wifi_connected = false; esp_wifi_connect(); } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) { wifi_connected = true; printf("WIFI CONNECTED: Wokwi-GUEST\n"); } }
static void sntp_callback(struct timeval *tv) { (void)tv; ntp_received = true; printf("NTP SYNCHRONIZED: EAT (UTC+3)\n"); }
static bool wifi_init_clock(void) { esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); err = nvs_flash_init(); }
    if (err != ESP_OK) { printf("NVS INIT FAILED: %s\n", esp_err_to_name(err)); return false;
    } ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default()); esp_netif_create_default_wifi_sta(); wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT(); ESP_ERROR_CHECK(esp_wifi_init(&config)); ESP_ERROR_CHECK(esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_handler, NULL)); wifi_config_t wifi = {0}; strcpy((char *)wifi.sta.ssid, "Wokwi-GUEST"); wifi.sta.channel = 6; wifi.sta.threshold.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi)); ESP_ERROR_CHECK(esp_wifi_start()); printf("CONNECTING TO Wokwi-GUEST...\n"); return true;
}
static void clock_task(void *arg) { (void)arg; bool announced = false;
    while (1) {
        if (wifi_connected && !sntp_started) { esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org"); config.sync_cb = sntp_callback; esp_err_t result = esp_netif_sntp_init(&config);
            if (result == ESP_OK) { sntp_started = true; printf("SNTP REQUEST STARTED\n"); } else { printf("SNTP INIT ERROR: %s\n", esp_err_to_name(result)); } }
        if (clock_valid() && !announced) { show_time(); show_schedule(); announced = true; } vTaskDelay(pdMS_TO_TICKS(2000)); } }

static void execute_command(const char *command) {
    if (strcmp(command, RESET_COMMAND) == 0) { reset_emergency(); } else if (strcmp(command, "TIME") == 0) { show_time(); } else if (strcmp(command, "SCHEDULE") == 0) { show_schedule(); } else if (strcmp(command, "LOGIN 2468") == 0) { admin_session = true; printf("ADMINISTRATOR SESSION STARTED (DEMO PIN)\n");
    } else if (strncmp(command, "LOGIN ", 6) == 0) { printf("INVALID DEMONSTRATION PIN\n"); } else if (strcmp(command, "LOGOUT") == 0) { admin_session = false; printf("ADMINISTRATOR SESSION ENDED\n"); } else if (strncmp(command, "OPEN ", 5) == 0 || strncmp(command, "CLOSE ", 6) == 0) {
        if (!admin_session) { printf("LOGIN 2468 REQUIRED FOR SCHEDULE CHANGES\n"); return; } bool opening = strncmp(command, "OPEN ", 5) == 0; int value;
        if (!parse_hours(command + (opening ? 5 : 6), &value)) { printf("USE OPEN HH:MM OR CLOSE HH:MM\n"); return; }
        if (opening) open_minute = value;
        else close_minute = value;
        show_schedule();
        if (sd_ready && !schedule_save()) { printf("WARNING: SCHEDULE NOT SAVED TO MICROSD\n"); } log_event("ADMIN", "SCHEDULE", "Open %02d:%02d Close %02d:%02d", open_minute / 60, open_minute % 60, close_minute / 60, close_minute % 60); } else if (strncmp(command, "SETTIME ", 8) == 0) {
        if (!admin_session) { printf("LOGIN 2468 REQUIRED FOR MANUAL TIME SETTING\n"); return; }
        if (!set_manual_time(command + 8)) { printf("USE SETTIME YYYY-MM-DD HH:MM:SS (VALID EAT DATE/TIME)\n"); return; } printf("MANUAL SOFTWARE CLOCK SET\n"); show_time(); log_event("ADMIN", "TIME_SET", "Manual EAT clock set"); } else if (strcmp(command, "STATUS") == 0) { print_status(); } else if (strcmp(command, "TERMINALS") == 0) {
        print_terminal_status(TERM_MAIN); print_terminal_status(TERM_ROOM1); print_terminal_status(TERM_ROOM2); } else if (strcmp(command, "VIEW MAIN") == 0) { active_view = TERM_MAIN; print_terminal_status(TERM_MAIN); } else if (strcmp(command, "VIEW ROOM1") == 0) { active_view = TERM_ROOM1; print_terminal_status(TERM_ROOM1);
    } else if (strcmp(command, "VIEW ROOM2") == 0) { active_view = TERM_ROOM2; print_terminal_status(TERM_ROOM2); } else if (strcmp(command, "VIEW ALL") == 0) { active_view = TERM_ALL; printf("ALL THREE LOGICAL TERMINAL STREAMS VISIBLE\n"); } else if (strcmp(command, "SDSTATUS") == 0) { show_sd_status();
    } else if (strcmp(command, "LOGS") == 0) { show_logs(); } else if (strcmp(command, "HELP") == 0) { printf("COMMANDS: STATUS, TERMINALS, VIEW MAIN, VIEW ROOM1, " "VIEW ROOM2, VIEW ALL, TIME, SCHEDULE, SDSTATUS, LOGS, " "RESET, LOGIN 2468, LOGOUT, OPEN HH:MM, CLOSE HH:MM, " "SETTIME YYYY-MM-DD HH:MM:SS\n");
    } else if (command[0] != '\0') { printf("UNKNOWN COMMAND - TYPE HELP\n"); } }
static void serial_initialize(void) { uart_config_t config = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT }; ESP_ERROR_CHECK( uart_param_config(UART_NUM_0, &config) );
    if (!uart_is_driver_installed(UART_NUM_0)) { ESP_ERROR_CHECK( uart_driver_install( UART_NUM_0, 512, 0, 0, NULL, 0 ) ); } }
static void update_serial(void) { uint8_t ch;
    for (int i = 0; i < 32; i++) { int received = uart_read_bytes(UART_NUM_0, &ch, 1, 0);
        if (received <= 0) { break;
        }
        if (ch == '\r' || ch == '\n') {
            if (serial_length > 0) { serial_buffer[serial_length] = '\0'; execute_command(serial_buffer); serial_length = 0; } } else if (ch >= 32 && ch <= 126) {
            if ( serial_length < sizeof(serial_buffer) - 1 ) { serial_buffer[serial_length++] = (char)ch; } else { serial_length = 0; printf("COMMAND TOO LONG\n"); } } } }

/* Initialization and main FreeRTOS control loop */

void app_main(void) { setenv("TZ", "EAT-3", 1); tzset(); printf("\n"); printf("========================================\n"); printf("COCIS SMART CONTROL AND MONITORING SYSTEM\n"); printf("STAGE 23 - RESTORED SD + WIFI CLOCK + CORRECT COUNTS\n");
    printf("========================================\n"); gpio_initialize(); servo_initialize(); buzzer_initialize(); serial_initialize();
    for (int i = 0; i < DOOR_COUNT; i++) { servo_move(i, CLOSED); }
    for (int i = 0; i < 2; i++) { rooms[i].last_motion = gpio_get_level(rooms[i].pir); } previous_raw = shift_read(); stable_buttons = previous_raw; last_change = xTaskGetTickCount(); rfid_queue = xQueueCreate( 12, sizeof(rfid_event_t) ); log_queue = xQueueCreate( 32, sizeof(log_record_t) ); file_mutex = xSemaphoreCreateMutex();
    if ( rfid_queue == NULL || log_queue == NULL || file_mutex == NULL ) { printf("ERROR: FREERTOS RESOURCE FAILURE\n"); return; } bool bus_ok = spi_initialize(); bool sd_ok = false;
    if (bus_ok) { sd_ok = sd_initialize(); }
    if (sd_ok) { BaseType_t log_task_result = xTaskCreate( sd_logger_task, "sd_logger", 6144, NULL, 1, NULL );
        if (log_task_result == pdPASS) { log_event( "SYSTEM", "STARTUP", "CoCIS system initialized" ); } else { printf("ERROR: SD LOGGER TASK FAILED\n"); sd_ready = false; } rfid_initialize(); BaseType_t rfid_task_result = xTaskCreate( rfid_task, "rfid_scanner", 6144, NULL, 3, NULL );
        if (rfid_task_result != pdPASS) { printf("ERROR: RFID TASK FAILED\n"); } } else { printf("\nWARNING: MICROSD NOT INITIALIZED\n"); printf("RFID DISABLED TO AVOID SPI CONFLICT\n"); printf("ROOM CONTROL AND EMERGENCY REMAIN ACTIVE\n"); }
    if (wifi_init_clock()) {
        if (xTaskCreate(clock_task, "clock_sync", 4096, NULL, 2, NULL) != pdPASS) { printf("CLOCK TASK FAILED; USE ADMIN MANUAL TIME\n"); } } printf("\nSYSTEM READY\n"); printf("COMMANDS: HELP, TIME, SCHEDULE, STATUS, TERMINALS, VIEW ..., LOGS, RESET\n"); print_status();
    while (1) { update_buttons(); update_serial(); process_rfid_queue(); update_doors(); update_room_lighting(); update_alarm(); vTaskDelay(pdMS_TO_TICKS(10)); } }
