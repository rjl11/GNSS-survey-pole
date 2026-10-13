#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "tilt.h"
#include "tm171.h"

static const char *TAG = "survey";

// ---- Config ---------------------------------------------------------------
#define GNSS_UART       UART_NUM_1
#define GNSS_TX_PIN     17
#define GNSS_RX_PIN     18
#define GNSS_BAUD       115200

#define IMU_UART        UART_NUM_2
#define IMU_TX_PIN      15
#define IMU_RX_PIN      16
#define IMU_BAUD        115200

// Fallback magnetic declination (east positive), used until/unless the
// receiver sends one in RMC.
#define DECLINATION_DEG 4.0
#define IMU_MAX_AGE_US  100000   // reject IMU samples older than 100 ms

// Sign/offset adapter between TM171 angles and tilt.c. Verify with the axis
// test: tilt top of rod north -> dN negative (flip PITCH_SIGN if not);
// tilt top east -> dE negative (flip ROLL_SIGN); rotate rod counter-clockwise
// seen from above -> yaw should increase (else set YAW_SIGN to -1.0).
#define ROLL_SIGN   (+1.0)
#define PITCH_SIGN  (+1.0)
#define YAW_SIGN    (+1.0)
#define YAW_OFFSET  (0.0)
// ---------------------------------------------------------------------------

typedef struct {
    double roll, pitch, yaw;     // degrees
    int64_t t_us;                // esp_timer time of reception
    bool valid;
} imu_sample_t;

static imu_sample_t s_imu;
static SemaphoreHandle_t s_imu_mtx;

// ---- IMU ------------------------------------------------------------------
static void imu_task(void *arg)
{
    uint8_t buf[128];
    tm_parser_t parser = {0};
    for (;;) {
        int n = uart_read_bytes(IMU_UART, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            float r, p, y; uint32_t ts; uint16_t id;
            if (tm_feed(&parser, buf[i], &r, &p, &y, &ts, &id)) {
                imu_sample_t s = {
                    .roll  = ROLL_SIGN  * r,
                    .pitch = PITCH_SIGN * p,
                    .yaw   = YAW_SIGN   * y + YAW_OFFSET,
                    .t_us  = esp_timer_get_time(),   // sensor timestamp wraps every 1.19 h
                    .valid = true,
                };
                xSemaphoreTake(s_imu_mtx, portMAX_DELAY);
                s_imu = s;
                xSemaphoreGive(s_imu_mtx);
            }
        }
    }
}

// ---- NMEA -----------------------------------------------------------------
static bool nmea_checksum_ok(const char *line)
{
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t cs = 0;
    for (const char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;
    return cs == (uint8_t)strtol(star + 1, NULL, 16);
}

// ddmm.mmmm / dddmm.mmmm + hemisphere -> decimal degrees
static double nmea_to_deg(const char *s, char hemi)
{
    double v = atof(s);
    int deg = (int)(v / 100.0);
    double d = deg + (v - deg * 100.0) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -d : d;
}

// Strips "*CS" and splits on commas in place. Returns number of fields.
static int nmea_split(char *line, char **f, int max)
{
    char *star = strchr(line, '*');
    if (star) *star = '\0';
    int n = 0;
    char *p = line, *tok;
    while (n < max && (tok = strsep(&p, ",")) != NULL) f[n++] = tok;
    return n;
}

typedef struct {
    double lat, lon;
    double alt_msl, geoid_sep;
    int quality, sats;
} gga_t;

// f[0] = "$xxGGA". Returns false if no fix.
static bool parse_gga(char **f, int n, gga_t *g)
{
    if (n < 13) return false;
    g->quality = atoi(f[6]);
    if (g->quality == 0 || !f[2][0] || !f[4][0]) return false;
    g->lat       = nmea_to_deg(f[2], f[3][0]);
    g->lon       = nmea_to_deg(f[4], f[5][0]);
    g->sats      = atoi(f[7]);
    g->alt_msl   = atof(f[9]);
    g->geoid_sep = atof(f[11]);
    return true;
}

// $xxRMC,time,status,lat,N,lon,E,speed,course,date,magvar,E/W,mode
// Returns true if a magnetic variation was present. *decl_deg is east-positive.
static bool parse_rmc(char **f, int n, double *decl_deg)
{
    if (n < 12 || !f[10][0]) return false;
    char hemi = f[11][0];
    if (hemi != 'E' && hemi != 'W') return false;
    double v = atof(f[10]);
    *decl_deg = (hemi == 'W') ? -v : v;
    return true;
}

// ---- GNSS task --------------------------------------------------------------
static void gnss_task(void *arg)
{
    char line[128];
    size_t len = 0;
    uint8_t c;

    double decl_deg     = DECLINATION_DEG;
    bool   decl_from_rx = false;

    for (;;) {
        if (uart_read_bytes(GNSS_UART, &c, 1, pdMS_TO_TICKS(100)) != 1) continue;
        int64_t t_rx = esp_timer_get_time();

        if (c == '$') len = 0;                       // resync on start of sentence
        if (len < sizeof(line) - 1) line[len++] = (char)c;
        if (c != '\n') continue;
        line[len] = '\0';
        len = 0;

        if (!nmea_checksum_ok(line)) continue;

        char *f[16];
        int n = nmea_split(line, f, 16);
        if (n < 1 || strlen(f[0]) < 6) continue;
        const char *type = f[0] + 3;                 // skips "$GP"/"$GN"/"$GL"...

        if (strcmp(type, "RMC") == 0) {
            double d;
            if (parse_rmc(f, n, &d)) {
                if (!decl_from_rx)
                    ESP_LOGI(TAG, "Using receiver magnetic variation: %.2f deg", d);
                decl_deg = d;
                decl_from_rx = true;
            }
            continue;
        }

        gga_t g;
        if (strcmp(type, "GGA") != 0 || !parse_gga(f, n, &g)) continue;

        imu_sample_t imu;
        xSemaphoreTake(s_imu_mtx, portMAX_DELAY);
        imu = s_imu;
        xSemaphoreGive(s_imu_mtx);

        if (!imu.valid || (t_rx - imu.t_us) > IMU_MAX_AGE_US) {
            ESP_LOGW(TAG, "No fresh IMU sample, skipping fix");
            continue;
        }

        double alt_ell = g.alt_msl + g.geoid_sep;    // ellipsoidal height
        tip_position_t tip = tilt_compensate(g.lat, g.lon, alt_ell,
                                             imu.roll, imu.pitch, imu.yaw,
                                             decl_deg, ROD_LENGTH_M);

        printf("fix=%d sats=%d tip=%.9f,%.9f h_ell=%.3f h_msl=%.3f "
               "dN=%.3f dE=%.3f decl=%.2f%s\n",
               g.quality, g.sats, tip.lat, tip.lon, tip.alt,
               tip.alt - g.geoid_sep, tip.dN, tip.dE,
               decl_deg, decl_from_rx ? "(rx)" : "(fixed)");
    }
}

// ---- Setup ----------------------------------------------------------------
static void uart_init_port(uart_port_t port, int baud, int tx, int rx)
{
    uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void)
{
    s_imu_mtx = xSemaphoreCreateMutex();

    uart_init_port(GNSS_UART, GNSS_BAUD, GNSS_TX_PIN, GNSS_RX_PIN);
    uart_init_port(IMU_UART,  IMU_BAUD,  IMU_TX_PIN,  IMU_RX_PIN);

    xTaskCreate(imu_task,  "imu",  4096, NULL, 6, NULL);
    xTaskCreate(gnss_task, "gnss", 4096, NULL, 5, NULL);
}
