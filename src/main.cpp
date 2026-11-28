/**
 * SPDX-License-Identifier: CC-BY-NC-SA-4.0
 * Copyright (c) 2026 FigVam (figvam2050@gmail.com)
 *
 * Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International.
 * Commercial use, resale of hardware pre-flashed with this software, or paid services
 * are strictly prohibited without prior written permission of the author.
 *
 * Hardware disclaimer: Third-party commercial PCB/hardware (Felicity Solar / OEM).
 * The author is not affiliated with the hardware manufacturer. Aftermarket firmware.
 *
 * BMS 485 to CAN Converter for Deye Inverter (Deye PCS CAN V3.3 / DY001,
 * Lithium Mode 00). Hardware: GD32F305 + ISO1050 (CAN) + CA-IS3080WX (RS485)
 *
 * COMBOX_HUAWEI: адаптація для промислових батарей Huawei ESM
 * (ESM-48100 / ESM-48150, Modbus RTU RS485: адреси 0xD6–0xD7).
 * База знань та реєстри — HUAWEI_REFERENCE.md (§5, §6, §7).
 * Вихідний CAN-бік (Deye PCS CAN V3.3 / DY001), fail-closed, балансування,
 * профілі DIP та PCS-485 (діагностичне дзеркало) активні.
 *
 * Frame set and layout follow "PCS CAN-Bus-protocol-DY-low-voltage V3.3" and
 * "PYLON low voltage Protocol CAN Bus v2.0.6": standard frames, 500 kbit/s,
 * cycle 1 s, BMS transmits 0x351/0x355/0x356/0x359/0x35C/0x35E.
 * IDs 0x300..0x30F are reserved for the PCS downlink (0x305 is the inverter
 * heartbeat) and must never be transmitted by the battery side.
 *
 * CAN bus speed: 500 kbit/s, measured manually on real hardware (BTR = 0x030A0000).
 * The 250 kbit/s value was never confirmed and is not a valid option.
 */

#include "gd32f30x.h"
#include <stdbool.h>
#include <string.h>

#define BMS_BAUDRATE 9600
// Таймаут відповіді: 400 мс (за нормальної відповіді 114 мс це 3.5-кратний запас)
#define BMS_TIMEOUT_MS 400
#define BMS_DEFAULT_TIMEOUT_MS 5000

// --- Huawei ESM: опитування строго 2 акумуляторів (0xD6 = 214, 0xD7 = 215) ---
#define HW_MODULE_COUNT 2
static const uint8_t HW_ADDR[HW_MODULE_COUNT] = {0xD6, 0xD7};

#define HW_INTER_QUERY_MS 30   // пауза між регістрами одного модуля (30 мс)
#define HW_INTER_MODULE_MS 80  // пауза між модулями D6 -> D7 (80 мс)

typedef struct {
  uint16_t reg;
  uint8_t qty;
} HwQuery;

enum HwQueryId : uint8_t {
  HW_Q_BASE = 0, // 0x0000 ×7: bus V, pack V, струм, SOC, SOH, t max, t min
  HW_Q_ALARM,    // 0x0042 ×9: cycles (0x0042..43), ah (0x0044..45), alarm[0..4] (0x0046..4A)
  HW_Q_CELL,     // 0x0022 ×15: напруги 15 комірок, U16 мВ BE
  HW_Q_FAST_COUNT,
  HW_Q_CELL_TEMP = HW_Q_FAST_COUNT, // 0x0012 ×15: температури 15 комірок
  HW_Q_SYS_INFO, // 0x0101 ×15: версії ПЗ/апаратури, кількість комірок (0x010F)
  HW_Q_MODEL,    // 0x0332 ×12: модель акумулятора (ESM-48100U2)
  HW_Q_SERIAL,   // 0x0340 ×10: серійний номер (20 байтів ASCII)
  HW_Q_COUNT
};

static const HwQuery HW_QUERIES[HW_Q_COUNT] = {
    {0x0000, 7}, {0x0042, 9}, {0x0022, 15},
    {0x0012, 15}, {0x0101, 15}, {0x0332, 12}, {0x0340, 10},
};



static uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// No-data presentation (Q5, рішення 13.10.2026): «порожня інертна батарея».
// SOC=0 — не запрошує інвертор до розряду; T=25 — нейтральна (45 °C могла
// тригерити derating/тривоги); SOH=100 — без фейкового «старіння»; V=52.2
// залишається номіналом (V=0 лякає інвертори; при лімітах 0 А не впливає).
// Разом із CCL/DCL=0 і 0x35C=0x00 (без прапорців) — чесний стан «нема даних».
#define BATTERY_SOC_DEFAULT 0
#define BATTERY_SOH_DEFAULT 100
#define BATTERY_VOLTAGE_DEFAULT 52.2f
#define BATTERY_CURRENT_DEFAULT 0.0f
#define BATTERY_TEMP_DEFAULT 25.0f
#define CHARGE_CURRENT_LIMIT_DEFAULT 0
#define DISCHARGE_CURRENT_LIMIT_DEFAULT 0

#define CHARGE_VOLTAGE_LIMIT_STATIC 535 // 53.5 V (3.567 V/cell * 15): Huawei ESM Equalized charge voltage
#define CHARGE_VOLTAGE_LIMIT_DIP90 518
// Per-pack charge current limit, selected by DIP4 (see DIP_CHARGE_CURRENT_15A):
// OFF = 50 A (battery manufacturer maximum), ON = 15 A (soft charge limit).
#define CHARGE_CURRENT_LIMIT_PER_PACK_DIP4_ON_X10 150
#define CHARGE_CURRENT_LIMIT_PER_PACK_X10 500
#define DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 1000
#define DISCHARGE_VOLTAGE_LIMIT_X10 450

// Max-cell charge taper: 50 A/pack to 0 A between 3.450 V and active stop threshold.
// Re-enabling below 3.380 V (Standard) prevents charge-limit oscillation at the cutoff.
#define CELL_TAPER_START_MV 3450
#define CELL_CHARGE_STOP_MV 3560
#define CELL_CHARGE_REENABLE_MV 3380
#define CELL_TAPER_START_MV_DIP90 3400
#define CELL_CHARGE_STOP_MV_DIP90 3455
#define CELL_CHARGE_REENABLE_MV_DIP90 3350
// 90% profile = SW1 ON (weight 1 => code 0b00001), moved from SW4 on 17.10.2026
// (convenient switch); stock SW1 protocol meaning (PYLON) is not used by us.
#define DIP_PROFILE_CODE_90 1
// DIP4 (SW4, weight 8, PB15) selects the per-pack charge current limit:
// OFF = 50 A, ON = 15 A. Orthogonal to the SW1 voltage profile.
// DIP5 (SW5, weight 16, PC6) selects 54.0 V equalization/balancing mode (priority over DIP1).
#define DIP_BALANCE_54V 0x10
#define CHARGE_VOLTAGE_LIMIT_54V 540
#define DIP_CHARGE_CURRENT_15A 0x08

// Physical cell over-voltage threshold for BMS hardware protection confirmation (Q17).
// Independent of DIP90 taper target so that software 90% profile never generates false alarms:
#define CELL_PHYSICAL_OV_CONFIRM_MV 3600
#define CELL_PHYSICAL_OV_WARN_CONFIRM_MV 3550

// LiFePO4 cell protection thresholds (Warn 2800 mV, Protect 2500 mV, Release 3000 mV):
#define CELL_UV_WARN_MV 2800
#define CELL_DISCHARGE_STOP_MV 2650
#define CELL_DISCHARGE_REENABLE_MV 3000
#define CELL_MAX_DELTA_MV 400

// Temperature thresholds: LiFePO4 charging blocked < +2°C (lithium plating) and > 55°C.
// Charging re-enables with hysteresis (>= +3°C and <= 50°C) to prevent chattering.
#define TEMP_CHARGE_MIN_C 2.0f
#define TEMP_CHARGE_MIN_REENABLE_C 3.0f
#define TEMP_CHARGE_MAX_C 55.0f
#define TEMP_CHARGE_MAX_REENABLE_C 50.0f
#define TEMP_DISCHARGE_MIN_C -20.0f
#define TEMP_DISCHARGE_MIN_REENABLE_C -18.0f
#define TEMP_DISCHARGE_MAX_C 60.0f
#define TEMP_DISCHARGE_MAX_REENABLE_C 58.0f

// ===== Balance session engine =====
// DIP5 ON: forced 54.0 V equalization (overrides DIP1 and auto sessions).
// DIP1 OFF (standard): CVL 53.5 V; auto balance windows (BAL_AUTO_ENABLED)
// open at 54.0 V on cell-spread analysis: full thresholds + 2 A per pack cap.
// DIP1 ON (51.8 V option): auto logic disabled, profile holds 51.8 V with
// the same 2 A trickle from 3400 mV (~90%) so passive balancing has gentle
// CV time.
#define BAL_AUTO_ENABLED 1
#define BAL_SPREAD_HIGH_MV 100     // anomaly bucket (>= 100 mV): retry in 24 h
#define BAL_SPREAD_MED_MV 30       // regular spread bucket (30..100 mV): retry in 72 h
#define BAL_SPREAD_EXIT_MV 25      // session success target (spread <= 25 mV)
#define BAL_TREND_DELTA_MV_12H 5   // 5 mV over 12 h == 10 mV/24 h -> HIGH
#define BAL_INTERVAL_HIGH_MS (24UL * 3600UL * 1000UL)
#define BAL_INTERVAL_MED_MS (72UL * 3600UL * 1000UL)
#define BAL_INTERVAL_OK_MS (30UL * 24UL * 3600UL * 1000UL)
#define BAL_COOLDOWN_TRY_MS (12UL * 3600UL * 1000UL)
#define BAL_COOLDOWN_OUTAGE_MS (1UL * 3600UL * 1000UL)
#define BAL_MIN_SOC_START 85
#define BAL_TRICKLE_CELL_MV 3400   // ~90% -> 2 A per pack phase
#define BAL_TRICKLE_A_X10 20       // 2.0 A per pack cap
#define BAL_NO_PROGRESS_MS (90UL * 60UL * 1000UL)
#define BAL_SESSION_MAX_MS (8UL * 3600UL * 1000UL)
#define BAL_START_I_MAX_A 1.0f     // start only when not discharging
#define BAL_OUTAGE_I_MIN_A 1.5f    // sustained discharge -> abort session
#define BAL_OUTAGE_AGG_N 60        // ~60 s при швидкості циклу ~1.0 с на агрегат
#define BAL_EXIT_STREAK 5          // 5 поспіль циклів зі спредом <= 25 мВ (~5 с) для фіксації успіху
#define BAL_TEMP_MIN_C 0.0f
#define BAL_TEMP_MAX_C 45.0f
#define BAL_HIST_HOURS 24          // hourly spread history for trend

// 0x359 Pylontech v2.0.6 Protection & Alarm bit definitions
#define CAN_PROT_CELL_OV       0x01
#define CAN_PROT_PACK_OV       0x02
#define CAN_PROT_CELL_UV       0x04
#define CAN_PROT_OT            0x08
#define CAN_PROT_UT            0x10
#define CAN_PROT_DISCHARGE_OC  0x80

#define CAN_PROT_CHARGE_OC     0x01
#define CAN_PROT_SYSTEM_ERROR  0x08

// Huawei ESM register units -> physical units.
// 0x0002 current: 1 = BMS reports charge as positive (negated for Deye CAN),
//                 0 = BMS reports discharge as positive (passed through).
#define BMS_CURRENT_CHARGE_IS_POSITIVE 1
// 0x0005/0x0006 temperature raw divisor: 1 = whole °C.
#define BMS_TEMP_RAW_DIV 1.0f

#define CAN_PROTOCOL_DEYE_V33 1

#define CAN_ID_LIMITS 0x351
#define CAN_ID_SOC 0x355
#define CAN_ID_STATUS 0x356
#define CAN_ID_PROTECT 0x359
#define CAN_ID_FLAGS 0x35C
#define CAN_ID_BRAND 0x35E
#define CAN_ID_CELL_EXTREMES 0x361
#define CAN_ID_DEYE_CFG 0x363
#define CAN_ID_DEYE_PACK_COUNT 0x364
#define CAN_ID_GRID_LIMITS 0x371
#define CAN_ID_DEYE_PACK_STATUS_BASE 0x110
#define CAN_ID_DEYE_PACK_VI_BASE 0x150
#define CAN_ID_DEYE_PACK_CELL_BASE 0x200
#define CAN_ID_DEYE_PACK_CYCLES_BASE 0x400
#define CAN_ID_DEYE_PACK_FW_BASE 0x500
#define CAN_ID_DEYE_PACK_MODEL_BASE 0x600
#define CAN_ID_DEYE_PACK_SER_BASE 0x650
#define CAN_ID_INV_HEARTBEAT 0x305U
#define INVERTER_CAN_TIMEOUT_MS 10000U

#define UART_BMS USART0
#define UART_BMS_RCU RCU_USART0
#define UART_BMS_GPIO_PORT GPIOA
#define UART_BMS_GPIO_RCU RCU_GPIOA
#define UART_BMS_TX_PIN GPIO_PIN_9
#define UART_BMS_RX_PIN GPIO_PIN_10

// PCS-485 port (UART4, PC12/PD2 — PORT_MAPPING.md §3/§5; no UART5 on GD32F30x)
#define PCS_UART UART4
#define PCS_UART_RCU RCU_UART4
#define PCS_TX_PORT GPIOC
#define PCS_TX_RCU RCU_GPIOC
#define PCS_TX_PIN GPIO_PIN_12
#define PCS_RX_PORT GPIOD
#define PCS_RX_RCU RCU_GPIOD
#define PCS_RX_PIN GPIO_PIN_2

#define DEYE_CAN CAN0
#define DEYE_CAN_RCU RCU_CAN0
#define DEYE_CAN_GPIO_PORT GPIOB
#define DEYE_CAN_GPIO_RCU RCU_GPIOB
#define DEYE_CAN_RX_PIN GPIO_PIN_8
#define DEYE_CAN_TX_PIN GPIO_PIN_9

// LED assignment per PORT_MAPPING.md §4 (factory panel labels, 17.10.2026):
//   LED5 (PB0)  = system running (program alive) — steady ON after init;
//   LED6 (PB1)  = BMS RS485 exchange — blinks on TX/RX with battery;
//   LED4 (PC5)  = CAN to inverter — blinks on TX bursts;
//   LED3 (PC4)  = PCS-485 port (UART4, Q23): blinks on TX/RX — mirror mode
//                 (DIP2 OFF) or inverter bridge traffic (DIP2 ON);
//   LED1 (PC13) = balance/profile status (see balance_led_tick()).
//   LED2 (VDDA) = power — wired to rail, not MCU-controlled.
// PC5 is NOT an RS485 direction pin (PC5=DE disproven, RS485 has
// auto-direction transceivers — ORIGINAL_FIRMWARE.md §14.3).
#define LED_RUNNING_PORT GPIOB
#define LED_RUNNING_PIN GPIO_PIN_0
#define LED_BMS485_PORT GPIOB
#define LED_BMS485_PIN GPIO_PIN_1
#define LED_PCSCAN_PORT GPIOC
#define LED_PCSCAN_PIN GPIO_PIN_5
#define LED_INV485_PORT GPIOC
#define LED_INV485_PIN GPIO_PIN_4
#define LED1_DEBUG_PORT GPIOC
#define LED1_DEBUG_PIN GPIO_PIN_13

// Stock boot state for PB10 (PP HIGH) and PB11 (OD HIGH) — likely transceiver
// enables (ORIGINAL_FIRMWARE.md §14.4); released HIGH like the factory firmware.
#define TRANSCEIVER_ENA_PORT GPIOB
#define TRANSCEIVER_ENA_PIN GPIO_PIN_10
#define TRANSCEIVER_ENB_PORT GPIOB
#define TRANSCEIVER_ENB_PIN GPIO_PIN_11

#define DIP_GPIO_PORT GPIOB
#define DIP_GPIO_RCU RCU_GPIOB
#define DIP_GPIO_PINS (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define DIP5_GPIO_PORT GPIOC
#define DIP5_GPIO_RCU RCU_GPIOC
#define DIP5_GPIO_PIN GPIO_PIN_6

static volatile uint32_t g_systick_ms = 0;
static volatile uint8_t uart_rx_buffer[256];
static volatile uint16_t uart_rx_index = 0;

static float bms_voltage = BATTERY_VOLTAGE_DEFAULT;
static float bms_current = BATTERY_CURRENT_DEFAULT;
static uint8_t bms_soc = BATTERY_SOC_DEFAULT;
static float bms_temp = BATTERY_TEMP_DEFAULT;
static uint8_t bms_soh = BATTERY_SOH_DEFAULT;
// Packs seen in the last full cycle; kept (not cleared) on comm loss so
// 0x359 byte 4 stays the fixed system size the inverter expects.
static uint8_t bms_pack_count = 0;
static uint16_t bank_max_cell_mv = 0;
static uint16_t bank_min_cell_mv = 0;
static float bank_temp_max = BATTERY_TEMP_DEFAULT;
static float bank_temp_min = BATTERY_TEMP_DEFAULT;

static float agg_voltage_sum = 0.0f;
static float agg_current_sum = 0.0f;
static uint32_t agg_soc_sum = 0;
static float agg_temp_max = -1000.0f;
static float agg_temp_min = 1000.0f;
static uint8_t agg_count = 0;
static uint16_t agg_max_cell_mv = 0;
static uint16_t agg_min_cell_mv = 0xFFFF;
static bool charge_cell_ov_inhibit = false;
static bool charge_temp_inhibit = false;
static bool discharge_cell_uv_inhibit = false;
static bool discharge_temp_inhibit = false;

// 0x359 CAN protection and alarm flags (active state and sweep accumulators)
static uint8_t can_protect_b0 = 0;
static uint8_t can_protect_b1 = 0;
static uint8_t can_alarm_b2 = 0;
static uint8_t can_alarm_b3 = 0;
static uint8_t sweep_protect_b0 = 0;
static uint8_t sweep_protect_b1 = 0;
static uint8_t sweep_alarm_b2 = 0;
static uint8_t sweep_alarm_b3 = 0;

// 0x359 Deye PCS CAN V3.3 Tables 1-7 active bytes and sweep accumulators
static uint8_t can_dy_p0 = 0, can_dy_p1 = 0, can_dy_p2 = 0, can_dy_p3 = 0;
static uint8_t can_dy_a4 = 0, can_dy_a5 = 0, can_dy_e6 = 0;
static uint8_t sweep_dy_p0 = 0, sweep_dy_p1 = 0, sweep_dy_p2 = 0, sweep_dy_p3 = 0;
static uint8_t sweep_dy_a4 = 0, sweep_dy_a5 = 0, sweep_dy_e6 = 0;

static uint32_t last_inverter_can_rx_ms = 0;
static bool inverter_can_alive = false;

// Multi-pack debounce (Q21): tolerate 10-20% RS485 packet loss without flapping limits
static uint8_t pack_miss_count[HW_MODULE_COUNT] = {0};
static uint16_t pack_ever_seen_mask = 0x0003; // D6 (214) та D7 (215) відомі одразу від старту
static uint16_t sweep_seen_mask = 0;

// --- Huawei ESM: буфер даних модулів (індекс 0..1 ↔ HW_ADDR) --------------
// У агрегат ідуть лише модулі, чия відповідь потрапила у sweep: базові суми
// дедуплікуються hw_base_seen_mask (клітинки/температури/аварії — ідемпотентні).
typedef struct {
  float bus_v;           // 0x0000 ×0.01 В
  float pack_v;          // 0x0001 ×0.01 В (клеми модуля)
  float current;         // 0x0002 S16 ×0.01 А, +заряд / −розряд (§5.1)
  uint8_t soc;           // 0x0003 %
  uint8_t soh;           // 0x0004 %
  int16_t t_max;         // 0x0005 °C
  int16_t t_min;         // 0x0006 °C
  uint32_t cycles;       // 0x0042–0x0043 U32 BE
  uint32_t ah;           // 0x0044–0x0045 U32 BE
  uint16_t alarm[5];     // 0x0046..0x004A (бітові мапи — §5.4)
  uint8_t ncell;         // 0x010F
  uint16_t cell_mv[15];  // 0x0022..0x0030, мВ
  int16_t cell_temp[15]; // 0x0012..0x0020, °C
  uint16_t cell_max;     // перерахунок на HW_Q_CELL (для мапінгу аварій)
  uint16_t cell_min;
  uint32_t last_ok_ms;   // остання валідна відповідь (будь-який запит)
} HwModule;

static HwModule hw_mod[HW_MODULE_COUNT];
static uint16_t hw_base_seen_mask = 0;  // базові суми цього sweep уже погоджені
static uint16_t hw_alarm_seen_mask = 0; // свіжі аварійні слова для мапінгу 0x359
static uint8_t agg_soh_min = 100;       // §6: SOH агрегату = мінімум по модулях

static uint8_t hw_addr_idx = 0;
static uint8_t hw_q_idx = 0;
static uint32_t hw_next_slot_ms = 0;
static uint32_t hw_uart_ready_ms = 0;

static uint16_t charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
static uint16_t charge_current_limit = CHARGE_CURRENT_LIMIT_DEFAULT;
static uint16_t discharge_current_limit = DISCHARGE_CURRENT_LIMIT_DEFAULT;

static uint16_t profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
static uint16_t profile_taper_start_mv = CELL_TAPER_START_MV;
static uint16_t profile_charge_stop_mv = CELL_CHARGE_STOP_MV;
static uint16_t profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
static uint16_t profile_charge_current_per_pack_x10 = CHARGE_CURRENT_LIMIT_PER_PACK_X10;

// Active thresholds seen by calculate_charge_limit(): a running balance
// session swaps the 90% profile taper for the full-charge triple, otherwise
// they mirror the current DIP profile.
static uint16_t calc_taper_start_mv = CELL_TAPER_START_MV;
static uint16_t calc_charge_stop_mv = CELL_CHARGE_STOP_MV;
static uint16_t calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;

// Balance session engine state
static bool bal_session_active = false;
static bool bal_trickle = false;
static bool bal_manual_prev = false;
static uint32_t bal_last_success_ms = 0;
static uint32_t bal_cooldown_until_ms = 0;
static uint32_t bal_session_start_ms = 0;
static uint32_t bal_last_improve_ms = 0;
static uint16_t bal_best_spread_mv = 0xFFFF;
static uint8_t bal_exit_streak = 0;
static uint8_t bal_outage_agg = 0;
static uint8_t bal_prev_packs = 0;
static uint16_t bal_hist_mv[BAL_HIST_HOURS];
static uint8_t bal_hist_n = 0;
static uint32_t bal_last_hist_ms = 0;

static uint8_t dip_applied_code = 0xFF;
static uint8_t dip_last_code = 0;
static uint8_t dip_stable_reads = 0;

static volatile uint32_t last_bms_response_ms = 0;
// True while BMS answers are fresh (< BMS_DEFAULT_TIMEOUT_MS): drives the
// 0x35C request flags (Q5). False at boot and on comm loss.
static bool bms_data_valid = false;
static uint32_t can_led_off_ms = 0;

extern "C" void osSystickHandler(void) { g_systick_ms++; }
static uint32_t millis(void) { return g_systick_ms; }

static void leds_init(void) {
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  // LED5 (PB0): system running — steady ON (program alive).
  gpio_init(LED_RUNNING_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_RUNNING_PIN);
  gpio_bit_set(LED_RUNNING_PORT, LED_RUNNING_PIN);
  // LED6 (PB1): BMS RS485 exchange — blinks on TX/RX (set at poll TX,
  // cleared when the response window closes).
  gpio_init(LED_BMS485_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_BMS485_PIN);
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN);
  // LED4 (PC5): CAN to inverter — blinks on TX bursts.
  gpio_init(LED_PCSCAN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_PCSCAN_PIN);
  gpio_bit_reset(LED_PCSCAN_PORT, LED_PCSCAN_PIN);
  // LED3 (PC4): PCS-485 (UART4) activity — mirror/bridge, see pcs_poll_service().
  gpio_init(LED_INV485_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED_INV485_PIN);
  gpio_bit_reset(LED_INV485_PORT, LED_INV485_PIN);
  // LED1 (PC13): free/debug — dark.
  gpio_init(LED1_DEBUG_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            LED1_DEBUG_PIN);
  gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
  // Transceiver enables: match factory boot state.
  gpio_init(TRANSCEIVER_ENA_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ,
            TRANSCEIVER_ENA_PIN);
  gpio_bit_set(TRANSCEIVER_ENA_PORT, TRANSCEIVER_ENA_PIN);
  gpio_init(TRANSCEIVER_ENB_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_2MHZ,
            TRANSCEIVER_ENB_PIN);
  gpio_bit_set(TRANSCEIVER_ENB_PORT, TRANSCEIVER_ENB_PIN);
}

static void rs485_init(void) {
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(UART_BMS_GPIO_RCU);
  rcu_periph_clock_enable(UART_BMS_RCU);
  gpio_init(UART_BMS_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            UART_BMS_TX_PIN);
  gpio_init(UART_BMS_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
            UART_BMS_RX_PIN);
  usart_deinit(UART_BMS);
  usart_baudrate_set(UART_BMS, BMS_BAUDRATE);
  usart_parity_config(UART_BMS, USART_PM_NONE);
  usart_word_length_set(UART_BMS, USART_WL_8BIT);
  usart_stop_bit_set(UART_BMS, USART_STB_1BIT);
  usart_transmit_config(UART_BMS, USART_TRANSMIT_ENABLE);
  usart_receive_config(UART_BMS, USART_RECEIVE_ENABLE);
  usart_enable(UART_BMS);
}

static void rs485_send_byte(uint8_t data) {
  usart_data_transmit(USART0, data);
  while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET)
    ;
}

static void rs485_clear_errors(void) {
  usart_flag_clear(USART0, USART_FLAG_ORERR);
  usart_flag_clear(USART0, USART_FLAG_NERR);
  usart_flag_clear(USART0, USART_FLAG_FERR);
  usart_flag_clear(USART0, USART_FLAG_PERR);
}

static void pcs_mirror_push(uint8_t b); // defined with the PCS-485 block below

static void rs485_flush_rx(void) {
  while (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET)
    pcs_mirror_push((uint8_t)usart_data_receive(USART0)); // залишкові байти обірваного кадру
  rs485_clear_errors();
}

// ===== PCS-485 port (UART4, Q23) — two modes selected by DIP2 (18.10.2026) =====
// DIP2 OFF (default): MIRROR — the box works as always (own polls, CAN) and a
//   byte-level copy of its BMS exchange (both directions) shifts out of UART4,
//   so a USB-UART adapter on the PCS-485 jack hears everything the box says to
//   the battery and everything the battery answers, without tapping the cable.
// DIP2 ON: PURE BRIDGE — the box sends NO commands of its own to the battery;
//   inverter requests arriving on UART4 are forwarded to the BMS line and the
//   answer is relayed straight back. The same traffic is pushed through
//   bms_parse_response() so CAN keeps broadcasting (listen/observe) and the
//   balance engine keeps its aggregates. Safeguards: only fixed 8-byte
//   FC03/FC04 read requests with a good CRC are forwarded (writes and noise
//   are dropped, the inverter just times out).
//   PRODUCTION KEEPS DIP2 OFF — 13.10 analysis: the inverter would read raw
//   packs over RS485, bypassing our CCL stop-tap aggregation; this mode is
//   research/diagnostics (DEYE_CAN_RS485_BASE.md §Q23, TODO §5.1).
#define DIP2_BRIDGE_MODE 0x02
#define PCS_GAP_MS 5           // ~3.5 idle chars at 9600 (3.65 ms): Modbus frame gap
#define PCS_RESP_FIRST_MS 300  // BMS must start answering within this
#define PCS_RESP_MAX_MS 1000   // whole bridged response hard cap
#define PCS_RESP_STALL_MS 200  // give up on a 1..3 byte fragment after this
#define PCS_NOISE_MIN_LEN 4    // shorter bursts (floating bus) are noise
#define BR_SWEEP_MAX_MS 15000UL // inverter loop ≈13 s for 16 addrs @ ~400 ms

static void watchdog_feed(void);
static void send_all_frames(void);
static uint32_t last_can = 0;
static void bms_aggregate_data(void);
static void agg_reset(void);
static void hw_send_query(uint8_t addr_idx, uint8_t q);
static uint8_t hw_parse_response(uint8_t addr_idx, uint8_t q);
static void hw_poll_service(uint32_t now);

static void pcs_uart_init(void) {
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(PCS_TX_RCU);
  rcu_periph_clock_enable(PCS_UART_RCU);
  gpio_init(PCS_TX_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PCS_TX_PIN);
  gpio_init(PCS_RX_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, PCS_RX_PIN);
  usart_deinit(PCS_UART);
  usart_baudrate_set(PCS_UART, BMS_BAUDRATE);
  usart_parity_config(PCS_UART, USART_PM_NONE);
  usart_word_length_set(PCS_UART, USART_WL_8BIT);
  usart_stop_bit_set(PCS_UART, USART_STB_1BIT);
  usart_transmit_config(PCS_UART, USART_TRANSMIT_ENABLE);
  usart_receive_config(PCS_UART, USART_RECEIVE_ENABLE);
  usart_enable(PCS_UART);
}

// UART4 error flags, same treatment as rs485_clear_errors() gives USART0:
// without this an overrun taken while the main loop is blocked (CAN burst)
// could stick on the PCS port and wedge its receive path.
static void pcs_clear_errors(void) {
  usart_flag_clear(PCS_UART, USART_FLAG_ORERR);
  usart_flag_clear(PCS_UART, USART_FLAG_NERR);
  usart_flag_clear(PCS_UART, USART_FLAG_FERR);
  usart_flag_clear(PCS_UART, USART_FLAG_PERR);
}

static uint32_t pcs_led_off_ms = 0;
static void pcs_led_activity(void) {
  gpio_bit_set(LED_INV485_PORT, LED_INV485_PIN);
  pcs_led_off_ms = millis() + 100;
}

// --- mirror ring buffer (DIP2 OFF); single-threaded polling, no volatile ---
static uint8_t pcs_mirror_q[256];
static uint16_t pcs_mirror_head = 0;
static uint16_t pcs_mirror_tail = 0;
static bool pcs_bridge_mode = false;

static void pcs_mirror_push(uint8_t b) {
  if (pcs_bridge_mode)
    return; // PCS port belongs to the inverter in bridge mode
  uint16_t next = (uint16_t)((pcs_mirror_head + 1) & 0xFF);
  if (next == pcs_mirror_tail)
    return; // listener too slow — drop, never block the BMS path
  pcs_mirror_q[pcs_mirror_head] = b;
  pcs_mirror_head = next;
  pcs_led_activity();
}

static void pcs_mirror_pump(void) {
  if (pcs_mirror_head != pcs_mirror_tail &&
      usart_flag_get(PCS_UART, USART_FLAG_TBE) != RESET) {
    usart_data_transmit(PCS_UART, pcs_mirror_q[pcs_mirror_tail]);
    pcs_mirror_tail = (uint16_t)((pcs_mirror_tail + 1) & 0xFF);
    pcs_led_activity(); // LED3 follows the line until the queue drains
  }
}

// --- bridge state (DIP2 ON) ---
enum { BR_IDLE = 0, BR_SEND, BR_WAIT_RESP, BR_FWD_RESP };
static uint8_t br_state = BR_IDLE;
static uint8_t inv_frame[256]; // one complete inverter request
static uint16_t inv_frame_n = 0;
static uint8_t inv_rx[256]; // request bytes still collecting
static uint16_t inv_rx_n = 0;
static uint32_t inv_last_rx_ms = 0;
static uint32_t br_sent_ms = 0;
static uint32_t br_first_ms = 0;
static uint32_t br_last_ms = 0;
static bool br_got = false;

// Modbus RTU request sanity: fixed 8-byte read (FC03/FC04), good CRC.
static bool pcs_request_ok(const uint8_t *f) {
  if (f[1] != 0x03 && f[1] != 0x04)
    return false;
  uint16_t crc = modbus_crc16(f, 6);
  return f[6] == (uint8_t)(crc & 0xFF) && f[7] == (uint8_t)(crc >> 8);
}

static void pcs_uart_rx_service(uint32_t now) {
  if (usart_flag_get(PCS_UART, USART_FLAG_RBNE) == RESET)
    return;
  uint8_t b = (uint8_t)usart_data_receive(PCS_UART);
  pcs_led_activity();
  if (inv_rx_n < sizeof(inv_rx))
    inv_rx[inv_rx_n++] = b;
  inv_last_rx_ms = now;
}

// Called every main-loop pass and inside bms_receive_response(): mirror mode
// drains the ring; bridge mode reads UART4 and closes a request on silence.
static void pcs_poll_service(uint32_t now) {
  if (!pcs_bridge_mode) {
    pcs_mirror_pump();
    return;
  }
  pcs_uart_rx_service(now);
  if (inv_frame_n == 0 && inv_rx_n != 0 && (now - inv_last_rx_ms) >= PCS_GAP_MS) {
    if (inv_rx_n >= PCS_NOISE_MIN_LEN && pcs_request_ok(inv_rx)) {
      memcpy(inv_frame, inv_rx, inv_rx_n);
      inv_frame_n = inv_rx_n;
    }
    inv_rx_n = 0;
  }
}

// Bridge mode drives aggregation itself (the box sends no own polls): each
// parsed bridged response feeds the sums once per sweep (dedupe in the
// parser), and the sweep closes when every known pack has been seen or after
// BR_SWEEP_MAX_MS — the inverter's own loop takes ~13 s for 16 addresses.
// A timeout with agg_count == 0 still runs bms_aggregate_data(): it only
// does the presence bookkeeping and clears sweep_seen_mask (after a
// fail-closed agg_reset mid-sweep the mask could otherwise stick full and
// dedupe would starve every later response).
static uint32_t br_sweep_start_ms = 0;

static void bridge_sweep_tick(uint32_t now) {
  if (br_sweep_start_ms == 0) {
    br_sweep_start_ms = now;
    return;
  }
  bool complete = pack_ever_seen_mask != 0 && agg_count != 0 &&
                  ((sweep_seen_mask & pack_ever_seen_mask) == pack_ever_seen_mask);
  if (complete || (now - br_sweep_start_ms) >= BR_SWEEP_MAX_MS) {
    bms_aggregate_data();
    br_sweep_start_ms = 0;
  }
}

static bool pcs_bridge_finish(void) {
  inv_frame_n = 0;
  uart_rx_index = 0;
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN);
  br_state = BR_IDLE;
  return true; // informational only — own polling never resumes in bridge mode
}

// Bridge step: runs once per main-loop pass. Returns true when a bridged
// transaction just finished (ignored by the caller — while DIP2 is ON the
// box sends no polls of its own).
static bool pcs_bridge_step(uint32_t now) {
  switch (br_state) {
  case BR_IDLE: {
    if (inv_frame_n == 0)
      return false;
    rs485_flush_rx();
    pcs_clear_errors(); // drop UART4 overrun left by a blocked loop pass
    uart_rx_index = 0;
    gpio_bit_set(LED_BMS485_PORT, LED_BMS485_PIN);
    for (uint16_t i = 0; i < inv_frame_n; i++) {
      rs485_send_byte(inv_frame[i]);
      pcs_poll_service(millis()); // keep UART4 RX from overrunning
    }
    br_sent_ms = millis();
    br_got = false;
    br_state = BR_WAIT_RESP;
    return false;
  }

  case BR_WAIT_RESP: {
    pcs_poll_service(millis());
    if (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET) {
      uint8_t b = (uint8_t)usart_data_receive(USART0);
      if (!br_got) {
        br_got = true;
        br_first_ms = now;
      }
      br_last_ms = now;
      if (uart_rx_index < sizeof(uart_rx_buffer))
        uart_rx_buffer[uart_rx_index++] = b;
    }
    if (br_got && uart_rx_index >= PCS_NOISE_MIN_LEN &&
        (now - br_last_ms) >= PCS_GAP_MS) {
      // Pure transparent bridge: relay BMS answer back to the master.
      last_bms_response_ms = millis();
      br_state = BR_FWD_RESP; // complete answer — relay it as-is
      return false;
    }
    if (br_got && (now - br_first_ms) >= PCS_RESP_MAX_MS)
      return pcs_bridge_finish(); // too slow — let the inverter time out
    if (br_got && (now - br_last_ms) >= PCS_RESP_STALL_MS)
      return pcs_bridge_finish(); // truncated fragment (1..3 bytes)
    if (!br_got && (now - br_sent_ms) >= PCS_RESP_FIRST_MS)
      return pcs_bridge_finish(); // BMS stayed silent
    return false;
  }

  case BR_FWD_RESP: {
    for (uint16_t i = 0; i < uart_rx_index; i++) {
      usart_data_transmit(PCS_UART, uart_rx_buffer[i]);
      while (usart_flag_get(PCS_UART, USART_FLAG_TBE) == RESET)
        ;
      pcs_led_activity();
      pcs_poll_service(millis());
    }
    return pcs_bridge_finish();
  }

  default:
    return pcs_bridge_finish();
  }
}

// DIP2 mode switch; deferred while a bridge transaction is in flight
// (retried from the 250 ms DIP poll until the bridge goes idle).
static void pcs_mode_apply(uint8_t code) {
  bool bridge = (code & DIP2_BRIDGE_MODE) != 0;
  if (bridge == pcs_bridge_mode)
    return;
  if (br_state != BR_IDLE || inv_frame_n != 0)
    return; // busy — keep the old mode for now
  pcs_bridge_mode = bridge;
  pcs_mirror_head = pcs_mirror_tail = 0;
  inv_rx_n = 0;
  // Drop a half-built sweep from the other mode so dedupe/sums start clean.
  br_sweep_start_ms = 0;
  sweep_seen_mask = 0;
  if (agg_count != 0)
    agg_reset();
}

static void hw_send_query(uint8_t addr_idx, uint8_t q) {
  // FC03-запит серії HW_QUERIES: <addr> 03 <reg:2BE> <qty:2BE> <CRC:2LE>
  uint8_t frame[8];
  frame[0] = HW_ADDR[addr_idx];
  frame[1] = 0x03;
  frame[2] = (uint8_t)(HW_QUERIES[q].reg >> 8);
  frame[3] = (uint8_t)(HW_QUERIES[q].reg & 0xFF);
  frame[4] = 0;
  frame[5] = HW_QUERIES[q].qty;
  uint16_t crc = modbus_crc16(frame, 6);
  frame[6] = (uint8_t)(crc & 0xFF); // lo-byte first (§3, як і у нас)
  frame[7] = (uint8_t)(crc >> 8);

  rs485_flush_rx();
  uart_rx_index = 0;
  gpio_bit_set(LED_BMS485_PORT, LED_BMS485_PIN); // poll in flight
  // No DE pin on the transceiver (auto-direction, ORIGINAL_FIRMWARE.md §14.3):
  // just shift the frame out; direction switches in hardware.
  for (uint8_t i = 0; i < 8; i++) {
    rs485_send_byte(frame[i]);
    pcs_mirror_push(frame[i]); // no-op in bridge mode
  }
  while (usart_flag_get(USART0, USART_FLAG_TC) == RESET)
    ;
  // Flush any TX echo or line bounce after transmission finishes:
  rs485_flush_rx();
  uart_rx_index = 0;
}

// Прийом до expected_len байт: тиша BMS_TIMEOUT_MS (400 мс),
// достроковий стоп на повному кадрі або на кадрі винятку Modbus (5 Б).
static uint16_t bms_receive_response(uint16_t expected_len) {
  uart_rx_index = 0;
  uint32_t start_ms = millis();
  uint32_t wait_start = start_ms;
  while (millis() - wait_start < BMS_TIMEOUT_MS) {
    pcs_poll_service(millis()); // mirror drain / inverter RX while we wait
    if (millis() - start_ms >= BMS_TIMEOUT_MS + 200)
      break; // жорсткий дедлайн (шум не може тягнути прийом безкінечно)
    if (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET) {
      uint8_t byte = usart_data_receive(USART0);
      if (uart_rx_index == 0 && byte == 0x00)
        continue; // відсікаємо брязкіт лінії при переході передавача
      pcs_mirror_push(byte); // no-op in bridge mode
      if (uart_rx_index < sizeof(uart_rx_buffer)) {
        uart_rx_buffer[uart_rx_index++] = byte;
      } else {
        uart_rx_index = 0; // overflow
      }
      if (uart_rx_index >= expected_len)
        break; // кадр повний
      if (uart_rx_index >= 5 && (uart_rx_buffer[1] & 0x80))
        break; // виняток Modbus: <addr> 83 <code> <CRC> (5 байт)
      wait_start = millis();
    }
  }
  rs485_clear_errors();
  gpio_bit_reset(LED_BMS485_PORT, LED_BMS485_PIN); // poll finished
  return uart_rx_index;
}

// --- Huawei ESM: декод відповіді у hw_mod + живлення агрегат-сум (§5, §6) --
// Повертає: 0 — не наш або битий кадр; 1 — дані FC03; 2 — виняток Modbus
// (модуль на шині, але відмовився від реєстра — присутність враховується).
static uint8_t hw_parse_response(uint8_t addr_idx, uint8_t q) {
  uint8_t addr = HW_ADDR[addr_idx];
  if (uart_rx_index < 5 || uart_rx_buffer[0] != addr)
    return 0;

  if (uart_rx_buffer[1] == 0x83) { // виняток Modbus, §4.4: <addr> 83 code CRC
    if (uart_rx_index < 5)
      return 0;
    uint16_t crc = modbus_crc16((uint8_t *)uart_rx_buffer, 3);
    if (((uint16_t)uart_rx_buffer[4] << 8 | uart_rx_buffer[3]) != crc)
      return 0;
    return 2;
  }

  uint16_t qty = HW_QUERIES[q].qty;
  if (uart_rx_buffer[1] != 0x03 || uart_rx_buffer[2] != qty * 2)
    return 0;
  uint16_t frame_len = (uint16_t)(5 + 2 * qty);
  if (uart_rx_index < frame_len)
    return 0;
  uint16_t received_crc = (uart_rx_buffer[frame_len - 1] << 8) |
                          uart_rx_buffer[frame_len - 2];
  uint16_t calculated_crc =
      modbus_crc16((uint8_t *)uart_rx_buffer, frame_len - 2);
  if (received_crc != calculated_crc)
    return 0;

  HwModule *m = &hw_mod[addr_idx];
  const volatile uint8_t *d = &uart_rx_buffer[3]; // початок даних FC03

  switch (q) {
  case HW_Q_BASE: {
    // 0x0000 ×7: bus V, pack V, струм, SOC, SOH, t max, t min — усі ×0.01/°C
    m->bus_v = ((uint16_t)(d[0] << 8) | d[1]) * 0.01f;
    m->pack_v = ((uint16_t)(d[2] << 8) | d[3]) * 0.01f;
    m->current = (float)(int16_t)((uint16_t)(d[4] << 8) | d[5]) * 0.01f;
    uint16_t soc = (uint16_t)(d[6] << 8) | d[7];
    uint16_t soh = (uint16_t)(d[8] << 8) | d[9];
    int16_t t_max = (int16_t)((uint16_t)(d[10] << 8) | d[11]);
    int16_t t_min = (int16_t)((uint16_t)(d[12] << 8) | d[13]);
    m->soc = (uint8_t)soc;
    m->soh = (uint8_t)soh;
    m->t_max = t_max;
    m->t_min = t_min;
    // Базові суми — один раз на sweep на модуль
    if (!(hw_base_seen_mask & (1u << addr_idx))) {
      hw_base_seen_mask |= (1u << addr_idx);
      agg_voltage_sum += m->pack_v; // §6: середнє по модулях
      agg_current_sum += m->current; // §6: сума; +заряд — CAN інвертує знак
      agg_soc_sum += soc;           // §6: середнє (однакова ємність)
      if (soh < agg_soh_min)
        agg_soh_min = soh; // §6: SOH агрегату = мінімум по модулях
      agg_count++;
    }
    if (t_max >= -40 && t_max <= 120 && (float)t_max > agg_temp_max)
      agg_temp_max = (float)t_max;
    if (t_min >= -40 && t_min <= 120 && (float)t_min < agg_temp_min)
      agg_temp_min = (float)t_min;
    break;
  }

  case HW_Q_ALARM:
    m->cycles = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                ((uint32_t)d[2] << 8) | d[3]; // 0x0042–0x0043 U32 BE
    m->ah = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) |
            ((uint32_t)d[6] << 8) | d[7];     // 0x0044–0x0045 U32 BE
    for (uint8_t i = 0; i < 5; i++)
      m->alarm[i] = (uint16_t)(d[(i + 4) * 2] << 8) | d[(i + 4) * 2 + 1]; // 0x0046..0x004A
    hw_alarm_seen_mask |= (1u << addr_idx); // мапінг у 0x359 — у aggregate()
    break;

  case HW_Q_CELL: { // 0x0022 ×15: U16 мВ
    uint16_t cmax = 0, cmin = 0xFFFF;
    for (uint8_t i = 0; i < 15; i++) {
      uint16_t mv = (uint16_t)(d[i * 2] << 8) | d[i * 2 + 1];
      m->cell_mv[i] = mv;
      if (mv > cmax)
        cmax = mv;
      if (mv > 0 && mv < cmin)
        cmin = mv;
      if (mv > agg_max_cell_mv)
        agg_max_cell_mv = mv;
      if (mv > 0 && mv < agg_min_cell_mv)
        agg_min_cell_mv = mv;
    }
    m->cell_max = cmax;
    m->cell_min = cmin;
    break;
  }


  case HW_Q_CELL_TEMP: // 0x0012 ×15: температури комірок
    for (uint8_t i = 0; i < 15; i++)
      m->cell_temp[i] = (int16_t)((uint16_t)(d[i * 2] << 8) | d[i * 2 + 1]);
    break;

  case HW_Q_SYS_INFO: // 0x0101 ×15: версії та кількість комірок (0x010F)
    m->ncell = (uint8_t)(((uint16_t)(d[14 * 2] << 8) | d[14 * 2 + 1]) & 0xFFU);
    break;

  case HW_Q_MODEL: // 0x0332 ×12: модель (транслюється в PCS-485 дзеркало)
    break;

  case HW_Q_SERIAL: // 0x0340 ×10: серійний номер (транслюється в PCS-485 дзеркало)
    break;

  default:
    return 0;
  }

  return 1;
}

static void hw_advance_addr(void) {
  hw_q_idx = HW_Q_BASE;
  if (++hw_addr_idx >= HW_MODULE_COUNT) {
    hw_addr_idx = 0;
    bms_aggregate_data(); // Обидва акумулятори опитані -> агрегація та формування CAN відповіді
  }
}

// Машина станів опиту строго 2 модулів Huawei ESM (D6 = 214 та D7 = 215):
//  - кожен модуль опитується 3 ключовими реєстрами (BASE, ALARM, CELL);
//  - після завершення циклу по обох акумуляторах bms_aggregate_data() одразу
//    відправляє агреговані дані в CAN шину;
//  - швидкість повного циклу становить ~1.0–1.1 секунди, синхронно з вимогами Deye CAN.
static void hw_poll_service(uint32_t now) {
  if (pcs_bridge_mode)
    return; // DIP2 ON: чистий міст, жодних власних команд на шині
  if ((int32_t)(now - hw_uart_ready_ms) < 0)
    return; // старт після запуску UART
  if ((int32_t)(now - hw_next_slot_ms) < 0)
    return; // міжкадрова пауза

  uint8_t idx = hw_addr_idx;

  hw_send_query(idx, hw_q_idx);
  bms_receive_response(HW_QUERIES[hw_q_idx].qty * 2 + 5);
  uint8_t r = hw_parse_response(idx, hw_q_idx);
  if (r == 0) {
    hw_advance_addr(); // таймаут/битий кадр: не затримувати опит
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
    return;
  }

  last_bms_response_ms = millis();
  hw_mod[idx].last_ok_ms = now;
  sweep_seen_mask |= (1u << idx);
  pack_miss_count[idx] = 0;

  if (r == 2) {
    hw_advance_addr(); // виняток Modbus: перехід до наступного модуля
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
  } else if (hw_q_idx >= HW_Q_FAST_COUNT) {
    // Повільний запит завершено — переходимо до наступного модуля
    hw_advance_addr();
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
  } else if (++hw_q_idx >= HW_Q_FAST_COUNT) {
    // Раз на ~5 циклів кожного модуля додаємо один повільний запит для повного моніторингу на PCS-485
    static uint8_t slow_tick[HW_MODULE_COUNT] = {0};
    static uint8_t slow_q[HW_MODULE_COUNT] = {HW_Q_CELL_TEMP, HW_Q_CELL_TEMP};
    if (++slow_tick[idx] >= 5) {
      slow_tick[idx] = 0;
      hw_q_idx = slow_q[idx];
      if (++slow_q[idx] >= HW_Q_COUNT)
        slow_q[idx] = HW_Q_CELL_TEMP;
      hw_next_slot_ms = now + HW_INTER_QUERY_MS;
    } else {
      hw_advance_addr(); // всі швидкі регістри модуля прочитано
      hw_next_slot_ms = now + HW_INTER_MODULE_MS;
    }
  } else {
    hw_next_slot_ms = now + HW_INTER_QUERY_MS;
  }
}

static uint16_t calculate_charge_limit(uint8_t pack_count) {
  uint32_t per_pack_x10 = profile_charge_current_per_pack_x10;
  // Balance/manual top-off phase: cap at 2.0 A per pack so passive balancing
  // gets hours of gentle CV time instead of a rushed stop at the threshold.
  if (bal_trickle && per_pack_x10 > BAL_TRICKLE_A_X10)
    per_pack_x10 = BAL_TRICKLE_A_X10;
  uint32_t max_current_x10 = per_pack_x10 * pack_count;

  // Temperature cut-offs: LiFePO4 plating occurs below +2°C; over-temp at 55°C.
  // Hysteresis release: re-enable only when temp is within [+3°C .. 50°C] to prevent chattering.
  if (agg_temp_min < TEMP_CHARGE_MIN_C || agg_temp_max > TEMP_CHARGE_MAX_C) {
    charge_temp_inhibit = true;
  } else if (charge_temp_inhibit) {
    if (agg_temp_min >= TEMP_CHARGE_MIN_REENABLE_C && agg_temp_max <= TEMP_CHARGE_MAX_REENABLE_C) {
      charge_temp_inhibit = false;
    }
  }

  // System error or over-current protection from 0x359
  if (can_protect_b1 & (CAN_PROT_CHARGE_OC | CAN_PROT_SYSTEM_ERROR)) {
    return 0;
  }

  // Cell voltage cut-off: stop at active stop, re-enable below re-enable threshold
  // (active thresholds = DIP profile, or the full-charge triple during a
  // balance session — see balance_update())
  if (agg_max_cell_mv >= calc_charge_stop_mv) {
    charge_cell_ov_inhibit = true;
  } else if (charge_cell_ov_inhibit) {
    if (agg_max_cell_mv < calc_charge_reenable_mv) {
      charge_cell_ov_inhibit = false;
    }
  }

  if (charge_temp_inhibit || charge_cell_ov_inhibit) {
    return 0;
  }

  if (agg_max_cell_mv <= calc_taper_start_mv)
    return (uint16_t)max_current_x10;

  return (uint16_t)(max_current_x10 *
                    (calc_charge_stop_mv - agg_max_cell_mv) /
                    (calc_charge_stop_mv - calc_taper_start_mv));
}

static uint16_t calculate_discharge_limit(uint8_t pack_count) {
  uint32_t max_current_x10 =
      (uint32_t)DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 * pack_count;

  uint16_t min_cell = agg_min_cell_mv;
  if (min_cell == 0xFFFF) {
    for (uint8_t i = 0; i < HW_MODULE_COUNT; i++) {
      if (hw_mod[i].cell_min > 0 && hw_mod[i].cell_min < min_cell) {
        min_cell = hw_mod[i].cell_min;
      }
    }
  }
  if (min_cell == 0xFFFF && bms_voltage > 40.0f) {
    min_cell = (uint16_t)((bms_voltage / 15.0f) * 1000.0f);
  }

  if (min_cell == 0xFFFF)
    return 0;

  // Temperature cut-offs for discharge (-20°C .. +60°C) with hysteresis
  if (agg_temp_min < TEMP_DISCHARGE_MIN_C || agg_temp_max > TEMP_DISCHARGE_MAX_C) {
    discharge_temp_inhibit = true;
  } else if (discharge_temp_inhibit) {
    if (agg_temp_min >= TEMP_DISCHARGE_MIN_REENABLE_C && agg_temp_max <= TEMP_DISCHARGE_MAX_REENABLE_C) {
      discharge_temp_inhibit = false;
    }
  }

  // System error protection
  if (can_protect_b1 & CAN_PROT_SYSTEM_ERROR) {
    return 0;
  }

  // Cell voltage cut-off: hard cut <= 2650 mV, re-enable >= 3000 mV
  if (min_cell <= CELL_DISCHARGE_STOP_MV) {
    discharge_cell_uv_inhibit = true;
  } else if (discharge_cell_uv_inhibit) {
    if (min_cell >= CELL_DISCHARGE_REENABLE_MV) {
      discharge_cell_uv_inhibit = false;
    }
  }

  if (discharge_temp_inhibit || discharge_cell_uv_inhibit) {
    return 0;
  }

  // Soft cut / Cell UV warning (<= 2800 mV): derate discharge current to 50%
  if (min_cell <= CELL_UV_WARN_MV) {
    max_current_x10 /= 2;
  }

  // Severe cell imbalance check (> 400 mV delta): derate to 50%
  if (agg_max_cell_mv > min_cell && (agg_max_cell_mv - min_cell) > CELL_MAX_DELTA_MV) {
    max_current_x10 /= 2;
  }

  return (uint16_t)max_current_x10;
}

// ===== Balance session engine =====
// DIP5 ON: forced 54.0 V equalization (overrides DIP1 and auto sessions).
// DIP1 OFF (standard): CVL 53.5 V + auto sessions (BAL_AUTO_ENABLED) at 54.0 V,
// driven by cell-spread analysis. DIP1 ON: 90% conservative mode (51.8 V), no auto sessions.
// Both modes cap the current at 2 A per pack once min cell >= 3400 mV (~90%),
// giving passive balancing hours of gentle CV time. Sessions exit on spread
// convergence (<= 25 mV), BMS loss, sustained discharge (grid outage), no progress
// or an 8 h timeout.

static void balance_session_end(bool success, uint32_t cooldown_until) {
  bal_session_active = false;
  bal_trickle = false;
  bal_exit_streak = 0;
  bal_outage_agg = 0;
  if (success)
    bal_last_success_ms = millis();
  bal_cooldown_until_ms = cooldown_until;
}

static void balance_on_bms_lost(void) {
  if (bal_session_active)
    balance_session_end(false, millis() + BAL_COOLDOWN_OUTAGE_MS);
}

static void balance_hist_push(uint16_t spread, uint32_t now) {
  if (bal_last_hist_ms != 0 && (now - bal_last_hist_ms) < 3600000UL)
    return;
  bal_last_hist_ms = now;
  if (bal_hist_n < BAL_HIST_HOURS) {
    bal_hist_mv[bal_hist_n++] = spread;
  } else {
    memmove(&bal_hist_mv[0], &bal_hist_mv[1],
            (BAL_HIST_HOURS - 1) * sizeof(uint16_t));
    bal_hist_mv[BAL_HIST_HOURS - 1] = spread;
  }
}

// >=5 mV growth over 12 h == 10 mV/day -> anomaly regardless of the bucket.
static bool balance_trend_high(void) {
  if (bal_hist_n < 13)
    return false;
  return ((int32_t)bal_hist_mv[bal_hist_n - 1] -
          (int32_t)bal_hist_mv[bal_hist_n - 12]) > BAL_TREND_DELTA_MV_12H;
}

static void balance_update(uint8_t packs) {
  bool force_54v = (dip_applied_code & DIP_BALANCE_54V) != 0;
  if (force_54v) {
    bal_session_active = false;
    bal_trickle = false;
    calc_taper_start_mv = profile_taper_start_mv;
    calc_charge_stop_mv = profile_charge_stop_mv;
    calc_charge_reenable_mv = profile_charge_reenable_mv;
    return;
  }
  bool manual = (dip_applied_code & DIP_PROFILE_CODE_90) != 0;
  uint32_t now = millis();
  bool have_cells = (agg_max_cell_mv != 0) && (agg_min_cell_mv != 0xFFFF) &&
                    (agg_max_cell_mv >= agg_min_cell_mv);
  uint16_t spread =
      have_cells ? (uint16_t)(agg_max_cell_mv - agg_min_cell_mv) : 0;

  if (have_cells)
    balance_hist_push(spread, now);

  // DIP1 transitions: switching the 51.8 V option ON aborts an active
  // session; leaving the option returns to standard 54 V behind a cooldown
  // so an auto session cannot re-trigger instantly.
  if (manual && !bal_manual_prev) {
    bal_session_active = false;
    bal_manual_prev = true;
  } else if (!manual && bal_manual_prev) {
    bal_session_active = false;
    bal_manual_prev = false;
    bal_cooldown_until_ms = now + BAL_COOLDOWN_TRY_MS;
  }

  if (bal_session_active) {
    // Sustained discharge (~60 s) = grid outage or heavy load -> abort.
    if (bms_current > BAL_OUTAGE_I_MIN_A) {
      if (bal_outage_agg < 255)
        bal_outage_agg++;
      if (bal_outage_agg >= BAL_OUTAGE_AGG_N) {
        balance_session_end(false, now + BAL_COOLDOWN_OUTAGE_MS);
        return;
      }
    } else {
      bal_outage_agg = 0;
    }

    // Top-off phase: min cell reached ~90% -> 2 A per pack cap
    // (applied by calculate_charge_limit() through bal_trickle).
    bal_trickle = (agg_min_cell_mv != 0xFFFF) &&
                  (agg_min_cell_mv >= BAL_TRICKLE_CELL_MV);

    if (bal_trickle) {
      if (bal_best_spread_mv == 0xFFFF) {
        bal_best_spread_mv = spread;
        bal_last_improve_ms = now;
      } else if (spread < bal_best_spread_mv) {
        bal_best_spread_mv = spread;
        bal_last_improve_ms = now;
      } else if ((now - bal_last_improve_ms) >= BAL_NO_PROGRESS_MS) {
        // Balancing not helping (possible weak cell) -> back off.
        balance_session_end(false, now + BAL_COOLDOWN_TRY_MS);
        return;
      }
    }

    if (spread <= BAL_SPREAD_EXIT_MV) {
      if (++bal_exit_streak >= BAL_EXIT_STREAK) {
        balance_session_end(true, now);
        return;
      }
    } else {
      bal_exit_streak = 0;
    }

    if ((now - bal_session_start_ms) >= BAL_SESSION_MAX_MS) {
      balance_session_end(false, now + BAL_COOLDOWN_TRY_MS);
      return;
    }

    // In-session thresholds: full-charge triple so the 54 V session is not
    // cut by the 90% profile stop; 3600 mV OV inhibit stays as hard guard.
    calc_taper_start_mv = CELL_TAPER_START_MV;
    calc_charge_stop_mv = 3600;
    calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
    return;
  }

  // Not in a session: active thresholds mirror the DIP profile.
  calc_taper_start_mv = profile_taper_start_mv;
  calc_charge_stop_mv = profile_charge_stop_mv;
  calc_charge_reenable_mv = profile_charge_reenable_mv;
  bal_trickle = manual && have_cells &&
                (agg_min_cell_mv >= BAL_TRICKLE_CELL_MV);

  if (manual || !BAL_AUTO_ENABLED)
    return;
  if (!bms_data_valid)
    return;

  // Pack composition change (a pack was switched on/off) -> HIGH session due.
  if (bal_prev_packs != 0 && packs != bal_prev_packs)
    bal_last_success_ms = now - BAL_INTERVAL_HIGH_MS;
  bal_prev_packs = packs;

  if (bms_soc < BAL_MIN_SOC_START)
    return;
  if (bms_current >= BAL_START_I_MAX_A)
    return;
  if (charge_cell_ov_inhibit || charge_temp_inhibit)
    return;
  if (agg_temp_min < BAL_TEMP_MIN_C || agg_temp_max > BAL_TEMP_MAX_C)
    return;
  if ((int32_t)(now - bal_cooldown_until_ms) < 0)
    return;

  // Severity: spread bucket + 12 h trend. Outliers >= 100 mV are covered by
  // the HIGH bucket (interval: 24 h).
  uint32_t interval;
  if ((spread >= BAL_SPREAD_HIGH_MV) || balance_trend_high())
    interval = BAL_INTERVAL_HIGH_MS;
  else if (spread >= BAL_SPREAD_MED_MV)
    interval = BAL_INTERVAL_MED_MS;
  else
    interval = BAL_INTERVAL_OK_MS;

  if ((now - bal_last_success_ms) < interval)
    return;

  // Already well balanced: renew the interval instead of burning a session.
  if (spread <= BAL_SPREAD_EXIT_MV) {
    bal_last_success_ms = now;
    return;
  }

  // Start a session.
  bal_session_active = true;
  bal_session_start_ms = now;
  bal_exit_streak = 0;
  bal_outage_agg = 0;
  bal_best_spread_mv = 0xFFFF;
  bal_last_improve_ms = now;
  bal_trickle = false;
  calc_taper_start_mv = CELL_TAPER_START_MV;
  calc_charge_stop_mv = 3600;
  calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
}

// LED1 (PC13): fast blink (200 ms) = DIP5 54.0 V forced balance,
// solid = 51.8 V option (DIP1 ON), slow blink (500 ms) = auto session,
// off = standard 53.5 V without session. Called from the main loop.
static void balance_led_tick(uint32_t now) {
  static bool on = false;
  static uint32_t last_toggle = 0;
  if (dip_applied_code & DIP_BALANCE_54V) {
    if ((now - last_toggle) >= 200) {
      last_toggle = now;
      on = !on;
      if (on)
        gpio_bit_set(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
      else
        gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
    }
    return;
  }
  if (dip_applied_code & DIP_PROFILE_CODE_90) {
    gpio_bit_set(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
    on = true;
    return;
  }
  if (bal_session_active) {
    if ((now - last_toggle) >= 500) {
      last_toggle = now;
      on = !on;
      if (on)
        gpio_bit_set(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
      else
        gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
    }
  } else if (on) {
    on = false;
    gpio_bit_reset(LED1_DEBUG_PORT, LED1_DEBUG_PIN);
  }
}

static void agg_reset(void) {
  agg_voltage_sum = 0.0f;
  agg_current_sum = 0.0f;
  agg_soc_sum = 0;
  agg_soh_min = 100; // §6: SOH агрегату = мінімум по модулях
  agg_temp_max = -1000.0f;
  agg_temp_min = 1000.0f;
  agg_count = 0;
  agg_max_cell_mv = 0;
  agg_min_cell_mv = 0xFFFF;
}

// Мапінг аварій Huawei → біти 0x359 (з фізичною перехресною перевіркою).
// Дискретні аварії (0x0048/0x004A: OC, anti-theft, внутрішнє OV тощо) лишаються
// log-only у hw_mod.alarm до стенд-верифікації на реальному обладнанні.
static void hw_map_alarms(void) {
  for (uint8_t i = 0; i < HW_MODULE_COUNT; i++) {
    if (!(hw_alarm_seen_mask & (1u << i)))
      continue;
    HwModule *m = &hw_mod[i];
    uint16_t prot = m->alarm[2]; // 0x0048: захисти (§5.4)
    uint16_t warn = m->alarm[3]; // 0x0049: попередження
    uint16_t sysf = m->alarm[0]; // 0x0046: біти 1–6 = fault
    uint16_t modst = m->alarm[4]; // 0x004A: біт0 shutdown, біт1 lithium fault

    // Protection → 0x359 байти 0–1 (Pylontech) + Tables 1-4 (Deye V3.3)
    if ((prot & (1u << 5)) && m->cell_max >= CELL_PHYSICAL_OV_CONFIRM_MV) {
      sweep_protect_b0 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
      sweep_dy_p0 |= (0x01U | 0x04U); // T1: Cell OV | Module OV
    }
    if ((prot & (1u << 4)) && m->pack_v >= 54.5f) {
      sweep_protect_b0 |= CAN_PROT_PACK_OV;
      sweep_dy_p0 |= 0x04U; // T1: Module OV
    }
    if ((prot & (1u << 7)) && (m->cell_min <= CELL_UV_WARN_MV || m->soc <= 5)) {
      sweep_protect_b0 |= CAN_PROT_CELL_UV;
      sweep_dy_p0 |= 0x02U; // T1: Cell UV
    }
    if ((prot & (1u << 6)) && m->pack_v <= 43.2f) {
      sweep_protect_b0 |= CAN_PROT_CELL_UV;
      sweep_dy_p0 |= 0x08U; // T1: Module UV
    }
    if ((prot & ((1u << 1) | (1u << 3) | (1u << 9))) &&
        (float)m->t_max >= TEMP_DISCHARGE_MAX_C) {
      sweep_protect_b0 |= CAN_PROT_OT;
      sweep_dy_p0 |= 0x40U; // T1: Charge OT
      sweep_dy_p1 |= 0x10U; // T2: Discharge OT
    }
    if ((prot & ((1u << 0) | (1u << 2))) &&
        (float)m->t_min <= TEMP_DISCHARGE_MIN_C) {
      sweep_protect_b0 |= CAN_PROT_UT;
      sweep_dy_p0 |= 0x80U; // T1: Charge UT
      sweep_dy_p1 |= 0x20U; // T2: Discharge UT
    }
    if ((sysf & 0x7E) || (modst & 0x0003)) {
      sweep_protect_b1 |= CAN_PROT_SYSTEM_ERROR;
      sweep_dy_e6 |= 0x01U; // T7: System Error
    }

    // Warning → 0x359 байти 2–3 (Pylontech) + Tables 5-6 (Deye V3.3)
    if ((warn & (1u << 5)) && m->cell_max >= CELL_PHYSICAL_OV_WARN_CONFIRM_MV) {
      sweep_alarm_b2 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
      sweep_dy_a4 |= (0x01U | 0x04U); // T5: Cell HV | Module HV
    }
    if ((warn & (1u << 4)) && m->pack_v >= 54.0f) {
      sweep_alarm_b2 |= CAN_PROT_PACK_OV;
      sweep_dy_a4 |= 0x04U; // T5: Module HV
    }
    if ((warn & (1u << 7)) && (m->cell_min <= CELL_UV_WARN_MV || m->soc <= 10)) {
      sweep_alarm_b2 |= CAN_PROT_CELL_UV;
      sweep_dy_a4 |= 0x02U; // T5: Cell LV
    }
    if ((warn & (1u << 6)) && m->pack_v <= 44.0f) {
      sweep_alarm_b2 |= CAN_PROT_CELL_UV;
      sweep_dy_a4 |= 0x08U; // T5: Module LV
    }
    if ((warn & ((1u << 1) | (1u << 3))) && (float)m->t_max >= 50.0f) {
      sweep_alarm_b2 |= CAN_PROT_OT;
      sweep_dy_a4 |= 0x40U; // T5: Charge HT
      sweep_dy_a5 |= 0x10U; // T6: Discharge HT
    }
    if ((warn & ((1u << 0) | (1u << 2))) &&
        (float)m->t_min <= TEMP_DISCHARGE_MIN_C + 5.0f) {
      sweep_alarm_b2 |= CAN_PROT_UT;
      sweep_dy_a4 |= 0x80U; // T5: Charge LT
      sweep_dy_a5 |= 0x20U; // T6: Discharge LT
    }
  }
}

static void bms_aggregate_data(void) {
  // Update pack presence with 5-miss debounce (Q21)
  uint8_t active_pack_count = 0;
  for (uint8_t i = 0; i < HW_MODULE_COUNT; i++) {
    if (sweep_seen_mask & (1 << i)) {
      pack_ever_seen_mask |= (1 << i);
      pack_miss_count[i] = 0;
      active_pack_count++;
    } else if (pack_ever_seen_mask & (1 << i)) {
      if (pack_miss_count[i] < 255)
        pack_miss_count[i]++;
      if (pack_miss_count[i] <= 5 && hw_mod[i].last_ok_ms != 0) {
        active_pack_count++;
      } else if (pack_miss_count[i] >= 10) {
        pack_ever_seen_mask &= ~(1u << i);
      }
    }
  }

  // Свіжі аварії саме цього sweep — у sweep_protect/sweep_alarm ДО їх латчу
  // (інакше біти зависнуть до наступного вдалого aggregate).
  hw_map_alarms();
  sweep_seen_mask = 0;

  // Якщо активний модуль (в межах debounce <= 5) пропустив цей конкретний sweep,
  // підхоплюємо його останню валідну телеметрію, щоб суми струмів, напруг та лімітів не смикались:
  for (uint8_t i = 0; i < HW_MODULE_COUNT; i++) {
    if (!(hw_base_seen_mask & (1u << i)) && (pack_ever_seen_mask & (1u << i)) &&
        pack_miss_count[i] > 0 && pack_miss_count[i] <= 5 && hw_mod[i].pack_v > 40.0f) {
      agg_voltage_sum += hw_mod[i].pack_v;
      agg_current_sum += hw_mod[i].current;
      agg_soc_sum += hw_mod[i].soc;
      if (hw_mod[i].soh < agg_soh_min)
        agg_soh_min = hw_mod[i].soh;
      agg_count++;
    }
  }

  hw_base_seen_mask = 0;
  hw_alarm_seen_mask = 0;

  if (agg_count == 0)
    return;


  bms_voltage = agg_voltage_sum / (float)agg_count;
  // Deye PCS CAN V3.3: 0x356 current is discharge positive, charge negative.
  bms_current = BMS_CURRENT_CHARGE_IS_POSITIVE ? -agg_current_sum
                                               : agg_current_sum;
  bms_soc = (uint8_t)(agg_soc_sum / agg_count);
  bms_soh = agg_soh_min; // §6: SOH = мінімум по модулях (найгірший)
  bms_temp = agg_temp_max / BMS_TEMP_RAW_DIV;

  // Latch sweep alarms and protection into active CAN frame registers
  can_protect_b0 = sweep_protect_b0;
  can_protect_b1 = sweep_protect_b1;
  can_alarm_b2 = sweep_alarm_b2;
  can_alarm_b3 = sweep_alarm_b3;
  sweep_protect_b0 = 0;
  sweep_protect_b1 = 0;
  sweep_alarm_b2 = 0;
  sweep_alarm_b3 = 0;

  can_dy_p0 = sweep_dy_p0;
  can_dy_p1 = sweep_dy_p1;
  can_dy_p2 = sweep_dy_p2;
  can_dy_p3 = sweep_dy_p3;
  can_dy_a4 = sweep_dy_a4;
  can_dy_a5 = sweep_dy_a5;
  can_dy_e6 = sweep_dy_e6;
  sweep_dy_p0 = sweep_dy_p1 = sweep_dy_p2 = sweep_dy_p3 = 0;
  sweep_dy_a4 = sweep_dy_a5 = sweep_dy_e6 = 0;

  uint8_t effective_packs = (active_pack_count > 0) ? active_pack_count : agg_count;
  bms_pack_count = effective_packs;

  balance_update(effective_packs);
  charge_voltage_limit = bal_session_active ? CHARGE_VOLTAGE_LIMIT_54V
                                            : profile_charge_voltage_limit;
  charge_current_limit = calculate_charge_limit(effective_packs);
  discharge_current_limit = calculate_discharge_limit(effective_packs);

  // Latch bank cell/temp extremes before agg_reset() clears them
  bank_max_cell_mv = (agg_max_cell_mv != 0) ? agg_max_cell_mv : 0;
  bank_min_cell_mv = (agg_min_cell_mv != 0xFFFF) ? agg_min_cell_mv : 0;
  bank_temp_max = (agg_temp_max > -500.0f) ? agg_temp_max : BATTERY_TEMP_DEFAULT;
  bank_temp_min = (agg_temp_min < 500.0f) ? agg_temp_min : BATTERY_TEMP_DEFAULT;

  // Set data valid now that fresh aggregated values are computed
  bms_data_valid = true;

  // Формуємо та надсилаємо агреговану відповідь у CAN шину ТІЛЬКИ ПІСЛЯ збору та агрегації даних від обох акумуляторів
  send_all_frames();
  last_can = millis();

  agg_reset();
}

static void can_init_500k(void) {
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(DEYE_CAN_GPIO_RCU);
  rcu_periph_clock_enable(DEYE_CAN_RCU);
  gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);
  gpio_init(DEYE_CAN_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
            DEYE_CAN_RX_PIN);
  gpio_init(DEYE_CAN_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            DEYE_CAN_TX_PIN);

  can_deinit(DEYE_CAN);
  can_parameter_struct params;
  can_struct_para_init(CAN_INIT_STRUCT, &params);
  params.auto_bus_off_recovery = ENABLE;
  params.auto_retrans = ENABLE;
  params.working_mode = CAN_NORMAL_MODE;
  params.resync_jump_width = CAN_BT_SJW_1TQ;
  params.time_segment_1 = CAN_BT_BS1_11TQ;
  params.time_segment_2 = CAN_BT_BS2_4TQ;
  params.prescaler = 1;
  can_init(DEYE_CAN, &params);

  can_filter_parameter_struct filter;
  can_struct_para_init(CAN_FILTER_STRUCT, &filter);
  filter.filter_number = 0;
  filter.filter_mode = CAN_FILTERMODE_MASK;
  filter.filter_bits = CAN_FILTERBITS_32BIT;
  filter.filter_list_high = 0x0000;
  filter.filter_list_low = 0x0000;
  filter.filter_mask_high = 0x0000;
  filter.filter_mask_low = 0x0000;
  filter.filter_fifo_number = CAN_FIFO0;
  filter.filter_enable = ENABLE;
  can_filter_init(&filter);
}

static bool can_mailbox_free(void) {
  return (can_flag_get(DEYE_CAN, CAN_FLAG_TME0) == SET) ||
         (can_flag_get(DEYE_CAN, CAN_FLAG_TME1) == SET) ||
         (can_flag_get(DEYE_CAN, CAN_FLAG_TME2) == SET);
}

static bool send_can_std(uint32_t id, const uint8_t *data, uint8_t len) {
  uint32_t wait_start = millis();
  while (!can_mailbox_free()) {
    if (millis() - wait_start >= 5)
      return false; // скриньки зайняті (немає ACK) — кидати кадр
  }

  can_transmit_message_struct tx_msg;
  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_msg);
  tx_msg.tx_sfid = id;
  tx_msg.tx_ff = CAN_FF_STANDARD;
  tx_msg.tx_ft = CAN_FT_DATA;
  tx_msg.tx_dlen = len;
  for (uint8_t i = 0; i < len; i++)
    tx_msg.tx_data[i] = data[i];

  uint8_t mbox = can_message_transmit(DEYE_CAN, &tx_msg);
  if (mbox == CAN_NOMAILBOX)
    return false;

  uint32_t tx_start = millis();
  while (can_transmit_states(DEYE_CAN, mbox) == CAN_TRANSMIT_PENDING) {
    if (millis() - tx_start >= 3)
      return false;
  }
  return true;
}

// Drain CAN_FIFO0 to prevent hardware FIFO overflow and track inverter 0x305 heartbeat.
static void can_rx_service(uint32_t now) {
  uint8_t pending = can_receive_message_length_get(DEYE_CAN, CAN_FIFO0);
  while (pending > 0) {
    can_receive_message_struct rx_msg;
    can_message_receive(DEYE_CAN, CAN_FIFO0, &rx_msg);
    if (rx_msg.rx_ff == CAN_FF_STANDARD && rx_msg.rx_sfid == CAN_ID_INV_HEARTBEAT) {
      last_inverter_can_rx_ms = now;
      inverter_can_alive = true;
    }
    pending = can_receive_message_length_get(DEYE_CAN, CAN_FIFO0);
  }
  if (inverter_can_alive && (now - last_inverter_can_rx_ms > INVERTER_CAN_TIMEOUT_MS)) {
    inverter_can_alive = false;
  }
}

static int16_t round_to_i16(float v) {
  return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static void put_u16le(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
}

#if CAN_PROTOCOL_DEYE_V33
static bool send_deye_pack_identity(uint8_t idx) {
  uint8_t data[8];
  static const uint8_t fw[8] = {0x20, 0x08, 0xAA, 'V', '5', '.', '1', '8'};
  static const uint8_t model[8] = {'E', 'S', 'M', '4', '8', '1', '0', '0'};
  if (!send_can_std((uint32_t)(CAN_ID_DEYE_PACK_FW_BASE + idx), fw, 8))
    return false;
  if (!send_can_std((uint32_t)(CAN_ID_DEYE_PACK_MODEL_BASE + idx), model, 8))
    return false;

  uint8_t pack_num = (uint8_t)(idx + 1U);
  data[0] = 'H';
  data[1] = 'W';
  data[2] = '0';
  data[3] = '0';
  data[4] = '0';
  data[5] = '0';
  data[6] = (uint8_t)('0' + (pack_num / 10U));
  data[7] = (uint8_t)('0' + (pack_num % 10U));
  return send_can_std((uint32_t)(CAN_ID_DEYE_PACK_SER_BASE + idx), data, 8);
}

static void send_deye_per_pack_frames(void) {
  static uint32_t tx_cycle = 0;
  static uint8_t slow_cursor = 0;
  tx_cycle++;

  uint8_t data[8];
  uint8_t online_indices[HW_MODULE_COUNT];
  uint8_t online_count = 0;

  for (uint8_t i = 0; i < HW_MODULE_COUNT; i++) {
    if (hw_mod[i].last_ok_ms == 0 || !(pack_ever_seen_mask & (1u << i)) ||
        pack_miss_count[i] > 5)
      continue;
    online_indices[online_count++] = i;
    const HwModule *m = &hw_mod[i];
    bool protect_active = ((m->alarm[0] & 0x7E) != 0) || ((m->alarm[4] & 0x0003) != 0);

    // 0x110+i: Pack relay/status flag (byte 7: 0x01=online | 0x10=CHG_OK | 0x20=DSG_OK; 0x00=protect)
    memset(data, 0, 8);
    data[7] = protect_active
                  ? 0x00U
                  : (uint8_t)(0x01U | (!charge_cell_ov_inhibit && !charge_temp_inhibit ? 0x10U : 0U) |
                              (!discharge_cell_uv_inhibit && !discharge_temp_inhibit ? 0x20U : 0U));
    if (!send_can_std((uint32_t)(CAN_ID_DEYE_PACK_STATUS_BASE + i), data, 8))
      return;

    // 0x150+i: V (0.1 V), I (0.1 A, discharge positive / charge negative), SOC (0.1%), SOH (0.1%)
    uint16_t v_01v = (uint16_t)round_to_i16(m->pack_v * 10.0f);
    int16_t cur_01a = (int16_t)round_to_i16(-m->current * 10.0f);
    put_u16le(&data[0], v_01v);
    put_u16le(&data[2], (uint16_t)cur_01a);
    put_u16le(&data[4], (uint16_t)m->soc * 10U);
    put_u16le(&data[6], (uint16_t)m->soh * 10U);
    if (!send_can_std((uint32_t)(CAN_ID_DEYE_PACK_VI_BASE + i), data, 8))
      return;

    // 0x200+i: CellMax (mV), CellMin (mV), Tmax (0.1 C), Tmin (0.1 C)
    put_u16le(&data[0], m->cell_max);
    put_u16le(&data[2], m->cell_min);
    put_u16le(&data[4], (uint16_t)(m->t_max * 10));
    put_u16le(&data[6], (uint16_t)(m->t_min * 10));
    if (!send_can_std((uint32_t)(CAN_ID_DEYE_PACK_CELL_BASE + i), data, 8))
      return;

    // 0x400+i: Cycle count
    memset(data, 0, 8);
    data[0] = 0x02U;
    put_u16le(&data[2], (uint16_t)(m->cycles > 0xFFFFU ? 0xFFFFU : m->cycles));
    data[6] = 0x09U;
    if (!send_can_std((uint32_t)(CAN_ID_DEYE_PACK_CYCLES_BASE + i), data, 8))
      return;
  }

  if (online_count > 0 && (tx_cycle % 5U) == 1U) {
    if (slow_cursor >= online_count)
      slow_cursor = 0;
    send_deye_pack_identity(online_indices[slow_cursor]);
    slow_cursor = (uint8_t)((slow_cursor + 1U) % online_count);
  }
}
#endif

static void send_all_frames(void) {
  gpio_bit_set(LED_PCSCAN_PORT, LED_PCSCAN_PIN); // LED4: CAN activity
  bool can_ok = true;
  uint8_t data[8] = {0};
  put_u16le(&data[0], charge_voltage_limit);
  put_u16le(&data[2], charge_current_limit);
  put_u16le(&data[4], discharge_current_limit);
  put_u16le(&data[6], DISCHARGE_VOLTAGE_LIMIT_X10);
  can_ok = send_can_std(CAN_ID_LIMITS, data, 8) && can_ok;

  memset(data, 0, 8);
  // Deye off-grid workaround (Battery-Emulator BYD-CAN.cpp): оф-грід Deye
  // ігнорує CCL/DCL і розряджає до SOC=0. Дзеркалити ліміти в SOC:
  // DCL=0 (розряд заборонено / порожній) → SOC=0; CCL=0 (заряд заборонено /
  // повний) → SOC=100. Обидва 0 (нема даних) → SOC=0 (чесний «не запрошуй»).
  uint8_t can_soc = bms_soc;
  if (discharge_current_limit == 0)
    can_soc = 0;
  else if (charge_current_limit == 0)
    can_soc = 100;
  put_u16le(&data[0], can_soc);
  put_u16le(&data[2], bms_soh);
  can_ok = send_can_std(CAN_ID_SOC, data, 4) && can_ok;

  memset(data, 0, 8);
  put_u16le(&data[0], (uint16_t)round_to_i16(bms_voltage * 100.0f));
  put_u16le(&data[2], (uint16_t)round_to_i16(bms_current * 10.0f));
  put_u16le(&data[4], (uint16_t)round_to_i16(bms_temp * 10.0f));
  can_ok = send_can_std(CAN_ID_STATUS, data, CAN_PROTOCOL_DEYE_V33 ? 8 : 6) && can_ok;

  memset(data, 0, 8);
#if CAN_PROTOCOL_DEYE_V33
  // 0x359 Deye PCS CAN V3.3: Bytes 0..3 Protection Tables 1-4, Bytes 4..5 Alarm Tables 5-6, Byte 6 Error Table 7
  data[0] = can_dy_p0;
  data[1] = can_dy_p1;
  data[2] = can_dy_p2;
  data[3] = can_dy_p3;
  data[4] = can_dy_a4;
  data[5] = can_dy_a5;
  data[6] = can_dy_e6;
  data[7] = 0;
  can_ok = send_can_std(CAN_ID_PROTECT, data, 8) && can_ok;
#else
  // 0x359: Bytes 0..1 Protection, Bytes 2..3 Alarm, Byte 4 Module count, Bytes 5..6 'P' 'N', Byte 7 0
  data[0] = can_protect_b0;
  data[1] = can_protect_b1;
  data[2] = can_alarm_b2;
  data[3] = can_alarm_b3;
  data[4] = bms_pack_count;
  data[5] = 'P';
  data[6] = 'N';
  data[7] = 0;
  can_ok = send_can_std(CAN_ID_PROTECT, data, 8) && can_ok;
#endif

  // 0x35C: Request / Enable flags
  // Bit 7 (0x80) = Charge enable: active only if data valid, not inhibited, CCL > 0
  // Bit 6 (0x40) = Discharge enable: active only if data valid, not inhibited, DCL > 0
  memset(data, 0, 8);
  uint8_t req_flags = 0x00;
  if (bms_data_valid) {
    if (!charge_cell_ov_inhibit && !charge_temp_inhibit && charge_current_limit > 0)
      req_flags |= 0x80;
    if (!discharge_cell_uv_inhibit && !discharge_temp_inhibit && discharge_current_limit > 0)
      req_flags |= 0x40;
  }
  data[0] = req_flags;
  can_ok = send_can_std(CAN_ID_FLAGS, data, CAN_PROTOCOL_DEYE_V33 ? 8 : 2) && can_ok;

#if CAN_PROTOCOL_DEYE_V33
  memset(data, 0, 8);
  data[0] = 'D';
  data[1] = 'Y';
  data[2] = '0';
  data[3] = '0';
  data[4] = '1';
  data[5] = bms_pack_count;
  uint32_t total_cap_01ah = (uint32_t)bms_pack_count * 1000U; // ESM-48100 = 100Ah (1000 * 0.1Ah)
  if (total_cap_01ah > 0xFFFFU)
    total_cap_01ah = 0xFFFFU;
  put_u16le(&data[6], (uint16_t)total_cap_01ah);
  can_ok = send_can_std(CAN_ID_BRAND, data, 8) && can_ok;

  memset(data, 0, 8);
  uint16_t cell_max = bms_data_valid ? bank_max_cell_mv : 0;
  uint16_t cell_min = bms_data_valid ? bank_min_cell_mv : 0;
  float t_max = bms_data_valid ? bank_temp_max : BATTERY_TEMP_DEFAULT;
  float t_min = bms_data_valid ? bank_temp_min : BATTERY_TEMP_DEFAULT;
  put_u16le(&data[0], cell_max);
  put_u16le(&data[2], cell_min);
  put_u16le(&data[4], (uint16_t)round_to_i16(t_max * 10.0f));
  put_u16le(&data[6], (uint16_t)round_to_i16(t_min * 10.0f));
  can_ok = send_can_std(CAN_ID_CELL_EXTREMES, data, 8) && can_ok;

  static const uint8_t deye_cfg[8] = {0x20, 0x08, 0x21, 0x02, 0x00, 0x00, 0x00, 0x00};
  can_ok = send_can_std(CAN_ID_DEYE_CFG, deye_cfg, 8) && can_ok;

  memset(data, 0, 8);
  data[0] = bms_pack_count;
  data[1] = (!charge_cell_ov_inhibit && !charge_temp_inhibit && charge_current_limit > 0) ? 0 : bms_pack_count;
  data[2] = (!discharge_cell_uv_inhibit && !discharge_temp_inhibit && discharge_current_limit > 0) ? 0 : bms_pack_count;
  data[3] = 0;
  data[4] = bms_pack_count;
  can_ok = send_can_std(CAN_ID_DEYE_PACK_COUNT, data, 8) && can_ok;

  memset(data, 0, 8);
  put_u16le(&data[0], charge_current_limit);
  put_u16le(&data[2], discharge_current_limit);
  can_ok = send_can_std(CAN_ID_GRID_LIMITS, data, 8) && can_ok;

  watchdog_feed();
  if (bms_data_valid && can_ok) {
    send_deye_per_pack_frames();
  }
#else
  static const uint8_t brand[8] = {'P', 'Y', 'L', 'O', 'N', 0, 0, 0};
  send_can_std(CAN_ID_BRAND, brand, 8);
#endif

  can_led_off_ms = millis() + 100; // keep the LED on briefly after TX
}

/* FWDGT — незалежний watchdog (LSI). SPL-джерела gd32f30x_fwdgt.c у проєкті нема,
 * тому регістри пишуться напряму. FWDGT_PSC_DIV256 + RLD 400 →
 * ≈3.13 с при LSI 32.768 кГц або ≈2.56 с при 40 кГц — запас над найгіршим
 * проходом циклу (Huawei-прийом ≤1 с + CAN ≤40 мс). Після старту не вимикається. */
static void watchdog_init(void) {
  uint32_t spin;
  FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;
  FWDGT_PSC = FWDGT_PSC_DIV256;
  for (spin = FWDGT_PSC_TIMEOUT; spin && (FWDGT_STAT & FWDGT_STAT_PUD); spin--)
    ;
  FWDGT_RLD = 400U;
  for (spin = FWDGT_RLD_TIMEOUT; spin && (FWDGT_STAT & FWDGT_STAT_RUD); spin--)
    ;
  FWDGT_CTL = FWDGT_KEY_RELOAD;
  FWDGT_CTL = FWDGT_KEY_ENABLE;
}

static void watchdog_feed(void) { FWDGT_CTL = FWDGT_KEY_RELOAD; }

static uint8_t dip_read_code(void) {
  uint8_t code = (uint8_t)((~GPIO_ISTAT(DIP_GPIO_PORT) >> 12) & 0x0FU);
  if (!(GPIO_ISTAT(DIP5_GPIO_PORT) & DIP5_GPIO_PIN)) code |= 0x10;
  return code;
}

static void dip_charge_profile_apply(uint8_t code) {
  if (code & DIP_BALANCE_54V) {
    profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_54V;
    profile_taper_start_mv = CELL_TAPER_START_MV;
    profile_charge_stop_mv = 3600;
    profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
  } else if (code & DIP_PROFILE_CODE_90) {
    profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_DIP90;
    profile_taper_start_mv = CELL_TAPER_START_MV_DIP90;
    profile_charge_stop_mv = CELL_CHARGE_STOP_MV_DIP90;
    profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV_DIP90;
  } else {
    profile_charge_voltage_limit = CHARGE_VOLTAGE_LIMIT_STATIC;
    profile_taper_start_mv = CELL_TAPER_START_MV;
    profile_charge_stop_mv = CELL_CHARGE_STOP_MV;
    profile_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
  }
  profile_charge_current_per_pack_x10 =
      (code & DIP_CHARGE_CURRENT_15A) ? CHARGE_CURRENT_LIMIT_PER_PACK_DIP4_ON_X10
                                      : CHARGE_CURRENT_LIMIT_PER_PACK_X10;
  charge_voltage_limit = profile_charge_voltage_limit;
  if (!bal_session_active || (code & DIP_BALANCE_54V)) {
    calc_taper_start_mv = profile_taper_start_mv;
    calc_charge_stop_mv = profile_charge_stop_mv;
    calc_charge_reenable_mv = profile_charge_reenable_mv;
  }
  dip_applied_code = code;
  pcs_mode_apply(code); // DIP2: mirror (OFF) vs inverter bridge (ON)
}

static void dip_charge_profile_init(void) {
  rcu_periph_clock_enable(DIP_GPIO_RCU);
  gpio_init(DIP_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, DIP_GPIO_PINS);
  rcu_periph_clock_enable(DIP5_GPIO_RCU);
  gpio_init(DIP5_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_2MHZ, DIP5_GPIO_PIN);
  dip_last_code = dip_read_code();
  dip_stable_reads = 2;
  dip_charge_profile_apply(dip_last_code);
}

static void dip_charge_profile_poll(void) {
  uint8_t code = dip_read_code();
  if (code != dip_last_code) {
    dip_last_code = code;
    dip_stable_reads = 0;
    return;
  }
  if (dip_stable_reads < 2)
    dip_stable_reads++;
  if (dip_stable_reads >= 2 && code != dip_applied_code)
    dip_charge_profile_apply(code);
}

int main(void) {
  SystemInit();
  if (SysTick_Config(SystemCoreClock / 1000U))
    while (1)
      ;
  dip_charge_profile_init();
  leds_init();
  rs485_init();
  // §4.2: старт скану Huawei-адрес не раніше ~2 с після старту UART
  // Старт опиту Huawei через 500 мс після старту UART
  hw_uart_ready_ms = millis() + 500;
  pcs_uart_init(); // PCS-485 port (mirror/bridge, mode from DIP2)
  can_init_500k();
  watchdog_init();

  uint32_t last_dip_poll = 0;

  while (1) {
    watchdog_feed();
    uint32_t now = millis();
    can_rx_service(now);
    pcs_poll_service(now); // mirror drain / bridge RX every pass
    if (now - last_dip_poll >= 250) {
      last_dip_poll = now;
      dip_charge_profile_poll();
      pcs_mode_apply(dip_applied_code); // retry a DIP2 switch deferred by a busy bridge
      balance_led_tick(now);
    }
    // Власний опит Huawei: опитування строго двох акумуляторів (0xD6 та 0xD7);
    // DIP2 ON всередині hw_poll_service() — це міст (без власних
    // команд; для промислової роботи DIP2 завжди вимкнено — дзеркало).
    hw_poll_service(now);

    // Pure bridge (DIP2 ON): relay inverter <-> BMS and keep CAN fed from the
    // same traffic (parse happens inside the step; aggregates close per sweep).
    if (pcs_bridge_mode) {
      pcs_bridge_step(now);
      bridge_sweep_tick(now);
    }

    if (millis() - last_bms_response_ms > BMS_DEFAULT_TIMEOUT_MS) {
      balance_on_bms_lost(); // an auto session cannot run blind
      agg_reset(); // не змішувати старі накопичення з новими після відновлення
      bms_data_valid = false;
      bms_voltage = BATTERY_VOLTAGE_DEFAULT;
      bms_current = BATTERY_CURRENT_DEFAULT;
      bms_soc = BATTERY_SOC_DEFAULT;
      bms_soh = BATTERY_SOH_DEFAULT;
      bms_temp = BATTERY_TEMP_DEFAULT;
      bank_max_cell_mv = 0;
      bank_min_cell_mv = 0;
      bank_temp_max = BATTERY_TEMP_DEFAULT;
      bank_temp_min = BATTERY_TEMP_DEFAULT;
      charge_voltage_limit = profile_charge_voltage_limit;
      charge_current_limit = CHARGE_CURRENT_LIMIT_DEFAULT;
      discharge_current_limit = DISCHARGE_CURRENT_LIMIT_DEFAULT;
      charge_cell_ov_inhibit = false;
      charge_temp_inhibit = false;
      discharge_cell_uv_inhibit = false;
      discharge_temp_inhibit = false;
      can_protect_b0 = 0;
      can_protect_b1 = 0;
      can_alarm_b2 = 0;
      can_alarm_b3 = 0;
      sweep_protect_b0 = 0;
      sweep_protect_b1 = 0;
      sweep_alarm_b2 = 0;
      sweep_alarm_b3 = 0;
      can_dy_p0 = can_dy_p1 = can_dy_p2 = can_dy_p3 = 0;
      can_dy_a4 = can_dy_a5 = can_dy_e6 = 0;
      sweep_dy_p0 = sweep_dy_p1 = sweep_dy_p2 = sweep_dy_p3 = 0;
      sweep_dy_a4 = sweep_dy_a5 = sweep_dy_e6 = 0;
    }

    // Fallback keepalive: якщо зв'язок RS485 раптом замовк більше ніж на 2000 мс (обрив),
    // періодично підтримуємо CAN шину.
    // За нормальної роботи CAN-кадри надсилаються з bms_aggregate_data() одразу
    // після завершення опитування та агрегації даних від обох акумуляторів.
    if (now - last_can >= 2000) {
      last_can = now;
      if (bms_data_valid || (millis() - last_bms_response_ms > BMS_DEFAULT_TIMEOUT_MS)) {
        send_all_frames();
      }
    }

    // LED4 (PC5, CAN): release ~100 ms after the last CAN burst.
    if (can_led_off_ms && (int32_t)(millis() - can_led_off_ms) >= 0) {
      gpio_bit_reset(LED_PCSCAN_PORT, LED_PCSCAN_PIN);
      can_led_off_ms = 0;
    }

    // LED3 (PC4, PCS-485): release ~100 ms after the last mirror/bridge byte.
    if (pcs_led_off_ms && (int32_t)(millis() - pcs_led_off_ms) >= 0) {
      gpio_bit_reset(LED_INV485_PORT, LED_INV485_PIN);
      pcs_led_off_ms = 0;
    }
  }
}
