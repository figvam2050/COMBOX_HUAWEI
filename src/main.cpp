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
 * BMS 485 to CAN Converter for Deye Inverter (Lithium Mode 00 - Deye generic
 * CAN / Pylontech). Hardware: GD32F305 + ISO1050 (CAN) + CA-IS3080WX (RS485)
 *
 * COMBOX_HUAWEI: адаптація для промислових батарей Huawei ESM
 * (ESM-48100 / ESM-48150, Modbus RTU RS485: адреси 0xD6–0xDD / 0xE0–0xE7).
 * База знань та реєстри — HUAWEI_REFERENCE.md (§5, §6, §7).
 * Вихідний CAN-бік (Pylontech для Deye), fail-closed, балансування,
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
// Таймаут відповіді: 400 мс (як в інверторі Deye).
#define BMS_TIMEOUT_MS 400
#define BMS_DEFAULT_TIMEOUT_MS 5000

// --- Huawei ESM: опитування до 16 акумуляторів (0xD6–0xDD, 0xE0–0xE7 = 214–221, 224–231) ---
#define HW_MODULE_COUNT 16
static const uint8_t HW_ADDR[HW_MODULE_COUNT] = {
    0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD,
    0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7};

#define HW_PROBE_PERIOD_MS 1500 // пауза між спробами скану відсутньої адреси
#define HW_INTER_QUERY_MS 30    // ≥10 мс, рекомендовано 20–50 мс (§3, §7.3)
#define HW_INTER_MODULE_MS 60   // пауза між модулями (§4.1: D7 мовчить, якщо лізти одразу після D6)

typedef struct {
  uint16_t reg;
  uint8_t qty;
} HwQuery;

enum HwQueryId : uint8_t {
  HW_Q_WAKE = 0, // 0x0000 ×1: wake-up/проба наявності (§4.1) — лише для скану
  HW_Q_BASE,     // 0x0000 ×7: bus V, pack V, струм, SOC, SOH, t max, t min
  HW_Q_STATE,    // 0x000A ×1: стан модуля (0 Charge … 3 Free)
  HW_Q_CYCLES,   // 0x0042 ×2: цикли розряду, U32 BE
  HW_Q_AH,       // 0x0044 ×2: накопичений розряд Ah, U32 BE
  HW_Q_ALARM,    // 0x0046 ×5: аварійні слова 0x0046..0x004A
  HW_Q_NCELL,    // 0x010F ×1: кількість комірок
  HW_Q_TEMP,     // 0x0012 ×15: температури комірок S16 °C
  HW_Q_CELL,     // 0x0022 ×15: напруги комірок U16 мВ
  HW_Q_COUNT
  // 0x100B/0x100D (ліміти ×0.001 C) — відкладено (HUAWEI_REFERENCE §8, Q5);
  // CCL/DCL на цьому етапі задаються профілями від DIP (40 А / 50 А на модуль).
};

static const HwQuery HW_QUERIES[HW_Q_COUNT] = {
    {0x0000, 7}, {0x0000, 7}, {0x000A, 1}, {0x0042, 2}, {0x0044, 2},
    {0x0046, 5}, {0x010F, 1}, {0x0012, 15}, {0x0022, 15},
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

#define CHARGE_VOLTAGE_LIMIT_STATIC 540
#define CHARGE_VOLTAGE_LIMIT_DIP90 518
// Per-pack charge current limit, selected by DIP5 (see DIP_CHARGE_CURRENT_50A):
// OFF = 40 A (soft margin), ON = 50 A (battery manufacturer maximum).
#define CHARGE_CURRENT_LIMIT_PER_PACK_DIP5_OFF_X10 400
#define CHARGE_CURRENT_LIMIT_PER_PACK_X10 500
#define DISCHARGE_CURRENT_LIMIT_PER_PACK_X10 1000
#define DISCHARGE_VOLTAGE_LIMIT_X10 450

// Max-cell charge taper: 50 A/pack to 0 A between 3.450 V and 3.600 V.
// Re-enabling below 3.550 V prevents charge-limit oscillation at the cutoff.
#define CELL_TAPER_START_MV 3450
#define CELL_CHARGE_STOP_MV 3600
#define CELL_CHARGE_REENABLE_MV 3550
#define CELL_TAPER_START_MV_DIP90 3400
#define CELL_CHARGE_STOP_MV_DIP90 3455
#define CELL_CHARGE_REENABLE_MV_DIP90 3405
// 90% profile = SW1 ON (weight 1 => code 0b00001), moved from SW4 on 17.10.2026
// (convenient switch); stock SW1 protocol meaning (PYLON) is not used by us.
#define DIP_PROFILE_CODE_90 1
// DIP5 (SW5, weight 16, PC6) selects the per-pack charge current limit:
// OFF = 40 A, ON = 50 A. Orthogonal to the SW1 voltage profile.
#define DIP_CHARGE_CURRENT_50A 0x10

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

// ===== Balance session engine (17.10.2026) =====
// DIP1 OFF (standard): CVL 54.0 V; auto balance windows (BAL_AUTO_ENABLED)
// open on cell-spread analysis: full thresholds + 2 A per pack cap.
// DIP1 ON (51.8 V option): auto logic disabled, profile holds 51.8 V with
// the same 2 A trickle from 3400 mV (~90%) so passive balancing has gentle
// CV time.
#define BAL_AUTO_ENABLED 1 // verdict (A) 18.10.2026: weak cell #7 confirmed
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
#define BAL_OUTAGE_AGG_N 19        // ~60 s at one aggregate per 3.2 s (full poll cycle)
#define BAL_EXIT_STREAK 3          // consecutive aggregates with spread <= exit
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

// RS485 register units -> physical units. Confirm against the battery model.
// reg1 current: 1 = BMS reports charge as positive (negated for Deye CAN),
//               0 = BMS reports discharge as positive (passed through).
#define BMS_CURRENT_CHARGE_IS_POSITIVE 1
// reg19 temperature raw divisor: 1 = reg 0x13 holds whole C (dump 08.10.2026).
#define BMS_TEMP_RAW_DIV 1.0f

#define CAN_ID_LIMITS 0x351
#define CAN_ID_SOC 0x355
#define CAN_ID_STATUS 0x356
#define CAN_ID_PROTECT 0x359
#define CAN_ID_FLAGS 0x35C
#define CAN_ID_BRAND 0x35E

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
//   LED1 (PC13) = free/debug — dark;
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

static float agg_voltage_sum = 0.0f;
static float agg_current_sum = 0.0f;
static uint32_t agg_soc_sum = 0;
static uint32_t agg_soh_sum = 0;
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

// Multi-pack debounce (Q21): tolerate 10-20% RS485 packet loss without flapping limits
static uint8_t pack_miss_count[16] = {0};
static uint16_t pack_ever_seen_mask = 0x0003; // D6 (214) та D7 (215) відомі одразу від старту
static uint16_t sweep_seen_mask = 0;

// --- Huawei ESM: буфер даних модулів (індекс 0..15 ↔ HW_ADDR) --------------
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
  uint8_t state;         // 0x000A: 0 Charge, 1 Discharge, 2 Full, 3 Free
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

static HwModule hw_mod[16];
static uint16_t hw_base_seen_mask = 0;  // базові суми цього sweep уже погоджені
static uint16_t hw_alarm_seen_mask = 0; // свіжі аварійні слова для мапінгу 0x359
static uint8_t agg_soh_min = 100;       // §6: SOH агрегату = мінімум по модулях

// Машина станів опиту: адреса × запит; скан відсутніх — ≥3 с на адресу (§4.1),
// старт скану — +2 с після старту UART (§4.2).
static uint8_t hw_addr_idx = 0;
static uint8_t hw_q_idx = 0;
static uint32_t hw_next_slot_ms = 0;
static uint32_t hw_probe_ms[16] = {0}; // 0 = ще не пробували
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
static bool bms_parse_response(uint8_t expected_addr); // Legacy 83-byte frame (DIP2 bridge test path)
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
      // Feed CAN/balance from the bridged exchange: in bridge mode the box
      // sends no own polls, so this is the ONLY data source. Keeps
      // last_bms_response_ms fresh -> fail-closed still trips when the
      // inverter stops polling.
      if (bms_parse_response(uart_rx_buffer[0]))
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

static bool bms_parse_response(uint8_t expected_addr) {
  if (expected_addr < 1 || expected_addr > 16)
    return false; // not one of our pack addresses (bridge forwards anything)
  if (uart_rx_index < 5)
    return false;
  if (uart_rx_buffer[0] != expected_addr || uart_rx_buffer[1] != 0x03)
    return false;
  // Byte count: our own window is 0x4E (39 regs, 83 B); a bridged inverter
  // frame (Deye) asks 0x38 (28 regs, 61 B). Other windows (e.g. reg 0x50)
  // are not ours — they stay transparent and are never aggregated.
  uint16_t data_bytes = uart_rx_buffer[2];
  if (data_bytes != 0x4E && data_bytes != 0x38)
    return false;
  uint16_t frame_len = (uint16_t)(5 + data_bytes);
  if (uart_rx_index < frame_len)
    return false;

  uint16_t received_crc = (uart_rx_buffer[frame_len - 1] << 8) |
                          uart_rx_buffer[frame_len - 2];
  uint16_t calculated_crc =
      modbus_crc16((uint8_t *)uart_rx_buffer, frame_len - 2);
  if (received_crc != calculated_crc)
    return false;

  uint16_t raw_voltage = (uart_rx_buffer[3] << 8) | uart_rx_buffer[4];
  int16_t raw_current = (int16_t)((uart_rx_buffer[5] << 8) | uart_rx_buffer[6]);
  uint16_t raw_soc = (uart_rx_buffer[45] << 8) | uart_rx_buffer[46];
  uint16_t raw_soh = (uart_rx_buffer[47] << 8) | uart_rx_buffer[48];

  for (uint8_t cell = 0; cell < 15; cell++) {
    uint16_t cell_mv = ((uint16_t)uart_rx_buffer[7 + cell * 2] << 8) |
                       uart_rx_buffer[8 + cell * 2];
    if (cell_mv > agg_max_cell_mv)
      agg_max_cell_mv = cell_mv;
    if (cell_mv > 0 && cell_mv < agg_min_cell_mv)
      agg_min_cell_mv = cell_mv;
  }

  // Multi-sensor temperature tracking: regs 18, 19, 20 (bytes 39..44)
  // Sanity check all sensor channels (-40..120 °C) to reject disconnected (0x7FFF) or corrupted probes:
  int16_t t_a = (int16_t)((uart_rx_buffer[39] << 8) | uart_rx_buffer[40]);
  int16_t t_main = (int16_t)((uart_rx_buffer[41] << 8) | uart_rx_buffer[42]);
  int16_t t_b = (int16_t)((uart_rx_buffer[43] << 8) | uart_rx_buffer[44]);

  float pack_t_max = -1000.0f;
  float pack_t_min = 1000.0f;
  if (t_main >= -40 && t_main <= 120) {
    pack_t_max = (float)t_main;
    pack_t_min = (float)t_main;
  }
  if (t_a >= -40 && t_a <= 120) {
    if (pack_t_max < -500.0f || (float)t_a > pack_t_max) pack_t_max = (float)t_a;
    if (pack_t_min > 500.0f || (float)t_a < pack_t_min) pack_t_min = (float)t_a;
  }
  if (t_b >= -40 && t_b <= 120) {
    if (pack_t_max < -500.0f || (float)t_b > pack_t_max) pack_t_max = (float)t_b;
    if (pack_t_min > 500.0f || (float)t_b < pack_t_min) pack_t_min = (float)t_b;
  }
  if (pack_t_max < -500.0f) {
    pack_t_max = BATTERY_TEMP_DEFAULT;
    pack_t_min = BATTERY_TEMP_DEFAULT;
  }
  if (pack_t_max > agg_temp_max)
    agg_temp_max = pack_t_max;
  if (pack_t_min < agg_temp_min)
    agg_temp_min = pack_t_min;

  // Status, Warning, Protection (regs 25..27, bytes 53..58): present in both
  // accepted windows (data_bytes >= 56). Error (reg 28, bytes 59..60) exists
  // only in the full 39-reg window — the 28-reg inverter window ends at reg 27.
  uint16_t reg_status  = ((uint16_t)uart_rx_buffer[53] << 8) | uart_rx_buffer[54];
  uint16_t reg_warning = ((uint16_t)uart_rx_buffer[55] << 8) | uart_rx_buffer[56];
  uint16_t reg_protect = ((uint16_t)uart_rx_buffer[57] << 8) | uart_rx_buffer[58];
  uint16_t reg_error   = 0;
  if (data_bytes >= 58)
    reg_error = ((uint16_t)uart_rx_buffer[59] << 8) | uart_rx_buffer[60];

  // Cross-validate with physical measurements to reject transient Boot Latch flags (Q17):
  // Confirmation uses physical threshold CELL_PHYSICAL_OV_CONFIRM_MV (3600 mV), NOT calc_charge_stop_mv,
  // so that activating the 90% software taper profile never generates false emergency cell OV alarms:
  if ((reg_protect & 0x0002) && agg_max_cell_mv >= CELL_PHYSICAL_OV_CONFIRM_MV)
    sweep_protect_b0 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
  if ((reg_protect & 0x0008) && (agg_min_cell_mv <= CELL_UV_WARN_MV || (raw_soc & 0xFF) <= 5))
    sweep_protect_b0 |= CAN_PROT_CELL_UV;
  if ((reg_protect & 0x0001) && raw_voltage >= 5450)
    sweep_protect_b0 |= CAN_PROT_PACK_OV;
  if ((reg_protect & 0x0004) && raw_voltage <= 4320)
    sweep_protect_b0 |= CAN_PROT_CELL_UV;
  if ((reg_protect & 0x0300) || pack_t_max >= TEMP_DISCHARGE_MAX_C)
    sweep_protect_b0 |= CAN_PROT_OT;
  // System-level UT protection stops both charge and discharge in Deye; trigger only if battery
  // is under the physical discharge limit (-20°C) or BMS asserts protection (charge limit handled via CCL=0):
  if ((reg_protect & 0x0C00) || pack_t_min <= TEMP_DISCHARGE_MIN_C)
    sweep_protect_b0 |= CAN_PROT_UT;
  if (reg_protect & 0x0020)
    sweep_protect_b0 |= CAN_PROT_DISCHARGE_OC;

  if (reg_protect & 0x0010)
    sweep_protect_b1 |= CAN_PROT_CHARGE_OC;
  if ((reg_protect & 0x2000) || (reg_status & 0x0004) || reg_error != 0)
    sweep_protect_b1 |= CAN_PROT_SYSTEM_ERROR;

  // Mirror confirmed warnings into CAN alarm bytes 2 and 3:
  if ((reg_warning & 0x0002) && agg_max_cell_mv >= CELL_PHYSICAL_OV_WARN_CONFIRM_MV)
    sweep_alarm_b2 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
  if ((reg_warning & 0x0008) && (agg_min_cell_mv <= CELL_UV_WARN_MV || (raw_soc & 0xFF) <= 10))
    sweep_alarm_b2 |= CAN_PROT_CELL_UV;
  if ((reg_warning & 0x0001) && raw_voltage >= 5400)
    sweep_alarm_b2 |= CAN_PROT_PACK_OV;
  if ((reg_warning & 0x0004) && raw_voltage <= 4400)
    sweep_alarm_b2 |= CAN_PROT_CELL_UV;
  if ((reg_warning & 0x0300) || pack_t_max >= 50.0f)
    sweep_alarm_b2 |= CAN_PROT_OT;
  if ((reg_warning & 0x0C00) || pack_t_min <= (TEMP_DISCHARGE_MIN_C + 5.0f))
    sweep_alarm_b2 |= CAN_PROT_UT;
  if (reg_warning & 0x0020)
    sweep_alarm_b2 |= CAN_PROT_DISCHARGE_OC;
  if (reg_warning & 0x0010)
    sweep_alarm_b3 |= CAN_PROT_CHARGE_OC;

  bool dup = (sweep_seen_mask & (1 << (expected_addr - 1))) != 0;
  sweep_seen_mask |= (1 << (expected_addr - 1));

  // Feed the sums once per address per sweep: in bridge mode the inverter may
  // repeat an address and would bias the averages toward it. Cells/temps and
  // the |= latches above stay idempotent, so only the sums are gated.
  if (!dup) {
    agg_voltage_sum += raw_voltage / 100.0f;
    agg_current_sum += raw_current / 100.0f;
    agg_soc_sum += (uint8_t)(raw_soc & 0xFF);
    agg_soh_sum += (uint8_t)(raw_soh & 0xFF);
    if ((uint8_t)(raw_soh & 0xFF) < agg_soh_min)
      agg_soh_min = (uint8_t)(raw_soh & 0xFF); // єдине правило §6 для обох шляхів
    agg_count++;
  }

  return true;
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
  case HW_Q_WAKE:
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
    // Базові суми — один раз на sweep на модуль (hw_base_seen_mask, бо
    // sweep_seen_mask виставляється вже на першому успішному запиті).
    if (!(hw_base_seen_mask & (1u << addr_idx))) {
      hw_base_seen_mask |= (1u << addr_idx);
      agg_voltage_sum += m->pack_v; // §6: середнє по модулях
      agg_current_sum += m->current; // §6: сума; +заряд — CAN інвертує знак
      agg_soc_sum += soc;           // §6: середнє (однакова ємність)
      agg_soh_sum += soh;
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

  case HW_Q_STATE:
    m->state = (uint8_t)((uint16_t)(d[0] << 8) | d[1]); // 0..3, §7.2 п.4
    break;

  case HW_Q_CYCLES:
    m->cycles = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                ((uint32_t)d[2] << 8) | d[3]; // U32 BE, §5.3
    break;

  case HW_Q_AH:
    m->ah = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
            ((uint32_t)d[2] << 8) | d[3]; // U32 BE, §5.3
    break;

  case HW_Q_ALARM:
    for (uint8_t i = 0; i < 5; i++)
      m->alarm[i] = (uint16_t)(d[i * 2] << 8) | d[i * 2 + 1]; // 0x0046..0x004A
    hw_alarm_seen_mask |= (1u << addr_idx); // мапінг у 0x359 — у aggregate()
    break;

  case HW_Q_NCELL:
    m->ncell = (uint8_t)((uint16_t)(d[0] << 8) | d[1]); // 15 у ESM-48100
    break;

  case HW_Q_TEMP: // 0x0012 ×15: S16 °C, саніті -40..120 (відсікає 0x7FFF)
    for (uint8_t i = 0; i < 15; i++) {
      int16_t t = (int16_t)((uint16_t)(d[i * 2] << 8) | d[i * 2 + 1]);
      m->cell_temp[i] = t;
      if (t >= -40 && t <= 120) {
        if ((float)t > agg_temp_max)
          agg_temp_max = (float)t;
        if ((float)t < agg_temp_min)
          agg_temp_min = (float)t;
      }
    }
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

  default:
    return 0;
  }

  return 1;
}

static bool probed_unknown_this_sweep = false;

static void hw_advance_addr(void) {
  hw_q_idx = HW_Q_BASE;
  if (++hw_addr_idx >= HW_MODULE_COUNT) {
    hw_addr_idx = 0;
    probed_unknown_this_sweep = false;
    bms_aggregate_data(); // sweep закрито: агрегація + мапінг аварій (§6)
  }
}

// Машина станів опиту Huawei ESM (неблокуючий FSM):
//  - модуль із pack_ever_seen_mask — повна серія HW_QUERIES, пауза
//    HW_INTER_QUERY_MS між кадрами (§7.3: 20–50 мс);
//  - невідома адреса — тільки скан-проба wake-up не частіше
//    HW_PROBE_PERIOD_MS (≥3 с, §4.1), старт — після hw_uart_ready_ms (§4.2);
//  - будь-яка валідна відповідь (дані чи виняток) оновлює
//    last_bms_response_ms (fail-closed Q5) і sweep_seen_mask (присутність);
//  - таймаут/битий кадр → перехід на НАСТУПНУ АДРЕСУ: мертвий модуль з'їдає
//    не більше ніж один слот на sweep, а 3-miss debounce (Q21) вилучає його
//    приблизно за 15 с (§6) за типового темпу sweep.
static void hw_poll_service(uint32_t now) {
  if (pcs_bridge_mode)
    return; // DIP2 ON: чистий міст, жодних власних команд на шині
  if ((int32_t)(now - hw_uart_ready_ms) < 0)
    return; // §4.2: скан не раніше ~2 с після старту UART
  if ((int32_t)(now - hw_next_slot_ms) < 0)
    return; // міжкадрова пауза

  uint8_t idx = hw_addr_idx;

  if (!(pack_ever_seen_mask & (1u << idx))) {
    // --- Невідома адреса: скан-проба (wake-up = FC03 0x0000 ×1, §4.1) ---
    // Якщо у цьому sweep вже пробувалась невідома адреса або пауза ще не минула —
    // одразу пропускаємо без затримки, щоб не гальмувати зв'язок з активними модулями.
    if (probed_unknown_this_sweep ||
        (hw_probe_ms[idx] != 0 && (now - hw_probe_ms[idx]) < HW_PROBE_PERIOD_MS)) {
      hw_advance_addr(); // щойно пробувалась або вже була проба у цьому sweep — далі
      hw_next_slot_ms = now; // без зайвої паузи
      return;
    }
    probed_unknown_this_sweep = true;
    hw_probe_ms[idx] = now;
    hw_send_query(idx, HW_Q_WAKE);
    bms_receive_response(HW_QUERIES[HW_Q_WAKE].qty * 2 + 5);
    uint8_t r = hw_parse_response(idx, HW_Q_WAKE);
    if (r != 0) {
      // Валідна відповідь будь-якого виду = модуль на шині (§4.1: критерій
      // успіху — валідна відповідь взагалі, 0x0000 вже успіх).
      last_bms_response_ms = millis();
      hw_mod[idx].last_ok_ms = now;
      sweep_seen_mask |= (1u << idx);
      pack_ever_seen_mask |= (1u << idx); // → повна серія вже в цьому ж візиті
      pack_miss_count[idx] = 0;
      hw_q_idx = HW_Q_STATE;
      hw_next_slot_ms = now + HW_INTER_QUERY_MS;
      return;
    }
    hw_advance_addr(); // тиша/битий кадр — далі
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
    return;
  }

  // --- Відома модуль: черговий запит серії ---
  hw_send_query(idx, hw_q_idx);
  bms_receive_response(HW_QUERIES[hw_q_idx].qty * 2 + 5);
  uint8_t r = hw_parse_response(idx, hw_q_idx);
  if (r == 0) {
    hw_advance_addr(); // таймаут/битий кадр: не тримати sweep, retry наступного
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
    return;
  }

  last_bms_response_ms = millis();
  hw_mod[idx].last_ok_ms = now;
  sweep_seen_mask |= (1u << idx);
  pack_miss_count[idx] = 0;
  if (r == 2) {
    hw_advance_addr(); // виняток: реєстр відхилено — решту серії не чіпаємо
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
  } else if (++hw_q_idx >= HW_Q_COUNT) {
    hw_advance_addr();
    hw_next_slot_ms = now + HW_INTER_MODULE_MS;
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

// ===== Balance session engine (17.10.2026) =====
// DIP1 OFF (standard): CVL 54.0 V + auto sessions (BAL_AUTO_ENABLED), driven by
// cell-spread analysis. DIP1 ON: 90% conservative mode (51.8 V), no auto sessions.
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
    // cut by the 90% profile stop; the existing 3600 mV OV inhibit stays
    // as the hard guard.
    calc_taper_start_mv = CELL_TAPER_START_MV;
    calc_charge_stop_mv = CELL_CHARGE_STOP_MV;
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
  calc_charge_stop_mv = CELL_CHARGE_STOP_MV;
  calc_charge_reenable_mv = CELL_CHARGE_REENABLE_MV;
}

// LED1 (PC13): solid = 51.8 V option (DIP1 ON), slow blink = auto session,
// off = standard 54 V without session. Called from the main loop.
static void balance_led_tick(uint32_t now) {
  static bool on = false;
  static uint32_t last_toggle = 0;
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
  agg_soh_sum = 0;
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

    // Protection → 0x359 байти 0–1
    if ((prot & (1u << 5)) && m->cell_max >= CELL_PHYSICAL_OV_CONFIRM_MV)
      sweep_protect_b0 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
    if ((prot & (1u << 4)) && m->pack_v >= 54.5f)
      sweep_protect_b0 |= CAN_PROT_PACK_OV;
    if ((prot & (1u << 7)) && (m->cell_min <= CELL_UV_WARN_MV || m->soc <= 5))
      sweep_protect_b0 |= CAN_PROT_CELL_UV;
    if ((prot & (1u << 6)) && m->pack_v <= 43.2f)
      sweep_protect_b0 |= CAN_PROT_CELL_UV;
    if ((prot & ((1u << 1) | (1u << 3) | (1u << 9))) &&
        (float)m->t_max >= TEMP_DISCHARGE_MAX_C)
      sweep_protect_b0 |= CAN_PROT_OT;
    if ((prot & ((1u << 0) | (1u << 2))) &&
        (float)m->t_min <= TEMP_DISCHARGE_MIN_C)
      sweep_protect_b0 |= CAN_PROT_UT;
    if ((sysf & 0x7E) || (modst & 0x0003))
      sweep_protect_b1 |= CAN_PROT_SYSTEM_ERROR;

    // Warning → 0x359 байти 2–3 (пороги раннього попередження)
    if ((warn & (1u << 5)) && m->cell_max >= CELL_PHYSICAL_OV_WARN_CONFIRM_MV)
      sweep_alarm_b2 |= (CAN_PROT_CELL_OV | CAN_PROT_PACK_OV);
    if ((warn & (1u << 4)) && m->pack_v >= 54.0f)
      sweep_alarm_b2 |= CAN_PROT_PACK_OV;
    if ((warn & (1u << 7)) && (m->cell_min <= CELL_UV_WARN_MV || m->soc <= 10))
      sweep_alarm_b2 |= CAN_PROT_CELL_UV;
    if ((warn & (1u << 6)) && m->pack_v <= 44.0f)
      sweep_alarm_b2 |= CAN_PROT_CELL_UV;
    if ((warn & ((1u << 1) | (1u << 3))) && (float)m->t_max >= 50.0f)
      sweep_alarm_b2 |= CAN_PROT_OT;
    if ((warn & ((1u << 0) | (1u << 2))) &&
        (float)m->t_min <= TEMP_DISCHARGE_MIN_C + 5.0f)
      sweep_alarm_b2 |= CAN_PROT_UT;
  }
}

static void bms_aggregate_data(void) {
  // Update pack presence with 3-miss debounce (Q21)
  uint8_t active_pack_count = 0;
  for (uint8_t i = 0; i < HW_MODULE_COUNT; i++) {
    if (sweep_seen_mask & (1 << i)) {
      pack_ever_seen_mask |= (1 << i);
      pack_miss_count[i] = 0;
      active_pack_count++;
    } else if (pack_ever_seen_mask & (1 << i)) {
      if (pack_miss_count[i] < 255)
        pack_miss_count[i]++;
      if (pack_miss_count[i] <= 2) {
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

  uint8_t effective_packs = (active_pack_count > 0) ? active_pack_count : agg_count;
  bms_pack_count = effective_packs;

  balance_update(effective_packs);
  charge_voltage_limit = bal_session_active ? CHARGE_VOLTAGE_LIMIT_STATIC
                                            : profile_charge_voltage_limit;
  charge_current_limit = calculate_charge_limit(effective_packs);
  discharge_current_limit = calculate_discharge_limit(effective_packs);

  // Set data valid now that fresh aggregated values are computed
  bms_data_valid = true;

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

static void send_can_std(uint32_t id, const uint8_t *data, uint8_t len) {
  uint32_t wait_start = millis();
  while (!can_mailbox_free()) {
    if (millis() - wait_start >= 5)
      return; // скриньки зайняті (немає ACK) — кидати кадр
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
    return;

  uint32_t tx_start = millis();
  while (can_transmit_states(DEYE_CAN, mbox) == CAN_TRANSMIT_PENDING) {
    if (millis() - tx_start >= 3)
      return;
  }
}

static int16_t round_to_i16(float v) {
  return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static void put_u16le(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void send_all_frames(void) {
  gpio_bit_set(LED_PCSCAN_PORT, LED_PCSCAN_PIN); // LED4: CAN activity
  uint8_t data[8] = {0};
  put_u16le(&data[0], charge_voltage_limit);
  put_u16le(&data[2], charge_current_limit);
  put_u16le(&data[4], discharge_current_limit);
  put_u16le(&data[6], DISCHARGE_VOLTAGE_LIMIT_X10);
  send_can_std(CAN_ID_LIMITS, data, 8);

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
  send_can_std(CAN_ID_SOC, data, 4);

  memset(data, 0, 8);
  put_u16le(&data[0], (uint16_t)round_to_i16(bms_voltage * 100.0f));
  put_u16le(&data[2], (uint16_t)round_to_i16(bms_current * 10.0f));
  put_u16le(&data[4], (uint16_t)round_to_i16(bms_temp * 10.0f));
  send_can_std(CAN_ID_STATUS, data, 6);

  // 0x359: Bytes 0..1 Protection, Bytes 2..3 Alarm, Byte 4 Module count, Bytes 5..6 'P' 'N', Byte 7 0
  memset(data, 0, 8);
  data[0] = can_protect_b0;
  data[1] = can_protect_b1;
  data[2] = can_alarm_b2;
  data[3] = can_alarm_b3;
  data[4] = bms_pack_count;
  data[5] = 'P';
  data[6] = 'N';
  data[7] = 0;
  send_can_std(CAN_ID_PROTECT, data, 8);

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
  send_can_std(CAN_ID_FLAGS, data, 2);

  static const uint8_t brand[8] = {'P', 'Y', 'L', 'O', 'N', 0, 0, 0};
  send_can_std(CAN_ID_BRAND, brand, 8);

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
  if (!(GPIO_ISTAT(DIP5_GPIO_PORT) & DIP5_GPIO_PIN)) code |= DIP_CHARGE_CURRENT_50A;
  return code;
}

static void dip_charge_profile_apply(uint8_t code) {
  if (code & DIP_PROFILE_CODE_90) {
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
      (code & DIP_CHARGE_CURRENT_50A) ? CHARGE_CURRENT_LIMIT_PER_PACK_X10
                                      : CHARGE_CURRENT_LIMIT_PER_PACK_DIP5_OFF_X10;
  charge_voltage_limit = profile_charge_voltage_limit;
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
  hw_uart_ready_ms = millis() + 2000;
  pcs_uart_init(); // PCS-485 port (mirror/bridge, mode from DIP2)
  can_init_500k();
  watchdog_init();

  uint32_t last_can = 0, last_dip_poll = 0;

  while (1) {
    watchdog_feed();
    uint32_t now = millis();
    pcs_poll_service(now); // mirror drain / bridge RX every pass
    if (now - last_dip_poll >= 250) {
      last_dip_poll = now;
      dip_charge_profile_poll();
      pcs_mode_apply(dip_applied_code); // retry a DIP2 switch deferred by a busy bridge
      balance_led_tick(now);
    }
    // Власний опит Huawei: серія запитів на відомі модулі + скан невідомих;
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
    }

    if (now - last_can >= 1000) {
      last_can = now;
      send_all_frames();
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
