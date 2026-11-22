/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_TYPES_H
#define JC2_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JC2_SIDE_RIGHT 0u
#define JC2_SIDE_LEFT 1u
#define JC2_MAX_LINKS 2u

#define JC2_REPORT05_SIZE 64u
#define JC2_REPORT0708_SIZE 63u
#define JC2_CMD_RESP_MAX 96u

#define JC2_BOND_MAGIC 0x4A433242u /* "JC2B" */

/*
 * Starting epoch mixed into base_mac to derive the Nintendo OUI (48:F1:EB)
 * Joy-Con 2 BLE addresses. Calling jc2_start_pairing() with clear_bonds = true
 * increments the epoch so Switch 2 treats the pair as fresh hardware.
 * Override at compile time (e.g. -DJC2_DEFAULT_MAC_EPOCH=50) on dev/test rigs
 * to bypass stale console pairings or isolate multiple boards.
 */
#ifndef JC2_DEFAULT_MAC_EPOCH
#define JC2_DEFAULT_MAC_EPOCH 0u
#endif

/*
 * Controller operating mode (selected via jc2_config_t.mode).
 * Zero-initialized default is JC2_MODE_PAIR_LR (dual-link Joy-Con 2 L + R).
 * Single-side modes advertise and maintain only one BLE link and pulse SL+SR
 * instead of L+R for Switch 2 "Change Grip/Order" registration.
 */
typedef enum {
	JC2_MODE_PAIR_LR = 0,
	JC2_MODE_RIGHT_ONLY = 1,
	JC2_MODE_LEFT_ONLY = 2
} jc2_mode_t;

/* Authentic Joy-Con 2 GATT attribute handles (0x0001..0x0030). */
#define JC2_HDL_INPUT05_VAL 0x000Au
#define JC2_HDL_INPUT05_CCCD 0x000Bu
#define JC2_HDL_INPUT0708_VAL 0x000Eu
#define JC2_HDL_INPUT0708_CCCD 0x000Fu
#define JC2_HDL_INPUT0708_RATE 0x0010u
#define JC2_HDL_VIB_VAL 0x0012u
#define JC2_HDL_CMD_WRITE_VAL 0x0014u
#define JC2_HDL_RUMBLE_CMD_VAL 0x0016u
#define JC2_HDL_FW_UPD_VAL 0x0018u
#define JC2_HDL_CMD_RESP_VAL 0x001Au
#define JC2_HDL_CMD_RESP_CCCD 0x001Bu
#define JC2_HDL_EXT_RESP_VAL 0x001Eu
#define JC2_HDL_EXT_RESP_CCCD 0x001Fu
#define JC2_HDL_UNK22_VAL 0x0022u
#define JC2_HDL_UNK22_CCCD 0x0023u
#define JC2_HDL_UNK26_VAL 0x0026u
#define JC2_HDL_UNK26_CCCD 0x0027u
#define JC2_HDL_UNK2A_VAL 0x002Au

/* Joy-Con 2 BLE command IDs (cmd[0]). */
#define JC2_CMD_CONN_PARAM 0x01u
#define JC2_CMD_SPI 0x02u
#define JC2_CMD_HOST_INFO 0x03u
#define JC2_CMD_SET_MODE 0x07u
#define JC2_CMD_LED 0x09u
#define JC2_CMD_SOUND 0x0Au
#define JC2_CMD_POWER 0x0Bu
#define JC2_CMD_FEATURE 0x0Cu
#define JC2_CMD_FW_INFO 0x10u
#define JC2_CMD_IMU 0x11u
#define JC2_CMD_MOUSE 0x13u
#define JC2_CMD_PAIR 0x15u
#define JC2_CMD_EXT_INFO 0x16u
#define JC2_CMD_AUDIO 0x18u

/* Unified L + R Joy-Con 2 button bitmask for jc2_input_t.buttons. */
#define JC2_BTN_Y (1u << 0)
#define JC2_BTN_B (1u << 1)
#define JC2_BTN_X (1u << 2)
#define JC2_BTN_A (1u << 3)
#define JC2_BTN_R_SR (1u << 4)
#define JC2_BTN_R_SL (1u << 5)
#define JC2_BTN_R (1u << 6)
#define JC2_BTN_ZR (1u << 7)
#define JC2_BTN_GR (1u << 8)
#define JC2_BTN_PLUS (1u << 9)
#define JC2_BTN_R3 (1u << 10)
#define JC2_BTN_HOME (1u << 11)
#define JC2_BTN_CHAT (1u << 12)

#define JC2_BTN_DOWN (1u << 16)
#define JC2_BTN_UP (1u << 17)
#define JC2_BTN_RIGHT (1u << 18)
#define JC2_BTN_LEFT (1u << 19)
#define JC2_BTN_L_SR (1u << 20)
#define JC2_BTN_L_SL (1u << 21)
#define JC2_BTN_L (1u << 22)
#define JC2_BTN_ZL (1u << 23)
#define JC2_BTN_MINUS (1u << 24)
#define JC2_BTN_GL (1u << 25)
#define JC2_BTN_L3 (1u << 26)
#define JC2_BTN_CAPTURE (1u << 27)

/* Joy-Con 2 accelerometer scale for +1g resting flat on a surface. */
#define JC2_ACCEL_1G 4096

/* Link-layer diagnostic event IDs. */
#define JC2_EVT_CONNECTED 0x10u
#define JC2_EVT_DISCONNECTED 0x11u
#define JC2_EVT_CONN_PARAM_UPD 0x12u
#define JC2_EVT_SEC_UPD 0x14u
#define JC2_EVT_PHY_UPD 0x21u
#define JC2_EVT_DLE_UPD 0x23u
#define JC2_EVT_WRITE 0x50u
#define JC2_EVT_MTU_REQ 0x55u

/*
 * Controller-agnostic input state for Joy-Con 2 (L + R or single-side).
 *
 * - Stick axes are signed 16-bit centered at 0 (-32768..32767).
 * - accel_* / gyro_* (scaled to 4096 = 1g) populate PC/Mac Report 0x05
 *   (0x000A). Switch 2 Console Report 0x07/0x08 (0x000E) currently emits
 *   stationary and mouse-rail IMU blobs while live compressed FIFO encoding
 *   is deferred to a future release.
 * - left_trigger / right_trigger (0..255) populate PC/Mac Report 0x05 bytes
 *   0x3C/0x3D; digital ZL/ZR on Switch 2 are driven strictly by JC2_BTN_ZL
 *   and JC2_BTN_ZR in buttons.
 */
typedef struct {
	uint32_t buttons;
	int16_t lx;
	int16_t ly;
	int16_t rx;
	int16_t ry;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t mouse_dx;
	int16_t mouse_dy;
	uint8_t left_trigger;
	uint8_t right_trigger;
	uint16_t battery_mv;
	uint8_t charge_state;
	bool mouse_on_surface;
} jc2_input_t;

typedef struct {
	uint8_t body_rgb[JC2_MAX_LINKS][3];
	uint8_t button_rgb[JC2_MAX_LINKS][3];
	uint8_t accent_rgb[JC2_MAX_LINKS][3];
	char serial[JC2_MAX_LINKS][16];
} jc2_identity_t;

/* Persistent Switch 2 host bond and per-side LTKs (stored via callbacks). */
typedef struct {
	uint32_t magic;
	uint8_t mac_epoch;
	uint8_t has_host;
	uint8_t has_ltk[JC2_MAX_LINKS];
	uint8_t host_addr[6];
	uint8_t ltk[JC2_MAX_LINKS][16];
} jc2_bond_t;

/*
 * Platform and application callbacks.
 *
 * Execution context contract:
 * - load_bond, save_bond, on_connected, on_disconnected, on_player_led, and
 *   on_rumble execute exclusively in caller thread context (inside jc2_init()
 *   or jc2_poll()), so they may safely use blocking I/O, mutexes, Serial, or
 *   I2C/SPI drivers.
 * - save_bond runs inside an acquired radio coexistence gap (>= 1500 us while
 *   connected, or unrestricted while disconnected). On nRF52840, keep connected
 *   writes under ~1 ms via append-only flash records (~41 us per 32-bit word)
 *   and defer full 4 KB page erases (~85 ms) to disconnect or slice them with
 *   NRF_NVMC->ERASEPAGEPARTIAL (1 ms pulses).
 * - on_rumble_isr (optional) fires immediately inside the bottom-half IRQ
 *   context (priority-2 SWI0 on nRF52840) for low-latency PWM/I2S haptics and
 *   must be strictly ISR-safe and non-blocking.
 */
typedef struct {
	bool (*load_bond)(jc2_bond_t *out, void *user_ctx);
	bool (*save_bond)(const jc2_bond_t *bond, void *user_ctx);
	void (*on_rumble)(uint8_t side, const uint8_t *data, uint16_t len,
			  void *user_ctx);
	void (*on_rumble_isr)(uint8_t side, const uint8_t *data, uint16_t len,
			      void *user_ctx);
	void (*on_player_led)(uint8_t side, uint8_t led_mask, void *user_ctx);
	void (*on_connected)(uint8_t side, const uint8_t peer_addr[6],
			     uint8_t peer_addr_type, void *user_ctx);
	void (*on_disconnected)(uint8_t side, uint8_t reason, void *user_ctx);
} jc2_callbacks_t;

typedef struct {
	jc2_mode_t mode;
	uint8_t base_mac[6];
	jc2_identity_t identity;
	jc2_callbacks_t cbs;
	void *user_ctx;
} jc2_config_t;

typedef struct {
	uint16_t conn_interval_125;
	uint16_t slave_latency;
	uint16_t sup_timeout_10ms;
	uint16_t mtu;
	uint16_t dle_tx_octets;
	uint16_t dle_rx_octets;
	uint16_t dle_tx_time_us;
	uint16_t dle_rx_time_us;
	uint16_t rate_desc;
	uint16_t tx_hz;
	uint16_t evt_hz;
	uint32_t hvx_ok;
	uint32_t hvx_drop;
	uint32_t rx_vib;
	uint32_t tx_win_cnt;
	uint32_t evt_win_cnt;
	uint32_t tx_win_us;
	uint8_t phy_tx;
	uint8_t phy_rx;
	uint8_t cp_upd_cnt;
	uint8_t disc_cnt;
	uint8_t last_disc_reason;
} jc2_link_diag_t;

typedef struct {
	uint8_t evt_id;
	uint8_t conn;
	uint16_t hdl;
} jc2_evt_log_t;

typedef struct {
	uint32_t grants;
	uint32_t skips;
	uint32_t min_us;
	uint32_t avg_us;
	uint32_t max_us;
	uint32_t samples;
	uint32_t hist[8];
} jc2_gap_stats_t;

typedef struct {
	jc2_mode_t mode;
	bool started;
	bool advertising;
	bool game_ready;
	uint8_t adv_side;
	uint8_t mac_epoch;
	uint8_t last_cmd_id;
	uint8_t last_sub_id;
	uint32_t cmd_count;
	bool connected[JC2_MAX_LINKS];
	bool secured[JC2_MAX_LINKS];
	bool streaming[JC2_MAX_LINKS];
	bool has_ltk[JC2_MAX_LINKS];
	bool adv_reconnect[JC2_MAX_LINKS];
	uint8_t mac[JC2_MAX_LINKS][6];
	uint8_t peer_addr[JC2_MAX_LINKS][6];
	uint8_t peer_addr_type[JC2_MAX_LINKS];
	uint16_t mtu[JC2_MAX_LINKS];
	uint8_t tx_queue_depth[JC2_MAX_LINKS];
	jc2_link_diag_t link[JC2_MAX_LINKS];
} jc2_status_t;

/* Portable 4-byte-aligned DMA buffer unions (pure ISO C99 / C++17 / Rust FFI). */
typedef union {
	uint32_t align4;
	uint8_t bytes[260];
} jc2_dma_buf260_t;

typedef union {
	uint32_t align4;
	uint8_t bytes[4];
} jc2_dma_buf4_t;

/* Little-endian 16-bit integer helpers shared across protocol, ATT, and LL. */
static inline uint16_t jc2_read_u16_le(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline void jc2_write_u16_le(uint8_t *buf, uint16_t v)
{
	buf[0] = (uint8_t)(v & 0xFFu);
	buf[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline int16_t jc2_read_s16_le(const uint8_t *buf)
{
	return (int16_t)jc2_read_u16_le(buf);
}

static inline void jc2_write_s16_le(uint8_t *buf, int16_t v)
{
	jc2_write_u16_le(buf, (uint16_t)v);
}

#ifdef __cplusplus
}
#endif

#endif /* JC2_TYPES_H */
