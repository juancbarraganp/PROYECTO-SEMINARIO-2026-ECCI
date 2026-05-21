#include <Arduino.h>
#include <math.h>
#include "esp_task_wdt.h"
#include <driver/gpio.h>
#include <Wire.h>

extern "C" {
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
}

// ===== Config =====
#define ENA_ALWAYS_ON 1
#define SINK_ACTIVE 1
#define PULSE_HIGH_US 1

// ===== Configuración I2C como esclava =====
#define I2C_SLAVE_ADDRESS 0x08
#define I2C_SLAVE_SDA 8
#define I2C_SLAVE_SCL 9

// Variables para I2C esclava
String i2cReceivedCommand = "";
String i2cResponse = "";
bool i2cSlaveInitialized = false;

float CLEAR_HOME_OFFSET_MM = 5.0f;
uint32_t LIMIT_DEBOUNCE_MS = 150;

// ===== Sistema de Ejecución Secuencial por Serial =====
bool sequentialMode = false;
String commandBuffer = "";

// ===== Tipos =====
struct MoveCmd {
  uint32_t pasos, hz, acc_ms, dec_ms;
  uint8_t dir;
};

struct GCmd {
  char line[48];
};

const uint32_t MIN_HZ = 1, MAX_HZ = 60000;

// Pines
const int X_STEP = 20, X_DIR = 21, Enable = 47, X_H0 = 3, X_HF = 46;
const int Y_STEP = 48, Y_DIR = 42, Y_H0 = 5, Y_HF = 10;
const int Z_STEP = 37, Z_DIR = 38, Z_H0 = 18, Z_HF = 4;

// ===== Métricas para todos los ejes =====
volatile int64_t x_pos_steps = 0, y_pos_steps = 0, z_pos_steps = 0;
volatile uint32_t x_span_steps = 0, y_span_steps = 0, z_span_steps = 0;
volatile float x_mm_total = 330.0f, y_mm_total = 200.0f, z_mm_total = 100.0f;
float x_steps_per_mm = 80.0f, y_steps_per_mm = 80.0f, z_steps_per_mm = 80.0f;

// ===== RTOS =====
QueueHandle_t qX, qY, qZ;
TaskHandle_t tX, tY, tZ, tUI, tG;
volatile bool x_busy = false, y_busy = false, z_busy = false;

// ===== GCODE =====
QueueHandle_t qG = nullptr;
volatile bool g_abortSeq = false;

// ===== Variables para JOG continuo =====
volatile bool jogging = false;
volatile char jog_axis = ' ';  // Cambiar de String a char
volatile bool jog_direction = false;
volatile uint32_t jog_speed = 5000;
volatile uint32_t time_acelydecel = 100;  // tiempo para acelerar y desacelerar



// Variables para almacenar el estado anterior de los sensores
bool last_x_h0_state = false;
bool last_x_hf_state = false;
bool last_y_h0_state = false;
bool last_y_hf_state = false;
bool last_z_h0_state = false;
bool last_z_hf_state = false;

// ===== GPIO rápidos =====
IRAM_ATTR static inline void _w1ts(int pin) {
  uint32_t m = 1UL << (pin & 31);
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG, m);
  else REG_WRITE(GPIO_OUT1_W1TS_REG, m);
}

IRAM_ATTR static inline void _w1tc(int pin) {
  uint32_t m = 1UL << (pin & 31);
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG, m);
  else REG_WRITE(GPIO_OUT1_W1TC_REG, m);
}

IRAM_ATTR static inline void _hi(int p) {
  _w1ts(p);
}
IRAM_ATTR static inline void _lo(int p) {
  _w1tc(p);
}

IRAM_ATTR static inline void _act(int p, bool on) {
  if (SINK_ACTIVE) {
    on ? _lo(p) : _hi(p);
  } else {
    on ? _hi(p) : _lo(p);
  }
}

IRAM_ATTR static inline void _dir_fast(int p, bool fwd) {
  _act(p, fwd);
  esp_rom_delay_us(10);
}

IRAM_ATTR static inline int _gin(int p) {
  if (p < 32) return (REG_READ(GPIO_IN_REG) >> p) & 1;
  else return (REG_READ(GPIO_IN1_REG) >> (p - 32)) & 1;
}

// Funciones de límites para todos los ejes
inline bool x_limH0() {
  return _gin(X_H0) == 0;
}
inline bool x_limHF() {
  return _gin(X_HF) == 0;
}
inline bool y_limH0() {
  return _gin(Y_H0) == 0;
}
inline bool y_limHF() {
  return _gin(Y_HF) == 0;
}
inline bool z_limH0() {
  return _gin(Z_H0) == 0;
}
inline bool z_limHF() {
  return _gin(Z_HF) == 0;
}


void verificarYEnviarCambiosSensores() {
  // Verificar sensor X_H0
  bool current_x_h0 = x_limH0();
  if (current_x_h0 != last_x_h0_state) {
    i2cResponse = "x_h0 " + String(current_x_h0 ? "on" : "off");
    last_x_h0_state = current_x_h0;
    Serial.println("Cambio X_H0: " + i2cResponse);
  }

  // Verificar sensor X_HF
  bool current_x_hf = x_limHF();
  if (current_x_hf != last_x_hf_state) {
    i2cResponse = "x_hf " + String(current_x_hf ? "on" : "off");
    last_x_hf_state = current_x_hf;
    Serial.println("Cambio X_HF: " + i2cResponse);
  }

  // Verificar sensor Y_H0
  bool current_y_h0 = y_limH0();
  if (current_y_h0 != last_y_h0_state) {
    i2cResponse = "y_h0 " + String(current_y_h0 ? "on" : "off");
    last_y_h0_state = current_y_h0;
    Serial.println("Cambio Y_H0: " + i2cResponse);
  }

  // Verificar sensor Y_HF
  bool current_y_hf = y_limHF();
  if (current_y_hf != last_y_hf_state) {
    i2cResponse = "y_hf " + String(current_y_hf ? "on" : "off");
    last_y_hf_state = current_y_hf;
    Serial.println("Cambio Y_HF: " + i2cResponse);
  }

  // Verificar sensor Z_H0
  bool current_z_h0 = z_limH0();
  if (current_z_h0 != last_z_h0_state) {
    i2cResponse = "z_h0 " + String(current_z_h0 ? "on" : "off");
    last_z_h0_state = current_z_h0;
    Serial.println("Cambio Z_H0: " + i2cResponse);
  }

  // Verificar sensor Z_HF
  bool current_z_hf = z_limHF();
  if (current_z_hf != last_z_hf_state) {
    i2cResponse = "z_hf " + String(current_z_hf ? "on" : "off");
    last_z_hf_state = current_z_hf;
    Serial.println("Cambio Z_HF: " + i2cResponse);
  }
}

// Agrega estas 3 líneas JUSTO ANTES de donde aparece el primer error:
typedef struct Axis Axis;  // Declaración forward

static void runMoveExactAxis(volatile Axis& axis, const MoveCmd& c);
static uint32_t runUntilLimit(volatile Axis& axis, bool towardHF, uint32_t hz);

// ===== Timers y Estructuras Axis =====
hw_timer_t *tx = nullptr, *ty = nullptr, *tz = nullptr;

struct Axis {
  int pinSTEP, pinDIR;
  volatile uint32_t period_us;
  volatile uint32_t steps_remaining;
  volatile uint32_t steps_done;
  volatile bool running;
  volatile bool dirFwd;
  volatile bool until_limit;
  volatile bool towardHF;
  volatile uint32_t lim_active_us;
  volatile int64_t* pos_steps;
  volatile uint32_t* span_steps;
  float* steps_per_mm;
  float mm_total;
  bool (*limH0)();
  bool (*limHF)();
};

volatile Axis AX_X = {}, AX_Y = {}, AX_Z = {};

// ===== ISRs para todos los ejes =====


// ===== ISRs para todos los ejes =====
void IRAM_ATTR onTimerX() {
  if (!AX_X.running) return;

  _act(AX_X.pinSTEP, true);
  esp_rom_delay_us(PULSE_HIGH_US);
  _act(AX_X.pinSTEP, false);
  AX_X.steps_done++;

  // Verificar cambios en sensores X
  bool current_x_h0 = x_limH0();
  bool current_x_hf = x_limHF();
  
  // Detectar cambios y preparar respuesta I2C
  if (current_x_h0 != last_x_h0_state) {
    i2cResponse = "x_h0 " + String(current_x_h0 ? "on" : "off");
    Serial.println("sensor detectadp");
    last_x_h0_state = current_x_h0;
  }
  if (current_x_hf != last_x_hf_state) {
    i2cResponse = "x_hf " + String(current_x_hf ? "on" : "off");
    last_x_hf_state = current_x_hf;
  }

  // Solo verificar límites si NO estamos en modo JOG continuo para este eje
  if (!jogging || jog_axis != 'X') {
    if (AX_X.until_limit) {
      bool limActive = AX_X.towardHF ? AX_X.limHF() : AX_X.limH0();
      if (limActive) {
        AX_X.lim_active_us += AX_X.period_us;
        if (AX_X.lim_active_us >= (LIMIT_DEBOUNCE_MS * 1000UL)) {
          AX_X.running = false;
          return;
        }
      } else {
        AX_X.lim_active_us = 0;
      }
    } else {
      // Protección contra límites en movimiento normal
      if (AX_X.dirFwd && AX_X.limHF()) {
        AX_X.running = false;
        return;
      }
      if (!AX_X.dirFwd && AX_X.limH0()) {
        AX_X.running = false;
        return;
      }
    }
  }

  // Actualizar posición (siempre)
  if (AX_X.dirFwd) {
    if (*AX_X.pos_steps < (int64_t)*AX_X.span_steps) (*AX_X.pos_steps)++;
  } else {
    if (*AX_X.pos_steps > 0) (*AX_X.pos_steps)--;
  }

  // En modo JOG (steps_remaining = 0), movimiento continuo infinito
  if (!AX_X.until_limit && AX_X.steps_remaining > 0) {
    if (AX_X.steps_remaining) AX_X.steps_remaining--;
    if (AX_X.steps_remaining == 0) AX_X.running = false;
  }
}

void IRAM_ATTR onTimerY() {
  if (!AX_Y.running) return;

  _act(AX_Y.pinSTEP, true);
  esp_rom_delay_us(PULSE_HIGH_US);
  _act(AX_Y.pinSTEP, false);
  AX_Y.steps_done++;

  // Verificar cambios en sensores Y
  bool current_y_h0 = y_limH0();
  bool current_y_hf = y_limHF();
  
  // Detectar cambios y preparar respuesta I2C
  if (current_y_h0 != last_y_h0_state) {
    i2cResponse = "y_h0 " + String(current_y_h0 ? "on" : "off");
    last_y_h0_state = current_y_h0;
  }
  if (current_y_hf != last_y_hf_state) {
    i2cResponse = "y_hf " + String(current_y_hf ? "on" : "off");
    last_y_hf_state = current_y_hf;
  }

  // Solo verificar límites si NO estamos en modo JOG continuo para este eje
  if (!jogging || jog_axis != 'Y') {
    if (AX_Y.until_limit) {
      bool limActive = AX_Y.towardHF ? AX_Y.limHF() : AX_Y.limH0();
      if (limActive) {
        AX_Y.lim_active_us += AX_Y.period_us;
        if (AX_Y.lim_active_us >= (LIMIT_DEBOUNCE_MS * 1000UL)) {
          AX_Y.running = false;
          return;
        }
      } else {
        AX_Y.lim_active_us = 0;
      }
    } else {
      // Protección contra límites en movimiento normal
      if (AX_Y.dirFwd && AX_Y.limHF()) {
        AX_Y.running = false;
        return;
      }
      if (!AX_Y.dirFwd && AX_Y.limH0()) {
        AX_Y.running = false;
        return;
      }
    }
  }

  // Actualizar posición (siempre)
  if (AX_Y.dirFwd) {
    if (*AX_Y.pos_steps < (int64_t)*AX_Y.span_steps) (*AX_Y.pos_steps)++;
  } else {
    if (*AX_Y.pos_steps > 0) (*AX_Y.pos_steps)--;
  }

  // En modo JOG (steps_remaining = 0), movimiento continuo infinito
  if (!AX_Y.until_limit && AX_Y.steps_remaining > 0) {
    if (AX_Y.steps_remaining) AX_Y.steps_remaining--;
    if (AX_Y.steps_remaining == 0) AX_Y.running = false;
  }
}

void IRAM_ATTR onTimerZ() {
  if (!AX_Z.running) return;

  _act(AX_Z.pinSTEP, true);
  esp_rom_delay_us(PULSE_HIGH_US);
  _act(AX_Z.pinSTEP, false);
  AX_Z.steps_done++;

  // Verificar cambios en sensores Z
  bool current_z_h0 = z_limH0();
  bool current_z_hf = z_limHF();
  
  // Detectar cambios y preparar respuesta I2C
  if (current_z_h0 != last_z_h0_state) {
    i2cResponse = "z_h0 " + String(current_z_h0 ? "on" : "off");
    last_z_h0_state = current_z_h0;
  }
  if (current_z_hf != last_z_hf_state) {
    i2cResponse = "z_hf " + String(current_z_hf ? "on" : "off");
    last_z_hf_state = current_z_hf;
  }

  // Solo verificar límites si NO estamos en modo JOG continuo para este eje
  if (!jogging || jog_axis != 'Z') {
    if (AX_Z.until_limit) {
      bool limActive = AX_Z.towardHF ? AX_Z.limHF() : AX_Z.limH0();
      if (limActive) {
        AX_Z.lim_active_us += AX_Z.period_us;
        if (AX_Z.lim_active_us >= (LIMIT_DEBOUNCE_MS * 1000UL)) {
          AX_Z.running = false;
          return;
        }
      } else {
        AX_Z.lim_active_us = 0;
      }
    } else {
      // Protección contra límites en movimiento normal
      if (AX_Z.dirFwd && AX_Z.limHF()) {
        AX_Z.running = false;
        return;
      }
      if (!AX_Z.dirFwd && AX_Z.limH0()) {
        AX_Z.running = false;
        return;
      }
    }
  }

  // Actualizar posición (siempre)
  if (AX_Z.dirFwd) {
    if (*AX_Z.pos_steps < (int64_t)*AX_Z.span_steps) (*AX_Z.pos_steps)++;
  } else {
    if (*AX_Z.pos_steps > 0) (*AX_Z.pos_steps)--;
  }

  // En modo JOG (steps_remaining = 0), movimiento continuo infinito
  if (!AX_Z.until_limit && AX_Z.steps_remaining > 0) {
    if (AX_Z.steps_remaining) AX_Z.steps_remaining--;
    if (AX_Z.steps_remaining == 0) AX_Z.running = false;
  }
}

// ===== Utilidades =====
static inline void abortSet() {
  g_abortSeq = true;
}
static inline void abortClear() {
  g_abortSeq = false;
}
static inline bool shouldAbort() {
  return g_abortSeq;
}

static void waitAllIdle() {
  while ((AX_X.running || AX_Y.running || AX_Z.running) && !shouldAbort()) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void coopDelayMs(uint32_t ms) {
  uint32_t t = 0;
  while (t < ms && !shouldAbort()) {
    vTaskDelay(pdMS_TO_TICKS(5));
    t += 5;
  }
}

static inline void timer_set_period(hw_timer_t* t, uint32_t per_us) {
  timerAlarmWrite(t, per_us, true);
}

// ===== Movimiento con rampas para todos los ejes =====
static void runMoveExactAxis(volatile Axis& axis, const MoveCmd& c) {
  if (c.pasos == 0) return;

  // Verificar límites
  if ((c.dir == 0) && axis.limH0()) {
    Serial.printf("BLOCK: límite H0 activo en eje\n");
    return;
  }
  if ((c.dir != 0) && axis.limHF()) {
    Serial.printf("BLOCK: límite HF activo en eje\n");
    return;
  }

  uint32_t hzC = constrain(c.hz, MIN_HZ, MAX_HZ);
  float acc_s = max(1u, c.acc_ms) / 1000.0f, dec_s = max(1u, c.dec_ms) / 1000.0f;

  float S_up = 0.5f * hzC * acc_s, S_dn = 0.5f * hzC * dec_s;
  if (c.pasos < (uint32_t)(S_up + S_dn)) {
    float peak = (2.0f * c.pasos) / (acc_s + dec_s);
    hzC = constrain((uint32_t)peak, MIN_HZ, MAX_HZ);
    S_up = 0.5f * hzC * acc_s;
    S_dn = 0.5f * hzC * dec_s;
  }

  uint32_t n_up = (uint32_t)lroundf(S_up);
  uint32_t n_dn = (uint32_t)lroundf(S_dn);
  uint32_t n_cru = (c.pasos > (n_up + n_dn)) ? (c.pasos - n_up - n_dn) : 0;
  const float f0 = (float)MIN_HZ, fC = (float)hzC;

  if (!ENA_ALWAYS_ON) {
    _act(Enable, true);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  _dir_fast(axis.pinDIR, c.dir != 0);

  axis.steps_done = 0;
  axis.steps_remaining = c.pasos;
  axis.running = true;
  axis.dirFwd = (c.dir != 0);
  axis.until_limit = false;

  axis.period_us = (uint32_t)(1000000.0f / f0);
  hw_timer_t* timer = (axis.pinSTEP == X_STEP) ? tx : (axis.pinSTEP == Y_STEP) ? ty
                                                                               : tz;
  timer_set_period(timer, axis.period_us);
  timerAlarmEnable(timer);

  // Marcar eje como ocupado
  volatile bool* busy_flag = (axis.pinSTEP == X_STEP) ? &x_busy : (axis.pinSTEP == Y_STEP) ? &y_busy
                                                                                           : &z_busy;
  *busy_flag = true;

  while (axis.running) {
    if (shouldAbort()) {
      axis.running = false;
      break;
    }

    uint32_t k = axis.steps_done;
    float f = f0;

    if (k < n_up) f = f0 + (fC - f0) * ((float)k / (float)max(1u, n_up));
    else if (k < n_up + n_cru) f = fC;
    else {
      uint32_t kd = k - n_up - n_cru;
      f = fC - (fC - f0) * ((float)kd / (float)max(1u, n_dn));
    }

    if (f < MIN_HZ) f = MIN_HZ;
    if (f > MAX_HZ) f = MAX_HZ;

    uint32_t per = (uint32_t)(1000000.0f / f);
    if (per != axis.period_us) {
      axis.period_us = per;
      timer_set_period(timer, per);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  *busy_flag = false;
  timerAlarmDisable(timer);
}

// Wrappers para cada eje
static void runMoveExactX(const MoveCmd& c) {
  runMoveExactAxis(AX_X, c);
}
static void runMoveExactY(const MoveCmd& c) {
  runMoveExactAxis(AX_Y, c);
}
static void runMoveExactZ(const MoveCmd& c) {
  runMoveExactAxis(AX_Z, c);
}

// ===== HOME para todos los ejes =====
static uint32_t runUntilLimit(volatile Axis& axis, bool towardHF, uint32_t hz) {
  uint32_t per = (uint32_t)(1000000.0f / constrain(hz, MIN_HZ, MAX_HZ));

  if (!ENA_ALWAYS_ON) {
    _act(Enable, true);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  _dir_fast(axis.pinDIR, towardHF);

  axis.steps_done = 0;
  axis.steps_remaining = 0;
  axis.running = true;
  axis.dirFwd = towardHF;
  axis.until_limit = true;
  axis.towardHF = towardHF;
  axis.period_us = per;
  axis.lim_active_us = 0;

  hw_timer_t* timer = (axis.pinSTEP == X_STEP) ? tx : (axis.pinSTEP == Y_STEP) ? ty
                                                                               : tz;
  timer_set_period(timer, per);
  timerAlarmEnable(timer);

  while (axis.running) vTaskDelay(1);

  timerAlarmDisable(timer);
  return axis.steps_done;
}

bool hasHome() {
  return (x_span_steps > 0 && x_steps_per_mm > 0.0f && y_span_steps > 0 && y_steps_per_mm > 0.0f && z_span_steps > 0 && z_steps_per_mm > 0.0f);
}

void doHOME() {
  Serial.println("Iniciando HOME para todos los ejes...");

  // HOME X
  Serial.println("HOME X...");
  (void)runUntilLimit(AX_X, false, 5000);
  x_pos_steps = 0;
  uint32_t x_span = runUntilLimit(AX_X, true, 10000);
  x_span_steps = x_span;
  x_steps_per_mm = (x_mm_total > 0 && x_span_steps > 0) ? ((float)x_span_steps / x_mm_total) : 0.0f;

  MoveCmd backX{ x_span_steps, 30000, 500, 500, 0 };
  runMoveExactX(backX);
  x_pos_steps = 0;

  // HOME Y
  Serial.println("HOME Y...");
  (void)runUntilLimit(AX_Y, false, 5000);
  y_pos_steps = 0;
  uint32_t y_span = runUntilLimit(AX_Y, true, 10000);
  y_span_steps = y_span;
  y_steps_per_mm = (y_mm_total > 0 && y_span_steps > 0) ? ((float)y_span_steps / y_mm_total) : 0.0f;

  MoveCmd backY{ y_span_steps, 30000, 500, 500, 0 };
  runMoveExactY(backY);
  y_pos_steps = 0;

  // HOME Z
  Serial.println("HOME Z...");
  (void)runUntilLimit(AX_Z, false, 5000);
  z_pos_steps = 0;
  uint32_t z_span = runUntilLimit(AX_Z, true, 10000);
  z_span_steps = z_span;
  z_steps_per_mm = (z_mm_total > 0 && z_span_steps > 0) ? ((float)z_span_steps / z_mm_total) : 0.0f;

  MoveCmd backZ{ z_span_steps, 30000, 500, 500, 0 };
  runMoveExactZ(backZ);
  z_pos_steps = 0;

  // Despegar de los límites
  if (x_steps_per_mm > 0.0f) {
    uint32_t off = (uint32_t)lroundf(CLEAR_HOME_OFFSET_MM * x_steps_per_mm);
    if (off > 0) {
      MoveCmd offMove{ off, 5000, 100, 100, 1 };
      runMoveExactX(offMove);
    }
  }

  if (y_steps_per_mm > 0.0f) {
    uint32_t off = (uint32_t)lroundf(CLEAR_HOME_OFFSET_MM * y_steps_per_mm);
    if (off > 0) {
      MoveCmd offMove{ off, 5000, 100, 100, 1 };
      runMoveExactY(offMove);
    }
  }

  if (z_steps_per_mm > 0.0f) {
    uint32_t off = (uint32_t)lroundf(CLEAR_HOME_OFFSET_MM * z_steps_per_mm);
    if (off > 0) {
      MoveCmd offMove{ off, 5000, 100, 100, 1 };
      runMoveExactZ(offMove);
    }
  }

  Serial.println(F("HOME_STATUS:true"));
  i2cResponse = "HOME:DONE";
}

// ===== Encoladores / STOP =====
inline void x_enqueue(const MoveCmd& m) {
  xQueueSend(qX, &m, 0);
}
inline void y_enqueue(const MoveCmd& m) {
  xQueueSend(qY, &m, 0);
}
inline void z_enqueue(const MoveCmd& m) {
  xQueueSend(qZ, &m, 0);
}

inline void x_stopNow() {
  AX_X.running = false;
  timerAlarmDisable(tx);
}
inline void y_stopNow() {
  AX_Y.running = false;
  timerAlarmDisable(ty);
}
inline void z_stopNow() {
  AX_Z.running = false;
  timerAlarmDisable(tz);
}

// ===== Tareas movimiento =====
void TaskX(void*) {
  for (;;) {
    MoveCmd m;
    if (xQueueReceive(qX, &m, portMAX_DELAY) != pdTRUE) continue;
    if (shouldAbort()) continue;
    runMoveExactX(m);
  }
}

void TaskY(void*) {
  for (;;) {
    MoveCmd m;
    if (xQueueReceive(qY, &m, portMAX_DELAY) != pdTRUE) continue;
    if (shouldAbort()) continue;
    runMoveExactY(m);
  }
}

void TaskZ(void*) {
  for (;;) {
    MoveCmd m;
    if (xQueueReceive(qZ, &m, portMAX_DELAY) != pdTRUE) continue;
    if (shouldAbort()) continue;
    runMoveExactZ(m);
  }
}

// ===== Interprete GCODE mejorado para 3 ejes =====
bool modoAbsoluto = true;

void gcodeExecuteLine(const String& line) {
  if (shouldAbort()) return;

  String s = line;
  s.trim();
  if (s.length() == 0) return;

  Serial.println(">> Ejecutando: " + s);

  // Parsear G-code
  int gcode = -1;
  float xVal = NAN, yVal = NAN, zVal = NAN, fVal = NAN, pVal = NAN;

  if (sscanf(s.c_str(), "G%d", &gcode) == 1) {
    // G-code detectado
  } else {
    return;
  }

  // Extraer parámetros
  int xi = s.indexOf('X');
  if (xi >= 0) xVal = s.substring(xi + 1).toFloat();

  int yi = s.indexOf('Y');
  if (yi >= 0) yVal = s.substring(yi + 1).toFloat();

  int zi = s.indexOf('Z');
  if (zi >= 0) zVal = s.substring(zi + 1).toFloat();

  int fi = s.indexOf('F');
  if (fi >= 0) fVal = s.substring(fi + 1).toFloat();

  int pi = s.indexOf('P');
  if (pi >= 0) pVal = s.substring(pi + 1).toFloat();

  switch (gcode) {
    case 0:  // Movimiento rápido
    case 1:  // Movimiento lineal
      {
        if (!hasHome()) {
          Serial.println("ERR: haga HOME primero");
          return;
        }

        // Calcular destinos para cada eje
        float xDest = NAN, yDest = NAN, zDest = NAN;

        if (!isnan(xVal)) {
          xDest = modoAbsoluto ? xVal : (x_pos_steps / x_steps_per_mm) + xVal;
          xDest = constrain(xDest, 0, x_mm_total);
        }

        if (!isnan(yVal)) {
          yDest = modoAbsoluto ? yVal : (y_pos_steps / y_steps_per_mm) + yVal;
          yDest = constrain(yDest, 0, y_mm_total);
        }

        if (!isnan(zVal)) {
          zDest = modoAbsoluto ? zVal : (z_pos_steps / z_steps_per_mm) + zVal;
          zDest = constrain(zDest, 0, z_mm_total);
        }

        uint32_t feedHz = isnan(fVal) ? 3000 : (uint32_t)fVal;

        // Preparar movimientos para cada eje
        if (!isnan(xDest)) {
          int64_t tgt = (int64_t)lroundf(xDest * x_steps_per_mm);
          int64_t d = tgt - x_pos_steps;
          if (d != 0) {
            MoveCmd m{ (uint32_t)llabs(d), feedHz, time_acelydecel, time_acelydecel, (uint8_t)(d > 0 ? 1 : 0) };
            x_enqueue(m);
          }
        }

        if (!isnan(yDest)) {
          int64_t tgt = (int64_t)lroundf(yDest * y_steps_per_mm);
          int64_t d = tgt - y_pos_steps;
          if (d != 0) {
            MoveCmd m{ (uint32_t)llabs(d), feedHz, time_acelydecel, time_acelydecel, (uint8_t)(d > 0 ? 1 : 0) };
            y_enqueue(m);
          }
        }

        if (!isnan(zDest)) {
          int64_t tgt = (int64_t)lroundf(zDest * z_steps_per_mm);
          int64_t d = tgt - z_pos_steps;
          if (d != 0) {
            MoveCmd m{ (uint32_t)llabs(d), feedHz, time_acelydecel, time_acelydecel, (uint8_t)(d > 0 ? 1 : 0) };
            z_enqueue(m);
          }
        }

        // Esperar a que todos los movimientos terminen
        waitAllIdle();
        break;
      }

    case 4:  // Pausa
      {
        uint32_t delayMs = isnan(pVal) ? 1000 : (uint32_t)pVal;
        Serial.printf("Pausa: %lu ms\n", delayMs);
        coopDelayMs(delayMs);
        break;
      }

    case 28:  // Volver a origen
      {
        if (!hasHome()) {
          Serial.println("ERR: haga HOME primero");
          return;
        }

        // Mover todos los ejes a 0
        if (x_pos_steps != 0) {
          MoveCmd m{ (uint32_t)x_pos_steps, 30000, 500, 500, 0 };
          x_enqueue(m);
        }

        if (y_pos_steps != 0) {
          MoveCmd m{ (uint32_t)y_pos_steps, 30000, 500, 500, 0 };
          y_enqueue(m);
        }

        if (z_pos_steps != 0) {
          MoveCmd m{ (uint32_t)z_pos_steps, 30000, 500, 500, 0 };
          z_enqueue(m);
        }

        waitAllIdle();
        break;
      }

    case 90:  // Modo absoluto
      modoAbsoluto = true;
      Serial.println("Modo absoluto (G90)");
      break;

    case 91:  // Modo relativo
      modoAbsoluto = false;
      Serial.println("Modo relativo (G91)");
      break;

    default:
      Serial.printf("Comando G%d no soportado\n", gcode);
      break;
  }
}

// ===== Tarea GCODE =====
void TaskG(void*) {
  for (;;) {
    GCmd c;
    if (xQueueReceive(qG, &c, portMAX_DELAY) != pdTRUE) continue;
    if (shouldAbort()) continue;
    gcodeExecuteLine(c.line);
  }
}

void gcodeSubmit(const String& s) {
  GCmd c{};
  size_t n = min(s.length(), sizeof(c.line) - 1);
  memcpy(c.line, s.c_str(), n);
  xQueueSend(qG, &c, 0);
}

// ===== Rutina de demostración para 3 ejes =====
void rutinaCNC_submit() {
  abortClear();
  const char* programa[] = {
    "G90",                    // Modo absoluto
    "G0 X0 Y0 Z0 F30000",     // Ir al origen
    "G4 P1000",               // Pausa 1 segundo
    "G0 X50 Y25 Z10 F20000",  // Movimiento en los 3 ejes
    "G1 X100 Y50 Z5 F10000",  // Movimiento lineal lento
    "G4 P500",
    "G0 X75 Y37 Z8 F30000",
    "G4 P500",
    "G28",  // Volver a home
    nullptr
  };

  for (int i = 0; programa[i]; ++i) {
    gcodeSubmit(programa[i]);
  }
}

// ===== I2C ESCLAVA =====

// Inicializar I2C como esclava
void initI2CSlave() {
  Wire.begin(I2C_SLAVE_ADDRESS, I2C_SLAVE_SDA, I2C_SLAVE_SCL, 100000);
  Wire.onReceive(receiveI2CEvent);
  Wire.onRequest(requestI2CEvent);
  i2cSlaveInitialized = true;
  Serial.println("I2C Esclava inicializada - Dirección: 0x" + String(I2C_SLAVE_ADDRESS, HEX));
}

// Evento cuando se reciben datos del maestro
void receiveI2CEvent(int howMany) {
  i2cReceivedCommand = "";
  while (Wire.available()) {
    char c = Wire.read();
    i2cReceivedCommand += c;
  }

  // Eliminar caracteres nulos si los hay
  i2cReceivedCommand.trim();

  // Procesar comando y preparar respuesta
  processI2CCommand(i2cReceivedCommand);
}

// Procesar comandos I2C recibidos
void processI2CCommand(const String& command) {
  String cmd = command;
  cmd.toUpperCase();
  cmd.trim();

  // Comandos de consulta y estado
  if (cmd == "STATUS") {
    i2cResponse = "OK:CNC_READY";
  } else if (cmd == "PING") {
    i2cResponse = "PONG";
  } else if (cmd == "VERSION") {
    i2cResponse = "CNC_ESP32_v1.0";

    // Comandos de movimiento individuales
  } else if (cmd.startsWith("MOVEX")) {
    uint32_t p, h, a, d, dr;
    if (sscanf(cmd.c_str(), "MOVEX %u %u %u %u %u", &p, &h, &a, &d, &dr) == 5) {
      x_enqueue({ p, h, a, d, (uint8_t)dr });
      i2cResponse = "OK:MOVEX";
    } else {
      i2cResponse = "ERROR:MOVEX_FORMAT";
    }
  } else if (cmd.startsWith("MOVEY")) {
    uint32_t p, h, a, d, dr;
    if (sscanf(cmd.c_str(), "MOVEY %u %u %u %u %u", &p, &h, &a, &d, &dr) == 5) {
      y_enqueue({ p, h, a, d, (uint8_t)dr });
      i2cResponse = "OK:MOVEY";
    } else {
      i2cResponse = "ERROR:MOVEY_FORMAT";
    }
  } else if (cmd.startsWith("MOVEZ")) {
    uint32_t p, h, a, d, dr;
    if (sscanf(cmd.c_str(), "MOVEZ %u %u %u %u %u", &p, &h, &a, &d, &dr) == 5) {
      z_enqueue({ p, h, a, d, (uint8_t)dr });
      i2cResponse = "OK:MOVEZ";
    } else {
      i2cResponse = "ERROR:MOVEZ_FORMAT";
    }

    // Comandos de configuración de dimensiones
  } else if (cmd.startsWith("SET_X_MM")) {
    float mm;
    if (sscanf(cmd.c_str(), "SET_X_MM %f", &mm) == 1) {
      x_mm_total = mm;
      i2cResponse = "OK:X_MM:" + String(mm, 2);
    } else {
      i2cResponse = "ERROR:SET_X_MM_FORMAT";
    }
  } else if (cmd.startsWith("SET_Y_MM")) {
    float mm;
    if (sscanf(cmd.c_str(), "SET_Y_MM %f", &mm) == 1) {
      y_mm_total = mm;
      i2cResponse = "OK:Y_MM:" + String(mm, 2);
    } else {
      i2cResponse = "ERROR:SET_Y_MM_FORMAT";
    }
  } else if (cmd.startsWith("SET_Z_MM")) {
    float mm;
    if (sscanf(cmd.c_str(), "SET_Z_MM %f", &mm) == 1) {
      z_mm_total = mm;
      i2cResponse = "OK:Z_MM:" + String(mm, 2);
    } else {
      i2cResponse = "ERROR:SET_Z_MM_FORMAT";
    }

    // Comandos de consulta de dimensiones y posición
  } else if (cmd == "GET_DIMENSIONS") {
    char dimBuffer[128];
    snprintf(dimBuffer, sizeof(dimBuffer), "DIM:X%.2f:Y%.2f:Z%.2f",
             x_mm_total, y_mm_total, z_mm_total);
    i2cResponse = String(dimBuffer);
  } else if (cmd == "GET_POSITION") {
    char posBuffer[128];
    snprintf(posBuffer, sizeof(posBuffer), "POS:X%.2f:Y%.2f:Z%.2f",
             x_pos_steps / x_steps_per_mm,
             y_pos_steps / y_steps_per_mm,
             z_pos_steps / z_steps_per_mm);
    i2cResponse = String(posBuffer);
  } else if (cmd == "GET_POSITION_STEPS") {
    char posBuffer[128];
    snprintf(posBuffer, sizeof(posBuffer), "STEPS:X%lld:Y%lld:Z%lld",
             x_pos_steps, y_pos_steps, z_pos_steps);
    i2cResponse = String(posBuffer);

    // Comandos de control general
  } else if (cmd == "STOP") {
    abortSet();
    x_stopNow();
    y_stopNow();
    z_stopNow();
    sequentialMode = false;
    i2cResponse = "OK:STOP";
  } else if (cmd == "HOME") {
    doHOME();
    i2cResponse = "OK:HOME";
  } else if (cmd == "RESET") {
    abortClear();
    sequentialMode = false;
    i2cResponse = "OK:RESET";
  } else if (cmd == "RUNCNC") {
    rutinaCNC_submit();
    i2cResponse = "OK:RUNCNC";

    // Comandos de modo secuencial
  } else if (cmd == "SEQUENTIAL_START") {
    sequentialMode = true;
    i2cResponse = "OK:SEQUENTIAL_START";
  } else if (cmd == "SEQUENTIAL_STOP") {
    sequentialMode = false;
    i2cResponse = "OK:SEQUENTIAL_STOP";

    // Comandos JOG
  } else if (cmd.startsWith("START_JOG")) {
    char axis;
    uint8_t direction;
    uint32_t speed = 5000;

    if (sscanf(cmd.c_str(), "START_JOG %c %hhu %u", &axis, &direction, &speed) >= 2) {
      startJogContinuous(axis, (direction == 1), speed);
      i2cResponse = "OK:JOG_START";
    } else {
      i2cResponse = "ERROR:JOG_FORMAT";
    }
  } else if (cmd == "STOP_JOG") {
    stopJogContinuous();
    i2cResponse = "OK:JOG_STOP";

    // Comandos de consulta de estado
  } else if (cmd == "GET_EMERGENCY_STOP") {
    if (g_abortSeq) {
      i2cResponse = "EMERGENCY:ACTIVE";
    } else {
      i2cResponse = "EMERGENCY:INACTIVE";
    }
  } else if (cmd == "GET_HOME_STATUS") {
    if (hasHome()) {
      i2cResponse = "HOME:DONE";
    } else {
      i2cResponse = "HOME:NOT_DONE";
    }
  } else if (cmd == "GET_STAT") {
    char statBuffer[256];
    snprintf(statBuffer, sizeof(statBuffer),
             "STAT:X%.2f:Y%.2f:Z%.2f:SEQ:%s:JOG:%s",
             x_pos_steps / x_steps_per_mm,
             y_pos_steps / y_steps_per_mm,
             z_pos_steps / z_steps_per_mm,
             sequentialMode ? "ACTIVE" : "INACTIVE",
             jogging ? "ACTIVE" : "INACTIVE");
    i2cResponse = String(statBuffer);

    // Comandos de configuración de parámetros
  } else if (cmd.startsWith("SET_HOME_OFFSET")) {
    float offset;
    if (sscanf(cmd.c_str(), "SET_HOME_OFFSET %f", &offset) == 1) {
      CLEAR_HOME_OFFSET_MM = offset;
      i2cResponse = "OK:HOME_OFFSET:" + String(offset, 2);
    } else {
      i2cResponse = "ERROR:HOME_OFFSET_FORMAT";
    }
  } else if (cmd.startsWith("SET_DEBOUNCE")) {
    uint32_t debounce;
    if (sscanf(cmd.c_str(), "SET_DEBOUNCE %u", &debounce) == 1) {
      LIMIT_DEBOUNCE_MS = debounce;
      i2cResponse = "OK:DEBOUNCE:" + String(debounce);
    } else {
      i2cResponse = "ERROR:DEBOUNCE_FORMAT";
    }
  } else if (cmd.startsWith("SET_TIMEACELYDECEL")) {
    uint32_t time_acel;
    if (sscanf(cmd.c_str(), "SET_TIMEACELYDECEL %u", &time_acel) == 1) {
      time_acelydecel = time_acel;
      i2cResponse = "OK:TIME_ACEL:" + String(time_acel);
    } else {
      i2cResponse = "ERROR:TIME_ACEL_FORMAT";
    }

    // Comandos G-code (enviar directamente a la cola)
  } else if (cmd == "HELP") {
    i2cResponse = "COMANDOS:MOVEX|MOVEY|MOVEZ|SET_X_MM|SET_Y_MM|SET_Z_MM|GET_DIMENSIONS|GET_POSITION|STOP|HOME|RESET|RUNCNC|START_JOG|STOP_JOG|GET_EMERGENCY_STOP|GET_HOME_STATUS|GET_STAT|G0-G91";
  } else {
    i2cResponse = "ERROR:UNKNOWN_COMMAND";
  }

  Serial.println("I2C Procesado: " + command + " -> " + i2cResponse);
}

// Evento cuando el maestro solicita datos
void requestI2CEvent() {
  if (i2cResponse.length() > 0) {
    Wire.write(i2cResponse.c_str());
    Wire.write('\0');  // Agregar terminador nulo
    Serial.println("I2C Enviado al maestro: " + i2cResponse);
  } else {
    Wire.write("NO_DATA");
    Wire.write('\0');  // Agregar terminador nulo
  }
}

// Función para verificar estado I2C
void checkI2CStatus() {
  if (!i2cSlaveInitialized) {
    Serial.println("I2C Esclava no inicializada");
  } else {
    Serial.println("I2C Esclava activa - Dirección: 0x" + String(I2C_SLAVE_ADDRESS, HEX));
  }
}

// ===== Setup =====
void timersInit() {
  // Inicializar estructura AX_X
  AX_X.pinSTEP = X_STEP;
  AX_X.pinDIR = X_DIR;
  AX_X.pos_steps = &x_pos_steps;
  AX_X.span_steps = &x_span_steps;
  AX_X.steps_per_mm = &x_steps_per_mm;
  AX_X.mm_total = x_mm_total;
  AX_X.limH0 = x_limH0;
  AX_X.limHF = x_limHF;

  // Inicializar estructura AX_Y
  AX_Y.pinSTEP = Y_STEP;
  AX_Y.pinDIR = Y_DIR;
  AX_Y.pos_steps = &y_pos_steps;
  AX_Y.span_steps = &y_span_steps;
  AX_Y.steps_per_mm = &y_steps_per_mm;
  AX_Y.mm_total = y_mm_total;
  AX_Y.limH0 = y_limH0;
  AX_Y.limHF = y_limHF;

  // Inicializar estructura AX_Z
  AX_Z.pinSTEP = Z_STEP;
  AX_Z.pinDIR = Z_DIR;
  AX_Z.pos_steps = &z_pos_steps;
  AX_Z.span_steps = &z_span_steps;
  AX_Z.steps_per_mm = &z_steps_per_mm;
  AX_Z.mm_total = z_mm_total;
  AX_Z.limH0 = z_limH0;
  AX_Z.limHF = z_limHF;


  last_x_h0_state = x_limH0();
  last_x_hf_state = x_limHF();
  last_y_h0_state = y_limH0();
  last_y_hf_state = y_limHF();
  last_z_h0_state = z_limH0();
  last_z_hf_state = z_limHF();

  // Configurar timers
  tx = timerBegin(0, 80, true);
  ty = timerBegin(1, 80, true);
  tz = timerBegin(2, 80, true);

  timerAttachInterrupt(tx, &onTimerX, true);
  timerAttachInterrupt(ty, &onTimerY, true);
  timerAttachInterrupt(tz, &onTimerZ, true);

  timerAlarmWrite(tx, 1000, true);
  timerAlarmWrite(ty, 1000, true);
  timerAlarmWrite(tz, 1000, true);

  timerAlarmDisable(tx);
  timerAlarmDisable(ty);
  timerAlarmDisable(tz);
}

// ===== JOG Continuo =====
void startJogContinuous(char axis, bool towardHF, uint32_t hz) {

  if (jogging) {
    stopJogContinuous();
  }

  jogging = true;
  jog_axis = axis;
  jog_direction = towardHF;
  jog_speed = hz;

  //Serial.printf("JOG START: %c %s %d Hz\n", axis, towardHF ? "HF" : "H0", hz);

  // Configurar el eje correspondiente
  volatile Axis* current_axis = nullptr;
  hw_timer_t* current_timer = nullptr;

  if (axis == 'X') {
    current_axis = &AX_X;
    current_timer = tx;
  } else if (axis == 'Y') {
    current_axis = &AX_Y;
    current_timer = ty;
  } else if (axis == 'Z') {
    current_axis = &AX_Z;
    current_timer = tz;
  } else {
    jogging = false;
    return;
  }

  // Configurar movimiento continuo
  uint32_t per = (uint32_t)(1000000.0f / constrain(hz, MIN_HZ, MAX_HZ));

  _dir_fast(current_axis->pinDIR, towardHF);

  current_axis->steps_done = 0;
  current_axis->steps_remaining = 0;  // 0 = movimiento infinito
  current_axis->running = true;
  current_axis->dirFwd = towardHF;
  current_axis->until_limit = false;
  current_axis->period_us = per;

  timer_set_period(current_timer, per);
  timerAlarmEnable(current_timer);

  Serial.println("OK JOG START");
}

void stopJogContinuous() {
  if (jogging) {
    Serial.printf("OK JOG STOP");

    // Detener el eje que está moviéndose
    if (jog_axis == 'X') {
      AX_X.running = false;
      timerAlarmDisable(tx);
    } else if (jog_axis == 'Y') {
      AX_Y.running = false;
      timerAlarmDisable(ty);
    } else if (jog_axis == 'Z') {
      AX_Z.running = false;
      timerAlarmDisable(tz);
    }

    jogging = false;

    // Pequeña pausa para asegurar que se detuvo
    vTaskDelay(pdMS_TO_TICKS(50));
    Serial.println("<< LISTO");
  }
}



// Función para ejecutar un comando G-code y esperar a que termine
void executeGCodeAndWait(const String& gcode) {
  if (gcode.length() == 0) return;

  Serial.println(">> Ejecutando: " + gcode);
  gcodeSubmit(gcode);

  // Esperar a que termine el movimiento actual
  waitAllIdle();

  // Enviar confirmación de que terminó
  Serial.println("<< LISTO");
}

// ===== Comandos por Serial actualizados =====
void pcHandle(const String& s) {
  if (s.startsWith("MOVEX")) {
    uint32_t p, h, a, d, dr;
    if (sscanf(s.c_str(), "MOVEX %u %u %u %u %u", &p, &h, &a, &d, &dr) == 5) {
      x_enqueue({ p, h, a, d, (uint8_t)dr });
      Serial.println(F("OK MOVEX"));
    }
  } else if (s.startsWith("MOVEY")) {
    uint32_t p, h, a, d, dr;
    if (sscanf(s.c_str(), "MOVEY %u %u %u %u %u", &p, &h, &a, &d, &dr) == 5) {
      y_enqueue({ p, h, a, d, (uint8_t)dr });
      Serial.println(F("OK MOVEY"));
    }
  } else if (s.startsWith("MOVEZ")) {
    uint32_t p, h, a, d, dr;
    if (sscanf(s.c_str(), "MOVEZ %u %u %u %u %u", &p, &h, &a, &d, &dr) == 5) {
      z_enqueue({ p, h, a, d, (uint8_t)dr });
      Serial.println(F("OK MOVEZ"));
    }

    // ===== NUEVOS COMANDOS PARA CONFIGURAR DIMENSIONES =====
  } else if (s.startsWith("SET_X_MM")) {
    float mm;
    if (sscanf(s.c_str(), "SET_X_MM %f", &mm) == 1) {
      x_mm_total = mm;
      Serial.printf("OK X_MM:%.2f\n", x_mm_total);
    }
  } else if (s.startsWith("SET_Y_MM")) {
    float mm;
    if (sscanf(s.c_str(), "SET_Y_MM %f", &mm) == 1) {
      y_mm_total = mm;
      Serial.printf("OK Y_MM:%.2f\n", y_mm_total);
    }
  } else if (s.startsWith("SET_Z_MM")) {
    float mm;
    if (sscanf(s.c_str(), "SET_Z_MM %f", &mm) == 1) {
      z_mm_total = mm;
      Serial.printf("OK Z_MM:%.2f\n", z_mm_total);
    }
  } else if (s == "GET_DIMENSIONS") {
    Serial.printf("DIMENSIONS X:%.2f Y:%.2f Z:%.2f\n", x_mm_total, y_mm_total, z_mm_total);

  } else if (s == "STOP") {
    abortSet();
    x_stopNow();
    y_stopNow();
    z_stopNow();
    sequentialMode = false;
    Serial.println(F("OK STOP"));
  } else if (s == "HOME") {
    doHOME();
  } else if (s == "RESET") {
    abortClear();
    sequentialMode = false;
    Serial.println(F("OK RESET"));
  } else if (s == "RUNCNC") {
    rutinaCNC_submit();
  }
  // ELIMINAR el caso de G-code aquí, ya que ahora van directamente a la cola
  else if (s == "SEQUENTIAL_START") {
    sequentialMode = true;
    Serial.println(F("OK MODO SECUENCIAL ACTIVADO"));
  } else if (s == "SEQUENTIAL_STOP") {
    sequentialMode = false;
    Serial.println(F("OK MODO SECUENCIAL DESACTIVADO"));
  } else if (s == "STAT") {
    Serial.printf("Pos X: %.2f mm, Y: %.2f mm, Z: %.2f mm\n",
                  x_pos_steps / x_steps_per_mm,
                  y_pos_steps / y_steps_per_mm,
                  z_pos_steps / z_steps_per_mm);
    Serial.printf("Steps X: %lld, Y: %lld, Z: %lld\n",
                  x_pos_steps, y_pos_steps, z_pos_steps);
    Serial.printf("Modo secuencial: %s\n", sequentialMode ? "ACTIVO" : "INACTIVO");
    Serial.printf("Dimensiones: X:%.2fmm Y:%.2fmm Z:%.2fmm\n", x_mm_total, y_mm_total, z_mm_total);
  } else if (s.startsWith("START_JOG ")) {
    char axis;
    uint8_t direction;
    uint32_t speed = 5000;

    if (sscanf(s.c_str(), "START_JOG %c %hhu %u", &axis, &direction, &speed) >= 2) {
      startJogContinuous(axis, (direction == 1), speed);
    } else {
      Serial.println("ERR: Formato START_JOG incorrecto");
    }
  } else if (s == "STOP_JOG") {
    stopJogContinuous();

    // ===== COMANDOS DE CONSULTA =====
  } else if (s == "GET_EMERGENCY_STOP") {
    if (g_abortSeq) {
      Serial.println(F("EMERGENCY_STOP:true"));
    } else {
      Serial.println(F("EMERGENCY_STOP:false"));
    }

  } else if (s == "GET_HOME_STATUS") {
    if (hasHome()) {
      Serial.println(F("HOME_STATUS:true"));
    } else {
      Serial.println(F("HOME_STATUS:false"));
    }

  } else if (s.startsWith("SET_HOME_OFFSET")) {
    float offset;
    if (sscanf(s.c_str(), "SET_HOME_OFFSET %f", &offset) == 1) {
      CLEAR_HOME_OFFSET_MM = offset;
      Serial.printf("OK HOME_OFFSET:%.2f\n", CLEAR_HOME_OFFSET_MM);
    }
  } else if (s.startsWith("SET_DEBOUNCE")) {
    uint32_t debounce;
    if (sscanf(s.c_str(), "SET_DEBOUNCE %u", &debounce) == 1) {
      LIMIT_DEBOUNCE_MS = debounce;
      Serial.printf("OK DEBOUNCE:%ums\n", LIMIT_DEBOUNCE_MS);
    }
  } else if (s.startsWith("SET_TIMEACELYDECEL")) {
    uint32_t TIEMPODECELACEL;
    if (sscanf(s.c_str(), "SET_TIMEACELYDECEL %u", &TIEMPODECELACEL) == 1) {
      time_acelydecel = TIEMPODECELACEL;
      Serial.printf("OK TIMEACELYDECEL:%ums\n", time_acelydecel);
    }

    // ===== COMANDOS I2C =====
  } else if (s == "I2C_STATUS") {
    checkI2CStatus();

  } else if (s == "I2C_INIT") {
    initI2CSlave();

  } else if (s.startsWith("I2C_TEST")) {
    // Probar procesamiento interno de comandos I2C
    String testCmd = s.substring(8);
    testCmd.trim();
    if (testCmd.length() > 0) {
      processI2CCommand(testCmd);
      Serial.println("I2C Test - Comando: " + testCmd + " | Respuesta: " + i2cResponse);
    } else {
      Serial.println("Uso: I2C_TEST <comando>");
    }

  } else {
    Serial.println(F("CMDS: MOVEX|MOVEY|MOVEZ|SET_X_MM|SET_Y_MM|SET_Z_MM|GET_DIMENSIONS|STOP|HOME|RESET|RUNCNC|G0-G91|STAT|SEQUENTIAL_START|SEQUENTIAL_STOP|START_JOG|STOP_JOG|GET_EMERGENCY_STOP|GET_HOME_STATUS|I2C_STATUS|I2C_INIT|I2C_TEST"));
  }
}

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  esp_task_wdt_init(160, true);
  delay(3000);

  // Configurar pines
  auto prepOut = [&](int pin) {
    pinMode(pin, OUTPUT);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
  };

  prepOut(Enable);
  prepOut(X_STEP);
  prepOut(X_DIR);
  prepOut(Y_STEP);
  prepOut(Y_DIR);
  prepOut(Z_STEP);
  prepOut(Z_DIR);

  pinMode(X_H0, INPUT_PULLUP);
  pinMode(X_HF, INPUT_PULLUP);
  pinMode(Y_H0, INPUT_PULLUP);
  pinMode(Y_HF, INPUT_PULLUP);
  pinMode(Z_H0, INPUT_PULLUP);
  pinMode(Z_HF, INPUT_PULLUP);

  // Inicializar salidas
  _act(Enable, false);
  _act(X_STEP, false);
  _act(X_DIR, false);
  _act(Y_STEP, false);
  _act(Y_DIR, false);
  _act(Z_STEP, false);
  _act(Z_DIR, false);

  timersInit();

  // Inicializar I2C como esclava
  initI2CSlave();

  // Crear colas
  qX = xQueueCreate(8, sizeof(MoveCmd));
  qY = xQueueCreate(8, sizeof(MoveCmd));
  qZ = xQueueCreate(8, sizeof(MoveCmd));
  qG = xQueueCreate(24, sizeof(GCmd));

  // Crear tareas
  xTaskCreatePinnedToCore(TaskG, "GCODE", 4096, NULL, 3, &tG, 1);
  xTaskCreatePinnedToCore(TaskX, "MotionX", 4096, NULL, configMAX_PRIORITIES - 1, &tX, 0);
  xTaskCreatePinnedToCore(TaskY, "MotionY", 4096, NULL, configMAX_PRIORITIES - 1, &tY, 0);
  xTaskCreatePinnedToCore(TaskZ, "MotionZ", 4096, NULL, configMAX_PRIORITIES - 1, &tZ, 0);

#if ENA_ALWAYS_ON
  _act(Enable, true);
#endif

  Serial.println(F("Sistema CNC 3 ejes listo"));
  Serial.println(F("CMDS: MOVEX|MOVEY|MOVEZ|STOP|HOME|RESET|RUNCNC|G0-G91|STAT"));
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      if (buf.length()) {
        // Filtrar comandos: si es comando especial, va directamente a pcHandle
        // Si es G-code, va a la cola G
        if (buf.startsWith("GET_") || buf == "HOME" || buf == "STOP" || buf == "RESET" || buf == "STAT" || buf.startsWith("START_JOG") || buf == "STOP_JOG" || buf == "SEQUENTIAL_START" || buf == "SEQUENTIAL_STOP" || buf.startsWith("MOVEX") || buf.startsWith("MOVEY") || buf.startsWith("MOVEZ") || buf.startsWith("SET_X_MM") || buf.startsWith("SET_Y_MM") || buf.startsWith("SET_Z_MM") || buf.startsWith("SET_HOME_OFFSET") || buf.startsWith("SET_DEBOUNCE") || buf.startsWith("SET_TIMEACELYDECEL") || buf == "GET_DIMENSIONS" || buf == "RUNCNC" || buf.startsWith("I2C_")) {
          // Comandos especiales van directamente a pcHandle
          pcHandle(buf);
        } else if (buf.startsWith("G")) {
          // G-code va a la cola G
          GCmd gcmd{};
          size_t n = min(buf.length(), sizeof(gcmd.line) - 1);
          memcpy(gcmd.line, buf.c_str(), n);
          gcmd.line[n] = '\0';
          xQueueSend(qG, &gcmd, 0);
        } else {
          // Comando no reconocido
          Serial.println("ERR: Comando no reconocido: " + buf);
        }
      }
      buf = "";
    } else {
      buf += c;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
