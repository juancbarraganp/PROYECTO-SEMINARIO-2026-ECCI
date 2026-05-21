# PROYECTO-SEMINARIO-2026-ECCI

# Sistema de Control de Movimiento Triaxial X-Y-Z

Este repositorio contiene el desarrollo de un sistema de control de movimiento lineal de alta precisión para aplicaciones educativas e industriales.  
El proyecto fue desarrollado en la Universidad ECCI como parte del **Proyecto Integrador 2025-II** y posteriormente ampliado durante el **Seminario de Ingeniería Electrónica 2026**.

El sistema implementa el control de los ejes **X, Y y Z** mediante motores paso a paso de bucle cerrado, controlados tanto por un **ESP32 con MicroPython** como por un **PLC Siemens S7-1200** programado en **TIA Portal**.

---

## 📌 Descripción General

La plataforma integra:

- Motores paso a paso Closed-Loop.
- Guías lineales HGR20.
- Tornillos de bolas SFU1204.
- Sensores inductivos PNP.
- Sensores PT100/PT1000.
- Pantalla Nextion con asistencia mediante chatbot.
- Sistema de paro de emergencia industrial.
- Gemelo Digital con simulación industrial.

El sistema fue diseñado para prácticas de automatización, control industrial y manufactura inteligente.

---

# 🖼️ Vista General del Sistema

<p align="center">
  <img src="docs/img/sistema_general.png" width="850">
</p>

---

# ⚙️ Características Principales

- ✅ Control preciso de movimiento en los ejes X-Y-Z.
- ⚙️ Arquitectura híbrida ESP32 + PLC Siemens S7-1200.
- 🧠 Programación en MicroPython y Ladder.
- 📡 Sensores inductivos PNP para límites de recorrido.
- 🌡️ Monitoreo térmico mediante sensores PT100/PT1000.
- 🔒 Sistema de paro de emergencia industrial.
- 🚨 Baliza LED de señalización.
- 🧩 Diseño modular mediante PCB personalizadas.
- 🖥️ Interfaz Nextion con chatbot integrado.
- 🏭 Integración con Gemelo Digital y simulación industrial.

---

# 🎯 Aplicaciones

- Plataformas educativas de automatización industrial.
- Sistemas de control de movimiento CNC.
- Laboratorios de PLC y Microcontroladores.
- Simulación industrial y Gemelo Digital.
- Prototipos de manufactura automatizada.
- Desarrollo de sistemas embebidos industriales.

---

# 🧱 Arquitectura del Sistema

```text
                ┌────────────────────┐
                │   Sensores PNP     │
                └─────────┬──────────┘
                          │
                 ┌────────▼────────┐
                 │ ESP32 / S7-1200 │
                 └────────┬────────┘
                          │ STEP/DIR
                 ┌────────▼────────┐
                 │   Driver CL57T  │
                 └────────┬────────┘
                          │
                 ┌────────▼────────┐
                 │ Motor Stepper   │
                 └────────┬────────┘
                          │
                 ┌────────▼────────┐
                 │ Tornillo SFU1204│
                 └─────────────────┘
