# Seat-Heater-Control
A robust, multi-seat automotive thermal control system built on FreeRTOS and Texas Instruments TM4C123GH6PM (ARM Cortex-M4), featuring custom MCAL drivers, thread-safe synchronization, temperature hysteresis logic, and hardware-timer runtime profiling.
# 🚗 Real-Time Multi-Seat Heater Control System

> An embedded FreeRTOS-based automotive control system developed as a capstone project for the Advanced Embedded Systems Diploma at **Edges Academy**.

---

## 📌 Project Overview
This project implements a complete, real-time climate control system for driver and passenger seat heaters on the **TM4C123GH6PM (Tiva C LaunchPad)** microcontroller. Moving beyond simple bare-metal code, it leverages a Real-Time Operating System (FreeRTOS) to manage concurrent tasks, synchronize shared hardware resources, enforce strict temperature hysteresis logic, and monitor performance metrics manually using a hardware timer (GPTM).

---

## 🛠️ System Architecture & Tech Stack
* **Microcontroller:** Texas Instruments TM4C123GH6PM (ARM Cortex-M4, 80 MHz)
* **RTOS:** FreeRTOS (Task management, Mutexes, Event Groups, Queues)
* **Language:** Embedded C (MISRA-compliant practices)
* **Peripherals / MCAL:** GPIO, ADC (shared sequencers), UART (9600-8-N-1 serial logging), GPTM (General-Purpose Timer)

---

## ⚙️ Key Technical Features

1. **Modular Layered Architecture:**
   * **MCAL (Microcontroller Abstraction Layer):** Register-level custom drivers (`gpio.c`, `adc.c`, `uart0.c`, `gptm.c`) completely decoupled from FreeRTOS data structures.
   * **Application Layer (`main.c`):** Houses control algorithms, state machines, and task orchestration.

2. **Multitasking & Task Reusability:**
   * Built with **6+ distinct RTOS tasks**, passing data pointers to reuse task code safely across both driver and passenger seats without code duplication:
     * *Button Task (x2):* Event-driven debouncing & heating-level cycling.
     * *Temperature-Sense Task (x2):* Periodic (100 ms) ADC reads with $5^{\circ}\text{C}$–$40^{\circ}\text{C}$ range validation.
     * *Heater-Control Task (x2):* Intensity state machine with dynamic feedback.
     * *Diagnostics Task (x1):* Queue-driven timestamped RAM logging.
     * *Display Task (x1):* Periodic (1000 ms) UART live status broadcaster.
     * *Run-Time Stats Task (x1):* Periodic (5000 ms) CPU load and lock-time profiler.

3. **Precise Synchronization & Concurrency Control:**
   * **Mutexes:** Protected shared peripheral modules (ADC, UART), RAM log arrays, and per-seat state structures from race conditions.
   * **Event Groups:** Zero-polling, edge-triggered button wake-ups connecting multiple physical buttons to shared seat logic.
   * **Queues:** Thread-safe, drop-free diagnostic event passing.

4. **Thermal Management & Hysteresis:**
   * Proportional intensity scaling (Off, Low, Medium, High) driven by temperature differences.
   * Implements a **$2^{\circ}\text{C}$ hysteresis band** to prevent rapid relay/heater toggling around the setpoint.

5. **Manual Run-Time Profiling:**
   * Bypassed built-in RTOS tools to build a custom runtime measurement module utilizing **Task Tags** and **Trace Hook Macros** (`traceTASK_SWITCHED_IN/OUT`) driven by a $0.1\text{ ms}$ free-running hardware timer to track CPU load and resource lock times.

---

## 🔌 Hardware Pin Mapping

| Signal | Pin | Description |
| :--- | :--- | :--- |
| **Driver Console Button** | `PF4` (SW1) | Active low, cycles driver heating level |
| **Passenger Console Button** | `PF0` (SW2) | Active low, cycles passenger heating level |
| **Driver Steering-Wheel Button** | `PB3` (External) | Secondary control, ORed with driver console button |
| **Driver Potentiometer** | `PE3` (AIN0) | Simulated LM35 sensor ($0\text{V}–3.3\text{V} \rightarrow 0^{\circ}\text{C}–45^{\circ}\text{C}$) |
| **Passenger Potentiometer** | `PE2` (AIN1) | Secondary ADC channel for passenger seat |
| **Driver Seat LED** | `PF1/PF2/PF3` | On-board RGB LED status indicator |
| **Passenger Seat LED** | `PB0/PB1/PB2` | External RGB LED status indicator |
| **Serial Monitor** | `UART0` (`PA0/PA1`) | Live telemetry stream ($9600\text{-}8\text{-N-1}$) |

---

## 🚀 Getting Started

### Prerequisites
* **IDE:** Code Composer Studio (CCS) or Keil µVision.
* **Toolchain:** ARM GNU Toolchain.
* **Hardware:** Tiva C LaunchPad (`TM4C123G`) board, connecting wires, potentiometers, and external RGB LEDs.
