# UART Communication Between STM32 and Arduino

This repository contains the source code and documentation for **UART Communication Between STM32 and Arduino**,  
where an STM32F410RB microcontroller communicates with an Arduino Uno using UART protocol for serial data exchange.  
The project emphasizes low-level peripheral configuration, USART register-level access, and real-time signal verification using a logic analyzer.

## Features  
- **Custom GPIO and USART Drivers**: Developed from scratch to configure UART communication using direct register manipulation for precision and performance.  
- **Full-Duplex UART Communication**: Enabled bidirectional data exchange between STM32 and Arduino at a specified baud rate.  
- **Button-Triggered Transmission**: On button press, the STM32 sends data to the Arduino over UART. The Arduino echoes the message back.  
- **Signal Integrity Verification**: UART TX/RX signals were captured using a logic analyzer to validate data accuracy and transmission timings.  

## Components Used  
- **STM32 Nucleo Board** (STM32F410RB)  
- **Arduino Board** (Uno/Nano)  
- **Logic Level Shifter** (for voltage compatibility between 3.3V STM32 and 5V Arduino)  
- **Push Button**, **Breadboard**, and **Jumper Wires**  

## Tools Used  
- **STM32CubeIDE**: For code development, debugging, and uploading to STM32  
- **Arduino IDE**: For writing the UART echo program for Arduino  
- **Logic Analyzer**: For real-time signal analysis of UART communication  

## Project Structure  
- **Source Code**:  
  - STM32 firmware with low-level GPIO and USART initialization and transmission logic  
  - Arduino sketch for receiving and echoing UART data  
- **Signal Captures**: UART TX and RX lines captured via logic analyzer to validate transmission  
- **Project Photograph**: Demonstrates hardware setup, wiring, and actual test scenario  
