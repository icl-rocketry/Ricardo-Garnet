/*
**********************
* PINS               *
**********************
* Ricardo-Garnet BLDC controller (ESP32-S3, QFN56, U101)
* Traced from Ricardo-Garnet.kicad_sch netlist (rev after commit 4d7588f)
 */
#pragma once

namespace PinMap{

    // --- Gate driver (Mosfet driver sheet, DRV8302 U601, 6-PWM mode) ---
    static constexpr int EnGate = 34;
    static constexpr int H1 = 40;
    static constexpr int L1 = 39;
    static constexpr int H2 = 38;
    static constexpr int L2 = 37;
    static constexpr int H3 = 36;
    static constexpr int L3 = 35;
    static constexpr int DcCal = 41;
    static constexpr int Fault = 42;     // open drain, pull-up R603
    static constexpr int OcTw = 4;       // overcurrent / overtemp warning, open drain, pull-up R605

    // --- Current shunt amps (DRV8302 SO1/SO2) ---
    // NOTE: GPIO33 and GPIO47 have no ADC on the ESP32-S3, so these
    // cannot be sampled as analog inputs on this board revision.
    static constexpr int BrSo1 = 33;
    static constexpr int BrSo2 = 47;

    // --- Phase voltage sense (ADC1) ---
    static constexpr int Sens1 = 1;      // ADC1_CH0 -> SH_C
    static constexpr int Sens2 = 2;      // ADC1_CH1 -> SH_B
    static constexpr int Sens3 = 5;      // ADC1_CH4 -> SH_A

    // --- Misc analog (ADC2 - unusable while WiFi is active) ---
    static constexpr int AnIn = 13;      // V_SUPPLY divider (39k / 2k2)
    static constexpr int MosTemp = 14;   // NTC 10k

    // --- SPI bus (sensors sheet) ---
    static constexpr int H_MISO = 10;
    static constexpr int H_MOSI = 11;
    static constexpr int H_SCLK = 12;

    // --- SPI chip selects ---
    static constexpr int AdcCs = 15;     // net "ACD_CS"
    static constexpr int TcCs_0 = 16;
    static constexpr int TcCs_1 = 21;
    static constexpr int EncoderCs = 48; // J506

    static constexpr int AdcClk = 6;     // net "ACD_CLK"

    // --- Modbus / RS485 ---
    static constexpr int ModDir = 7;
    static constexpr int ModTx = 8;
    static constexpr int ModRx = 9;

    // --- CAN ---
    static constexpr int TxCan = 18;
    static constexpr int RxCan = 17;

    // --- UART0 (test points TP101/TP102, TX LED) ---
    static constexpr int TxD = 43;       // via R107 499R
    static constexpr int RxD = 44;

    // --- USB (native USB-Serial-JTAG) ---
    static constexpr int UsbDm = 19;
    static constexpr int UsbDp = 20;

    // --- Boot ---
    static constexpr int Boot = 0;

    // Unconnected: 3, 45, 46 (26-32 reserved for flash)
};
