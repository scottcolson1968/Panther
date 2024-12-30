// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of ScaleValue
//        bit 31~0 - ScaleValue[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of ZeroCross
//        bit 31~0 - ZeroCross[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of calibratedStep
//        bit 31~0 - calibratedStep[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of calibratedOffset
//        bit 31~0 - calibratedOffset[31:0] (Read/Write)
// 0x2c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA       0x10
#define XDRIVESCALE_CONTROL_BITS_SCALEVALUE_DATA       32
#define XDRIVESCALE_CONTROL_ADDR_ZEROCROSS_DATA        0x18
#define XDRIVESCALE_CONTROL_BITS_ZEROCROSS_DATA        32
#define XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_DATA   0x20
#define XDRIVESCALE_CONTROL_BITS_CALIBRATEDSTEP_DATA   32
#define XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_DATA 0x28
#define XDRIVESCALE_CONTROL_BITS_CALIBRATEDOFFSET_DATA 32

