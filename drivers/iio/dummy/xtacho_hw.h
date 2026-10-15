// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of out_data
//        bit 31~0 - out_data[31:0] (Read)
// 0x14 : Control signal of out_data
//        bit 0  - out_data_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XTACHO_CONTROL_ADDR_OUT_DATA_DATA 0x10
#define XTACHO_CONTROL_BITS_OUT_DATA_DATA 32
#define XTACHO_CONTROL_ADDR_OUT_DATA_CTRL 0x14

