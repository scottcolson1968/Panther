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
// 0x10 : Data signal of in_data_i
//        bit 31~0 - in_data_i[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of in_data_o
//        bit 31~0 - in_data_o[31:0] (Read)
// 0x1c : Control signal of in_data_o
//        bit 0  - in_data_o_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_I_DATA 0x10
#define XTACHO_CONTROL_CONTROL_BITS_IN_DATA_I_DATA 32
#define XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_O_DATA 0x18
#define XTACHO_CONTROL_CONTROL_BITS_IN_DATA_O_DATA 32
#define XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_O_CTRL 0x1c

