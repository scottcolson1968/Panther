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
// 0x10 : Data signal of bufStart
//        bit 31~0 - bufStart[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of bSpuriousSynch_i
//        bit 0  - bSpuriousSynch_i[0] (Read/Write)
//        others - reserved
// 0x1c : reserved
// 0x20 : Data signal of bSpuriousSynch_o
//        bit 0  - bSpuriousSynch_o[0] (Read)
//        others - reserved
// 0x24 : Control signal of bSpuriousSynch_o
//        bit 0  - bSpuriousSynch_o_ap_vld (Read/COR)
//        others - reserved
// 0x28 : Data signal of bRunningAcq
//        bit 0  - bRunningAcq[0] (Read/Write)
//        others - reserved
// 0x2c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XSYNCH_CONTROL_ADDR_BUFSTART_DATA         0x10
#define XSYNCH_CONTROL_BITS_BUFSTART_DATA         32
#define XSYNCH_CONTROL_ADDR_BSPURIOUSSYNCH_I_DATA 0x18
#define XSYNCH_CONTROL_BITS_BSPURIOUSSYNCH_I_DATA 1
#define XSYNCH_CONTROL_ADDR_BSPURIOUSSYNCH_O_DATA 0x20
#define XSYNCH_CONTROL_BITS_BSPURIOUSSYNCH_O_DATA 1
#define XSYNCH_CONTROL_ADDR_BSPURIOUSSYNCH_O_CTRL 0x24
#define XSYNCH_CONTROL_ADDR_BRUNNINGACQ_DATA      0x28
#define XSYNCH_CONTROL_BITS_BRUNNINGACQ_DATA      1

