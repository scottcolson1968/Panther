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
// 0x10 : Data signal of gainChanged
//        bit 31~0 - gainChanged[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of iCurrentRange
//        bit 31~0 - iCurrentRange[31:0] (Read/Write)
// 0x1c : reserved
// 0x60 : Data signal of iOverloadCount
//        bit 31~0 - iOverloadCount[31:0] (Read/Write)
// 0x64 : reserved
// 0x68 : Data signal of OLValid
//        bit 31~0 - OLValid[31:0] (Read)
// 0x6c : Control signal of OLValid
//        bit 0  - OLValid_ap_vld (Read/COR)
//        others - reserved
// 0x78 : Data signal of reset_i
//        bit 31~0 - reset_i[31:0] (Read/Write)
// 0x7c : reserved
// 0x80 : Data signal of reset_o
//        bit 31~0 - reset_o[31:0] (Read)
// 0x84 : Control signal of reset_o
//        bit 0  - reset_o_ap_vld (Read/COR)
//        others - reserved
// 0x88 : Data signal of pointOffset
//        bit 31~0 - pointOffset[31:0] (Read/Write)
// 0x8c : reserved
// 0x20 ~
// 0x3f : Memory 'cal_step' (8 * 32b)
//        Word n : bit [31:0] - cal_step[n]
// 0x40 ~
// 0x5f : Memory 'cal_offset' (8 * 32b)
//        Word n : bit [31:0] - cal_offset[n]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_GAINCHANGED_DATA    0x88
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_GAINCHANGED_DATA    32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_GAINCHANGED_CTRL    0x8c
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_ICURRENTRANGE_DATA  0x10
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_ICURRENTRANGE_DATA  32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_IOVERLOADCOUNT_DATA 0x60
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_IOVERLOADCOUNT_DATA 32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_OLVALID_DATA        0x70
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_OLVALID_DATA        32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_OLVALID_CTRL        0x74
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_RESET_I_DATA        0x18
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_RESET_I_DATA        32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_RESET_O_DATA        0x18
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_RESET_O_DATA        32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_RESET_O_CTRL        0x64
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_POINTOFFSET_DATA    0x80
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_POINTOFFSET_DATA    32 
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE       0x20
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH       0x3f
#define XADCCOUNTSTOFLOAT_CONTROL_WIDTH_CAL_STEP           32
#define XADCCOUNTSTOFLOAT_CONTROL_DEPTH_CAL_STEP           8
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE     0x40
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH     0x5f
#define XADCCOUNTSTOFLOAT_CONTROL_WIDTH_CAL_OFFSET         32
#define XADCCOUNTSTOFLOAT_CONTROL_DEPTH_CAL_OFFSET         8

