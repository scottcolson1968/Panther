// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of iCurrentRange
//        bit 31~0 - iCurrentRange[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of iOverloadCount
//        bit 31~0 - iOverloadCount[31:0] (Read)
// 0x1c : Control signal of iOverloadCount
//        bit 0  - iOverloadCount_ap_vld (Read/COR)
//        others - reserved
// 0x28 : Data signal of reset
//        bit 31~0 - reset[31:0] (Read/Write)
// 0x2c : reserved
// 0x40 ~
// 0x7f : Memory 'cal_step' (16 * 32b)
//        Word n : bit [31:0] - cal_step[n]
// 0x80 ~
// 0xbf : Memory 'cal_offset' (16 * 32b)
//        Word n : bit [31:0] - cal_offset[n]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)
 
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_ICURRENTRANGE_DATA  0x10
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_ICURRENTRANGE_DATA  32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_IOVERLOADCOUNT_DATA 0x60
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_IOVERLOADCOUNT_DATA 32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_IOVERLOADCOUNT_CTRL 0x64
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_OLVALID_DATA          0x70
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_OLVALID_DATA          32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_OLVALID_CTRL          0x74
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_RESET_DATA          0x18
#define XADCCOUNTSTOFLOAT_CONTROL_BITS_RESET_DATA          32
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE       0x20 
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH       0x3f
#define XADCCOUNTSTOFLOAT_CONTROL_WIDTH_CAL_STEP           32
#define XADCCOUNTSTOFLOAT_CONTROL_DEPTH_CAL_STEP           8
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE     0x40
#define XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH     0x5f
#define XADCCOUNTSTOFLOAT_CONTROL_WIDTH_CAL_OFFSET         32
#define XADCCOUNTSTOFLOAT_CONTROL_DEPTH_CAL_OFFSET         8

