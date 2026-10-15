// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XTACHO_CONTROL_H
#define XTACHO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <linux/module.h>   /* Needed by all modules */
#include <linux/kernel.h>   /* Needed for KERN_INFO */
#include <linux/slab.h>
#include <linux/dirent.h>
#include <linux/fcntl.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <linux/string.h>
//#include <sys/mman.h>
//#include <unistd.h>
#include <linux/stddef.h>
#include <linux/fs.h>
//#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/iomap.h>
#include <linux/io.h>
#endif
#include "xtacho_control_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    void __iomem *Control_BaseAddress;
} XTacho_control_Config;
#endif

typedef struct {
    void __iomem *Control_BaseAddress;
    u32 IsReady;
} XTacho_control;

typedef u32 word_type;
#define ASSERT(x)                                                       \
do {    if (x) break;                                                   \
        printk(KERN_EMERG "### ASSERTION FAILED %s: %s: %d: %s\n",      \
               __FILE__, __func__, __LINE__, #x); dump_stack(); BUG();  \
} while (0)
#define assert(x) ASSERT(x)

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XTacho_control_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XTacho_control_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else

void XTacho_control_WriteReg(void __iomem *base, int RegOffset, u32 Data);
u32  XTacho_control_ReadReg(void __iomem *base, int RegOffset);

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XTacho_control_Initialize(XTacho_control *InstancePtr, UINTPTR BaseAddress);
XTacho_control_Config* XTacho_control_LookupConfig(UINTPTR BaseAddress);
#else
int XTacho_control_Initialize(XTacho_control *InstancePtr, u16 DeviceId);
XTacho_control_Config* XTacho_control_LookupConfig(u16 DeviceId);
#endif
int XTacho_control_CfgInitialize(XTacho_control *InstancePtr, XTacho_control_Config *ConfigPtr);
#else
int XTacho_control_Initialize(XTacho_control *InstancePtr, const char* InstanceName);
int XTacho_control_Release(XTacho_control *InstancePtr);
#endif


void XTacho_control_Set_in_data_i(XTacho_control *InstancePtr, u32 Data);
u32 XTacho_control_Get_in_data_i(XTacho_control *InstancePtr);
u32 XTacho_control_Get_in_data_o(XTacho_control *InstancePtr);
u32 XTacho_control_Get_in_data_o_vld(XTacho_control *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
