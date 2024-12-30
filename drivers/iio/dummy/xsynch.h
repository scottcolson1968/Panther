// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XSYNCH_H
#define XSYNCH_H

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
#include "xsynch_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u64 Control_BaseAddress;
} XSynch_Config;
#endif

typedef struct {
    u32 Control_BaseAddress;
    u32 IsReady;
} XSynch;

typedef u32 word_type;
#define ASSERT(x)                                                       \
do {    if (x) break;                                                   \
        printk(KERN_EMERG "### ASSERTION FAILED %s: %s: %d: %s\n",      \
               __FILE__, __func__, __LINE__, #x); dump_stack(); BUG();  \
} while (0)
#define assert(x) ASSERT(x)
/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XSynch_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XSynch_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XSynch_Initialize(XSynch *InstancePtr, u16 DeviceId);
XSynch_Config* XSynch_LookupConfig(u16 DeviceId);
int XSynch_CfgInitialize(XSynch *InstancePtr, XSynch_Config *ConfigPtr);
#else
int XSynch_Initialize(XSynch *InstancePtr, const char* InstanceName);
int XSynch_Release(XSynch *InstancePtr);
#endif


void XSynch_Set_sync(XSynch *InstancePtr, u32 Data);
u32 XSynch_Get_sync(XSynch *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
