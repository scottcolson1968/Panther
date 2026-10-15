// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
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
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/stddef.h>
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif /* __KERNEL__ */
#endif
#include "xsynch_hw.h"

/**************************** Type Definitions ******************************/
#if defined(__linux__) && !defined(__KERNEL__)
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#elif !defined(__linux__)
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    u64 Control_BaseAddress;
} XSynch_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XSynch;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XSynch_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XSynch_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XSynch_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XSynch_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#ifdef __KERNEL__
#define Xil_AssertVoid(expr)    do { } while (0)
#define Xil_AssertNonvoid(expr) do { } while (0)
#else
#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)
#endif

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XSynch_Initialize(XSynch *InstancePtr, UINTPTR BaseAddress);
XSynch_Config* XSynch_LookupConfig(UINTPTR BaseAddress);
#else
int XSynch_Initialize(XSynch *InstancePtr, u16 DeviceId);
XSynch_Config* XSynch_LookupConfig(u16 DeviceId);
#endif
int XSynch_CfgInitialize(XSynch *InstancePtr, XSynch_Config *ConfigPtr);
#else
int XSynch_Initialize(XSynch *InstancePtr, const char* InstanceName);
int XSynch_Release(XSynch *InstancePtr);
#endif


void XSynch_Set_bufStart(XSynch *InstancePtr, u32 Data);
u32 XSynch_Get_bufStart(XSynch *InstancePtr);
void XSynch_Set_bSpuriousSynch_i(XSynch *InstancePtr, u32 Data);
u32 XSynch_Get_bSpuriousSynch_i(XSynch *InstancePtr);
u32 XSynch_Get_bSpuriousSynch_o(XSynch *InstancePtr);
u32 XSynch_Get_bSpuriousSynch_o_vld(XSynch *InstancePtr);
void XSynch_Set_bRunningAcq(XSynch *InstancePtr, u32 Data);
u32 XSynch_Get_bRunningAcq(XSynch *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
