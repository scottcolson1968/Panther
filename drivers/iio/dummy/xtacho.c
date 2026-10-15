// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xtacho.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XTacho_CfgInitialize(XTacho *InstancePtr, XTacho_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XTacho_WriteReg(void __iomem *base, int RegOffset, u32 Data) {
    iowrite32(Data, base + RegOffset);
}

u32 XTacho_ReadReg(void __iomem *base, int RegOffset) {
    return ioread32(base + RegOffset);
}


u32 XTacho_Get_out_data(XTacho *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XTacho_ReadReg(InstancePtr->Control_BaseAddress, XTACHO_CONTROL_ADDR_OUT_DATA_DATA);
    return Data;
}

u32 XTacho_Get_out_data_vld(XTacho *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XTacho_ReadReg(InstancePtr->Control_BaseAddress, XTACHO_CONTROL_ADDR_OUT_DATA_CTRL);
    return Data & 0x1;
}

