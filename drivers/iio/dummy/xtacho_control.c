// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xtacho_control.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XTacho_control_CfgInitialize(XTacho_control *InstancePtr, XTacho_control_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XTacho_control_WriteReg(void __iomem *base, int RegOffset, u32 Data) {
    iowrite32(Data, base + RegOffset);
}

u32 XTacho_control_ReadReg(void __iomem *base, int RegOffset) {
    return ioread32(base + RegOffset);
}


void XTacho_control_Set_in_data_i(XTacho_control *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XTacho_control_WriteReg(InstancePtr->Control_BaseAddress, XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_I_DATA, Data);
}

u32 XTacho_control_Get_in_data_i(XTacho_control *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XTacho_control_ReadReg(InstancePtr->Control_BaseAddress, XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_I_DATA);
    return Data;
}

u32 XTacho_control_Get_in_data_o(XTacho_control *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XTacho_control_ReadReg(InstancePtr->Control_BaseAddress, XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_O_DATA);
    return Data;
}

u32 XTacho_control_Get_in_data_o_vld(XTacho_control *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XTacho_control_ReadReg(InstancePtr->Control_BaseAddress, XTACHO_CONTROL_CONTROL_ADDR_IN_DATA_O_CTRL);
    return Data & 0x1;
}

