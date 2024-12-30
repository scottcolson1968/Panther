// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xsynch.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XSynch_CfgInitialize(XSynch *InstancePtr, XSynch_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XSynch_WriteReg(int BaseAddress, int RegOffset, int Data) {
    void *io = ioremap(BaseAddress+RegOffset, 4); 
 //   printk(KERN_INFO "write adc base 0x%x reg 0x%x data 0x%x\n",BaseAddress,RegOffset,Data);
	writel(Data, io); 
	iounmap(io);
}

int XSynch_ReadReg(int BaseAddress, int RegOffset) {
   	void *io = ioremap(BaseAddress+RegOffset, 4); 
 	int Data = readl(io); 
	iounmap(io); 
	return Data;
}

void XSynch_Set_sync(XSynch *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSynch_WriteReg(InstancePtr->Control_BaseAddress, XSYNCH_CONTROL_ADDR_SYNC_DATA, Data);
}

u32 XSynch_Get_sync(XSynch *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSynch_ReadReg(InstancePtr->Control_BaseAddress, XSYNCH_CONTROL_ADDR_SYNC_DATA);
    return Data;
}

