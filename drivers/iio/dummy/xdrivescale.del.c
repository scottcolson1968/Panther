// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xdrivescale.h"
 
/************************** Function Implementation *************************/
#ifndef __linux__
int XDrivescale_CfgInitialize(XDrivescale *InstancePtr, XDrivescale_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XDrivescale_WriteReg(int BaseAddress, int RegOffset, int Data) {
    void *io = ioremap(BaseAddress+RegOffset, 4); 
    printk(KERN_INFO "write adc base 0x%x reg 0x%x data 0x%x\n",BaseAddress,RegOffset,Data);
	writel(Data, io); 
	iounmap(io);
}

int XDrivescale_ReadReg(int BaseAddress, int RegOffset) {
   	void *io = ioremap(BaseAddress+RegOffset, 4); 
 	int Data = readl(io); 
	iounmap(io); 
	return Data;
}

void XDrivescale_Set_ScaleValue(XDrivescale *InstancePtr, u32 Data) {
 //   printk(KERN_INFO "remap address for write\n");
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	
	void *io = ioremap(InstancePtr->Control_BaseAddress+XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA, 4); 
	writel(Data, io); 
	iounmap(io);
 //   XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA, Data);
}

u32 XDrivescale_Get_ScaleValue(XDrivescale *InstancePtr) {
    u32 Data;
//	printk(KERN_INFO "remap address for read\n");
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	
	void *io = ioremap(InstancePtr->Control_BaseAddress+XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA, 4); 
	Data = readl(io); 
	iounmap(io);
	
 //   Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA);
    return Data;
}

void XDrivescale_Set_ZeroCross(XDrivescale *InstancePtr, u32 Data) {
     Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	
	void *io = ioremap(InstancePtr->Control_BaseAddress+XDRIVESCALE_CONTROL_ADDR_ZEROCROSS_DATA, 4); 
	writel(Data, io); 
	iounmap(io);
 }

u32 XDrivescale_Get_ZeroCross(XDrivescale *InstancePtr) {
    u32 Data;
   Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	
	void *io = ioremap(InstancePtr->Control_BaseAddress+XDRIVESCALE_CONTROL_ADDR_ZEROCROSS_DATA, 4); 
	Data = readl(io); 
	iounmap(io);
	
    return Data;
}

void XDrivescale_Set_calibratedStep(XDrivescale *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_DATA, Data);
}

u32 XDrivescale_Get_calibratedStep(XDrivescale *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_DATA);
    return Data;
}

void XDrivescale_Set_calibratedOffset(XDrivescale *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_DATA, Data);
}

u32 XDrivescale_Get_calibratedOffset(XDrivescale *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_DATA);
    return Data;
}


