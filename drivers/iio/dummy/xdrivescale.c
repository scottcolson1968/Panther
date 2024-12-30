// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
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
 //   printk(KERN_INFO "write adc base 0x%x reg 0x%x data 0x%x\n",BaseAddress,RegOffset,Data);
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
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA, Data);
}

u32 XDrivescale_Get_ScaleValue(XDrivescale *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_SCALEVALUE_DATA);
    return Data;
}

void XDrivescale_Set_ZeroCross(XDrivescale *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_ZEROCROSS_DATA, Data);
}

u32 XDrivescale_Get_ZeroCross(XDrivescale *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_ZEROCROSS_DATA);
    return Data;
}

void XDrivescale_Set_samp_index(XDrivescale *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_SAMP_INDEX_DATA, Data);
}

u32 XDrivescale_Get_samp_index(XDrivescale *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_SAMP_INDEX_DATA);
    return Data;
}

void XDrivescale_Set_iCurrentFilter(XDrivescale *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_ICURRENTFILTER_DATA, Data);
}

u32 XDrivescale_Get_iCurrentFilter(XDrivescale *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_ICURRENTFILTER_DATA);
    return Data;
}

u32 XDrivescale_Get_calibratedStep_BaseAddress(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE);
}

u32 XDrivescale_Get_calibratedStep_HighAddress(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_HIGH);
}

u32 XDrivescale_Get_calibratedStep_TotalBytes(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + 1);
}

u32 XDrivescale_Get_calibratedStep_BitWidth(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDRIVESCALE_CONTROL_WIDTH_CALIBRATEDSTEP;
}

u32 XDrivescale_Get_calibratedStep_Depth(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDRIVESCALE_CONTROL_DEPTH_CALIBRATEDSTEP;
}

u32 XDrivescale_Write_calibratedStep_Words(XDrivescale *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + (offset + i)*4, *(data + i));
    }
    return length;
}

u32 XDrivescale_Read_calibratedStep_Words(XDrivescale *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + (offset + i)*4);
    }
    return length;
}

u32 XDrivescale_Write_calibratedStep_Bytes(XDrivescale *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + offset + i, *(data + i));
    }
    return length;
}

u32 XDrivescale_Read_calibratedStep_Bytes(XDrivescale *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDSTEP_BASE + offset + i);
    }
    return length;
}


u32 XDrivescale_Get_calibratedOffset_BaseAddress(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE);
}

u32 XDrivescale_Get_calibratedOffset_HighAddress(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_HIGH);
}

u32 XDrivescale_Get_calibratedOffset_TotalBytes(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + 1);
}

u32 XDrivescale_Get_calibratedOffset_BitWidth(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDRIVESCALE_CONTROL_WIDTH_CALIBRATEDOFFSET;
}

u32 XDrivescale_Get_calibratedOffset_Depth(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDRIVESCALE_CONTROL_DEPTH_CALIBRATEDOFFSET;
}

u32 XDrivescale_Write_calibratedOffset_Words(XDrivescale *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + (offset + i)*4, *(data + i));
    }
    return length;
}

u32 XDrivescale_Read_calibratedOffset_Words(XDrivescale *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + (offset + i)*4);
    }
    return length;
}

u32 XDrivescale_Write_calibratedOffset_Bytes(XDrivescale *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + offset + i, *(data + i));
    }
    return length;
}

u32 XDrivescale_Read_calibratedOffset_Bytes(XDrivescale *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_HIGH - XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_CALIBRATEDOFFSET_BASE + (offset + i));
    }
    return length;
}

u32 XDrivescale_Get_OffsetCounts_BaseAddress(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE);
}

u32 XDrivescale_Get_OffsetCounts_HighAddress(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_HIGH);
}

u32 XDrivescale_Get_OffsetCounts_TotalBytes(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_HIGH - XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + 1);
}

u32 XDrivescale_Get_OffsetCounts_BitWidth(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDRIVESCALE_CONTROL_WIDTH_OFFSETCOUNTS;
}

u32 XDrivescale_Get_OffsetCounts_Depth(XDrivescale *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDRIVESCALE_CONTROL_DEPTH_OFFSETCOUNTS;
}

u32 XDrivescale_Write_OffsetCounts_Words(XDrivescale *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_HIGH - XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + (offset + i)*4, *(data + i));
    }
    return length;
}

u32 XDrivescale_Read_OffsetCounts_Words(XDrivescale *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_HIGH - XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + (offset + i)*4);
    }
    return length;
}

u32 XDrivescale_Write_OffsetCounts_Bytes(XDrivescale *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_HIGH - XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XDrivescale_WriteReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + offset + i, *(data + i));
    }
    return length;
}

u32 XDrivescale_Read_OffsetCounts_Bytes(XDrivescale *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_HIGH - XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XDrivescale_ReadReg(InstancePtr->Control_BaseAddress, XDRIVESCALE_CONTROL_ADDR_OFFSETCOUNTS_BASE + offset + i);
    }
    return length;
}


