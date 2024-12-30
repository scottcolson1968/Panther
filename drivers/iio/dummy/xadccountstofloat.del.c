// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xadccountstofloat.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XAdccountstofloat_CfgInitialize(XAdccountstofloat *InstancePtr, XAdccountstofloat_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
} 
#endif 

void XAdccountstofloat_WriteReg(int BaseAddress, int RegOffset, int Data) {
    void *io = ioremap(BaseAddress+RegOffset, 4); 
  
	writel(Data, io); 
	iounmap(io);
}

int XAdccountstofloat_ReadReg(int BaseAddress, int RegOffset) {
   	void *io = ioremap(BaseAddress+RegOffset, 4); 
 	int Data = readl(io); 
	iounmap(io); 
	return Data;
}

void XAdccountstofloat_Set_iCurrentRange(XAdccountstofloat *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	
    XAdccountstofloat_WriteReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_ICURRENTRANGE_DATA, Data);
} 

void XAdccountstofloat_Set_reset(XAdccountstofloat *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XAdccountstofloat_WriteReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_RESET_DATA, Data);
} 

u32 XAdccountstofloat_Get_iCurrentRange(XAdccountstofloat *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	
    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_ICURRENTRANGE_DATA); 
    return Data;
}

u32 XAdccountstofloat_Get_iOverloadCount(XAdccountstofloat *InstancePtr) {
    u32 Data;
 
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_IOVERLOADCOUNT_DATA);
    return Data;
}

u32 XAdccountstofloat_Get_iOverloadCount_vld(XAdccountstofloat *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_IOVERLOADCOUNT_CTRL);
    return Data & 0x1;
}

u32 XAdccountstofloat_Get_OLValid(XAdccountstofloat *InstancePtr) {
    u32 Data;
 //   printk(KERN_INFO "get_OLValid\n");
    Xil_AssertNonvoid(InstancePtr != NULL);
//    printk(KERN_INFO "get_OLValid non-void\n");
    
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_OLVALID_DATA);
     
    return Data;
}

u32 XAdccountstofloat_Get_cal_step_BaseAddress(XAdccountstofloat *InstancePtr) {
    u32 Data;
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE);
    return Data;    
}

u32 XAdccountstofloat_Get_cal_step_HighAddress(XAdccountstofloat *InstancePtr) {
    u32 Data;
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH);
    return Data;      
}

u32 XAdccountstofloat_Get_cal_step_TotalBytes(XAdccountstofloat *InstancePtr) {    
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
    
    return (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + 1);
}

u32 XAdccountstofloat_Get_cal_step_BitWidth(XAdccountstofloat *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XADCCOUNTSTOFLOAT_CONTROL_WIDTH_CAL_STEP;
}

u32 XAdccountstofloat_Get_cal_step_Depth(XAdccountstofloat *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XADCCOUNTSTOFLOAT_CONTROL_DEPTH_CAL_STEP;
}

u32 XAdccountstofloat_Write_cal_step_Words(XAdccountstofloat *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XAdccountstofloat_WriteReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + (offset + i)*4, *(data + i));
    }
    
    return length;
}

u32 XAdccountstofloat_Read_cal_step_Words(XAdccountstofloat *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
       
        *(data + i) = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + (offset + i)*4);
    }
    return length;
}

u32 XAdccountstofloat_Write_cal_step_Bytes(XAdccountstofloat *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
     
        XAdccountstofloat_WriteReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + (offset + i), *(data + i));
    }
    return length;
}

u32 XAdccountstofloat_Read_cal_step_Bytes(XAdccountstofloat *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        
        *(data + i) = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_STEP_BASE + (offset + i));
    }
    return length;
}

u32 XAdccountstofloat_Get_cal_offset_BaseAddress(XAdccountstofloat *InstancePtr) {
    u32 Data;
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
    Data = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE);
    return Data;
}

u32 XAdccountstofloat_Get_cal_offset_HighAddress(XAdccountstofloat *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH);
}

u32 XAdccountstofloat_Get_cal_offset_TotalBytes(XAdccountstofloat *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + 1);
}

u32 XAdccountstofloat_Get_cal_offset_BitWidth(XAdccountstofloat *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XADCCOUNTSTOFLOAT_CONTROL_WIDTH_CAL_OFFSET;
}

u32 XAdccountstofloat_Get_cal_offset_Depth(XAdccountstofloat *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XADCCOUNTSTOFLOAT_CONTROL_DEPTH_CAL_OFFSET;
}

u32 XAdccountstofloat_Write_cal_offset_Words(XAdccountstofloat *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XAdccountstofloat_WriteReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + (offset + i)*4, *(data + i));
    }
    return length;
}

u32 XAdccountstofloat_Read_cal_offset_Words(XAdccountstofloat *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);
   
    int i;

    if ((offset + length)*4 > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + 1)) {
        printk(KERN_INFO "offset + length to high");
        return 0;
    }
    for (i = 0; i < length; i++) {
        *(data + i) = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + (offset + i)*4);
    }
   
    return length;
}

u32 XAdccountstofloat_Write_cal_offset_Bytes(XAdccountstofloat *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        XAdccountstofloat_WriteReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + (offset + i), *(data + i));
    }
    return length;
}

u32 XAdccountstofloat_Read_cal_offset_Bytes(XAdccountstofloat *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_HIGH - XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = XAdccountstofloat_ReadReg(InstancePtr->Control_BaseAddress, XADCCOUNTSTOFLOAT_CONTROL_ADDR_CAL_OFFSET_BASE + (offset + i));
    }
    return length;
}

