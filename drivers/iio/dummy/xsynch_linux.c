// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.1 (64-bit)
// Tool Version Limit: 2023.05
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifdef __linux__

/***************************** Include Files *********************************/
#include "xsynch.h"
/***************** Macros (Inline Functions) Definitions *********************/
#define MAX_UIO_PATH_SIZE       256
#define MAX_UIO_NAME_SIZE       64
#define MAX_UIO_MAPS            5
#define UIO_INVALID_ADDR        0

/**************************** Type Definitions ******************************/
typedef struct {
    u32 addr;
    u32 size;
} XSynch_uio_map;

typedef struct {
    int  uio_fd; 
    int  uio_num;
    char name[ MAX_UIO_NAME_SIZE ];
    char version[ MAX_UIO_NAME_SIZE ];
    XSynch_uio_map maps[ MAX_UIO_MAPS ];
} XSynch_uio_info;

/***************** Variable Definitions **************************************/
static XSynch_uio_info uio_info;

/************************** Function Implementation *************************/
static int line_from_file(char* filename, char* linebuf) {
    char* s;
    int i;
	struct file *fp;
    void *in_buf;
    int ret=0;
 
	for(i=0;i<MAX_UIO_NAME_SIZE;i++)
        	linebuf[i] = 0;
	fp = filp_open(filename, O_RDONLY, 0);

    if(fp == NULL)
        printk(KERN_ALERT "filp_open error!!.\n");
    else{
  
       loff_t pos=0;
    //   printk(KERN_INFO "do kernel read");
       ret=kernel_read(fp,linebuf,MAX_UIO_NAME_SIZE,&pos);
       
        // See what we read from file
        printk(KERN_INFO "buf:%s\n",linebuf);
    }
    filp_close(fp,NULL);
  
    for (i=0; (linebuf[i])&&(i<MAX_UIO_NAME_SIZE); i++) 
        if (linebuf[i] == '\n') linebuf[i] = 0;
      
    
    return 0;
}

static int uio_info_read_name(XSynch_uio_info* info) {
    char file[ MAX_UIO_PATH_SIZE ];
    sprintf(file, "/sys/class/uio/uio9/name");//, info->uio_num);
    return line_from_file(file, info->name);
}

static int uio_info_read_version(XSynch_uio_info* info) {
    char file[ MAX_UIO_PATH_SIZE ];
    sprintf(file, "/sys/class/uio/uio9/version");//, info->uio_num);
    return line_from_file(file, info->version);
}

static int uio_info_read_map_addr(XSynch_uio_info* info, int n) {
    int ret;
    char file[ MAX_UIO_PATH_SIZE ];
	char sAddr[80];
	u32 iAddr;
    info->maps[n].addr = UIO_INVALID_ADDR;
    sprintf(file, "/sys/class/uio/uio%d/maps/map%d/addr", 9, n);
 //   FILE* fp = fopen(file, "r");
 //   if (!fp) return -1;
	line_from_file(file, sAddr);
	ret = sscanf(sAddr, "0x%x", &info->maps[n].addr);
//	info->maps[n].addr=bus_to_virt(iAddr);
	printk(KERN_INFO "sAddr:%s phys:%x\n",sAddr,info->maps[n].addr);
//    ret = sscanf(sAddr, "0x%x", &info->maps[n].addr);
 //   fclose(fp);
//printk(KERN_INFO "sAddr:%s\n",sAddr);
 //   if (ret < 0) return -2;
    return 0;
}

static int uio_info_read_map_size(XSynch_uio_info* info, int n) {
    int ret;
    char file[ MAX_UIO_PATH_SIZE ];
	char sAddr[80];
    sprintf(file, "/sys/class/uio/uio%d/maps/map%d/size", 9, n);
 /*   FILE* fp = fopen(file, "r");
    if (!fp) return -1;*/
	line_from_file(file, sAddr);
    ret = sscanf(sAddr, "0x%x", &info->maps[n].size);
	printk(KERN_INFO "sSize:%s\n",sAddr);
 //   fclose(fp);
    if (ret < 0) return -2;
    return 0;
}

int XSynch_Initialize(XSynch *InstancePtr, const char* InstanceName) {
	XSynch_uio_info *InfoPtr = &uio_info;
	struct dirent **namelist;
    int i, n;
    char* s;
    char file[ MAX_UIO_PATH_SIZE ];
    char name[ MAX_UIO_NAME_SIZE ];
    int flag = 0;

    assert(InstancePtr != NULL);

 //   n = scandir("/sys/class/uio", &namelist, 0, alphasort);
 //   if (n < 0)  return XST_DEVICE_NOT_FOUND;
   // for (i = 0;  i < n; i++) {
    	strcpy(file, "/sys/class/uio/uio9/name");
 //   	strcat(file, namelist[i]->d_name);
 //   	strcat(file, "/name");
        if ((line_from_file(file, name) == 0) && (strcmp(name, "Synch") == 0)) {
            flag = 1;
          /*  s = namelist[i]->d_name;
            s += 3; // "uio"*/
            InfoPtr->uio_num = 9;//atoi(s);
	    
		printk(KERN_INFO "name:%s\n",name);
   //         break;
        }
		else printk(KERN_INFO "synch not found at uio9\n");
    
    if (flag == 0)  return XST_DEVICE_NOT_FOUND;

    uio_info_read_name(InfoPtr);
    printk(KERN_INFO "read_name:%s\n",InfoPtr->name);
    uio_info_read_version(InfoPtr);
    printk(KERN_INFO "read_version:%s\n",InfoPtr->version);
    for (n = 0; n < 1; ++n) {
        uio_info_read_map_addr(InfoPtr, n);
        uio_info_read_map_size(InfoPtr, n);
    }

    sprintf(file, "/dev/uio9");//, InfoPtr->uio_num);
 /*   if ((InfoPtr->uio_fd = open(file, O_RDWR)) < 0) {
        return XST_OPEN_DEVICE_FAILED;
    }*/

    // NOTE: slave interface 'Axilites' should be mapped to uioX/map0
    InstancePtr->Control_BaseAddress = InfoPtr->maps[0].addr;//(u32)mmap(NULL, InfoPtr->maps[0].size, PROT_READ|PROT_WRITE, MAP_SHARED, InfoPtr->uio_fd, 0 * getpagesize());
    assert(InstancePtr->Control_BaseAddress);

    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}

int XSynch_Release(XSynch *InstancePtr) {
	XSynch_uio_info *InfoPtr = &uio_info;

    assert(InstancePtr != NULL);
    assert(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

 /*   munmap((void*)InstancePtr->Control_BaseAddress, InfoPtr->maps[0].size);

    close(InfoPtr->uio_fd);*/

    return XST_SUCCESS;
}

#endif
