#ifndef SHARED_MEM_H
#define SHARED_MEM_H

#include <stdint.h>

#define SHD_RAM_BASE      0x38000000
#define SHD_BUFFER_SIZE   400

//typedef struct {
//    uint8_t from_m7[SHD_BUFFER_SIZE];
//    uint8_t to_m7[SHD_BUFFER_SIZE];
//    uint32_t flag;
//} SharedMemory;

#define SHARED ((volatile SharedMemory *) SHD_RAM_BASE)
#define ptrStat ((uint8_t *)0x38000000)

#endif
