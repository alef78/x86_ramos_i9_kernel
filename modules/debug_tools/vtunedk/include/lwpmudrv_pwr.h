/*COPYRIGHT**
 * -------------------------------------------------------------------------
 *               INTEL CORPORATION PROPRIETARY INFORMATION
 *  This software is supplied under the terms of the accompanying license
 *  agreement or nondisclosure agreement with Intel Corporation and may not
 *  be copied or disclosed except in accordance with the terms of that
 *  agreement.
 *        Copyright (c) 2011-2012 Intel Corporation.  All Rights Reserved.
 * -------------------------------------------------------------------------
**COPYRIGHT*/

#ifndef _LWPMUDRV_PWR_H_
#define _LWPMUDRV_PWR_H_

#if defined(__cplusplus)
extern "C" {
#endif

#define MAX_EVENT_NAME_LEN         512
#define MAX_EVENT_DESC_LEN         1024

// Power event groups
enum PWR_EVENT_GROUPS {
    IO_DEV_STATES = 1,
    MMIO_DEV_STATES,
    MMIO_SYS_STATES,
    MMIO_IPC_DEV_RES,
    MMIO_IPC_SYS_RES
};

typedef struct PWR_EVENT_INFO_NODE_S  PWR_EVENT_INFO_NODE;
typedef        PWR_EVENT_INFO_NODE   *PWR_EVENT_INFO;

struct PWR_EVENT_INFO_NODE_S {
    U32                     event_id;
    U32                     group_id;
    char                    name[MAX_EVENT_NAME_LEN];
    char                    desc[MAX_EVENT_DESC_LEN];
    U32                     io_baseaddr1;
    U32                     io_range1;
    U32                     io_baseaddr2;
    U32                     io_range2;
    U32                     offset;
    U32                     virtual_address;
};

#define PWR_EVENT_INFO_event_id(pwr_event)         (pwr_event)->event_id
#define PWR_EVENT_INFO_group_id(pwr_event)         (pwr_event)->group_id
#define PWR_EVENT_INFO_name(pwr_event)             (pwr_event)->name
#define PWR_EVENT_INFO_desc(pwr_event)             (pwr_event)->desc
#define PWR_EVENT_INFO_io_baseaddr1(pwr_event)     (pwr_event)->io_baseaddr1
#define PWR_EVENT_INFO_io_range1(pwr_event)        (pwr_event)->io_range1
#define PWR_EVENT_INFO_io_baseaddr2(pwr_event)     (pwr_event)->io_baseaddr2
#define PWR_EVENT_INFO_io_range2(pwr_event)        (pwr_event)->io_range2
#define PWR_EVENT_INFO_offset(pwr_event)           (pwr_event)->offset
#define PWR_EVENT_INFO_virtual_address(pwr_event)  (pwr_event)->virtual_address

// IPC register offsets
#define IPC_BASE_ADDRESS                0xFF11C000
#define IPC_CMD_OFFSET                  0x00000000
#define IPC_STS_OFFSET                  0x00000004
#define IPC_SPTR_OFFSET                 0x00000008
#define IPC_DPTR_OFFSET                 0x0000000C
#define IPC_WBUF_OFFSET                 0x00000080
#define IPC_RBUF_OFFSET                 0x00000090
#define IPC_MAX_ADDR                    0x100

// Write 3bytes in IPC_WBUF (2bytes for address and 1byte for value)
#define IPC_ADC_WRITE_1                 0x000300FF
// Write 2bytes in IPC_WBUF (2bytes for address) and read 1byte from IPC_RBUF
#define IPC_ADC_READ_1                  0x000210FF

// IPC commands
#define IPC_MESSAGE_MSIC                0xFF
#define IPC_MESSAGE_CC                  0xEF
#define IPC_MESSAGE_D_RESIDENCY         0xEA
#define IPC_MESSAGE_S_RESIDENCY         0xEB
 
// IPC subcommands
#define IPC_COMMAND_WRITE               0x0
#define IPC_COMMAND_READ                0x1
#define IPC_COMMAND_START_RESIDENCY     0x0
#define IPC_COMMAND_STOP_RESIDENCY      0x1
#define IPC_COMMAND_DUMP_RESIDENCY      0x2
 
// IPC commands for S state residency counter
#define S_RESIDENCY_BASE_ADDRESS        0xFFFF71E0
#define S_RESIDENCY_MAX_COUNTERS        0x4
#define S_RESIDENCY_MAX_STATES          0x3
// IPC commands for D state residency counter
#define D_RESIDENCY_BASE_ADDRESS        0xFFFF7000
#define D_RESIDENCY_MAX_COUNTERS        0x78    // 40 LSS * 3 D states = 120
#define D_RESIDENCY_MAX_STATES          0x3
#define D_RESIDENCY_MAX_LSS             0x28    // 40 LSS

#if defined(__cplusplus)
}
#endif

#endif  /* _LWPMUDRV_PWR_H_ */
