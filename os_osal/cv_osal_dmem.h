
#ifndef __CV_OSAL_DMEM_H__
#define __CV_OSAL_DMEM_H__


#if (OSAL_DMEM_EN > 0)

#define OS_DMEM_NAME_SIZE 8
#define OS_DMEM_POOL_SIZE (1024*32)



typedef struct  CS_NODE_STRUCT
{
    struct CS_NODE_STRUCT  *cs_previous;
    struct CS_NODE_STRUCT  *cs_next;
}CS_NODE;

/* Define dynamic memory structs, added by wyf */
typedef unsigned int   UNSIGNED;
typedef unsigned char  CHAR; 
typedef unsigned char  DATA_ELEMENT; 
typedef void           VOID;

#ifndef TRUE
#define TRUE               1
#endif

#ifndef FALSE
#define FALSE              0
#endif

#ifndef NULL
#define NULL               0
#endif

/* Define constants local to this component.  */

#define         DM_DYNAMIC_ID          0x44594e41UL
#define         DM_OVERHEAD            ((sizeof(DM_HEADER) + sizeof(UNSIGNED) \
                                        - 1)/sizeof(UNSIGNED)) *    \
                                        sizeof(UNSIGNED)

#define KL_MAX_NAME   8
#define PAD_1         3
#define R1 	register


/* Define the Dynamic Pool Control Block data type.  */


typedef struct DM_PCB_STRUCT 
{
    CS_NODE             dm_created;            /* Node for linking to    */
                                               /* created dynamic pools  */
    UNSIGNED            dm_id;                 /* Internal PCB ID        */
    CHAR                dm_name[OS_DMEM_NAME_SIZE];  /* Dynamic Pool name      */
    VOID               *dm_start_address;      /* Starting pool address  */
    UNSIGNED            dm_pool_size;          /* Size of pool           */
    UNSIGNED            dm_min_allocation;     /* Minimum allocate size  */
    UNSIGNED            dm_available;          /* Total available bytes  */
    struct DM_HEADER_STRUCT    
                       *dm_memory_list;        /* Memory list            */
    struct DM_HEADER_STRUCT
                       *dm_search_ptr;         /* Search pointer         */
} DM_PCB;    


/* Define the header structure that is in front of each memory block.  */

typedef struct DM_HEADER_STRUCT
{
    struct DM_HEADER_STRUCT
                       *dm_next_memory,        /* Next memory block      */
                       *dm_previous_memory;    /* Previous memory block  */
    DATA_ELEMENT        dm_memory_free;        /* Memory block free flag */
#if     PAD_1
    DATA_ELEMENT        dm_padding[PAD_1];
#endif 
    DM_PCB             *dm_memory_pool;        /* Dynamic pool pointer   */
} DM_HEADER;

#endif /* OSAL_DMEM_EN */

#endif /* __CV_OSAL_DMEM_H__ */

