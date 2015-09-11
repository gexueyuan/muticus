/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_osal_rtt.h
 @brief  : This is the OS adapter layer realization of RT-thread.
 @author : wangyf
 @history:
           2014-11-12    wangyf    port from project 'cuckoo'. 
           ...
******************************************************************************/
#ifndef __CV_OS_RTT_H__
#define __CV_OS_RTT_H__

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <rthw.h>
#include <rtthread.h>	
#include <board.h>
#include <rtdevice.h>


#define osal_assert(c)          RT_ASSERT(c)
#define osal_printf(f, a...)    rt_kprintf(f, ##a)


#define HZ (RT_TICK_PER_SECOND)

/* legacy definition. */
typedef struct rt_thread        osal_task_t;
typedef struct rt_timer         osal_timer_t;
typedef struct rt_semaphore     osal_sem_t;
typedef struct rt_messagequeue  osal_queue_t;
typedef struct rt_mailbox       osal_mailbox_t;
typedef struct rt_mutex         osal_mutex_t;


/* Components definition for osal. */
typedef struct rt_thread        osal_task_st, *osal_task_st_ptr;
typedef struct rt_timer         osal_timer_st, *osal_timer_st_ptr;
typedef struct rt_semaphore     osal_sem_st, *osal_sem_st_ptr;
typedef struct rt_messagequeue  osal_queue_st, *osal_queue_st_ptr;
typedef struct rt_mailbox       osal_mailbox_st, *osal_mailbox_st_ptr;
typedef struct rt_mutex         osal_mutex_st, *osal_mutex_st_ptr;



typedef  void (*OSAL_TIMER_CALLBACK)(void *arg);

typedef int osal_status_t;

#define OSAL_STATUS_SUCCESS         (RT_EOK)
#define OSAL_STATUS_TIMEOUT         (-RT_ETIMEOUT)
#define OSAL_STATUS_FULL            (-RT_EFULL)
#define OSAL_STATUS_EMPTY           (-RT_EEMPTY)
#define OSAL_STATUS_NOMEM           (-RT_ENOMEM)
#define OSAL_STATUS_ERROR_UNDEFINED (-RT_ERROR)

#define OSAL_WAITING_FOREVER         RT_WAITING_FOREVER  /**< Block forever until get resource. */
#define OSAL_WAITING_NO              RT_WAITING_NO       /**< Non-block. */

#define OSAL_TASK_PRIOPRITY_HIGHEST 10 /* Reserved some for os */
#define OSAL_TASK_PRIOPRITY_LOWEST  30 /* Reserved one for idle task */


/**
 * [Task]
*/
static inline osal_task_t *osal_task_create
(
    /* The name of thread, which shall be unique. */
    const char *name,
        
    /* The entry function of thread. */
    void (*entry)(void *param),
        
    /* The parameter of thread enter function. */
    void *param,
        
    /* The size of thread stack. */
    uint32_t stk_size,
    
    /* The priority of thread. */
    uint32_t prio
)
{
    rt_thread_t task_ptr = RT_NULL;
    
    
    task_ptr = rt_thread_create(name, entry, param, stk_size, prio, 0xFFFFFFF0); 
    if (task_ptr != RT_NULL) 
    {
        rt_thread_startup(task_ptr);
    }
    
    return task_ptr;
}


static __inline osal_status_t osal_task_del(osal_task_t *task)
{
    rt_err_t error;
    error = rt_thread_delete(task);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_task_suspend(osal_task_t *task)
{
    rt_err_t error;
    error = rt_thread_suspend(task);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_task_resume(osal_task_t *task)
{
    rt_err_t error;
    error = rt_thread_resume(task);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

/**
 * [Mutex]
*/
static __inline osal_mutex_t *osal_mutex_create(const char* name)
{
    return rt_mutex_create(name, RT_IPC_FLAG_FIFO);
}

static __inline osal_status_t osal_mutex_delete(osal_mutex_t *mutex)
{
    rt_err_t error;
    error = rt_mutex_delete(mutex);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_mutex_take(osal_mutex_t *mutex, int32_t wait_time)
{
    rt_err_t error;
    osal_status_t status;
    
    error = rt_mutex_take(mutex, wait_time);
    if (error == RT_EOK) {
        status = OSAL_STATUS_SUCCESS;
    }
    else if (error == RT_ETIMEOUT) {
        status = OSAL_STATUS_TIMEOUT;
    }
    else {
        status = OSAL_STATUS_ERROR_UNDEFINED;
    }

    return status;
}

static __inline osal_status_t osal_mutex_release(osal_mutex_t *mutex)
{
    rt_err_t error;
    error = rt_mutex_release(mutex);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

/**
 * [Semphore]
*/
static __inline osal_sem_t *osal_sem_create(const char* name, uint32_t count)
{
    return rt_sem_create(name, count, RT_IPC_FLAG_FIFO);
}

static __inline osal_status_t osal_sem_delete(osal_sem_t *sem)
{
    rt_err_t error;
    error = rt_sem_delete(sem);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_sem_take(osal_sem_t *sem, int32_t wait_time)
{
    rt_err_t error;
    osal_status_t status;
    
    error = rt_sem_take(sem, wait_time);
    if (error == RT_EOK) {
        status = OSAL_STATUS_SUCCESS;
    }
    else if (error == RT_ETIMEOUT) {
        status = OSAL_STATUS_TIMEOUT;
    }
    else {
        status = OSAL_STATUS_ERROR_UNDEFINED;
    }

    return status;
}

static __inline osal_status_t osal_sem_release(osal_sem_t *sem)
{
    rt_err_t error;
    error = rt_sem_release(sem);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_sem_set(osal_sem_t *sem, uint32_t count)
{
    rt_err_t error;
    error = rt_sem_control(sem, RT_IPC_CMD_RESET, (void *)count);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

/**
 * [Message Queue]
 *   The size of message is always 4 bytes(length of pointer).
 *   Every message only carrys the pointer of actual content.
 *   When received a message from queue, you only get a pointer actually.
*/
static __inline osal_queue_t * osal_queue_create(const char* name, uint32_t max_msgs)
{
    return rt_mq_create(name, sizeof(void *), max_msgs, RT_IPC_FLAG_FIFO);
}

static __inline osal_status_t osal_queue_delete(osal_queue_t *queue)
{
    rt_err_t error;
    error = rt_mq_delete(queue);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_queue_send(osal_queue_t *queue, void *msg)
{ 
    rt_err_t error;

    
    error = rt_mq_send(queue, &msg, queue->msg_size);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_queue_recv
(
    osal_queue_t *queue, 
    void *p_msg, 
    int32_t wait_time
)
{
    rt_err_t error;
    osal_status_t status;
    uint32_t msg;

    error = rt_mq_recv(queue, &msg, queue->msg_size, wait_time);
    if (error == RT_EOK) 
    {
        *((uint32_t *)p_msg) = msg;
        status = OSAL_STATUS_SUCCESS;
    }
    else if (error == RT_ETIMEOUT) 
    {
        status = OSAL_STATUS_TIMEOUT;
    }
    else
    {
        status = OSAL_STATUS_ERROR_UNDEFINED;
    }

    return status;
}

/**
 * [Timer]
*/
static __inline osal_timer_t *osal_timer_create(
        const char* name,
        OSAL_TIMER_CALLBACK callback,
        void *callback_arg,
        uint32_t period,
        int repeat)
{
    return rt_timer_create(name, callback, callback_arg, period, \
                                          repeat ? RT_TIMER_FLAG_PERIODIC : RT_TIMER_FLAG_ONE_SHOT);
}

static __inline osal_status_t osal_timer_start(osal_timer_t *timer)
{
    rt_err_t error;
    error = rt_timer_start(timer);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_timer_stop(osal_timer_t *timer)
{
    rt_err_t error;
    error = rt_timer_stop(timer);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}

static __inline osal_status_t osal_timer_change(osal_timer_t *timer, uint32_t period)
{
    rt_err_t error;
    error = rt_timer_control(timer, RT_TIMER_CTRL_SET_TIME, &period);
    return (error == RT_EOK) ? OSAL_STATUS_SUCCESS: OSAL_STATUS_ERROR_UNDEFINED; 
}


/**
 * [Util]
*/
static __inline void osal_delay
(
    /* The delay ticks. */
    uint32_t ticks
)
{
    rt_thread_delay(ticks);
}

static __inline uint32_t osal_get_systemtime(void)
{
    return rt_hw_tick_get_millisecond();
}

/**
 * Notice: The follow IRQ functions are not able to be nested.
 */
static rt_base_t cpu_sr;
static __inline void osal_enter_critical(void)
{
    cpu_sr = rt_hw_interrupt_disable();
}

static __inline void osal_leave_critical(void)
{
    rt_hw_interrupt_enable(cpu_sr);
}

#if OSAL_DMEM_EN > 0
void *osal_malloc(uint32_t size);
void osal_free(void *pointer);
#else
static __inline void *osal_malloc(uint32_t size)
{
    return rt_malloc(size);
}

static __inline void osal_free(void *pointer)
{
    rt_free(pointer);
}
#endif

#endif /* __CV_OS_RTT_H__ */

