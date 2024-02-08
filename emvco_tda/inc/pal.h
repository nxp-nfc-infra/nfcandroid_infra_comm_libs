/******************************************************************************
 *
 *  Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of NXP nor the names of its contributors may be used
 * to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef PAL_H_
#define PAL_H_
/** \addtogroup PAL_API
 *  @brief  interface to perform the project/profile(NFC/EMVCo) specific
 * operations
 *  @{
 */
#include <emvco_cl.h>
#include <emvco_common.h>
#include <emvco_dm.h>
#include <emvco_log.h>
#include <emvco_ncif.h>
#include <errno.h>
#include <osal_memory.h>
#include <osal_thread.h>
#include <pthread.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define OSAL_LOG_EMVCOHAL_D(...)                                               \
  { LOG_EMVCOHAL_D(__VA_ARGS__); }
#define OSAL_LOG_EMVCOHAL_W(...)                                               \
  { LOG_EMVCOHAL_W(__VA_ARGS__); }
#define OSAL_LOG_EMVCOHAL_E(...)                                               \
  { LOG_EMVCOHAL_E(__VA_ARGS__); }

/**
 *
 * @brief           This function write the data to NFCC through physical
 *                  interface (e.g. I2C) using the PN7220 driver interface.
 *                  Before sending the data to NFCC, phEMVCoHal_write_ext
 *                  is called to check if there is any extension processing
 *                  is required for the NCI packet being sent out.
 *
 * @param[in]       data_len length of the data to be written
 * @param[in]       p_data actual data to be written
 * @param[in]       is_tda specifies data received from CT or CL
 *
 * @return          uint16_t - returns the number of bytes written to controller
 *
 */
uint16_t ct_osal_write(uint8_t *p_data, uint16_t data_len, bool is_tda);

/**
 *
 * @brief initialize an unnamed semaphore
 *
 * @param[in] sem_t                          Semaphore.
 *
 * @return #0                on success
 * @return #EINVAL           value exceeds SEM_VALUE_MAX
 * @return #ENOSYS           pshared is nonzero, but the system does not support
 *                           process-shared semaphores
 *
 */
int ct_osal_sem_init(sem_t *sem, int pshared, unsigned int value);

/**
 *
 * @brief  Allocates some memoryAllocates some memory
 *
 * @param[in] dwSize   Size, in uint32_t, to be allocated
 *
 * @return            NON-NULL value:  The memory is successfully allocated ;
 *                    the return value is a pointer to the allocated memory
 * location NULL:The operation is not successful.
 *
 */
void *ct_osal_malloc(int dwSize);

/**
 * @brief                Copies the values stored in the source memory to the
 *                       values stored in the destination memory.
 *
 * @param[in] pDest     Pointer to the Destination Memory
 * @param[in] pSrc      Pointer to the Source Memory
 * @param[in] dwSize    Number of bytes to be copied.
 *
 * @return    void
 */
void ct_osal_memcpy(void *pDest, const void *pSrc, int size);

/**
 *
 * @brief get the value of a semaphore
 *
 * @param[in] sem_t          Semaphore.
 *
 * @return #0                on success
 * @return #EINVAL           sem is not a valid semaphore
 *
 */
int ct_osal_sem_getvalue(sem_t *sem, int *sval);

/**
 *
 * @brief unlock a semaphore
 *
 * @param[in] sem_t                          Semaphore.
 *
 * @return #0                on success
 * @return #EINVAL           sem is not a valid semaphore..
 * @return #EOVERFLOW        The maximum allowable value for a semaphore would
 *                           be exceeded
 */
int ct_osal_sem_post(sem_t *sem);

/**
 *
 * @brief lock a semaphore with monotonic timeout
 *
 * @param[in] sem_t                          Semaphore.
 *
 * @return #0             on success
 * @return #EAGAIN        The operation could not be performed
 *                        without blocking
 *
 * @return #EINTR         The call was interrupted by
 *                        a signal handler.
 * @return #EINVAL        sem is not a valid semaphore..
 * @return #EINVAL        The value of abs_timeout.tv_nsecs is
 *                        less than 0, or greater than or
 *                        equal to 1000 million
 * @return #ETIMEDOUT     The call timed out before the semaphore
 *                        could be locked.
 */
int ct_osal_sem_timedwait_monotonic_np(sem_t *__sem,
                                       const struct timespec *__ts)
    __INTRODUCED_IN(28);

/**
 *
 * @brief Notifies the CT init completion
 *
 * @param[in] void
 *
 * @return void
 */
void ct_osal_init_completed();

/**
 *
 * @brief returns the HAL control
 *
 * @param[in] void
 *
 * @return nci_hal_ctrl_t HAL control
 */
nci_hal_ctrl_t ct_osal_get_nci_hal_ctrl();

#ifdef __cplusplus
}
#endif /*  C++ Compilation guard */
/** @}*/
#endif /* PAL_H_ */
