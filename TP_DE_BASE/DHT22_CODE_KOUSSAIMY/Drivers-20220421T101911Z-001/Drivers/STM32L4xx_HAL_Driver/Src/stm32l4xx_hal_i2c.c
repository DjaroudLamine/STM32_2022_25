/**
  ******************************************************************************
  * @file    stm32l4xx_hal_i2c.c
  * @author  MCD Application Team
  * @brief   I2C HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Inter Integrated Circuit (I2C) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
    The I2C HAL driver can be used as follows:

    (#) Declare a I2C_HandleTypeDef handle structure, for example:
        I2C_HandleTypeDef  hi2c;

    (#)Initialize the I2C low level resources by implementing the HAL_I2C_MspInit() API:
        (##) Enable the I2Cx interface clock
        (##) I2C pins configuration
            (+++) Enable the clock for the I2C GPIOs
            (+++) Configure I2C pins as alternate function open-drain
        (##) NVIC configuration if you need to use interrupt process
            (+++) Configure the I2Cx interrupt priority
            (+++) Enable the NVIC I2C IRQ Channel
        (##) DMA Configuration if you need to use DMA process
            (+++) Declare a DMA_HandleTypeDef handle structure for
                  the transmit or receive channel
            (+++) Enable the DMAx interface clock using
            (+++) Configure the DMA handle parameters
            (+++) Configure the DMA Tx or Rx channel
            (+++) Associate the initialized DMA handle to the hi2c DMA Tx or Rx handle
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on
                  the DMA Tx or Rx channel

    (#) Configure the Communication Clock Timing, Own Address1, Master Addressing mode, Dual Addressing mode,
        Own Address2, Own Address2 Mask, General call and Nostretch mode in the hi2c Init structure.

    (#) Initialize the I2C registers by calling the HAL_I2C_Init(), configures also the low level Hardware
        (GPIO, CLOCK, NVIC...etc) by calling the customized HAL_I2C_MspInit(&hi2c) API.

    (#) To check if target device is ready for communication, use the function HAL_I2C_IsDeviceReady()

    (#) For I2C IO and IO MEM operations, three operation modes are available within this driver :

    *** Polling mode IO operation ***
    =================================
    [..]
      (+) Transmit in master mode an amount of data in blocking mode using HAL_I2C_Master_Transmit()
      (+) Receive in master mode an amount of data in blocking mode using HAL_I2C_Master_Receive()
      (+) Transmit in slave mode an amount of data in blocking mode using HAL_I2C_Slave_Transmit()
      (+) Receive in slave mode an amount of data in blocking mode using HAL_I2C_Slave_Receive()

    *** Polling mode IO MEM operation ***
    =====================================
    [..]
      (+) Write an amount of data in blocking mode to a specific memory address using HAL_I2C_Mem_Write()
      (+) Read an amount of data in blocking mode from a specific memory address using HAL_I2C_Mem_Read()


    *** Interrupt mode IO operation ***
    ===================================
    [..]
      (+) Transmit in master mode an amount of data in non-blocking mode using HAL_I2C_Master_Transmit_IT()
      (+) At transmission end of transfer, HAL_I2C_MasterTxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MasterTxCpltCallback()
      (+) Receive in master mode an amount of data in non-blocking mode using HAL_I2C_Master_Receive_IT()
      (+) At reception end of transfer, HAL_I2C_MasterRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MasterRxCpltCallback()
      (+) Transmit in slave mode an amount of data in non-blocking mode using HAL_I2C_Slave_Transmit_IT()
      (+) At transmission end of transfer, HAL_I2C_SlaveTxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_SlaveTxCpltCallback()
      (+) Receive in slave mode an amount of data in non-blocking mode using HAL_I2C_Slave_Receive_IT()
      (+) At reception end of transfer, HAL_I2C_SlaveRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_SlaveRxCpltCallback()
      (+) In case of transfer Error, HAL_I2C_ErrorCallback() function is executed and users can
           add their own code by customization of function pointer HAL_I2C_ErrorCallback()
      (+) Abort a master I2C process communication with Interrupt using HAL_I2C_Master_Abort_IT()
      (+) End of abort process, HAL_I2C_AbortCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_AbortCpltCallback()
      (+) Discard a slave I2C process communication using __HAL_I2C_GENERATE_NACK() macro.
           This action will inform Master to generate a Stop condition to discard the communication.


    *** Interrupt mode or DMA mode IO sequential operation ***
    ==========================================================
    [..]
      (@) These interfaces allow to manage a sequential transfer with a repeated start condition
          when a direction change during transfer
    [..]
      (+) A specific option field manage the different steps of a sequential transfer
      (+) Option field values are defined through I2C_XFEROPTIONS and are listed below:
      (++) I2C_FIRST_AND_LAST_FRAME: No sequential usage, functional is same as associated interfaces in
           no sequential mode
      (++) I2C_FIRST_FRAME: Sequential usage, this option allow to manage a sequence with start condition, address
                            and data to transfer without a final stop condition
      (++) I2C_FIRST_AND_NEXT_FRAME: Sequential usage (Master only), this option allow to manage a sequence with
                            start condition, address and data to transfer without a final stop condition,
                            an then permit a call the same master sequential interface several times
                            (like HAL_I2C_Master_Seq_Transmit_IT() then HAL_I2C_Master_Seq_Transmit_IT()
                            or HAL_I2C_Master_Seq_Transmit_DMA() then HAL_I2C_Master_Seq_Transmit_DMA())
      (++) I2C_NEXT_FRAME: Sequential usage, this option allow to manage a sequence with a restart condition, address
                            and with new data to transfer if the direction change or manage only the new data to
                            transfer
                            if no direction change and without a final stop condition in both cases
      (++) I2C_LAST_FRAME: Sequential usage, this option allow to manage a sequance with a restart condition, address
                            and with new data to transfer if the direction change or manage only the new data to
                            transfer
                            if no direction change and with a final stop condition in both cases
      (++) I2C_LAST_FRAME_NO_STOP: Sequential usage (Master only), this option allow to manage a restart condition
                            after several call of the same master sequential interface several times
                            (link with option I2C_FIRST_AND_NEXT_FRAME).
                            Usage can, transfer several bytes one by one using
                              HAL_I2C_Master_Seq_Transmit_IT
                              or HAL_I2C_Master_Seq_Receive_IT
                              or HAL_I2C_Master_Seq_Transmit_DMA
                              or HAL_I2C_Master_Seq_Receive_DMA
                              with option I2C_FIRST_AND_NEXT_FRAME then I2C_NEXT_FRAME.
                             Then usage of this option I2C_LAST_FRAME_NO_STOP at the last Transmit or
                              Receive sequence permit to call the opposite interface Receive or Transmit
                              without stopping the communication and so generate a restart condition.
      (++) I2C_OTHER_FRAME: Sequential usage (Master only), this option allow to manage a restart condition after
                            each call of the same master sequential
                            interface.
                            Usage can, transfer several bytes one by one with a restart with slave address between
                            each bytes using
                              HAL_I2C_Master_Seq_Transmit_IT
                              or HAL_I2C_Master_Seq_Receive_IT
                              or HAL_I2C_Master_Seq_Transmit_DMA
                              or HAL_I2C_Master_Seq_Receive_DMA
                              with option I2C_FIRST_FRAME then I2C_OTHER_FRAME.
                            Then usage of this option I2C_OTHER_AND_LAST_FRAME at the last frame to help automatic
                            generation of STOP condition.

      (+) Different sequential I2C interfaces are listed below:
      (++) Sequential transmit in master I2C mode an amount of data in non-blocking mode using
            HAL_I2C_Master_Seq_Transmit_IT() or using HAL_I2C_Master_Seq_Transmit_DMA()
      (+++) At transmission end of current frame transfer, HAL_I2C_MasterTxCpltCallback() is executed and
            users can add their own code by customization of function pointer HAL_I2C_MasterTxCpltCallback()
      (++) Sequential receive in master I2C mode an amount of data in non-blocking mode using
            HAL_I2C_Master_Seq_Receive_IT() or using HAL_I2C_Master_Seq_Receive_DMA()
      (+++) At reception end of current frame transfer, HAL_I2C_MasterRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MasterRxCpltCallback()
      (++) Abort a master IT or DMA I2C process communication with Interrupt using HAL_I2C_Master_Abort_IT()
      (+++) End of abort process, HAL_I2C_AbortCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_AbortCpltCallback()
      (++) Enable/disable the Address listen mode in slave I2C mode using HAL_I2C_EnableListen_IT()
            HAL_I2C_DisableListen_IT()
      (+++) When address slave I2C match, HAL_I2C_AddrCallback() is executed and users can
           add their own code to check the Address Match Code and the transmission direction request by master
           (Write/Read).
      (+++) At Listen mode end HAL_I2C_ListenCpltCallback() is executed and users can
          add their own code by customization of function pointer HAL_I2C_ListenCpltCallback()
      (++) Sequential transmit in slave I2C mode an amount of data in non-blocking mode using
            HAL_I2C_Slave_Seq_Transmit_IT() or using HAL_I2C_Slave_Seq_Transmit_DMA()
      (+++) At transmission end of current frame transfer, HAL_I2C_SlaveTxCpltCallback() is executed and
            users can add their own code by customization of function pointer HAL_I2C_SlaveTxCpltCallback()
      (++) Sequential receive in slave I2C mode an amount of data in non-blocking mode using
            HAL_I2C_Slave_Seq_Receive_IT() or using HAL_I2C_Slave_Seq_Receive_DMA()
      (+++) At reception end of current frame transfer, HAL_I2C_SlaveRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_SlaveRxCpltCallback()
      (++) In case of transfer Error, HAL_I2C_ErrorCallback() function is executed and users can
           add their own code by customization of function pointer HAL_I2C_ErrorCallback()
      (++) Discard a slave I2C process communication using __HAL_I2C_GENERATE_NACK() macro.
           This action will inform Master to generate a Stop condition to discard the communication.

    *** Interrupt mode IO MEM operation ***
    =======================================
    [..]
      (+) Write an amount of data in non-blocking mode with Interrupt to a specific memory address using
          HAL_I2C_Mem_Write_IT()
      (+) At Memory end of write transfer, HAL_I2C_MemTxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MemTxCpltCallback()
      (+) Read an amount of data in non-blocking mode with Interrupt from a specific memory address using
          HAL_I2C_Mem_Read_IT()
      (+) At Memory end of read transfer, HAL_I2C_MemRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MemRxCpltCallback()
      (+) In case of transfer Error, HAL_I2C_ErrorCallback() function is executed and users can
           add their own code by customization of function pointer HAL_I2C_ErrorCallback()

    *** DMA mode IO operation ***
    ==============================
    [..]
      (+) Transmit in master mode an amount of data in non-blocking mode (DMA) using
          HAL_I2C_Master_Transmit_DMA()
      (+) At transmission end of transfer, HAL_I2C_MasterTxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MasterTxCpltCallback()
      (+) Receive in master mode an amount of data in non-blocking mode (DMA) using
          HAL_I2C_Master_Receive_DMA()
      (+) At reception end of transfer, HAL_I2C_MasterRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MasterRxCpltCallback()
      (+) Transmit in slave mode an amount of data in non-blocking mode (DMA) using
          HAL_I2C_Slave_Transmit_DMA()
      (+) At transmission end of transfer, HAL_I2C_SlaveTxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_SlaveTxCpltCallback()
      (+) Receive in slave mode an amount of data in non-blocking mode (DMA) using
          HAL_I2C_Slave_Receive_DMA()
      (+) At reception end of transfer, HAL_I2C_SlaveRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_SlaveRxCpltCallback()
      (+) In case of transfer Error, HAL_I2C_ErrorCallback() function is executed and users can
           add their own code by customization of function pointer HAL_I2C_ErrorCallback()
      (+) Abort a master I2C process communication with Interrupt using HAL_I2C_Master_Abort_IT()
      (+) End of abort process, HAL_I2C_AbortCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_AbortCpltCallback()
      (+) Discard a slave I2C process communication using __HAL_I2C_GENERATE_NACK() macro.
           This action will inform Master to generate a Stop condition to discard the communication.

    *** DMA mode IO MEM operation ***
    =================================
    [..]
      (+) Write an amount of data in non-blocking mode with DMA to a specific memory address using
          HAL_I2C_Mem_Write_DMA()
      (+) At Memory end of write transfer, HAL_I2C_MemTxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MemTxCpltCallback()
      (+) Read an amount of data in non-blocking mode with DMA from a specific memory address using
          HAL_I2C_Mem_Read_DMA()
      (+) At Memory end of read transfer, HAL_I2C_MemRxCpltCallback() is executed and users can
           add their own code by customization of function pointer HAL_I2C_MemRxCpltCallback()
      (+) In case of transfer Error, HAL_I2C_ErrorCallback() function is executed and users can
           add their own code by customization of function pointer HAL_I2C_ErrorCallback()


     *** I2C HAL driver macros list ***
     ==================================
     [..]
       Below the list of most used macros in I2C HAL driver.

      (+) __HAL_I2C_ENABLE: Enable the I2C peripheral
      (+) __HAL_I2C_DISABLE: Disable the I2C peripheral
      (+) __HAL_I2C_GENERATE_NACK: Generate a Non-Acknowledge I2C peripheral in Slave mode
      (+) __HAL_I2C_GET_FLAG: Check whether the specified I2C flag is set or not
      (+) __HAL_I2C_CLEAR_FLAG: Clear the specified I2C pending flag
      (+) __HAL_I2C_ENABLE_IT: Enable the specified I2C interrupt
      (+) __HAL_I2C_DISABLE_IT: Disable the specified I2C interrupt

     *** Callback registration ***
     =============================================
    [..]
     The compilation flag USE_HAL_I2C_REGISTER_CALLBACKS when set to 1
     allows the user to configure dynamically the driver callbacks.
     Use Functions HAL_I2C_RegisterCallback() or HAL_I2C_RegisterAddrCallback()
     to register an interrupt callback.
    [..]
     Function HAL_I2C_RegisterCallback() allows to register following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) MemTxCpltCallback    : callback for Memory transmission end of transfer.
       (+) MemRxCpltCallback    : callback for Memory reception end of transfer.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
     This function takes as parameters the HAL peripheral handle, the Callback ID
     and a pointer to the user callback function.
    [..]
     For specific callback AddrCallback use dedicated register callbacks : HAL_I2C_RegisterAddrCallback().
    [..]
     Use function HAL_I2C_UnRegisterCallback to reset a callback to the default
     weak function.
     HAL_I2C_UnRegisterCallback takes as parameters the HAL peripheral handle,
     and the Callback ID.
     This function allows to reset following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) MemTxCpltCallback    : callback for Memory transmission end of transfer.
       (+) MemRxCpltCallback    : callback for Memory reception end of transfer.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
    [..]
     For callback AddrCallback use dedicated register callbacks : HAL_I2C_UnRegisterAddrCallback().
    [..]
     By default, after the HAL_I2C_Init() and when the state is HAL_I2C_STATE_RESET
     all callbacks are set to the corresponding weak functions:
     examples HAL_I2C_MasterTxCpltCallback(), HAL_I2C_MasterRxCpltCallback().
     Exception done for MspInit and MspDeInit functions that are
     reset to the legacy weak functions in the HAL_I2C_Init()/ HAL_I2C_DeInit() only when
     these callbacks are null (not registered beforehand).
     If MspInit or MspDeInit are not null, the HAL_I2C_Init()/ HAL_I2C_DeInit()
     keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.
    [..]
     Callbacks can be registered/unregistered in HAL_I2C_STATE_READY state only.
     Exception done MspInit/MspDeInit functions that can be registered/unregistered
     in HAL_I2C_STATE_READY or HAL_I2C_STATE_RESET state,
     thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
     Then, the user first registers the MspInit/MspDeInit user callbacks
     using HAL_I2C_RegisterCallback() before calling HAL_I2C_DeInit()
     or HAL_I2C_Init() function.
    [..]
     When the compilation flag USE_HAL_I2C_REGISTER_CALLBACKS is set to 0 or
     not defined, the callback registration feature is not available and all callbacks
     are set to the corresponding weak functions.

     [..]
       (@) You can refer to the I2C HAL driver header file for more useful macros

  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup I2C I2C
  * @brief I2C HAL module driver
  * @{
  */

#ifdef HAL_I2C_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup I2C_Private_Define I2C Private Define
  * @{
  */
#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)  /*!< I2C TIMING clear register Mask */
#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_DIR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOPF   (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TC      (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TCR     (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TXIS    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

#define MAX_NBYTE_SIZE      255U
#define SLAVE_ADDR_SHIFT     7U
#define SLAVE_ADDR_MSK       0x06U

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK             ((uint32_t)((uint32_t)((uint32_t)HAL_I2C_STATE_BUSY_TX | \
                                                         (uint32_t)HAL_I2C_STATE_BUSY_RX) & \
                                              (uint32_t)(~((uint32_t)HAL_I2C_STATE_READY))))
/*!< Mask State define, keep only RX and TX bits */
#define I2C_STATE_NONE            ((uint32_t)(HAL_I2C_MODE_NONE))
/*!< Default Value */
#define I2C_STATE_MASTER_BUSY_TX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | \
                                              (uint32_t)HAL_I2C_MODE_MASTER))
/*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | \
                                              (uint32_t)HAL_I2C_MODE_MASTER))
/*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | \
                                              (uint32_t)HAL_I2C_MODE_SLAVE))
/*!< Slave Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_RX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | \
                                              (uint32_t)HAL_I2C_MODE_SLAVE))
/*!< Slave Busy RX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_MEM_BUSY_TX     ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | \
                                              (uint32_t)HAL_I2C_MODE_MEM))
/*!< Memory Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MEM_BUSY_RX     ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | \
                                              (uint32_t)HAL_I2C_MODE_MEM))
/*!< Memory Busy RX, combinaison of State LSB and Mode enum */


/* Private define to centralize the enable/disable of Interrupts */
#define I2C_XFER_TX_IT          (uint16_t)(0x0001U)   /*!< Bit field can be combinated with
                                                         @ref I2C_XFER_LISTEN_IT */
#define I2C_XFER_RX_IT          (uint16_t)(0x0002U)   /*!< Bit field can be combinated with
                                                         @ref I2C_XFER_LISTEN_IT */
#define I2C_XFER_LISTEN_IT      (uint16_t)(0x8000U)   /*!< Bit field can be combinated with @ref I2C_XFER_TX_IT
                                                         and @ref I2C_XFER_RX_IT */

#define I2C_XFER_ERROR_IT       (uint16_t)(0x0010U)   /*!< Bit definition to manage addition of global Error
                                                         and NACK treatment */
#define I2C_XFER_CPLT_IT        (uint16_t)(0x0020U)   /*!< Bit definition to manage only STOP evenement */
#define I2C_XFER_RELOAD_IT      (uint16_t)(0x0040U)   /*!< Bit definition to manage only Reload of NBYTE */

/* Private define Sequential Transfer Options default/reset value */
#define I2C_NO_OPTION_FRAME     (0xFFFF0000U)
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Macro to get remaining data to transfer on DMA side */
#define I2C_GET_DMA_REMAIN_DATA(__HANDLE__)     __HAL_DMA_GET_COUNTER(__HANDLE__)

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */
/* Private functions to handle DMA transfer */
static void I2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAError(DMA_HandleTypeDef *hdma);
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma);

/* Private functions to handle IT transfer */
static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITMasterSeqCplt(I2C_HandleTypeDef *hi2c);
static void I2C_ITSlaveSeqCplt(I2C_HandleTypeDef *hi2c);
static void I2C_ITMasterCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITListenCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_ITError(I2C_HandleTypeDef *hi2c, uint32_t ErrorCode);

/* Private functions to handle IT transfer */
static HAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                                uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout,
                                                uint32_t Tickstart);
static HAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                               uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout,
                                               uint32_t Tickstart);

/* Private functions for I2C transfer IRQ handler */
static HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                           uint32_t ITSources);
static HAL_StatusTypeDef I2C_Slave_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                          uint32_t ITSources);
static HAL_StatusTypeDef I2C_Master_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                            uint32_t ITSources);
static HAL_StatusTypeDef I2C_Slave_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                           uint32_t ITSources);

/* Private functions to handle flags during polling transfer */
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout,
                                                        uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout,
                                                        uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout,
                                                        uint32_t Tickstart);
static HAL_StatusTypeDef I2C_IsErrorOccurred(I2C_HandleTypeDef *hi2c, uint32_t Timeout,
                                             uint32_t Tickstart);

/* Private functions to centralize the enable/disable of Interrupts */
static void I2C_Enable_IRQ(I2C_HandleTypeDef *hi2c, uint16_t InterruptRequest);
static void I2C_Disable_IRQ(I2C_HandleTypeDef *hi2c, uint16_t InterruptRequest);

/* Private function to treat different error callback */
static void I2C_TreatErrorCallback(I2C_HandleTypeDef *hi2c);

/* Private function to flush TXDR register */
static void I2C_Flush_TXDR(I2C_HandleTypeDef *hi2c);

/* Private function to handle  start, restart or stop a transfer */
static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode,
                               uint32_t Request);

/* Private function to Convert Specific options */
static void I2C_ConvertOtherXferOptions(I2C_HandleTypeDef *hi2c);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the I2Cx peripheral:

      (+) User must Implement HAL_I2C_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_I2C_Init() to configure the selected device with
          the selected configuration:
        (++) Clock Timing
        (++) Own Address 1
        (++) Addressing mode (Master, Slave)
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) Own Address 2 Mask
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function HAL_I2C_DeInit() to restore the default configuration
          of the selected I2Cx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2C according to the specified parameters
  *         in the I2C_InitTypeDef and initialize the associated handle.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_I2C_OWN_ADDRESS1(hi2c->Init.OwnAddress1));
  assert_param(IS_I2C_ADDRESSING_MODE(hi2c->Init.AddressingMode));
  assert_param(IS_I2C_DUAL_ADDRESS(hi2c->Init.DualAddressMode));
  assert_param(IS_I2C_OWN_ADDRESS2(hi2c->Init.OwnAddress2));
  assert_param(IS_I2C_OWN_ADDRESS2_MASK(hi2c->Init.OwnAddress2Masks));
  assert_param(IS_I2C_GENERAL_CALL(hi2c->Init.GeneralCallMode));
  assert_param(IS_I2C_NO_STRETCH(hi2c->Init.NoStretchMode));

  if (hi2c->State == HAL_I2C_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hi2c->Lock = HAL_UNLOCKED;

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    /* Init the I2C Callback settings */
    hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
    hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
    hi2c->SlaveTxCpltCallback  = HAL_I2C_SlaveTxCpltCallback;  /* Legacy weak SlaveTxCpltCallback  */
    hi2c->SlaveRxCpltCallback  = HAL_I2C_SlaveRxCpltCallback;  /* Legacy weak SlaveRxCpltCallback  */
    hi2c->ListenCpltCallback   = HAL_I2C_ListenCpltCallback;   /* Legacy weak ListenCpltCallback   */
    hi2c->MemTxCpltCallback    = HAL_I2C_MemTxCpltCallback;    /* Legacy weak MemTxCpltCallback    */
    hi2c->MemRxCpltCallback    = HAL_I2C_MemRxCpltCallback;    /* Legacy weak MemRxCpltCallback    */
    hi2c->ErrorCallback        = HAL_I2C_ErrorCallback;        /* Legacy weak ErrorCallback        */
    hi2c->AbortCpltCallback    = HAL_I2C_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
    hi2c->AddrCallback         = HAL_I2C_AddrCallback;         /* Legacy weak AddrCallback         */

    if (hi2c->MspInitCallback == NULL)
    {
      hi2c->MspInitCallback = HAL_I2C_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    hi2c->MspInitCallback(hi2c);
#else
    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    HAL_I2C_MspInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }

  hi2c->State = HAL_I2C_STATE_BUSY;

  /* Disable the selected I2C peripheral */
  __HAL_I2C_DISABLE(hi2c);

  /*---------------------------- I2Cx TIMINGR Configuration ------------------*/
  /* Configure I2Cx: Frequency range */
  hi2c->Instance->TIMINGR = hi2c->Init.Timing & TIMING_CLEAR_MASK;

  /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Disable Own Address1 before set the Own Address1 configuration */
  hi2c->Instance->OAR1 &= ~I2C_OAR1_OA1EN;

  /* Configure I2Cx: Own Address1 and ack own address1 mode */
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | hi2c->Init.OwnAddress1);
  }
  else /* I2C_ADDRESSINGMODE_10BIT */
  {
    hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | I2C_OAR1_OA1MODE | hi2c->Init.OwnAddress1);
  }

  /*---------------------------- I2Cx CR2 Configuration ----------------------*/
  /* Configure I2Cx: Addressing Master mode */
  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
  {
    hi2c->Instance->CR2 = (I2C_CR2_ADD10);
  }
  /* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
  hi2c->Instance->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK);

  /*---------------------------- I2Cx OAR2 Configuration ---------------------*/
  /* Disable Own Address2 before set the Own Address2 configuration */
  hi2c->Instance->OAR2 &= ~I2C_DUALADDRESS_ENABLE;

  /* Configure I2Cx: Dual mode and Own Address2 */
  hi2c->Instance->OAR2 = (hi2c->Init.DualAddressMode | hi2c->Init.OwnAddress2 | \
                          (hi2c->Init.OwnAddress2Masks << 8));

  /*---------------------------- I2Cx CR1 Configuration ----------------------*/
  /* Configure I2Cx: Generalcall and NoStretch mode */
  hi2c->Instance->CR1 = (hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode);

  /* Enable the selected I2C peripheral */
  __HAL_I2C_ENABLE(hi2c);

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_READY;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = HAL_I2C_MODE_NONE;

  return HAL_OK;
}

/**
  * @brief  DeInitialize the I2C peripheral.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));

  hi2c->State = HAL_I2C_STATE_BUSY;

  /* Disable the I2C Peripheral Clock */
  __HAL_I2C_DISABLE(hi2c);

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
  if (hi2c->MspDeInitCallback == NULL)
  {
    hi2c->MspDeInitCallback = HAL_I2C_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  hi2c->MspDeInitCallback(hi2c);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  HAL_I2C_MspDeInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_RESET;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = HAL_I2C_MODE_NONE;

  /* Release Lock */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief Initialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspInit could be implemented in the user file
   */
}

/**
  * @brief DeInitialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspDeInit could be implemented in the user file
   */
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User I2C Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_RegisterCallback(I2C_HandleTypeDef *hi2c, HAL_I2C_CallbackIDTypeDef CallbackID,
                                           pI2C_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hi2c);

  if (HAL_I2C_STATE_READY == hi2c->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_MASTER_TX_COMPLETE_CB_ID :
        hi2c->MasterTxCpltCallback = pCallback;
        break;

      case HAL_I2C_MASTER_RX_COMPLETE_CB_ID :
        hi2c->MasterRxCpltCallback = pCallback;
        break;

      case HAL_I2C_SLAVE_TX_COMPLETE_CB_ID :
        hi2c->SlaveTxCpltCallback = pCallback;
        break;

      case HAL_I2C_SLAVE_RX_COMPLETE_CB_ID :
        hi2c->SlaveRxCpltCallback = pCallback;
        break;

      case HAL_I2C_LISTEN_COMPLETE_CB_ID :
        hi2c->ListenCpltCallback = pCallback;
        break;

      case HAL_I2C_MEM_TX_COMPLETE_CB_ID :
        hi2c->MemTxCpltCallback = pCallback;
        break;

      case HAL_I2C_MEM_RX_COMPLETE_CB_ID :
        hi2c->MemRxCpltCallback = pCallback;
        break;

      case HAL_I2C_ERROR_CB_ID :
        hi2c->ErrorCallback = pCallback;
        break;

      case HAL_I2C_ABORT_CB_ID :
        hi2c->AbortCpltCallback = pCallback;
        break;

      case HAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = pCallback;
        break;

      case HAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (HAL_I2C_STATE_RESET == hi2c->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = pCallback;
        break;

      case HAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c);
  return status;
}

/**
  * @brief  Unregister an I2C Callback
  *         I2C callback is redirected to the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_UnRegisterCallback(I2C_HandleTypeDef *hi2c, HAL_I2C_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hi2c);

  if (HAL_I2C_STATE_READY == hi2c->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_MASTER_TX_COMPLETE_CB_ID :
        hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
        break;

      case HAL_I2C_MASTER_RX_COMPLETE_CB_ID :
        hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
        break;

      case HAL_I2C_SLAVE_TX_COMPLETE_CB_ID :
        hi2c->SlaveTxCpltCallback = HAL_I2C_SlaveTxCpltCallback;   /* Legacy weak SlaveTxCpltCallback  */
        break;

      case HAL_I2C_SLAVE_RX_COMPLETE_CB_ID :
        hi2c->SlaveRxCpltCallback = HAL_I2C_SlaveRxCpltCallback;   /* Legacy weak SlaveRxCpltCallback  */
        break;

      case HAL_I2C_LISTEN_COMPLETE_CB_ID :
        hi2c->ListenCpltCallback = HAL_I2C_ListenCpltCallback;     /* Legacy weak ListenCpltCallback   */
        break;

      case HAL_I2C_MEM_TX_COMPLETE_CB_ID :
        hi2c->MemTxCpltCallback = HAL_I2C_MemTxCpltCallback;       /* Legacy weak MemTxCpltCallback    */
        break;

      case HAL_I2C_MEM_RX_COMPLETE_CB_ID :
        hi2c->MemRxCpltCallback = HAL_I2C_MemRxCpltCallback;       /* Legacy weak MemRxCpltCallback    */
        break;

      case HAL_I2C_ERROR_CB_ID :
        hi2c->ErrorCallback = HAL_I2C_ErrorCallback;               /* Legacy weak ErrorCallback        */
        break;

      case HAL_I2C_ABORT_CB_ID :
        hi2c->AbortCpltCallback = HAL_I2C_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
        break;

      case HAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = HAL_I2C_MspInit;                   /* Legacy weak MspInit              */
        break;

      case HAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = HAL_I2C_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (HAL_I2C_STATE_RESET == hi2c->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = HAL_I2C_MspInit;                   /* Legacy weak MspInit              */
        break;

      case HAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = HAL_I2C_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c);
  return status;
}

/**
  * @brief  Register the Slave Address Match I2C Callback
  *         To be used instead of the weak HAL_I2C_AddrCallback() predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pCallback pointer to the Address Match Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_RegisterAddrCallback(I2C_HandleTypeDef *hi2c, pI2C_AddrCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hi2c);

  if (HAL_I2C_STATE_READY == hi2c->State)
  {
    hi2c->AddrCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c);
  return status;
}

/**
  * @brief  UnRegister the Slave Address Match I2C Callback
  *         Info Ready I2C Callback is redirected to the weak HAL_I2C_AddrCallback() predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_UnRegisterAddrCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hi2c);

  if (HAL_I2C_STATE_READY == hi2c->State)
  {
    hi2c->AddrCallback = HAL_I2C_AddrCallback; /* Legacy weak AddrCallback  */
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c);
  return status;
}

#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
  *  @brief   Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_I2C_Master_Transmit()
        (++) HAL_I2C_Master_Receive()
        (++) HAL_I2C_Slave_Transmit()
        (++) HAL_I2C_Slave_Receive()
        (++) HAL_I2C_Mem_Write()
        (++) HAL_I2C_Mem_Read()
        (++) HAL_I2C_IsDeviceReady()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_I2C_Master_Transmit_IT()
        (++) HAL_I2C_Master_Receive_IT()
        (++) HAL_I2C_Slave_Transmit_IT()
        (++) HAL_I2C_Slave_Receive_IT()
        (++) HAL_I2C_Mem_Write_IT()
        (++) HAL_I2C_Mem_Read_IT()
        (++) HAL_I2C_Master_Seq_Transmit_IT()
        (++) HAL_I2C_Master_Seq_Receive_IT()
        (++) HAL_I2C_Slave_Seq_Transmit_IT()
        (++) HAL_I2C_Slave_Seq_Receive_IT()
        (++) HAL_I2C_EnableListen_IT()
        (++) HAL_I2C_DisableListen_IT()
        (++) HAL_I2C_Master_Abort_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_I2C_Master_Transmit_DMA()
        (++) HAL_I2C_Master_Receive_DMA()
        (++) HAL_I2C_Slave_Transmit_DMA()
        (++) HAL_I2C_Slave_Receive_DMA()
        (++) HAL_I2C_Mem_Write_DMA()
        (++) HAL_I2C_Mem_Read_DMA()
        (++) HAL_I2C_Master_Seq_Transmit_DMA()
        (++) HAL_I2C_Master_Seq_Receive_DMA()
        (++) HAL_I2C_Slave_Seq_Transmit_DMA()
        (++) HAL_I2C_Slave_Seq_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2C_MasterTxCpltCallback()
        (++) HAL_I2C_MasterRxCpltCallback()
        (++) HAL_I2C_SlaveTxCpltCallback()
        (++) HAL_I2C_SlaveRxCpltCallback()
        (++) HAL_I2C_MemTxCpltCallback()
        (++) HAL_I2C_MemRxCpltCallback()
        (++) HAL_I2C_AddrCallback()
        (++) HAL_I2C_ListenCpltCallback()
        (++) HAL_I2C_ErrorCallback()
        (++) HAL_I2C_AbortCpltCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                          uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE,
                         I2C_GENERATE_START_WRITE);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_WRITE);
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until TXIS flag is set */
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }
      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE,
                             I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                             I2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                         uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE,
                         I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_READ);
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE,
                             I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                             I2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmits in slave mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                         uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Wait until ADDR flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    /* If 10bit addressing mode is selected */
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
    {
      /* Wait until ADDR flag is set */
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hi2c->Instance->CR2 |= I2C_CR2_NACK;
        return HAL_ERROR;
      }

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
    }

    /* Wait until DIR flag is set Transmitter mode */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until TXIS flag is set */
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hi2c->Instance->CR2 |= I2C_CR2_NACK;
        return HAL_ERROR;
      }

      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
    }

    /* Wait until AF flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_AF, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Flush TX register */
    I2C_Flush_TXDR(hi2c);

    /* Clear AF flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Wait until STOP flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;

      return HAL_ERROR;
    }

    /* Clear STOP flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Disable Address Acknowledge */
    hi2c->Instance->CR2 |= I2C_CR2_NACK;

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                        uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferSize = hi2c->XferCount;
    hi2c->XferISR   = NULL;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Wait until ADDR flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    /* Wait until DIR flag is reset Receiver mode */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hi2c->Instance->CR2 |= I2C_CR2_NACK;

        /* Store Last receive data if any */
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
        {
          /* Read data from RXDR */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          hi2c->XferCount--;
          hi2c->XferSize--;
        }

        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;
    }

    /* Wait until STOP flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Clear STOP flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Disable Address Acknowledge */
    hi2c->Instance->CR2 |= I2C_CR2_NACK;

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size)
{
  uint32_t xfermode;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_GENERATE_START_WRITE);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
      I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size)
{
  uint32_t xfermode;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_GENERATE_START_READ);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
      I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Slave_ISR_IT;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
      I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT | I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Slave_ISR_IT;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
      I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                              uint16_t Size)
{
  uint32_t xfermode;
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_DMA;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmatx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferCpltCallback = I2C_DMAMasterTransmitCplt;

        /* Set the DMA error callback */
        hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmatx->XferHalfCpltCallback = NULL;
        hi2c->hdmatx->XferAbortCallback = NULL;

        /* Enable the DMA channel */
        dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR,
                                         hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }

      if (dmaxferstatus == HAL_OK)
      {
        /* Send Slave Address */
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_GENERATE_START_WRITE);

        /* Update XferCount value */
        hi2c->XferCount -= hi2c->XferSize;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        /* Enable ERR and NACK interrupts */
        I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

        /* Enable DMA Request */
        hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
    else
    {
      /* Update Transfer ISR function pointer */
      hi2c->XferISR = I2C_Master_ISR_IT;

      /* Send Slave Address */
      /* Set NBYTES to write and generate START condition */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_WRITE);

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
        I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
      I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size)
{
  uint32_t xfermode;
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_DMA;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmarx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmarx->XferCpltCallback = I2C_DMAMasterReceiveCplt;

        /* Set the DMA error callback */
        hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmarx->XferHalfCpltCallback = NULL;
        hi2c->hdmarx->XferAbortCallback = NULL;

        /* Enable the DMA channel */
        dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData,
                                         hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }

      if (dmaxferstatus == HAL_OK)
      {
        /* Send Slave Address */
        /* Set NBYTES to read and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_GENERATE_START_READ);

        /* Update XferCount value */
        hi2c->XferCount -= hi2c->XferSize;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        /* Enable ERR and NACK interrupts */
        I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

        /* Enable DMA Request */
        hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
    else
    {
      /* Update Transfer ISR function pointer */
      hi2c->XferISR = I2C_Master_ISR_IT;

      /* Send Slave Address */
      /* Set NBYTES to read and generate START condition */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_READ);

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
        I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
      I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Slave_ISR_DMA;

    if (hi2c->hdmatx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmatx->XferCpltCallback = I2C_DMASlaveTransmitCplt;

      /* Set the DMA error callback */
      hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmatx->XferHalfCpltCallback = NULL;
      hi2c->hdmatx->XferAbortCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR,
                                       hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Enable Address Acknowledge */
      hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, STOP, NACK, ADDR interrupts */
      I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

      /* Enable DMA Request */
      hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Slave_ISR_DMA;

    if (hi2c->hdmarx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmarx->XferCpltCallback = I2C_DMASlaveReceiveCplt;

      /* Set the DMA error callback */
      hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmarx->XferHalfCpltCallback = NULL;
      hi2c->hdmarx->XferAbortCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData,
                                       hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Enable Address Acknowledge */
      hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, STOP, NACK, ADDR interrupts */
      I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

      /* Enable DMA Request */
      hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                    uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
    }

    do
    {
      /* Wait until TXIS flag is set */
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE,
                             I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                             I2C_NO_STARTSTOP);
        }
      }

    } while (hi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE,
                         I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_READ);
    }

    do
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
          hi2c->XferSize = MAX_NBYTE_SIZE;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t) hi2c->XferSize, I2C_RELOAD_MODE,
                             I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                             I2C_NO_STARTSTOP);
        }
      }
    } while (hi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart;
  uint32_t xfermode;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart)
        != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_NO_STARTSTOP);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
      I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                      uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart;
  uint32_t xfermode;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_GENERATE_START_READ);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
      I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                        uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart;
  uint32_t xfermode;
  HAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_DMA;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart)
        != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }


    if (hi2c->hdmatx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmatx->XferCpltCallback = I2C_DMAMasterTransmitCplt;

      /* Set the DMA error callback */
      hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmatx->XferHalfCpltCallback = NULL;
      hi2c->hdmatx->XferAbortCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR,
                                       hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_READY;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Send Slave Address */
      /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_NO_STARTSTOP);

      /* Update XferCount value */
      hi2c->XferCount -= hi2c->XferSize;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR and NACK interrupts */
      I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

      /* Enable DMA Request */
      hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_READY;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be read
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                       uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  uint32_t tickstart;
  uint32_t xfermode;
  HAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Master_ISR_DMA;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = I2C_AUTOEND_MODE;
    }

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    if (hi2c->hdmarx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmarx->XferCpltCallback = I2C_DMAMasterReceiveCplt;

      /* Set the DMA error callback */
      hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmarx->XferHalfCpltCallback = NULL;
      hi2c->hdmarx->XferAbortCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData,
                                       hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_READY;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, I2C_GENERATE_START_READ);

      /* Update XferCount value */
      hi2c->XferCount -= hi2c->XferSize;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR and NACK interrupts */
      I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

      /* Enable DMA Request */
      hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_READY;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials,
                                        uint32_t Timeout)
{
  uint32_t tickstart;

  __IO uint32_t I2C_Trials = 0UL;

  FlagStatus tmp1;
  FlagStatus tmp2;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    do
    {
      /* Generate Start */
      hi2c->Instance->CR2 = I2C_GENERATE_START(hi2c->Init.AddressingMode, DevAddress);

      /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set or a NACK flag is set*/
      tickstart = HAL_GetTick();

      tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
      tmp2 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);

      while ((tmp1 == RESET) && (tmp2 == RESET))
      {
        if (Timeout != HAL_MAX_DELAY)
        {
          if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
          {
            /* Update I2C state */
            hi2c->State = HAL_I2C_STATE_READY;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
          }
        }

        tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
        tmp2 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);
      }

      /* Check if the NACKF flag has not been set */
      if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == RESET)
      {
        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Device is ready */
        hi2c->State = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
      }
      else
      {
        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        /* Clear NACK Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Clear STOP Flag, auto generated with autoend*/
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
      }

      /* Check if the maximum allowed number of trials has been reached */
      if (I2C_Trials == Trials)
      {
        /* Generate Stop */
        hi2c->Instance->CR2 |= I2C_CR2_STOP;

        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
      }

      /* Increment Trials */
      I2C_Trials++;
    } while (I2C_Trials < Trials);

    /* Update I2C state */
    hi2c->State = HAL_I2C_STATE_READY;

    /* Update I2C error code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_ERROR;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with Interrupt.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions)
{
  uint32_t xfermode;
  uint32_t xferrequest = I2C_GENERATE_START_WRITE;

  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = hi2c->XferOptions;
    }

    /* If transfer direction not change and there is no request to start another frame,
       do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_TX) && \
        (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
    {
      xferrequest = I2C_NO_STARTSTOP;
    }
    else
    {
      /* Convert OTHER_xxx XferOptions if any */
      I2C_ConvertOtherXferOptions(hi2c);

      /* Update xfermode accordingly if no reload is necessary */
      if (hi2c->XferCount <= MAX_NBYTE_SIZE)
      {
        xfermode = hi2c->XferOptions;
      }
    }

    /* Send Slave Address and set NBYTES to write */
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with DMA.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                  uint16_t Size, uint32_t XferOptions)
{
  uint32_t xfermode;
  uint32_t xferrequest = I2C_GENERATE_START_WRITE;
  HAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Master_ISR_DMA;

    /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = hi2c->XferOptions;
    }

    /* If transfer direction not change and there is no request to start another frame,
       do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_TX) && \
        (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
    {
      xferrequest = I2C_NO_STARTSTOP;
    }
    else
    {
      /* Convert OTHER_xxx XferOptions if any */
      I2C_ConvertOtherXferOptions(hi2c);

      /* Update xfermode accordingly if no reload is necessary */
      if (hi2c->XferCount <= MAX_NBYTE_SIZE)
      {
        xfermode = hi2c->XferOptions;
      }
    }

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmatx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferCpltCallback = I2C_DMAMasterTransmitCplt;

        /* Set the DMA error callback */
        hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmatx->XferHalfCpltCallback = NULL;
        hi2c->hdmatx->XferAbortCallback = NULL;

        /* Enable the DMA channel */
        dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR,
                                         hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }

      if (dmaxferstatus == HAL_OK)
      {
        /* Send Slave Address and set NBYTES to write */
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

        /* Update XferCount value */
        hi2c->XferCount -= hi2c->XferSize;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        /* Enable ERR and NACK interrupts */
        I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

        /* Enable DMA Request */
        hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
    else
    {
      /* Update Transfer ISR function pointer */
      hi2c->XferISR = I2C_Master_ISR_IT;

      /* Send Slave Address */
      /* Set NBYTES to write and generate START condition */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_WRITE);

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
        I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
      I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                uint16_t Size, uint32_t XferOptions)
{
  uint32_t xfermode;
  uint32_t xferrequest = I2C_GENERATE_START_READ;

  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Master_ISR_IT;

    /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = hi2c->XferOptions;
    }

    /* If transfer direction not change and there is no request to start another frame,
       do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_RX) && \
        (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
    {
      xferrequest = I2C_NO_STARTSTOP;
    }
    else
    {
      /* Convert OTHER_xxx XferOptions if any */
      I2C_ConvertOtherXferOptions(hi2c);

      /* Update xfermode accordingly if no reload is necessary */
      if (hi2c->XferCount <= MAX_NBYTE_SIZE)
      {
        xfermode = hi2c->XferOptions;
      }
    }

    /* Send Slave Address and set NBYTES to read */
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                 uint16_t Size, uint32_t XferOptions)
{
  uint32_t xfermode;
  uint32_t xferrequest = I2C_GENERATE_START_READ;
  HAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Master_ISR_DMA;

    /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
      xfermode = I2C_RELOAD_MODE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      xfermode = hi2c->XferOptions;
    }

    /* If transfer direction not change and there is no request to start another frame,
       do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_RX) && \
        (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
    {
      xferrequest = I2C_NO_STARTSTOP;
    }
    else
    {
      /* Convert OTHER_xxx XferOptions if any */
      I2C_ConvertOtherXferOptions(hi2c);

      /* Update xfermode accordingly if no reload is necessary */
      if (hi2c->XferCount <= MAX_NBYTE_SIZE)
      {
        xfermode = hi2c->XferOptions;
      }
    }

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmarx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmarx->XferCpltCallback = I2C_DMAMasterReceiveCplt;

        /* Set the DMA error callback */
        hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmarx->XferHalfCpltCallback = NULL;
        hi2c->hdmarx->XferAbortCallback = NULL;

        /* Enable the DMA channel */
        dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData,
                                         hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }

      if (dmaxferstatus == HAL_OK)
      {
        /* Send Slave Address and set NBYTES to read */
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

        /* Update XferCount value */
        hi2c->XferCount -= hi2c->XferSize;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        /* Enable ERR and NACK interrupts */
        I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

        /* Enable DMA Request */
        hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
    else
    {
      /* Update Transfer ISR function pointer */
      hi2c->XferISR = I2C_Master_ISR_IT;

      /* Send Slave Address */
      /* Set NBYTES to read and generate START condition */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                         I2C_GENERATE_START_READ);

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI |
        I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
      I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions)
{
  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave RX state to TX state */
    if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      /* Disable associated Interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

      /* Abort DMA Xfer if any */
      if ((hi2c->Instance->CR1 & I2C_CR1_RXDMAEN) == I2C_CR1_RXDMAEN)
      {
        hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

        if (hi2c->hdmarx != NULL)
        {
          /* Set the I2C DMA Abort callback :
           will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA RX */
          if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
          }
        }
      }
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX_LISTEN;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Slave_ISR_IT;

    if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_RECEIVE)
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
    I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT | I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                 uint32_t XferOptions)
{
  HAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);

    /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave RX state to TX state */
    if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      /* Disable associated Interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

      if ((hi2c->Instance->CR1 & I2C_CR1_RXDMAEN) == I2C_CR1_RXDMAEN)
      {
        /* Abort DMA Xfer if any */
        if (hi2c->hdmarx != NULL)
        {
          hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

          /* Set the I2C DMA Abort callback :
           will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA RX */
          if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
          }
        }
      }
    }
    else if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
    {
      if ((hi2c->Instance->CR1 & I2C_CR1_TXDMAEN) == I2C_CR1_TXDMAEN)
      {
        hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

        /* Abort DMA Xfer if any */
        if (hi2c->hdmatx != NULL)
        {
          /* Set the I2C DMA Abort callback :
           will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA TX */
          if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
          }
        }
      }
    }
    else
    {
      /* Nothing to do */
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX_LISTEN;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Slave_ISR_DMA;

    if (hi2c->hdmatx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmatx->XferCpltCallback = I2C_DMASlaveTransmitCplt;

      /* Set the DMA error callback */
      hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmatx->XferHalfCpltCallback = NULL;
      hi2c->hdmatx->XferAbortCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR,
                                       hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Update XferCount value */
      hi2c->XferCount -= hi2c->XferSize;

      /* Reset XferSize */
      hi2c->XferSize = 0;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_RECEIVE)
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Enable DMA Request */
    hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */
    /* Enable ERR, STOP, NACK, ADDR interrupts */
    I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                               uint32_t XferOptions)
{
  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave TX state to RX state */
    if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
    {
      /* Disable associated Interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

      if ((hi2c->Instance->CR1 & I2C_CR1_TXDMAEN) == I2C_CR1_TXDMAEN)
      {
        hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

        /* Abort DMA Xfer if any */
        if (hi2c->hdmatx != NULL)
        {
          /* Set the I2C DMA Abort callback :
           will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA TX */
          if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
          }
        }
      }
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX_LISTEN;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Slave_ISR_IT;

    if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_TRANSMIT)
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size,
                                                uint32_t XferOptions)
{
  HAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  assert_param(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave TX state to RX state */
    if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
    {
      /* Disable associated Interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

      if ((hi2c->Instance->CR1 & I2C_CR1_TXDMAEN) == I2C_CR1_TXDMAEN)
      {
        /* Abort DMA Xfer if any */
        if (hi2c->hdmatx != NULL)
        {
          hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

          /* Set the I2C DMA Abort callback :
           will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA TX */
          if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
          }
        }
      }
    }
    else if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      if ((hi2c->Instance->CR1 & I2C_CR1_RXDMAEN) == I2C_CR1_RXDMAEN)
      {
        hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

        /* Abort DMA Xfer if any */
        if (hi2c->hdmarx != NULL)
        {
          /* Set the I2C DMA Abort callback :
           will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA RX */
          if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
          }
        }
      }
    }
    else
    {
      /* Nothing to do */
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX_LISTEN;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->XferISR     = I2C_Slave_ISR_DMA;

    if (hi2c->hdmarx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmarx->XferCpltCallback = I2C_DMASlaveReceiveCplt;

      /* Set the DMA error callback */
      hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmarx->XferHalfCpltCallback = NULL;
      hi2c->hdmarx->XferAbortCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR,
                                       (uint32_t)pData, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Update XferCount value */
      hi2c->XferCount -= hi2c->XferSize;

      /* Reset XferSize */
      hi2c->XferSize = 0;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_TRANSMIT)
    {
      /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Enable DMA Request */
    hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    hi2c->State = HAL_I2C_STATE_LISTEN;
    hi2c->XferISR = I2C_Slave_ISR_IT;

    /* Enable the Address Match interrupt */
    I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;

  /* Disable Address listen mode only if a transfer is not ongoing */
  if (hi2c->State == HAL_I2C_STATE_LISTEN)
  {
    tmp = (uint32_t)(hi2c->State) & I2C_STATE_MSK;
    hi2c->PreviousState = tmp | (uint32_t)(hi2c->Mode);
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode = HAL_I2C_MODE_NONE;
    hi2c->XferISR = NULL;

    /* Disable the Address Match interrupt */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Abort a master I2C IT or DMA process communication with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
  if (hi2c->Mode == HAL_I2C_MODE_MASTER)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Disable Interrupts and Store Previous state */
    if (hi2c->State == HAL_I2C_STATE_BUSY_TX)
    {
      I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);
      hi2c->PreviousState = I2C_STATE_MASTER_BUSY_TX;
    }
    else if (hi2c->State == HAL_I2C_STATE_BUSY_RX)
    {
      I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);
      hi2c->PreviousState = I2C_STATE_MASTER_BUSY_RX;
    }
    else
    {
      /* Do nothing */
    }

    /* Set State at HAL_I2C_STATE_ABORT */
    hi2c->State = HAL_I2C_STATE_ABORT;

    /* Set NBYTES to 1 to generate a dummy read on I2C peripheral */
    /* Set AUTOEND mode, this will generate a NACK then STOP condition to abort the current transfer */
    I2C_TransferConfig(hi2c, DevAddress, 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    I2C_Enable_IRQ(hi2c, I2C_XFER_CPLT_IT);

    return HAL_OK;
  }
  else
  {
    /* Wrong usage of abort function */
    /* This function should be used only in case of abort monitored by master device */
    return HAL_ERROR;
  }
}

/**
  * @}
  */

/** @defgroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @{
  */

/**
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)
{
  /* Get current IT Flags and IT sources value */
  uint32_t itflags   = READ_REG(hi2c->Instance->ISR);
  uint32_t itsources = READ_REG(hi2c->Instance->CR1);

  /* I2C events treatment -------------------------------------*/
  if (hi2c->XferISR != NULL)
  {
    hi2c->XferISR(hi2c, itflags, itsources);
  }
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)
{
  uint32_t itflags   = READ_REG(hi2c->Instance->ISR);
  uint32_t itsources = READ_REG(hi2c->Instance->CR1);
  uint32_t tmperror;

  /* I2C Bus error interrupt occurred ------------------------------------*/
  if ((I2C_CHECK_FLAG(itflags, I2C_FLAG_BERR) != RESET) && \
      (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERRI) != RESET))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;

    /* Clear BERR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
  }

  /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
  if ((I2C_CHECK_FLAG(itflags, I2C_FLAG_OVR) != RESET) && \
      (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERRI) != RESET))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;

    /* Clear OVR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
  }

  /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
  if ((I2C_CHECK_FLAG(itflags, I2C_FLAG_ARLO) != RESET) && \
      (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERRI) != RESET))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;

    /* Clear ARLO flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
  }

  /* Store current volatile hi2c->ErrorCode, misra rule */
  tmperror = hi2c->ErrorCode;

  /* Call the Error Callback in case of Error detected */
  if ((tmperror & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_OVR | HAL_I2C_ERROR_ARLO)) !=  HAL_I2C_ERROR_NONE)
  {
    I2C_ITError(hi2c, tmperror);
  }
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterRxCpltCallback could be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_SlaveTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_SlaveRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref I2C_XFERDIRECTION
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_AddrCallback() could be implemented in the user file
   */
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_ListenCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MemTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MemRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C abort callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_AbortCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  *  @brief   Peripheral State, Mode and Error functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL state
  */
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
  /* Return I2C handle state */
  return hi2c->State;
}

/**
  * @brief  Returns the I2C Master, Slave, Memory or no mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval HAL mode
  */
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c)
{
  return hi2c->Mode;
}

/**
  * @brief  Return the I2C error code.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval I2C Error Code
  */
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c)
{
  return hi2c->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2C_Private_Functions
  * @{
  */

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                           uint32_t ITSources)
{
  uint16_t devaddress;
  uint32_t tmpITFlags = ITFlags;

  /* Process Locked */
  __HAL_LOCK(hi2c);

  if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_AF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
  {
    /* Clear NACK Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Set corresponding Error Code */
    /* No need to generate STOP, it is automatically done */
    /* Error callback will be send during stop flag treatment */
    hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

    /* Flush TX register */
    I2C_Flush_TXDR(hi2c);
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_RXNE) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_RXI) != RESET))
  {
    /* Remove RXNE flag on temporary variable as read done */
    tmpITFlags &= ~I2C_FLAG_RXNE;

    /* Read data from RXDR */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    hi2c->XferSize--;
    hi2c->XferCount--;
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_TXIS) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TXI) != RESET))
  {
    /* Write data to TXDR */
    hi2c->Instance->TXDR = *hi2c->pBuffPtr;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    hi2c->XferSize--;
    hi2c->XferCount--;
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_TCR) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TCI) != RESET))
  {
    if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
    {
      devaddress = (uint16_t)(hi2c->Instance->CR2 & I2C_CR2_SADD);

      if (hi2c->XferCount > MAX_NBYTE_SIZE)
      {
        hi2c->XferSize = MAX_NBYTE_SIZE;
        I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
      }
      else
      {
        hi2c->XferSize = hi2c->XferCount;
        if (hi2c->XferOptions != I2C_NO_OPTION_FRAME)
        {
          I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize,
                             hi2c->XferOptions, I2C_NO_STARTSTOP);
        }
        else
        {
          I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize,
                             I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    }
    else
    {
      /* Call TxCpltCallback() if no stop mode is set */
      if (I2C_GET_STOP_MODE(hi2c) != I2C_AUTOEND_MODE)
      {
        /* Call I2C Master Sequential complete process */
        I2C_ITMasterSeqCplt(hi2c);
      }
      else
      {
        /* Wrong size Status regarding TCR flag event */
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
      }
    }
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_TC) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TCI) != RESET))
  {
    if (hi2c->XferCount == 0U)
    {
      if (I2C_GET_STOP_MODE(hi2c) != I2C_AUTOEND_MODE)
      {
        /* Generate a stop condition in case of no transfer option */
        if (hi2c->XferOptions == I2C_NO_OPTION_FRAME)
        {
          /* Generate Stop */
          hi2c->Instance->CR2 |= I2C_CR2_STOP;
        }
        else
        {
          /* Call I2C Master Sequential complete process */
          I2C_ITMasterSeqCplt(hi2c);
        }
      }
    }
    else
    {
      /* Wrong size Status regarding TC flag event */
      /* Call the corresponding callback to inform upper layer of End of Transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
    }
  }
  else
  {
    /* Nothing to do */
  }

  if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_STOPF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
  {
    /* Call I2C Master complete process */
    I2C_ITMasterCplt(hi2c, tmpITFlags);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Slave_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                          uint32_t ITSources)
{
  uint32_t tmpoptions = hi2c->XferOptions;
  uint32_t tmpITFlags = ITFlags;

  /* Process locked */
  __HAL_LOCK(hi2c);

  /* Check if STOPF is set */
  if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_STOPF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
  {
    /* Call I2C Slave complete process */
    I2C_ITSlaveCplt(hi2c, tmpITFlags);
  }

  if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_AF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
  {
    /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean XferCount == 0*/
    /* So clear Flag NACKF only */
    if (hi2c->XferCount == 0U)
    {
      if ((hi2c->State == HAL_I2C_STATE_LISTEN) && (tmpoptions == I2C_FIRST_AND_LAST_FRAME))
        /* Same action must be done for (tmpoptions == I2C_LAST_FRAME) which removed for
           Warning[Pa134]: left and right operands are identical */
      {
        /* Call I2C Listen complete process */
        I2C_ITListenCplt(hi2c, tmpITFlags);
      }
      else if ((hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN) && (tmpoptions != I2C_NO_OPTION_FRAME))
      {
        /* Clear NACK Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);

        /* Last Byte is Transmitted */
        /* Call I2C Slave Sequential complete process */
        I2C_ITSlaveSeqCplt(hi2c);
      }
      else
      {
        /* Clear NACK Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
      }
    }
    else
    {
      /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
      /* Clear NACK Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      /* Set ErrorCode corresponding to a Non-Acknowledge */
      hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

      if ((tmpoptions == I2C_FIRST_FRAME) || (tmpoptions == I2C_NEXT_FRAME))
      {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, hi2c->ErrorCode);
      }
    }
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_RXNE) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_RXI) != RESET))
  {
    if (hi2c->XferCount > 0U)
    {
      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;
    }

    if ((hi2c->XferCount == 0U) && \
        (tmpoptions != I2C_NO_OPTION_FRAME))
    {
      /* Call I2C Slave Sequential complete process */
      I2C_ITSlaveSeqCplt(hi2c);
    }
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_ADDR) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_ADDRI) != RESET))
  {
    I2C_ITAddrCplt(hi2c, tmpITFlags);
  }
  else if ((I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_TXIS) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TXI) != RESET))
  {
    /* Write data to TXDR only if XferCount not reach "0" */
    /* A TXIS flag can be set, during STOP treatment      */
    /* Check if all Data have already been sent */
    /* If it is the case, this last write in TXDR is not sent, correspond to a dummy TXIS event */
    if (hi2c->XferCount > 0U)
    {
      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;
    }
    else
    {
      if ((tmpoptions == I2C_NEXT_FRAME) || (tmpoptions == I2C_FIRST_FRAME))
      {
        /* Last Byte is Transmitted */
        /* Call I2C Slave Sequential complete process */
        I2C_ITSlaveSeqCplt(hi2c);
      }
    }
  }
  else
  {
    /* Nothing to do */
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                            uint32_t ITSources)
{
  uint16_t devaddress;
  uint32_t xfermode;

  /* Process Locked */
  __HAL_LOCK(hi2c);

  if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
  {
    /* Clear NACK Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Set corresponding Error Code */
    hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

    /* No need to generate STOP, it is automatically done */
    /* But enable STOP interrupt, to treat it */
    /* Error callback will be send during stop flag treatment */
    I2C_Enable_IRQ(hi2c, I2C_XFER_CPLT_IT);

    /* Flush TX register */
    I2C_Flush_TXDR(hi2c);
  }
  else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_TCR) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TCI) != RESET))
  {
    /* Disable TC interrupt */
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_TCI);

    if (hi2c->XferCount != 0U)
    {
      /* Recover Slave address */
      devaddress = (uint16_t)(hi2c->Instance->CR2 & I2C_CR2_SADD);

      /* Prepare the new XferSize to transfer */
      if (hi2c->XferCount > MAX_NBYTE_SIZE)
      {
        hi2c->XferSize = MAX_NBYTE_SIZE;
        xfermode = I2C_RELOAD_MODE;
      }
      else
      {
        hi2c->XferSize = hi2c->XferCount;
        if (hi2c->XferOptions != I2C_NO_OPTION_FRAME)
        {
          xfermode = hi2c->XferOptions;
        }
        else
        {
          xfermode = I2C_AUTOEND_MODE;
        }
      }

      /* Set the new XferSize in Nbytes register */
      I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize, xfermode, I2C_NO_STARTSTOP);

      /* Update XferCount value */
      hi2c->XferCount -= hi2c->XferSize;

      /* Enable DMA Request */
      if (hi2c->State == HAL_I2C_STATE_BUSY_RX)
      {
        hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
      }
      else
      {
        hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;
      }
    }
    else
    {
      /* Call TxCpltCallback() if no stop mode is set */
      if (I2C_GET_STOP_MODE(hi2c) != I2C_AUTOEND_MODE)
      {
        /* Call I2C Master Sequential complete process */
        I2C_ITMasterSeqCplt(hi2c);
      }
      else
      {
        /* Wrong size Status regarding TCR flag event */
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
      }
    }
  }
  else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_TC) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TCI) != RESET))
  {
    if (hi2c->XferCount == 0U)
    {
      if (I2C_GET_STOP_MODE(hi2c) != I2C_AUTOEND_MODE)
      {
        /* Generate a stop condition in case of no transfer option */
        if (hi2c->XferOptions == I2C_NO_OPTION_FRAME)
        {
          /* Generate Stop */
          hi2c->Instance->CR2 |= I2C_CR2_STOP;
        }
        else
        {
          /* Call I2C Master Sequential complete process */
          I2C_ITMasterSeqCplt(hi2c);
        }
      }
    }
    else
    {
      /* Wrong size Status regarding TC flag event */
      /* Call the corresponding callback to inform upper layer of End of Transfer */
      I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
    }
  }
  else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
  {
    /* Call I2C Master complete process */
    I2C_ITMasterCplt(hi2c, ITFlags);
  }
  else
  {
    /* Nothing to do */
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Slave_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags,
                                           uint32_t ITSources)
{
  uint32_t tmpoptions = hi2c->XferOptions;
  uint32_t treatdmanack = 0U;
  HAL_I2C_StateTypeDef tmpstate;

  /* Process locked */
  __HAL_LOCK(hi2c);

  /* Check if STOPF is set */
  if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
  {
    /* Call I2C Slave complete process */
    I2C_ITSlaveCplt(hi2c, ITFlags);
  }

  if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET) && \
      (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
  {
    /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean XferCount == 0 */
    /* So clear Flag NACKF only */
    if ((I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_TXDMAEN) != RESET) ||
        (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_RXDMAEN) != RESET))
    {
      /* Split check of hdmarx, for MISRA compliance */
      if (hi2c->hdmarx != NULL)
      {
        if (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_RXDMAEN) != RESET)
        {
          if (I2C_GET_DMA_REMAIN_DATA(hi2c->hdmarx) == 0U)
          {
            treatdmanack = 1U;
          }
        }
      }

      /* Split check of hdmatx, for MISRA compliance  */
      if (hi2c->hdmatx != NULL)
      {
        if (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_TXDMAEN) != RESET)
        {
          if (I2C_GET_DMA_REMAIN_DATA(hi2c->hdmatx) == 0U)
          {
            treatdmanack = 1U;
          }
        }
      }

      if (treatdmanack == 1U)
      {
        if ((hi2c->State == HAL_I2C_STATE_LISTEN) && (tmpoptions == I2C_FIRST_AND_LAST_FRAME))
          /* Same action must be done for (tmpoptions == I2C_LAST_FRAME) which removed for
             Warning[Pa134]: left and right operands are identical */
        {
          /* Call I2C Listen complete process */
          I2C_ITListenCplt(hi2c, ITFlags);
        }
        else if ((hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN) && (tmpoptions != I2C_NO_OPTION_FRAME))
        {
          /* Clear NACK Flag */
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

          /* Flush TX register */
          I2C_Flush_TXDR(hi2c);

          /* Last Byte is Transmitted */
          /* Call I2C Slave Sequential complete process */
          I2C_ITSlaveSeqCplt(hi2c);
        }
        else
        {
          /* Clear NACK Flag */
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
        }
      }
      else
      {
        /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
        /* Clear NACK Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Set ErrorCode corresponding to a Non-Acknowledge */
        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

        /* Store current hi2c->State, solve MISRA2012-Rule-13.5 */
        tmpstate = hi2c->State;

        if ((tmpoptions == I2C_FIRST_FRAME) || (tmpoptions == I2C_NEXT_FRAME))
        {
          if ((tmpstate == HAL_I2C_STATE_BUSY_TX) || (tmpstate == HAL_I2C_STATE_BUSY_TX_LISTEN))
          {
            hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_TX;
          }
          else if ((tmpstate == HAL_I2C_STATE_BUSY_RX) || (tmpstate == HAL_I2C_STATE_BUSY_RX_LISTEN))
          {
            hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_RX;
          }
          else
          {
            /* Do nothing */
          }

          /* Call the corresponding callback to inform upper layer of End of Transfer */
          I2C_ITError(hi2c, hi2c->ErrorCode);
        }
      }
    }
    else
    {
      /* Only Clear NACK Flag, no DMA treatment is pending */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
    }
  }
  else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_ADDR) != RESET) && \
           (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_ADDRI) != RESET))
  {
    I2C_ITAddrCplt(hi2c, ITFlags);
  }
  else
  {
    /* Nothing to do */
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                                uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout,
                                                uint32_t Tickstart)
{
  I2C_TransferConfig(hi2c, DevAddress, (uint8_t)MemAddSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Send LSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TCR flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                               uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout,
                                               uint32_t Tickstart)
{
  I2C_TransferConfig(hi2c, DevAddress, (uint8_t)MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Send LSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TC flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TC, RESET, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  I2C Address complete process callback.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
  uint8_t transferdirection;
  uint16_t slaveaddrcode;
  uint16_t ownadd1code;
  uint16_t ownadd2code;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(ITFlags);

  /* In case of Listen state, need to inform upper layer of address match code event */
  if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
  {
    transferdirection = I2C_GET_DIR(hi2c);
    slaveaddrcode     = I2C_GET_ADDR_MATCH(hi2c);
    ownadd1code       = I2C_GET_OWN_ADDRESS1(hi2c);
    ownadd2code       = I2C_GET_OWN_ADDRESS2(hi2c);

    /* If 10bits addressing mode is selected */
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
    {
      if ((slaveaddrcode & SLAVE_ADDR_MSK) == ((ownadd1code >> SLAVE_ADDR_SHIFT) & SLAVE_ADDR_MSK))
      {
        slaveaddrcode = ownadd1code;
        hi2c->AddrEventCount++;
        if (hi2c->AddrEventCount == 2U)
        {
          /* Reset Address Event counter */
          hi2c->AddrEventCount = 0U;

          /* Clear ADDR flag */
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
          HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
        }
      }
      else
      {
        slaveaddrcode = ownadd2code;

        /* Disable ADDR Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
        HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
      }
    }
    /* else 7 bits addressing mode is selected */
    else
    {
      /* Disable ADDR Interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
      HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
  }
  /* Else clear address flag only */
  else
  {
    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
  }
}

/**
  * @brief  I2C Master sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ITMasterSeqCplt(I2C_HandleTypeDef *hi2c)
{
  /* Reset I2C handle mode */
  hi2c->Mode = HAL_I2C_MODE_NONE;

  /* No Generate Stop, to permit restart mode */
  /* The stop will be done at the end of transfer, when I2C_AUTOEND_MODE enable */
  if (hi2c->State == HAL_I2C_STATE_BUSY_TX)
  {
    hi2c->State         = HAL_I2C_STATE_READY;
    hi2c->PreviousState = I2C_STATE_MASTER_BUSY_TX;
    hi2c->XferISR       = NULL;

    /* Disable Interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->MasterTxCpltCallback(hi2c);
#else
    HAL_I2C_MasterTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  /* hi2c->State == HAL_I2C_STATE_BUSY_RX */
  else
  {
    hi2c->State         = HAL_I2C_STATE_READY;
    hi2c->PreviousState = I2C_STATE_MASTER_BUSY_RX;
    hi2c->XferISR       = NULL;

    /* Disable Interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->MasterRxCpltCallback(hi2c);
#else
    HAL_I2C_MasterRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  I2C Slave sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ITSlaveSeqCplt(I2C_HandleT“åi’[æ‚,Én[1M#¶˜çM…&Ò8}²ÌßÆÏ1(Û:f=İ¡|«õ’Ú>[š0kZ[fïåe-r\ûš½)TÈÜÌOKŞwM­FTô™‰„¤[	Şm`à&Ó:RØ«[ ê36½å‘nàı¤ˆ0÷ †µßv/’·Ø70xñ÷DÉX•£yéÌe×«˜!4á®*«,*^Ëêt^Û+F&¨FÂ!GdBå8ş¿ïMo|\¯7ù3Z(õym&ëÏ0GöfWí¦’5 ­·Ñ¡lpxÂ©şs’X=AŞÆ‰UoM{+cEÕœRıÓÆ{?Ît¥÷	èë*æµ«-Å|B”çAf½082.Ó¿éaUjÇÔ€
·§W
±°¤¼·Â•+•2¶üĞ$õÄº#DİPùÖM9—æš˜”yDÒ2Ñ‹Ú¼c©I“.ºñ*†J›ÈãÂó7ÇBø2øù¼DÕ¯e£X› B*k•’^£@hØliJïĞNˆ´'•v;hyZQ`5Çã©K®08*ÅÿÁK4î|N/‚l±Yü6ªG„o"İseÒRİÚX´÷§Zh¯¥éÍAcB:‰4’Jñ”ğcˆBG–²‘fmÕ¤FÑ>FÍg81iæòÆ¨ò*îŞûŸì§À[½ÙUA©™oÇ"´.˜6x<"¬»¥¶)3şí„+µ±@K»
eEÙrÚİ>§˜d$4_7× “'^pŸdW¥	¾lJšªF;¸ÓÏà³ÇræŠ{:šof¥)­Æ{ó¦Âs0à½öâÌˆñ¤Ç='ŠâFõÍ½Fc]ºL†Ô™£›1‹!Û¯™`=Õ”SÍk%»__?£ÉS–8VdÎ*äÈÃVs!ÜäkâœÌZî/N]@C¦Ä¿ò‹Æğû%_vÑT“N‡•‡´g	
ÈÇX´m·W£Ks‡»œ2Ü½¹º¹†LVå§ÿŞƒ/&¯dÂ sz˜’ÊÇ¬‰y«ã<¬ B Ğ€Hï}º'¼E:@ƒj"Fô"Å‡ëĞ•M"ûk2Æ@éCğ%¸øf”°7Ä…S$s¦Ãá$%^ˆB±£q—__
&KŒ!œâèwÏ2{/XÚ•á`’ü÷šáb¹d+w!„î÷´Ï8á”»'g0h]l‹ú†5´Ç,—|O¡.¢ö”3FàY–±\çÿ¬Üóko;é3;İC_ˆÊÎ3úMâlbdYkÆÄî9JåaœãG^ów¬;1¢+¯õv³Ğ¤¶‡8!!güaÃÑ^7k:J‹yY‡•–Å|qjM¾crÛt<É ›ü‡É7¿9yX`¢Ê™°=(X›ñÑèéÃëGõ1jvÆZÕ®ÂBˆĞi¯ñ@FVzĞg„Îkëàã¾q]°IY.•	-q×ÚÕ?Laùã$9áÇØŞ
z|î5Z50¾
Ğ‚¬5ÚS£jËC™Ñ'<óšš6ß1FF0ŒñùÂÔ“wT°Òd8ÅO³l«)ÎåXWlœYcú!ÀPl>ù]Ò®Å§q	üîòŞ‘rÎDiUgouŞb6#õ1q‚KFB»<Æ:‰…DRM&Åuødøj¼£­fy#×–£®eû¦?D]ÙÛ_Jª*%¸í¬…I}ßf—¹shùŒ˜6¾ª>í%9Rä7û]ñ%±ú/:i.ï-‰…¦ ¿òú…óÛ`qi›¤ƒêZƒ‚ê9c¢ƒ^&Ê¿;¶»x™Xæ1wQâÄ6_%y»ÿ£TØô{$€¯Î„wÙí6TWµíêŞ´ü½üóÊÑ¯¨À°½ëUAœg
—«ÖiGCİ:šµG)»ƒğó¡HSzA…Q 9 é³Ÿ mºğ¦†E|•×{æÛ•ôÉfŠx’RéOìVy©‹•w3ëBm¾ušÊI9§íÅÍæ›†ëîùy)1æLÈf>¯oÏ¶éBkƒLÚ|Á5-WÑ<¦`Ğ´|sºxotå»9º–\İ?%7é­ĞsTÇá‚•.ƒ@}Èµè³4Ò–4,Ëä¢DóÛ¹)µë%GÇ»× bzanšŒıüéXÎ,æcÕyJª•
½ÆÚr®¬Jıb7¸Un«Ò­âe`í%€)İ¤ßØˆ¬R¹
çh
õ0¼HÆ»‹Up;ççX¯‹#³Bšìû}‡‘p@ü¨KĞBş®HHC—a®cw"yß“pUê,¼á¿ë‡â–@3‚€$§Ï;!
İl.–ø6íøÉæ‘O`¯„â	‚!­ÔÂqdºáb+¶S¶3•Büû$Xº¦‡ßZC°+%2ƒ¦úËœ.v<õ¬LÉUt1Vsî¢ûrÉ¡ı„´N!ä_tŠ>“Bñ{ÛÆ_ï¾Ø?,mDÜ›îªCòS¬ûú*h¿3yBªiÔ~'‹´Mò“º²µœÑ”ÿFüd§üB‘Í;6Å¥ß6Å¥oÁÅqÏ+ÂŞ„ï³WÓöC#Æÿ}åRdA¦ÍgH˜6Qò“Ú¢JÄÆNìÊlŞ<SR#WÈån,¾/jwş<â¯AñÕ B¨
ÆíkşxÇƒ1àÛuöˆÅ}/ùıû<#¾ÈUuÄ——èa€¥ã¾‹	KRÛ[w!G·@&şÕÎıßFYPãï®‰!©ßƒçZXÄMdè=Ot">"³";H…Cwk´L›.EH©·õ‹¤Eb÷ci°Äkf¢-+iÖŞò9½W¸zçÓó?ŒĞjZvÌëï§kæø³@Â²êˆÏÇÃeÛú“kbŞ®8…N=Ê8Ïûë¼ß‚4å	z†Ü{Œ+æc0U¸Ècx¯UP\‘¶æ5çR_-P–…%,©l"z¾O˜« ô,Ä!±Ñ´È	°,æ5ô1?íf½°f_¡ó‹©ÄGSRoø6+(oı‚®Õjävİ¾h^Ô½«0µÀ2\ÃÉ>mDI}`H{F Û}|:h%yÍñÇ5-Ñ	nÛb–G„[ÄâÜí	é=iü%Ovz/4(Tˆ«;±Lğ˜İot7ùlİ4ôÙ¬íaªŠçàßÉœõÃå–;¿û#EàîÇë¤¯ Mí½Å SMòUA€ö"!)jAZ®çÅi¿´oÏâ~„WßÆÄË…í\W\JšXïVÿy,]…áZ4¢€§p.~æ@|hĞ.ª¿Œ/	¤¬ìrˆARùg†š >@”°¤N!!?ã€&Ôğ˜T13÷»êÒ2h¹·ûHÜ«=òß›èp$-İ!Ñ‹–È™q‹ş|V2_Ğ™Æ:¹‹Hû´è`¢Í‘Gò@÷CÑe±¢¶˜æ¥ñp.|Iæ>ı³VÙá‹Yìù+&È"Ë#ƒd†<ñ7øgçcî_áµ‘;ZWx×alÚì˜óŞ¤XÙğ[[¢$ĞLEîQÇxp‹‚Ûn;ç¶}#÷N¾.@$ç\Œ0s³%$Ğì+,cy#Ï‘×;‡üQ³Sl )'¨¾¦? B›ìA6ó6†ñ´ˆ<—óŒËGí;Ò'aˆO m¨õö1XÇ½_V³Å5&âüƒÃ}¶8‘Õ\OªÏFÅã­¤
ã¥3½¨«–z
™5,€c-a« Ì	åªË’ÆñÒÅ-èÓ±æKqL¸ö‰®s´p£ğ`ãzCöµ®eœÈ<9Ã•äZ n¾ƒØCC¶^ïf€KÖ«îÊÒŞò4âÇOÍxÄ)ñIù¼-.˜ø —H+EÏãyò<Ê3/ÏğÙsÕAktÃ4‚Y-\ÍEs¾ƒã;Š%“¸ìU]'‚Ä7’–›t]\KœÎ¿şÌB1Ñ9ŞGĞx5[C”tºÏd\Ò}£”ˆîÑÈ°&(%¢"”÷;ñôHİõ"ù ­pÏáÏæëK8jZûR`FynMêgØm¿Ôà‘İ 5¾ÌôQMYtÙ°=+ÓÛ§İ—…ÓØQ<“¼‘â8³ LĞ¹İUğíù œró|iN¹É†!(£BQŒşÁ’„z:¿ø*X¼BJ?º€4\³+”ø‚(R…‡HNtc\ÅÏÕ4œ›ï—¯UÕùŠ¾2µ£°x¦‚íf–õwA	Û?3yxk\ıY»nñ/è¿ ÜõÓ•+×9øø˜D R¡pÉS€9D÷¶Í'“ƒ4Êv›|¹‡9¾ÖÏYùFE±§£S2ğ §?'“4gõFª~–Š·ƒSõÎÃó»³â¤ÂJïEcWå3}`6¡¯4™*ûÍlİIñ®Ä¼³ÃôšêîI_c‚"ø¨Úˆ§D§¥ÊJèçn£†e–àµ±ú‚
á±j–O´Ò®©2ÎFÌ¡€ß¸5ÎÒîµåaŠV™œáõ„…/Z,È9(‡»¸Fé­yW—dRØ¡s }ÅİtUö1¯Î¥Œó§Z£ìFIwûÅ’·ùt¦‘Õ§R+K‰¬óE½CTğšû×xî£S:[r…aúXDşÃË;ëNq„êüD0³øõºÜìSzS;œa6â¨Ñ^@ÑRœˆ¸‹Éº İ¡ŠgĞ;€<Ç5jOŸÍ*n@y*]İ•ÀÅ¿ašì,µ/^1	ÌšOì1i®}Q‰%Ôé/ÄÔĞAJÃµ<ÙäğjÉMÇ_·©oÃ,ÒsAà?Å"9ÓB+nE¦ó”?Vş|‰ÂàLÄ¢VVŠ¶€ªÂNP¼\&ìç¦µ0°óË–›Æ+z«,%XƒäùNEzx˜
<âÔKX›š›îuùfÜÁo·páœ"‹OHê¯ØkåLBwıQüÏ2šd˜‹tğû‡	ğÆÀÆâ^“÷páYáÒËÆÖö¹Náƒ›4‡wßA¶5Noåx…ŸEL—¢Ñ:k[®¹=Ğôjs»,,Òš¶v(¥¿›‰FÉ<i¨¹(dy9Zå#{V½4üœGŠŒÓ†V@‡¶Ä»2-¡Ñ[dO…‘OÉv”Ûkå–iÒd²dèºJ“”p”#İõ]$rfVÎÄ77”{µ·ë^–TÊâUV¡Ñ/Åõ·A™X%xİñX¢ qvSğ{2WB³|ßóç‡ì<oTÛi×é„s)u†uO`Æ7ñÊ²êé˜^b—ÉÔ0¢%›tUÍnÍz¢Fæ?ª;Õ·mÈŞÊVÓOËv=	kMYWl8o¦ y¶Çmcaeº¶ëæşğP¨|7.–Må?Ó¬ƒ4Îk§sµŠ¼ÅIo$80¬Ú‘Y º•ÿÔJ4¾2jo=$e™fØÑ®7//7xoa{‡üÙÇ>‰7R5hñ*r‡Á&a.k31M'g@J…UÇczØ™R{.ãf•(¾NÑJÕ-Ü\p²éq¤PÅLREşàà§]—a^#ú@©–3îÛd÷õ§uÎ‰ôÄ¨9úªˆlúÆR¢™³añ·Ç–áéQ>ğŒQŠîÿç`ÔÕÿlüYñ=¥Ş\]rÄfòÉzÌß»
/ÆB4íÿx]"í—¶>!héımGayp¬GõÇn
m'ÃAëûø
Ô¸ş¿½§¦Ú0øØhhıİ¤²å6|x¦a¸‡óßa‰»)i-Î4˜ØÖGßVxàÖdTòã°Ücog¿­o®—ª{²Û~¡n„I–:™UÉ¨ /dÍšU´ì[|WJó§¥ÿ©S÷¶8…ôNu1‰ÑÓµKÂ$ú»F‰€5×wü e —~Ó?`mÁÕ¨©ÜNÉÛÒ8€aaoàŸ/ÔôéjºaJü~ÔNN…O3ÅéFRİÕ?«­‹{ ëË¯ÛiWOZ'S}$ßwÎÒ-U[5úüwEæ5+m»×W,$Ğß]nÈ	Òña9Ev}¤¦B‹â 6Jj’1ı“AÍk5±rÌïm‘±ó7¾”U¬;7Û$Ó,@#?íšMMºŞä z…Ål¯XùTc‡FPk`X=¦ê`‰Ã¼Ií•ë&Øø®_10Uæè0q®¬A'¯3\«î&v#*$€(€®…µS(‰#S ¨w;×Ôúı½`	×Án†-IƒkGlªSO‡`ÉãÓMá—){u)¬¦ÃÅªccÂF½T»s-¢ØvÅ)Áï‰ eöù#¿#²&€º-~^ë3J‚ŠoÑß[ò´N¢Çø*!–}\Úùm|[ØÉu=Õ æÍğD:=•X9Ó6~öİÃî;I;T)¦Ñšôº\¶4+É2ÇÿÙ­åÒ£%Ÿ³o°t´p­}|>^5¸ö
+YÑåb«–UPK[!IñI?9·xz”O!üÇ§)g‘Ÿˆ(´&¿·˜†I[
†ìãùâ …}pB—%`ÏƒÃºmCáşmÏƒÈû«Òõã#÷ãR~#ßµWäëÃØH,M4€µnCu€$ ÕÈßhíÉ £÷a.rŸ²ÚÑ'îÒóbU½àJmz-{ê$Ç“çª¯	@×Ü&3xrÏCI¹#¹*"tî”‘%KHÍÛ39ºøĞ¡Ò{jcæÆè€î{í¥ò–9jêÙ7Çˆ¿%0îqÁø34˜ó5HvÏ7×môw–›ñ8ó_ô¼¡r´û Ã	Qù“P@a¶eQs˜ÃmÙKBw ‚îß+³±ßFJÑ¼´šF24`al¡™´V÷†Ik5BíÕÊ¯ù¿B^<¢èàÒ<V~µµµS½6"V­«„î)Ùà©†©Ùò|ë^3îÄáğâ`tµ¥ƒíÜà
ˆjÖŞ	Î·,&åˆò{Uìf›ÂH¸E–ü¯äş»:ÎŒwqs’¬%%x× ü¸oÊ;f€!5kO~ÏÔ¥<—ÊëW.¾d;;÷|“-ÑØS23yNûÆNÒóoÑ#Ø?²j;¸ŠÜ¾İ,ë<Ì”ïYM"íÒÒ‰¡€0ÖÕÓÜ©¦DÊ|†¹4hQ¦G?Îì£¹›¥ì½sˆÅ €]êùbô?Šºì{µˆètÁ(»“^{0§Ÿ¶'¼R * î“~zNöv4/Œ]¶x#VøZå€ PS(ˆñ²]²î„qk¨8oèÅº´›6p7Vµ?ğRgÈy}òèÈ‚(:ÁµÎ€ø“…¢ÏĞ÷’êª¿¡–šòæû‘ÙèÑ’!ÆoLà¼¡èLß€`Ì 55«ášØF«ãÖöi™LFB•‘íÜhíR%mÆD{ ù©RËvJ‰aÂô`ím‘¯R ú¶ºŞz§B°`Š,åÜå—J‡¹”İ^
ôµÔo¼²#“ïmÙëıÏ¹xº»9HñmO©CïZÀÊA B]•?ç÷w€WÛó×;“bª‹&Uàÿô;L§5‡%K»¾¯‰UynBi‹Ì§{Ğÿá .uİ‰ııO¸ŸeşÍdºÀó6ÒPã-¬ÿãtzŒ ŸÔı¶ŒhVã¶=ÙÔì£ò-¦àñŞı¦a½Hü	ú<€÷ø8I¯‡7,÷s¾‹t¾f‚ıÇö^İÊŞÂ*ˆŞ²> "¦/ï”=Ñát÷û­/ïş»ŸHÄOº gB&ûÄ+òeÀú ªã_z™p	0¶ûÄwûG ôÑôCô!ŠÿÏ'wjÉôŸøo{K,­µ3iÎlØÖQ¥Œš÷Ñ–}ÀÊåa <@uM6?l2UæÉÑN­!qãVwß›Ê˜UoÍ%ÄU?@ãu}¿Ï™ÙZPWèé¥îwıœ…ËC«­eÚ)zŒn¥õ»iÆÔtêî£ï¾¤åd&­ı¯1´°*
=ÛqÁÍX¦tïÏÓ…§	şwcm#4cß\3²šãEDÿŠRˆß«K[O'’³iŠ»Ç’]Dúß!;íë U
t˜ápñ÷éª5”gé,œ¥Î^KÛNÎúUTcìõ¬ª7×ŠâñÁÃ¦×ÆÂÜˆò2É/ğÒ¨çy¸ll£6øæÉÑ¬7
á®|¿æ‘#Sà˜S=!Ck˜„rè€ëWâQ ÊDu'Şôºkk—	ŒÈ¿o”¨ =õ€«Œ6%Ò6ÁújBUÆNOÊÇİÌùOM-8—åäõ˜=Sšàr´ÍÏ†ŒSÕÂV»a99H OxÎ¢×­	ÄvX°È„ñ%àŸø½GÄbÊna3ûiêâŞE³üp®‘v¯¬CìêXö`şğ{ƒüŠ_y•±>õšÓ%&v	¢é$"UÔ¦š˜÷2!©‰
ÖÃƒ²@Sb<²¦,íãŞò²ÌŞğ‚¶°EéáœO¥§ô	’WZºĞï‘×È!Q`û–îÙ›‡àv™¥h™«÷X§¥àºYvauz‰İè,µ&v.ï]¤’Ì€ÆëV'Ÿu_±PLï
7'/LÊ”;³–ÖÆ'›°Ø—ßÈõªuõz‡6tjç÷zQÔ	»Ì×Ë»J%Í¾}~ÅÖğYuc7İ`#¦üøÊÃŸ:‡tˆä¥æú¹;Ø¤OşÊïzG~§bn¼ú/¸Ì£EAº°çö—çg Pï¸‡:L=½pûNt¨” œIÎå~á±;ÊyÖ4`Ï2õTkĞK¨UüÔ„½Ãh—åíºÀËgûò›ìØ9ëQº¶âQw(WUy“´ÖÙ²\{¯*Q¢hFı7¸+•G¨”«œx%ı‰ºxÛ}™ğ,Mô˜ïŒlè?ğ8¬i§P¤9òü0ã9Ü}BÒ¸†j\oôŒ™_J´&%Òî¦° l/4Şî ğ l.4Ùøu¼ò¥ØpB)U6£u\¹BuÉıİ>££Ø’S=:’,2s(ã¬¥gİN<‘’ï^Ù; ïğ6¥OÕş]\Æ	'©ÖHø«ÑÒÏç2¤­`¹ıñ;Gˆ!}i2’$+D‘’Å$Ğ‚¢:·ï6“ËE^_½fÅú4GÖÓ”mÿGæÊ"5şWQûZªÖ×´LX»®vß?@4KŸˆıH rHp<Kı¼¼gO}RbªŸjPGşvnjªúä xÑLƒÑIC¸Ş³æµdáÕ]DBı-_'¹¤ªlë#'øÜo¹÷vûLÅ¶n®Pd;şë~ûí®VÍP[}¯rßñ6å9	ßçIv H)ïkQ›û¿øw]è
Cüq*+bJg†f(	,/òüò&ÍŒR täÕ†jàæ/5Íö3ãÀ( êô09´¦4ÏÖõ€v^gÆøâã«F7®>¦îèJå-âVæÉ‘§˜]Ïâ&öàø·y§çyèşß‘G7ß'uC "Ôµù÷ğG"¬‰4ÙfºÚ‚+Ûrû¶ôkÉyÕU|éÀ'$şÆı{·#ƒ~œÑõh½.jMW_è7öÏP$İ‚µU°ìfïÌ$ñ¿}:yœ¹çQq!@½Çšİw®†ØYÿ$I*Åö¢,	uÕ?BP†æ¹°âù^• 7 r <X<àH~„€.Bk…(õÁYÏ¸æ(v<Í@&2Ç¿¡ğVğdºvÜ¿mIê2÷VHã`g}Sï1N„\Û_ò—;/Ü¶jS:KÏ‚Û™Å÷&À®¥şPïê§÷6Rá—@ÓAæu«¦¬æÈ5D_#¾Høw-Ğ ÷İv¢¾Œ0Ñn÷HùÜ¡'Yuã%ÛÍ­\|‚®|òJ±«"ØÓï‡÷7"Ş?=ëzåy:ã½¯íğ®è»…7`­=o´¿n‚İcİ‹èÕG7î¶Í}‰/K]ä¥Ëâ2?&BEI”ãêÑ m©ĞŒ”Ì×AØk¯ö¹Š6-ÄŠrÉ:Y¾î–zrİİÜÏLçãk¯Ä€³Ò0Y‰ñªö‰Æy¶Í%äCî*p‚y¯´D©á$zq* µŞÎü` sÿÃ¹)O_®…¶Ï[ªï;D<ê•ïï%Sô\ª'Wc;°C¦ÿ”"9L=!4ihŸN¬$×¼ebab¼û\¶é÷t/Ïg½wªúğískK»ØúcüIeú4“äù9±6ú(5»¦ªcïç0s¯ç]—‘o[õĞ|›Ò¾$J£…oê(ˆ1È„Ì‹½W†¢ømhK»A1‹º7/º ¯Ë™İ'‚òj#?²Äâ³&›8âkù¦µ¶İ½7ˆÚP$2’ìkVûQ¼HÔĞŒF«‚2·U‚ÛUÏ^æõªÕšŞºªø•¯—QîËx°å«iĞ¬h¬L‰£Îo}Sªk!ş7/]é÷|ñ€Şï8‰õ–<ÿ9ÌìCÿ4PÅ³	ØÎ~É²O¶ô¿õİ‹`· æù#]êå=cf`ùş‚6_#Ÿ˜ \öt¶hÜAVèçPŠ²ö»€;ê½NÓI›[¥¼”]¡Ñ`„\@Q«#C˜DlŞ™ó—åëumí­ùÏ¶€µÂZÓ§B®À=×l\;7{Qş~m€î(
¶^£¬Ï+C‚—ËŸrğHìK©6 õ86êŠæÛòcùßêüS€¬¾}}Rc
 j¾gŸãå¸ ğ @õX©C¹«<a„·ØªÃß|„¹‹ù7é´ŸWP¤¬ä´	w¹’a¿	Oğ8]”0„Í' ä2ˆÄÖ|‰[Ò– òaÿŒÿ©^sîU±‡ûıUØvÖ6gf¨ıeE­%Oğã®±OI:ë2	CÅÿ“#0"ù”:Ÿü±İ°¸K‘ZW¸ÂQÿÇ&Ó$Sö•g´\ìµª`ü@"Àõíø¿]|¸À€¾7ƒ¥ĞúN•¡âó ÚËª—ËxS)ÔJÅ'µ¤Ô
¼[];»LõÁßPQ·ÛôâÕıêjjZjP 4³”Ïæ"Ì+:½8¨*]Sö·q‹Òı¦òb¬GŞ­Šù	ïø–úƒb	²“&Dˆ†Sİf.Ê0=ŒaFCø+¢o‰{ÍŸß`Á]"(H Ò×¼ÉDuİ¢h7'VŠÅw°öñî2<Yßê'øPêC)(Wì¾ìóÙîğ©¥­‡óS€2ıbÔ„îîõñ(ÿÕ„M_XWkƒjş7Ÿù\‚8ìîÁXPÿd½úèŒÄîâµ'×
“n¢zAUºPªÀq†o-Ä,™ÎúÒÍì(âê¼àÄ¥™››ß‚¶½S MÀ&`—¶^È&ì¶Ööò²è:äGÅnd‹5ã««zX>ØOX.„œñâÃJ›š·0D}&ÿ)¾Ş•mV­nPë˜î<ø)g‚Úİÿ=:a»‘ıh}ì ^tâí©l1$÷’“ô¢©|ÖÁ
ÇÓà¢8ª'Óãú=ŠÔ¾ˆ©JQRD0Ámœ}hşR}üJ?Äª©ãÇ"Î|ù_ÿÓwuêN©¡©±yUA µêeØÏNÎ¤Ònâ:]SX°äär\ô]·”ËÉ5ÛøıpÀó_Ïeõ·üRß*3Àğäì/æ|vfğ›Î*şW¿f§tú¼/tGÚYÙÎ+ÿ…ÅÓqĞÉÿ„<è"¡wä5‘{q·}`ÌYZË¶8-J¾]ÍoÑÅ÷PF»G£}jüiö<@ …à
x§JVMŸ®.*ÿ”Š‰€ã¬³ÚÏü›îı\ G†±9hÙ“ní"Ğó+¸>0†êeğÔù#ÕÎÒ´ğ\İî”Â@N‹Ck_€]˜ó£mOÌ–”øÂíbg~Ç1‹¥÷±ºş€¤¥8˜ŸIı™Rª 5€ÂwC 9•G&e'Wú~­O]¢!ÚâUt¡âÚŒô›úé¡.½æ8˜†ßÖè«YïC~ÃÏÇÚ¹0yzT?mlè$³öÄÚİ³¿|û…~‚…)ïÆ¢åT3ÜÉˆDóÔN†ğ$EåÏk¨<0.ÙÃğÔ-ÇÓ¤e«}ê*¬L_%YÂrLs6Ÿ•ÇÅ¬Ã[ıÃïbê5ÅfQ®*(‹Ğîû]Ñ—Ûü,²\÷§°ûªå»”æÄÖ˜]"ÚT‰^_²½Ú^ã0»€?™Ç‘,dæYı”½y\ÍtÎ;g9![òNà>‚hx¼v¨®B7G÷èÄåº7h}qn>™õ%-û¡`³`sB~H=hu=ÆÂë…kbnäò1È›h„F¿[1FX‰kã%	Mä¹a0[0^`9‚3É@·Ö|?ıĞ!-k«ºÚõ¾ùHŞJo1×\6ımCÓÊá–H^¾œ‹ÿê‹`Î6V±à‡Hö#G‰[Ü½Ñä[L® 9kæ¯Ş3[YØ…‡±iÍ.æıX¡W îú'?ÅsôØ=WIˆXyBdDE7©8RĞ‘°ş«šlŞZåát{¹6‡9V#”–Ö­2“·z¥Œ[oâ0:&o™óè>8G~ûÆúÇGÒ©¦l;î'q½k·†&¯ÜudÊÚëâŒ ÒÁôÎ°œA|úÁã_ub&[À¨£"~ªæ
}ŞB][=Ù±¾sÏC•®vâRÇtnÃ¼vvm¦¨‹:™Á/÷iÒÁ¦óx©éÊ¡‰ÇÄÀ¾¸ı~Ùz×1_ò•9µ“´õs£"Â»^üdVÜD£5ìÖ¶aİúÔŞ=
é[¨/Ğ>³N=!LFŸÔçšgÕàè"¤ŒG†Øå¹iŠRûK)¾£æÑBÇàt7C\ÀáÉúâ3áôu^îÄ‚±\äºŒ¦'Ÿ^ ±¶xvQ0( áÜ#Ú¯˜è^Ö@”ãoyá1$å±ª42ÓÃ=®Ã“áŞR&Ø×·‹³i¶âÿ™˜}º÷ıª©«¼Oh™Ÿ,¿úK,Áõa{‘ù…˜,ş]‘ñ	lDÇ>õjÆ„ÿ¿óL-É­ël_Lªq3e )i*À¨C>o¬=Âö*°‡ÍöT‰”2\\İwr¦»sèÛ/›gÁêİ™9·^ßv)+!™:>âæK¾9ì`"Æê[%rÒ¾–›XÎ³Reî•ÏÙ÷Ğ·æP_­B|Èmæ(I@&Ó©k×1qNğ¶ôä]†Ów#ÓÎ;QXW²ŞN¨Î‚ëŞ–¡b–âxçœé'ûXµà½VÄUöS÷”­:fŠ„¶^á¥Íd3Æ4ÅóøĞt§°ä"áLÙ3Åú*	Y>?UzëS7zõ®×2é•ş@8+0 LaÿE ä©oúw£ ™=Ç¯Ô«¾¿{ÿf¨¼V_·Ë~wÅC´ÅŞC¶ÿÊ‚ÇËnÿJ&Åæùø"w˜d+ÙÒı
oSó*GÏı•-õ‡™âÜ_8ÿô'é–ûÇ2¥¯¨‰ìıÔïs™‡˜•tv ­î?]ĞçÙò2Ÿ–¥vÿsÛ¹o«Éá4²Ç9•ÿœ.Şªşfæ¦y–ú~ŸÚ&m1ªIsk´ÅmRº€3Ë€dÉ"/.°("bvveaöáyœpX£.y/oï+¢nkvÇÆ.è&$‘7#î±É\-eõ¼5[Xß®ó!5MÕ›<Õ©®4ÄÆî3™ÖyµvRZy Ağ €€?÷Ë@t\SŸ€æ×X>>|ôÕQj¸,“É¬â)äoÖÚÊ›‰JìàÓ2^™£ÙˆÉ‰&"éo²GÜp\uIÙ‹¾t¥&6|‡: G—ë1~²Nó³IÎd]ƒ%¹¹‚—Ã£ëƒ{D¾'ÈçL…mâiËÙ¶Lã××j©ÎµË§è?æ`:á4ÁÙÇ‘şŞ™…n±§RŸÉ‡7‰Ó%ñÎ¨»Vo`³9é™-UçèàˆœÉá†h½qdcñmq¶R–xÇf6€ÃvŞ«byŸM©Är:eĞˆ_¡O¹ª3É;}ë3±@—÷ó—³¦´Ãäe-šÔ©ßt<ë¹Á#´İ n<°³¿ØÅÊê=¶âiÈÔƒkYßk*ê¬X¶ÊnŞgKˆ³„•B_ğ§bÃøßA·ş²ëÑ—¦3ÑCïaQ={ÎE1,j¡–Ç±Ôì‘K-\£Ë«øq[o4óÎğ¹v4R’@aFOİŞ¢ğ¹ ¸ôTœHsO ¤Ø„{Ó5ëy<ş~’·n‡â‘oN3sí’(Ç-·©DK]åªF%­Â„.ÍJ=~¯¡µÏ'«İ2Œ³Ø&í»¦Î/B|q3œQŞ·™¸š.ïÙŞÁœÇ˜eÇ¤L ìÛ!aÉnè:©S¢g{Jˆb†æy‡»y¾8ñø6^d«wÇünmëüŞb‰Ì­üÉt1%Ü{öÉ·½á{A=4Ç5ŸÜg©†Á£urN ÌşLı*gW¹ wmOvŞóİ±Tt\ÄC*üUäİ5HéÚ¢q”3‰/Ù9¼¾p\!m…t¯p·ê­„€Æïıå¡ó×g·¡Åt"ã˜Ö· Å-~c9[ĞşÔ¨hxŒ(a¢V§€JÌ·ı¾‚/qrd3—íøtexG:×Œ÷-¨ö™Çqû’'£dì$Òñ~¦å`‡/UIX=ğÑ-}<Ş´äBÃüÛ2µCZ_¬Ü,–ä¸İ“ qáZ%·zt°®”-±€„C2ÑÛàÏ_–œ†œHºMğÉù’»5å7ÂI}½.¿Q0ÁÌ§s>8¼:\²±]*Ç4X\9v?
ıNå}–9˜¿ˆQAd_£L	a®WÙxBãqâ¨çÙhô]9E¬hT°{`Cp;(·ºU—“3‰èÍİFŞç/ÛÒrùˆ@îÈG›ş?Üv¶ÔtbWw\çŸÆ;_Ğ¯0MBí§¿Š·ˆÂ7ÛûvîÈÙ¯¸°©yp›ÉF[¾VÅ£;"/É<Äuì9Rº&ÛsF
Dµà±^€„êíz@Úß§ı^¸$ö.DK}vW‰¯Á«Ôr˜Æ"µ­øåŞQL`ƒïôüS‘´cúµ dœh·´ÑØ4\KˆŞZÙƒ­ÏÒ§Õ6½ÌAÅ˜	ZğNh„¾CFËbLGm~^Õ›Bì$“&hy‡òN©ÔCóÀš”C§òëª-$æmˆs¯Õ Z*OÃÙ¼8ãñYùAûfÍ[mé¼ñ†oˆ–<:yäù–ú'<]Xéß¨•œ£[¨\v İéíÙ X;‘"ÂÍW2œá*ĞÈ`'ŸÊrö0/ü“¯Ö¨_Üè,åxˆ™ßô³ƒìTTŠ”9;(}ÛMKâ°®wZqúÿÚR–
ÎıaP~ô	L	Zî Ñ2Á'30¤MÖmâ
$“Ù[S®ÚùèKŠÉtSİ–Bty¸”Jkè*<}Ş>YZDbvj_O˜üîøã0Œ~#ŞûvŞ¼<ØQaZUXÊl»‹j´”å}ñí»™c
&›Y§ñáw+—¨/Úv 4S]>oàrè¼˜é¡OY;<+«´h»fÛ(É¥÷pC]“Ú|%1Më|^-{!|ª½•Äz6KïãJ×õ™W€GRìUMe‰Æ)¢½îbşæĞğí${{{M}–½Ë.HÙ¦¨óóÁTXPá<Á{;£ AC¨IÈÎZ’óÊY8½Yl*^$æ¬ïË—YÇôÉØ¥N‘Ä)øi‰{ˆBÜ’•3Sp¿œû¯İáÓ¬@İåê€wã[rtÊè‰…ÙYE ¤Ç¶m0zßcÊíËæ•Y[²¿É2¹RB"ì½»å¢Eù([é	ì´²Ù(ÿ–‰ñzrÙÏ‘—F‹4j3 E$ZĞN'3d9k ÙF’(µcc˜§o®İw&‡„®s7Ô|În¿»aög†”fHŞc	“EOÛŞQ—–Òátõ¨æ79h,K¥tmŸåk¾àK¤6¸Ô«_Uµºñ¡‹8ùÖŞİéVÜ$ğÑŒÀ6“ÀN6ôòàcOŞqµî#Jx»[ä•u1W&eH£òšYªP-Û­çRüCŞ„l,5ûû¸8ÏIèØË)TéŠı‰VRZ9m=[-&;òÒ]ã\¢^Y-&™“p
×S¯ˆ9Ë×I@xaƒcîÉ`Õ½Iˆ¥ÉÎ¼QlÛn2';ßu°jPR½0ëìkDİ"K	y—ìå¨±İnÉJ¥vxq­ÀÖö˜Şlí ş0SVîÿàÏÃÌÖªğT!¼N(zjgTYãL
Ö™Ï°‚Ÿ[döÚIŸwVoÒlÆßAîP ›İ•gü(7ã‹ÅJoh %Ì|+ºVë3;şM²8ÓK‡¸fîæ¾»~ï)É6WA¹Ã`Ìİ$Ùr<õ¶É2K\/üæ‚=²İvˆEòtµx¦Ô—'BÉ—°@¦²-µH¹Mõ*ä=¨P«u.DÁ·¥ÌÁğ}ÄªNwÃÑ
„³ˆ~ö é÷Sº\»ô¯yî§	ôÚ¦ÁÜZSÊ…”ÉRñ~K[ŒËèN­Íö•IRâNPµ+•!u„Í­63÷åÜ[_8Dø¢l
Á‚PM›”fÈ,åğÍŒ†ZÎÜÔZ‡·Qé
é‚~.1ù‡f¸Ò¤¶äL4¿€¹÷Ìì`ãˆ?_m Êò£S4×7åóÆMÆçRWõí¯úö÷oáîü´×ùqãĞËÿO4²'0’ê‚ã?òßéàúú!ÆìZÛŸ´T?‘#Kö8ÿ\~ß7¼€vöHGÃ¡úğÏÓÊ®¡êMpÀ¯^¹8{Xöõù>wkwl|‹]~±Å.¹Èb]ì±‹.&ØÀ»íg]w°Å,ìÙì|mâaµ¦ıH"á‹îÌ-êÓYE÷1«ÇM´!aÚØMßëŠZ=˜µÏŞLX};m¥¬Æµ»9x@"'¥FÍî‹Ç«lØõ<8Gs8É8ìó%HN ‰±À4Ug‰ˆ(Ãågm2òc­`åqŒlU©Áæ<Ší'lúNquen‚3KìˆE8•~ìá9	)NUÔ:
,ê,4I;| ¾-5t@iâR%|;%Ø±_’n%7:”¯õ//2©5ÙY·$ñ†?Ô=œ"
Ï z±ï5’C¡B8·‡ÒZ¶YÀf<J?/³Æ
}
‘™§tg«/^œQ[u‡tÎÇ@{OZ—=‰´V£ítÖ õ~ÑÄ¦;©l4á­ŸÂõ{,XùğAÒY°$h±3G«x$yÙÜÚ#ü˜ä¦¾Ù>¢şøU©Õ±åFTÙåêQü˜ëI«Ç0ËåJG¹Ç?û³RÉµÄø3Nf‹Z;|>G÷Ó$?ŞˆuÔHÖª¯r8‡~eJA±pSÑ¹dÊål†ÙN}L3
RÙİ„t«>‚ô&£SkjÒ‚×°>F#ßxñê›¨..!ÅCPYn :`Ä!í!Æ±µ<¶³(B9R›şfxA†ŠÌ×ãã1@Ö‹mÍò’mª‰=w(_õDø3' Qº+#SnFf*…·§³¥[}Q€“h–m™Ríì0¨TÑ’rjroh0è%F¹JË¸,|?ó,ZßëîäzB\]ÍÙ{-„ş£µ6şá–ñĞÓÓşÄ-÷ZÍSÆ2Æ“)¸Âvó!:óCÀ¬ŞÒkøÒ­£FÂ¤<í±{>¶N¿`ˆ“ZMvŒºO4R%«_İ„ÈÚùÓÈî*Vo†Ét"ÊN4*ßçx´™Ğ·;…ıyGWÏø-€ItPbX+ZÌ¿mÁ¢ÎóòfµìÇ¤aÚv3Z®2n}£$áZæÁØEŸÉB|ìK3æİOP©øİÌsP^s()ukl•LwÈõİyµo…ñ G'ßìä©”•ûiaŞKi¤·2âè´ÿ‘¯69;ÑŒœâ“Ç˜İA.Û1
øÄCîúü™nè7õİMšéL=Zú\Ëi‰‚—­×.§ùwm²ÇŒ%¸@_÷ÓÚ£û1Ép
Ó3^¤½|Œ&Ú¨€G>šËfZ-	‡:vÅ†S«®.“–"ï;xÁéĞ‹–œâÀ‰…Î7A&ñQß“yŒ:Áøğ4»æ(™NpÑné]{>Ãá ªx¬zt#ÿ•u™­øõã9>*%…xÇÊEÊ_<§}ìñ-HÖõÆ¶|;<ˆ)ãoP'½ƒWùÑ¤|l’N“2¼¦3·Ú¨@Gş„ş {ñã¥€š°#²a7ßÙ[gË¾HÑN$e»Î©«•Ş× š0È:³i`78Ô<¯0ŒË~í…##m*ñ@5v±˜:<ÆVà°CV»ÑÖo]²i|ù9LFé·Fëx·Şà>¹BïıMÕØtLèÄJ|a˜¸Ââ:mÅSö}Rì€°äGÚ%ìN =/i½Tz#¦£dîzNR‡Õ¤ )”EóLb¢¯Î ú¢aÑPíNrÅ¬É4æËOİ,Ñ{C{ı´.<˜&JäSoÇ/í4Ş`Û £ÆÜ¬…ÆvŒ‚Õ2¹ÊÌ[œ`ÕÎ’æ_×l¶rÀÍ„ªG2@ÖE/Ò/pnä+ÃH)'î@¹Äûø *‡ÈÈİV”F	bL>Zæ½Ö[ÍŞe¢KŠŠ{¸±|û–2&Ğ/üµóğJ”ühİïB%U<İ‡É•ÙÃH¶ÚÑ«³êÚÿP|,PwæÊK¹)ô{nS-GVë¥İ›è½]nÑKWcW82œÓãìïlÈ×$‚0B-sµîŞ°û|Cz6«;¾,M×J|ş‡EÈKîViáÇWÏ«ZÂd‹8gÅÛõÚä“Úû¯«k º¡©T©ƒ>*øÂ9(Íw{[#ÄuCÑ‰àŠ,µX ½"^ÏÏ¿æ[M}%/EéØ"„´ÓRÍ­ÀÒIØ©bÊÚóŞøç|Öçé¹viˆ¨3,º³ÍW‰Î70“Ö&ùÙ3LÉì-O<V§$ß¾qö
,zdn7Ÿ—BB®lR‹É°3‡šÒ#e(¸¿#:5m\e1ínĞ}‹Û¯À¢Ä0É#Â·k¬®6ÂîìÅ§Z×
ñ‰tôø9%-›„
LÛuVƒØÌ¿.9*Æ}Û¬"¼}	íi+àO¿^´?XDCE|¼–%vCàbh¦_¼—´—Kd¹çÊ™‚aÃ#öı£
âèß#,™â¬ÓøûQ¿Á¢‚ÒÔÁ£†,É²7l/cWİBFUú:qš³€ H_I%ò.ZJTgıT ˜…Q¼ş³ïrğÛš6™
İ„
Zzµ½E)uXCˆlÂ^¾6[cNnãı¨½QIiú£Ù“¯&-xIò[´yAš0q¾T2y‚kã¢Ì_Ó,¤ô¡eÛI´Äí€›‹{ª&ô*|z¤í[DÊ•İöÿ€›iËÅVå[‘Tq6VõÊ¬;Á+)ğÍè%W>ß‚lXIQ’Õşªo’¯ğ‘o^J²¨¿]'‡YÚÀ1ı/™ä¹œLá…=Âq¯ÇÉü©âòÿònhÌeš"×aCox ¶¸¹½q&ÕÿÊ¸ºÈ'Ç"íé¹Ù %Òpû]Ró¢»ÚOC­‰s"^°œVÆ5³-³·>y8Á÷â¡2Ë;0d{qmµÚ¸)–ÆÁ6kšHWİ|,^†—ªô2*£É—kû¬á•Ù¾y~Õ½ÅC/J…±(İIa†Zr$âÃfÄ	¥Û)SåƒŠ¢¦âß—áõB€˜C4­œ›¸a¿GHvËš<n‹WNû&Ó;WìÕ¼{SÓ¹~8‹8N¨ÔÏ“^ªhÁƒƒå“µ(GÇßÎ2ÓıãüÔà!AqOŞäê{‡¾á3´æIÏ2àö ¢Ô,ğZM[_ªªæšÆû™?YoŞ¤º¿’Z;é,¡ÙøWÑƒ§ÃvûÁ’ıŸ'ŒWHŒï€mªì$ßO““¹·í¦²±[6üß±€ã½†§aû Ô9KóBëÔäQıjC"×õÁöãõÊŒ¸wZ.Ûo!>&@£ F,ö1•¯«óÜ¶-„÷İêC”%ù³qVÂ\bU\âÕ_€PgrÅâŒdÃâëş€4Ê%B«¿ı°lIHÆJûğØ& _şVøU°j·}ÚÆ?4ÏÜ{j(f¤‰è 7©(hí}è×zxş¶aí($7g¬Iy®¯ÜŠS‚ÖBxæ¤YK@©û‘ø[Ä Yç6iñ"8;“Æ{’,ÌÔjğ ÀîáÎÛÉ 52˜ĞÖÎÜ›#ZÙ—ú 	¾ÃŒj/6cätKIhX¬QÔ|ÃÓ!Eˆ>)$0ò¨d¢-û?6>cçÇÍrÖöÃCTş<Ôæm°Ã´¹˜lTÅµî¤Ïõ+{¹ˆµ°ß[Z¾g‚h‚‹ş¾0åÌf¢cFÇƒh‹Ç*£§OÜh)ÈHÎEÌ†Î—dÙš:„É´şp}I°2ú;ï_¿ÚË¹²*©ôF'Àeèç2ŠºFôå.(»nääÏèÑ2fŠ3jí°4Ğ£h³Úø#Ÿ¼Î¢ì|Ù·ü¢­pÔJlWz:y º,b“˜ŒÄ»¹Ã!¢¡± IûZ-–"–Ï$ºŞc$ñÈ^»ïà @ß¸³[™ü#òÂÒm-{ãŠì‹'½öŸaF?;¥§@´eœa½ŸBß9fO4d†ö	dùÁˆöTvÿÙ‰ĞÙ/œ²Ì	g÷S&™m÷»,ì.Ò˜Ñ×iîìì|W–z¸>MîØ_&ÿÂƒ,µõEchwÖš1'¨`dêh¡yhqá 'x'İˆ7nó2êc‰½|Vò98Æ¹1¡ƒ¶ß(	£mšñ§æºş¨-„Èúí¤álûì²_CL"œİ½¡Ç ®<Ì9¯µ&aû¨ï³îß€ Ê™êK÷'†.»î²tÄ­¤ìwÿHÕ&vøÄr-tğx‚Bç2"Îã—ÊÆW°‹ÀÙè‰„ì"(“Ï¥_Gˆkåéã…Ÿø3Ÿ÷“î•ôj‰J[»GÿÌ3}¶¿>gÛ¯$³×¯,ºcÔÜgï‰aÇ.¼°HĞ@…•e[r#{~VP³JvnçæCoÍâ&ÿ¾tyÒœ­½‹˜Åı&‡D¬÷Ó®×m±Û”Y#“Râì·È$Œëo”Q´osP[ª½1…¯'7ÒÜøá„pzæÿé×ş‹ëÒ ¨’R©jaŒC6mtDöş…:ø´\"Ÿ3ÕìEÌµûµy9ŞÚ=Ç˜q¢Ï˜Ûê€ùu.êŸJ¼ 'l˜4£ååñÊ_„ØÃÛ°àÁ/™ÿ´Ä¬OQƒû@h‹xmk5‹ÆgÉí—9Æß0ÓIÀëâªë«Î¦¾g–Ê:ŠFRrU¯¯%3Vı”Á!÷Ê×ÕùØRGj>J¹Ø¥?¢Á:OO­aÆù:Úzê!‘	»6;79»4ÿ=r ÌOk|ÓæM|‡9õšK1“cÖ°5Ci%Ú×4­mƒÜ‡XïC÷+<pU5e3
”üKdœ´½×(šâåÃôTÕÔ)†vØTo¿¡”\ÉµôVÔÏíiŸh]½¸éåîf¸W–·£ãv	îSûz\[dmÖËìyJéÏh@™´âHÎ‚¨j±ÅqÿY2ûT„$ãFÈ¦?v7Ùi”ù×sõºŠ–âmÀ$h	¿<ø”N\5%A_7jçT}$Öøj°«d'JëğïX(‡¥¥R,ïáåV”riî›â#‡>Ç(æÜ3pPLd8•³íõké¤åkÉØ_ÇWr®«,Ibõ8 è
Øepeßõ#¥V%œë´Z1şÖ\|Íºü”êz#†Û¹nª{¾§ÛR4¶ÁÏ>¥yBY&¿%ü¹ÕªâDŸŸBW­Û­³ëô¨µXVôEÚDSÇG­…äŒ¼ ¼¦r!ø•¦Bş43º²<~—İAˆ1²¼*,Bâ×j56·ËÉ£•¯úÌÁ«¨aıùs9†zŸó¥ÒÏ[…Lj¥XC®¼uÂ{V]¾MâM”`ÒÚCí(ù¤ó6á!twx¹t:hê Ò'±{Ğ¿f¢†K™f™Q_ZŞ
’µKßuÏàXªXÌ·ß6ÌJVÏ±YaBá®Ï|N÷¸N•/¯»Ó(Ü¯äı¸Ë_ÀOÉ±xß­›hÉ2=à9¨EŒ»†^‰*Õ«Ï—®Ö«?å4×ò94…•…k¼èë¯ÏNL&Ù@ã×AÓIëÙ4ëêeâ­¡f'Lr~—)o Í]Dò.w<aÄ ÇèK ró róîXm’.}ƒ^…_³yØZkQ3<êZÑÜöTW#¸ªkıQ]åCê¡$[f?ˆ+úiÊ×ğŸÖû…²Dù´:qöMË1@Ærû¾ƒr	(%¥äëv(ï„.„Ø×¨n6±pv!ÏŒMŞõ;3s*Ùù-N‘º„íògû‹Ñ.|Cøy]ñnßjZ’{OË=JÆC4‰…œËU"¦ê
=Õe³í˜ôğC¿÷2y§>j‹¸³@DïdÕršPÂwÔyM—ğpyn¹11P!P*!M$öXT]´W>£R>¤Í{å3_ÖéOÄ;4ú<‡¦.±€æx¥DZ„-Ÿë7ùzn[==@=Ú^¯.`”–së˜ŸÅÍtŞU6n¤5î4û›¾P¿Ü¡ìäı´ÊEeå°õ=ùfIéŸÀ®f›ÿÄ`ĞPÄİä€•ëD§²$§n1<M	5I–º‰yk'{ÓU({I1ü?vÁ¸K+íy	{)pÅ¬d2,üI^‹0N|ãåó`,ªÎÛg9¤ïğRVı‹Õª¦fˆ®]Vï¸?NxÙHf-D{Ù›2¶=’Å@	#Ô‚ ]wJøëÅûU<¤^Oi0ššúC$ÇV˜MÂ…»ïEÏ‹Ù©µ!Nšı;`”<Ìc¨8$E(×²Qé¤%]æÔæziå,B]Ï.D¯P£Í3î^3EÑ¢´Ğ'PÑT`¯>-7×&`Gáè\0Ç>’µü÷d9šjš±èI˜w•òèSœ¹ºõ†Œš²x››´lÏª:Ÿ©õ‡iîPn°ÿ!>»¶ÎA-% º ^còv…Ÿ‡	ûwÂİ£ Ãš  ø¢¤ kÀiÜŸ
±ğÈ?}b=;­Â=ôRrÖù0-¸¾“àWOgnÙâÚå(Ûõ;Ôwy¦…lÎW®L¶`ûMõ’Š
=jå²<ì-*]Ù¡=~ø_}ò¯œ/›Ïrl(ŸXŒ@Ê¦<n?h±¸­Æ¹·ñ€ö¤Ü:H1#Ê™˜áË±Á{]ìE€¤îGfóYûÍré³V]•äi¦RWœÁÄ§ö±.Ä–nBä0%~e
'n˜<–¿š'VBx *ßZFÛc'ò¤’½ÆÔOt±ßIrÃ„|ŸÎZC/líÃ êêz3Ä'²î„>ë‡¡ß®ó3»~LĞòò/Ò8âªUşØjî…¾óÔ+ÁqòÜûÿúĞYb…2bÎŸşØãêiê%Kš4Şçe(QïÇ`çpîõ™‡sô¼ÃÏÕ†.øÊV<Å
VF&z(‹?­ÊqfP9Œİ*ÂˆyO÷æ˜!ƒl“¿PØŒ|èì?Rµh³ß6Ëâkê`eºP€¢®Eo1n“S=ï)±í)e²òã(×Û˜´}äË<"Î`‚‡9³Édh®İ—å ì»ĞÕ'éáëKÈ¦¢õñX)#†ª“C¼QFN} ¯»ö…FXéÉnˆ2,¾C~Ä÷«• CÔÌ½ñ^#_tÀPm_vG×‘Ã$3"3çjİ-<ñ#ü2¢Éâ¾P¿‹´i¥0ÍîŠxëÂW°Š§Y#Ü¥ÁûÛaRËJ².\3õ?ÇhaÂ‰zÉ…55x†Ï»ĞT©zö¢¸/^5Ä¯5øÕş¹j‚o'[%é¬ËUgûü°#«¸#sc‹xæ]¸ââ#­ÂºÆù·Ûe'Âˆ;ßğÂg»­±†Óèp{”ñéà4ÎŞ¬L_E,_5®À½ø°c ¢Yì8Ì}[ÄIkQÒ¿_ 4ÂmOU¿¯õÃTÔáÕ{‘8R¯×ÎÆĞÏ «’íK,Ög®×æ±·È¦s¢·Õ§v!ÀS&è4’ZÅ'hOJbEc}ÊÌ¹CÊ×	6'RsT„7S$Âx6ƒ)ô{Nb5J"=úW<X¡9Q?³ô|ş]íUŸŞ·Ï
ûG…Û§ËsT¾SÄÑíÆLüêq->ù]D5ŸÉîùav¬–™\XébØsŞÄfrCrªuãˆ²AŸ­­k¼óğõE|ÌJî’áO~òhùIÎmF ™Öô,
õÅ¨[åû¹QÀaõQÀ|6×ªùs©Ï;§‚¢ÒšëÕ_"½KKÕL‹jÔ¿é]ğ§
€Èû1ß/è5Öôæu œêFµï¦(Ü­£í}ä¿ªV®ûÄë&ÿ¶•¸^üêò²~+Ìw›°nªH~ª•»M/-—Ÿ{¬ƒ¬&úóÌ+Æ•Â€›u¼RÙı>Èsjğrêovn2ëGòFËOæJ}zd¿vŞÍ.ÅgHÏNJ[V‘	~õúŠ=¢¿İÎcãÁtë–½¿Û‘;¾5§7\·äı;Ë5Q¸Kå½< ´ºÑNÏÀh-ÕÇÓù	\8˜ää‰ÿh®>ÛsÜ?JŠtùHä‰VWdálZgsf­Hc³ ë(ê´ÁÏ#ëH
íi’PM3ïêõsƒ[Š‰T )ké4ÆÕbÃBÔÍ-NPÿÇ}áqŒÔNµnºŸõ¾iRºô›$ìû³>W4Í™ë/õu5Åå=!ÖÂ‘®nÔ¦<ƒY‰nô«k §œø<©Y!ÇÎì&úI_…Œø·ÊùÜ+òúTN¯è>æ¿tcã”¡ÿU`<ÿwŞ¿ì¦x$­{?ğ«3“¨î…ü¾’oT›U¿Ô~T_TQ§{íÉ/äòË ,¢]µ£c×şójUÑg¬˜|cÇuÍşş÷ú•ß3qMæ;×;×ûòğ]Ğ=ÿê^ıİçpü>¿¯nl¦\|ÔÃÔo³3ì²]Ô˜<o³å’8Ó•-’î¤Ş!}:ß²<_®‚o#Yxğ~8RPÃ:yşËÇé¾l/O0EAUÑ[ªÙYRû´'ÈÕåhÁT~‘ˆÍˆûÈ‘¼­ ‚WV4wÈùúı§.;ÊèqüYÿŒO*µgkTjık¥{6êš ö8Ú¨Æú7U›Ôüğş™Û2µcJQ}*Mn^Õ³oNWçÛ,€Ôıáè_öUõ
V‹Ë€~ê/ÃUk²Qi™w†‚=k5A¬+C8\Œ[pÑi§çCZyOGŒØ%ZzC)¨“H´'÷–`	HÉUö(Vò´şsz2ÃŞÛ‘ñ‡ãRJtŠß®»ùëÓgÃÜJÜõ!ı¶nZO‡Æ] Æà¥ÇšÉëÆì:Û#&’ñuÚ‘&P—ÎÔøó°°_ÀÉRç¤“¶|Ç×Ê„­ }¬è
šZ½Z™Â^>‰»è‰#†…È•¹ëÌp… ø?4-™ú!©×¼GK­x§k?ˆ|ÅÔj[ÆõŒEÓ_\È(Ò¶áÕ‘C³N&<,Ïš:§Æò)²Ïšèp–1‚ç<`Oİ¦±x Án#b<§Ş›‚ïØysÁƒ–É‘YßDâ¨v	Øf#ñ·&bm8ßpi%¤­>m#¾•Ô µ0 b£"lH
‹+O,¡˜S9kÑü†š=ó­çQmè½}`ı	›m¡n[–hû(l¹v¤q"Ê/ÃøUÄ‹±\b1òá…IÆÍs-î9÷[”ë÷ îåwÙ6Q¢zûåÙBLG2½±
â¨sç|¸Dii/q&:ñÎ´!ÆÜİNøı äK(–ç.wúäfÁËH„]a˜œÚHŞe¾çÜ5u€ä7s,Á£æãÍ_şœ8&C ˜DÍòõtÍòê÷Z{2ĞGd÷0dãF½;ì¢1xş–·À¾í¢ä÷¿­#cˆ tî™úYa£k¨qÍV4´…ñ!?ü±n+{»!H1ŞˆÑŠ.ğĞ	U&ìgjdF) ƒAéQ›äZ–á.br@o(4ØÂş0¸%Ö0Ğ­5Ø)óîˆı\Úf	H«2’(|xRØÈXgi”(Šï>o…îjØ¡Ñy=›ªÙ5ZÕG»äE3lÑ€•©øàF|jF›.<¸WÙa[N·ì€‚@éqÏŞÜÈèÂ mŠ}–¢Bá²ÙÙ`.—X•ç3pã‘Z±Duæ_L•‹òé­²;zkñW¯ìBN1_^ÃFœÚ·J¨•^TôUÎv_§è6yÔË|ùÅ	Nî¡Ò ›yà€ÃÔÍÃ²cšƒ±y5¨]õ§Ó$Ü·ª5uo”ùÇ²¯³6?f×x•şŒ~ >ÉtuÑûûgó»·²<~Ôÿ¿Ÿ´åzôY?ÛøÂ¾ÏŞÇÈWÊ»Ø‡?ÍÎİğ‘ÍòiH¤!tƒjZŞX»ã¸Ğ
å:…McÀda	)ö‰ êycY ÒøĞ ®\^ôÍZ(kôÚ7ğĞ*Äèíuğkív\œqyëL]GÈŠo8²â§¼°‡EÃàJ:Jl^¥Îe¶ò_¶‰•ÔSN4FVÑ›:µC¥“ØRÍ\c5]Âï1ç]âNÚ´õÙîObBEU°Š~ÊğqlÄªVAYÇãcO›*í»«7ÄâÌŞæ´0Uä”-¬gŞìfVíj”L½sD0ÖË4kKøA‰4œ ­Eµ:¸}«[¸º;<m84?- úQ³%Ùlmdg}ÆÛœw-~–É—y†zgÚ¶t²Ì^şíš=_4¬Ãqí7\ïÒ
!ª¦ÎŸ±süóNw¢Ôµ]^…ÜŒek¯)Ÿ— Ze±Gz}ÜåÁ/åÑ¤©P°da¤aàÈøp=Ã‡†³ŸéàÍ¶´OdÎÍt	ÖE¸®Gù‘¯¸JJòG£6Ù˜VqwÕŞ»±0
èNÁõW}yıš Õræ\Ík’v]šÆı²¶kJ[b[€ò	ÍººÄ+šéƒÑ(òO‰¾Fn*.D›‡#|³¦—E9F/øµJj[%G´¾f~h±§ıC@ÓV}Ü;A[Î¹PI1e©Z4-…Zİ›Y†èO¹Ùse0ÖnWÛ—˜§ "pÛ;qĞb‚Ñ]ò¸ä»ßÎ&èâóQ[2™»İìä^ñ±DÙÆ±òÑrMG•.ÖàÆÑô¸pt‘ BãÌq®l…?ár’ÖMíÁ‚÷îÂ E·¢ÑÇ'éóĞaĞ–®ÍÍàÏOèöw)7½e-/@wv[€ôJfT¯PFá,%2FcÒzì»§ù$şøø5/İ*Ìwßı(F¾qE	œ
ô”î•êÈ¡¯¯cÑA†Ã³OHz£fd1`©£VÎ	9\;g{F¥Ú”F‘rF ¡}íI[³°BªºßIş–HNêkZ@ æ k8ü=.¾I#dÕ]ËË$¯¢t=Ì;<¨'Çª]Ç‹®³6ÀïAKŞ’.
{0Ø"šXe)oDXuø3ñåó#%XªÎ	|ºİŞhı3oúä†ÏN­ ®i*nF7¦Uƒf;Pñqrö’
ÿŠkàÆè^nö¡•Š|áÏŞƒY¼L—ÕkÁhö5úñ£Àğg±Ò¬¼¦D‡ksÿ¶<†)¿ô(Ñ°(Ú[#«6G	ƒK¾ho¸“{±Ç»*¾£h›Šú
\‰È‘‰—êô©†	ÜĞöå¦sìà®4Ä}½]ÿØ'_ÀsSÆBÇd7²¦¾™ºI¤èDò©ÆmÒÒ¥ŞÓÌ'ïË(Óô‡=óìo(5¨Ìkıçò(J»›‹ğÆÓ$+¯¢ŠŞŸ]5Àb­ŒÌÔÉX#«i9¥n|=úeÉ©|ªäUu¥Dd·í¼ùËjğ·­.¾ê:x4Œ÷Ë¢}÷.óöÄ`îÑãq‚±°¨ºË˜Kàj$kÈ5e@°•h*£v1Å=é$!ÍÖ&4ºH©G¬ãğ:ÇcøÈDÅ.ƒøˆiw¶Hy)ÕÍßw‰ºjâ<+H(’¢Én+>¤Oÿ»Dªc[#}@ŸdOˆx05ÅàØ_“ÂÀdüµ/şñˆÁaij,WÜÏ{xğq^U$VÄUP“¸™d@÷7.„4µähZ±›ÄZC)Ï/Çwfàìèıàãî¾èØ ˜Íœı›3¿	»Æ³gÎ€ºÖâlVjîìMbSĞH‘›¿û·+6”b9¢á€ÃF›¼eœ ;h<RÏ"óD@½êÙ$×œ³|‡Úä[ö2§Ø}ËË<\~hi§vé+n’7=¹Ë¼'•FmöF=ìœ`c>ÒÎIºæ?¿‚J¹íez§‡S®Qöèß·óõ¶³Ó4šs;Lµ²ÎÃŞ½%ÑÂã ¢œ»²Ã¬ãï32§Òšã/òĞRãñ]GYá>|¶‰FİnWôqİ\={5;›³õ´ÓíË¸\î´¹şOç¹K]|H€µïıŞÀ#Ã=»âe„k‹8» i¼·Y£Pø{oh»òâà¤ZO©°’Âû+-öaÌJåYª|ÿ)²Geæ"éş¹¬p­‹{Ã³3:àA¹ÇåÕ	=Ó_¼'e.WªC-Yòò²©ÿ ]TC0}°~A@qnê¨•i2JXæ*ßÇ„¸¢¦ÓdG¢Œéu ©pšEH<ó¬x\æH+Æş	Á+tÎ×Lƒ_º%	æ½í¦¯	<ÄƒRG %Ü
ƒfÇkÆ%#qÆR¨÷ù®®ç€yƒx.3?íc»Zø©N&íwDÈ'áŒ’64éĞ¦«DŞåÿDr\P_Dµ«˜.v Éğ¦|ä¯\DŞPBÊê÷°t½÷>ı€ê‘Dï@àulV¸-Â„‡ ¥íúQ^Ö#iÈœ ÑrJµìóôËMÊùÊ]ÔÄTâ3ïQ=SÈLúŠÔ›²¿>%Ç øuğùÕp€u ×Fm¤ÊéÔ„›¢ƒ¬M“„u« ù%ò–)Íù¾IQÎÄ àù	h ‘|,Õ»gù&»Ø_<6;v¾>uwvu¾dgY_Õoôpı?T§ìÀÿúf¸2;u¾Tß]Q«ìWçƒÿÕŸ6ÏÌ˜ù+{ÙÏcsgfÏ·ØÅ»ìb‹]v±Å.»Øb]ì±‹.vØÅ»ìb]Ç!ÎP{¶³q¾4õ²A$1W\Á¸êE6L¯_Õ{eú"¨ir6­Bf©ï–!Kt£ı°şp.=²òúŸ<´~Ş— 4¾B½5£81ƒ²¥ß7%Ø¤öI¡ú…+ƒ]â=Çm+İßÁ^¯¤S‹JÒª^7äğ¶®D»ËKõ&.Î©S‚K¡ø9e¥H±ıŒ:eD0q÷©{¦Õ÷>S¯„^÷g„½øØQ¾¨]îÛsÓ»\ĞkeQ_¡ò+3‘¨\lZw­k=É	F³1(€Ó³Œ2tgÿƒM—	«>çfV¸g	Ã.Fz’Õ-Ğ?(’º!p#£iÕñrâ(nNfĞB
@1µì2n7aßé;ÙCc!1e ^ĞÌ~ÛØ’4²Ô~ªİ»Y6Û+ía¸-·c{cĞÜ+ªV§ÅLí“âØ<n<èİ‹I0?µ÷¤^´Š7•Å±õØ¯EÜ+äA$V{½†õ9í¥{BH¯®ö—ÃlÊ,„.Ş'¸,ºì-Xª<‘k*O¤hhÏûË„¦oŠm6ìş†¹ÿÂ4%Y>¶‰¹m½ï’´ÁÁÜ…„™ŞUV:?‚S—WmH9xµ!Ğ„äÆÄèP(Ô{’­w*±kÑ•ZHbğè%u®µŞbÇë'=ıŒ”•ğ[7£A3¶ÑÑqOVMûÀÌm'Z^ÖÜÃ`P¢ÇÈ|5êFl·k`pH¹ÁA¹Â «y±ÉYİ_[„ ‡ÏW`åÙ¾³3C^&ëzÜ¬0Ä8Ê”©Wô¹´@˜Îf‹™mùeIjçaÄ	j‘0vk¢VGÙ’`A7"3­)¬˜¤|Ø¨¢³}o<°NçvlÅğcŸ1Ë³[2¼çñrÅ2À>À¡ç³×Ø4O´ãSy^ïsÜ]­ÔàÉ±M"9ÎÎÖ&¿ÚzÉîŸYæb¢+B2˜İ5]<ïö^úºw=µ5J?fy<täbGÎ±ïXJò>ß¬[jî÷:=—E2NîdúÆx8E‰+RxËµ­Y‡—˜¶°ˆT¤Òû¹û˜ÀqO˜ª\[ãéNŞ7Z›g
q­ÔÊ—ÆÍšÿFtRàœkBÔ}á»e![øİ?ÛeÉÀ¿²Øóz#§X*è¾cüÏÛ§®D<õÒ¹ıª#2ğÕ^QmÀI‹]TJ¡o!…]2²¤b%›’	(	Í|1Ê ÙÓÁi½›è$ÛÁ2¤Ü8(J×Ü>xÔQ«ñÃ²è¸A3‘X»7Ü²i65Ÿ[N5³»–Çwa"jî…Çí]Á#Oœé[…ÑD‡%»ç»ÏKøş-”¬U²æÑw½›vş#¼<O<¨Yl	—q¸TãÍ'\+­õ¶r^ihY~·|³¸~hFo&ÚZk"‹W3¢‘ÁÊÎ@U°Á,n¿è¹!(hÂ¡3/¢TœjıÅÍU†[\ˆíï<ôæ“
¥Ó†–Şˆ¥º¥åû!”îáàÄ“)‰U¤¦÷Å3|f„PAÅÓ”ïµU%‘n ÿè£È1m47—°mÉß»Ÿ®©H…	ÊÓ-ß3“¾™qÉéZˆÚ\;ØrCÃ¿ëy4Ã‘Î„^)
ÎéM?:~(ÚWV@`É{ã»¸öş!ùêÈ‘PÛ×aXV÷›ÂàíšWu	ER™àg‰g—Èóé„Ql‚YÔµ79»ìÕğöÔ´Ü¤3PÖLö°ä¹Ä3]ÍIib	3_Ö:àOàmœT_\€úLRÂ xfÌÂ$¹(~yÜ¥ËTøI_HUtë|—£‚d$ø_"		XûÁ¬)íìb<w/ZkÑ"z£‰ÕuéA¨e’^[Ëîî„GxÍÆñcdè¹{ôOJı¥V‡°c›óÁL7vòñëÛûGÊ}ò£‘˜›<™—«9Z"Œ|¹îw+ğÜë<U”‚¯MÑÄCÙ‡>óe­C
»0i¡^¥¾‰áÁiO15ãûøYÜÒ¥öH½óÉ6Ú´ÕÎa>s‰¬iP0¼øP:UbT`tjg4N¾›A×¸9hæÍÆÓ©³ÚÒ|Ã$·CB¤™Tˆ÷_ü
3.^ÊVo½Wõ®½£9º¯¨2†‡’¯’/oã`˜·5‡f’Ç/åjíMÔ–Lf‹¥·ƒ½Xlq‘Z•çÿy_ß—5ÈRNq×ûNêz€q—“Ä‚ïKótæ÷¿¶•Uµ2ì¨Ù†<×,I!fpU)T8¦êS¯{ÏØ~`Óö¹(/Ù<*^¤×Ê¾’HóOå ÑÉVd'Î‘µïú]Îc®L. Œ"¬—]Tâ-«ô_ìËÀ{¼ŸL@Ş\Í‹NßF[`„SmVíÑZ3&œK ›6.W“B<"‡3(˜ŞÇËh–¢K‰ça¾o |H¡÷&·ÍS:Qˆ÷Î%=š¼y¾5Ğdå
]}Ñ~Ù~ö°+’J§èåÀ­–è^ ]ªÃíüqë#?`"EÓ•ÕäáÕ1C¢k6V}JSú%Y„FõHä¹YYŠö”Şá¶ÎTµç,„Ë´ãPĞ£&zEÜÎ JàénÙÂÕñ:0²ÛzàŒiæHszÂ='ØQ	~nÛ•“şì¥x#Æ•ÄÔÅPº½û]`±Üî8^l3`I(‘U‚–:¢W#/ß-½†±‰¨öa©G CŸ·‚®¼SªîÕÂé:sØúÊ«Ø!·İì¤;ŞhÃ¦3íèb„¡9›/ï;É›×²ĞÏ°/1‘-V“ªlcÖ'ïF\µÇ¥<ùÀILqlUÈ]J¥0jñ¥X×GU¹‰<ú	Å¯úûe× µ¯½M;R¿Àá
¿ÊnÀDvŸ	ü·(Ò¦s8ââş©gV)Ø
ŒÑ¾;÷œÜ›¥¹Ölá
÷8§ñEwÁ<òŠz$È^h£@lÑ8Co†[,lvëF'Ş†oıCòíÇ;ƒé¨ ş%’¬øOù‰İæ gR¶r¡w6ÑÀÄxğÅpø>P–‹ºNÎ:OÔd4BÁìßGY‘—WhH1ç}ÏúkÉ"Ññsôv÷…N–šäu›Ew+Í"újŒ:dK'[ãZ{üœŠnÁ½Ã`İ~×27&j¡$b ê‚n¤¶Ü”í››Ê}ü¹¤Ç½ušş±viĞ-.á	¬€U{—¼±Õ\¨·üFpKÕ£7*˜kEØüÜ‚ø%Ãõ~1P°‘Êó—€×»Pò÷NW£·HÅ¤‚7<4ÊŠ°ÕªÇ%ºÄÈİ[ßuHûJvÙ´<”J½ÇŠs÷éC¢ßĞŠD¨[µ¡öÈñIã¬¸õŞ55-`‰œSÓÉBƒ¨ƒ fkà›^Í=ö‚>÷İs 3U¤&z‚…sÆÓBljòåß»…} CÀPà×¬óÍ @¨C¬Ã•«ëp[ {0¯ì–ëwĞ
PóØ°@­†İÓ~ šÊd?6­ZèÙƒö	«’·^ğ<^>‰…9:F÷’CÚq¯2oas*WFCšhğüKúü 3U½Ë^ 1üo×“;ç›ÌÅ¯/è•[‰ÊÑoò³]H¿ÍwŠV‡Â™	ü_œò´ñË‡VbuÔ1*;"ˆÅ˜Ù¬j’C¤û{VOGË¤¶Şªo]Şq;:q;G´Ó†#1¥Î/rßs'œ¥gx$Pï›Póû“{?)¾ÅP^l/RœÅ$›ş‘•¦^èÿ®ˆíp¸"^&A·šO–9 ÛØšO[÷,œ(ÅŸMÏPK»	b«ÍúXğf¶ı"sÂ—ä.]Z7¼Rj. q€ğV|®³=Áh-Rks­D+tù¸Š–i	+çÒéÍ×ú=ş}7ÍÔyP8ãÔ¨°rùKU{}ç·ëLlëP°¬MIÄ~»ÚöT…=rw1<j¨~Ûj»5o¹=N.,Â.S›´Ë3ùiö€|<‚LXJB{ÊVZ’jÅêQ7•¾­2¤Ü·Ä´”0ïé7î25(èâJªAHœ¤÷§SµpÊ“CrÎO8Éîy›©„J·_`åf{ûôÒÇ<ªılİŸb…=†lT4˜¢ÑY´nØÁÙh?4çì’Ê%Ï©Æ%šç3ÃxãšğiGI"@n;™&NßW{G¾3ÒKæºª ]•"ÀêkZ@:$Ù¦Œôj¬Ó;F1&âê€Ñ„&ç]ùv±* ÂÍ+=§­ß7t» rsÆH`D£»0Õbç Â^M0E×ÃğÚÌª¬µrO§®¨‘Å?P+êø®ûşëåßÈñı_ø Şı^ÏJEÛUç (èÖŠ€sÅ‡ÇÊæ$Š Æ#0ôÚ,?(1Y¦…'ÙwÑ1ÖÓø; L=ı×)ÙÓ~Z£ªÓNq?#º_eÔÚïÖD×GÇ?ã¼te{@~£wúu ôÕÓ™àë—0O~İƒ±ÊEˆY@ön¼/˜5,G" äc'ÚœÊ·ÊüA§„–ˆï½ªáÉ³ÓFr¨‹Äœ’‡Ò™ïK²ÒÄZ{çÓÎpàÇ.¸D“ÂİÈ4½K(Šs­po¬)6kJ­#j÷Ã­O­ğEÊÓÉ;Gèˆ¶qØ`Py²r¤¢B|QÍÎœOKºšR¾²=â~ág²w†°Ítuu®pMoÄ•ùCcYWwwÑHy6N:2®7•[U]âşuş%ûÙq«æ_Õ×îq³ü)ğõ¾bë&³úšèŞAËj=‰ZQ.à‹½×_d;Ï÷¦$$£[Äö±PÑ @ ß÷Û›{ò(ıÕ>:Î/ÓPy<Ñ[îDYñ\…öq6LEùìÛç Ä¯Oï¢÷”~¿ä]-âa
‡6Õ”tÂ1¯?3FÃÃ1YW\öÜ†òClˆjA9¶¾ø@˜ü!WQ"MB`Xß=€PÏ±ËÔ3—¿üQ VÀC5L×(ú…©	ã>·)(17ÖÊ)'08š>ğ>Õğû%ÿ‚Vƒ AËKüîù4%l1Ãõ€ä¾iè†?ÅŞªh§î[ëáÖçÂ´ñ%h8¬*¿TK‘<·Ÿ*Z‘İ)ŒÛĞIR4s°Õ¾9ÙÄF“šË•‰È\®u„¡MOõ=ãª-™z‡¬Ê#@3`ëØqG¿íZVNá… ëxÚ^î%œ¸Soò@Ø™ñÛñÚ?`CÌ3?ö½1¤˜ß«^Ço‰NF‘?´‚Aªˆï¨ı‰‹l÷ßÑ~³ª/8òş:ùgğa€_¥¾¼ÌÓSùË}ª]1q£ò%ËÃÌĞ"´%Œ÷"U–·”¢í"¢”R›Ú·‰²UÏnq7—jÓCy€1vW²¿âî“›ã¸wtÙ8=œşqÌß'*‡ÇèŞpŞP""ZïË9ûsğf@³€`BâÉ‰Óvv‡kOl è‰’ü3àıú{'>)7G½.ÀmŸ#JÓ÷ı<íâ£Gÿê5Oıv¼ù9ã~Î[Æ2òæ°šıšáëYÏ)yïÒ,c!ÔHÃœÌ~-¿ETˆ;íWwo ÷>œÎuª mrè[f2ïzâ*vOù±ÌÍ¾p*Ú^!î´Æœiíi|€_s¶ÇĞŠÌ¨6iw$¸E5Ş¬Û`ïÿ¦q¢¦|7Œx)-+Õ¶ËA¼}øùğùİ½÷MùXàŠ¯’ß„ìj}HÈ\awiúS\SÏé9I©pµ´œ¤³4~Êe©»õ›–ˆ¹ö>Ç™–kPÊÉªêä½§xŸKëV?Úf[W%[²è"3J(»*yqÛÂïÄ•9ú:7½s­aş	ò·'eäFqÕò¸šô®Êï%C7­S?Å¯çN†0 ÌË_å[Áîùñ:QÎ‘—Çğ,¸÷æ1”^N¥ñgüFıáÖ³baøİ¯n6'²QóRQ¾´{0=#0±š|ÎÓ¾>w´½ ü>“8Nš(MVRG—±œP›ı*áÊ jÚ#¿™ (¸Ÿúµı9ùSŸ©}Šêç=O;À‹‡WX+¾	,¯) æ·üydû‹ÉÊŸ!ä2Æìù€Ú{ëw à9Ø¼òæ1Ü´Z›÷ãîƒ(¤÷©÷kW_¤ÓÖ8€€ ¶Ã)ÿ ¡–§ÕØ$É§ªC» |O¤Ú0‡·N›‘­Ğı‡²ûQmIPÌğµ¡oäŸáü×T´©÷wºBú”ş…Üù²Õ\ßoê€ûˆàó“c’D ¡à$ ñ…¯Ynz}¢†UË.±ŠåÉûi]±C)=|M	÷TæÚŞ_…|Ìƒ#¥c,LØJGçÆ‰W)ô.¿<|‡Lã|şÈ&DÚ±Ù8.®mª0à ï2~ö‡ã¡U¤ĞÃıíÑdiJ7,;Ùå7]ÃÅÁv§èÌKo%#§&L;ÌÍß@®±¾¨¥Ç¿_êf˜òLÕàá„Wq·ımà×MÆ¿ğÕH¶àN T@GækñÍÕÕçØL½ØŸÑr~f¢K~^²ÛÃ?y¾vñWâs€²ê(}ô9Ø;¿^ç‹õËÑ¯xnMVårÖ—Ç>‹.›¯"IQOíêg‚mÈsT»63¯oì<{8¤œºlrÒŞùÓ–6/Ã¾Z)€xù=ê–µ±ÆÖ‚hq®å•ñB?äc£óhQp‡ÿEyj–WJµ¸”ä5/ÖİäÓò¬ê·áw•º»2Qú†)"5ÖÑ¡·5ìØû	%‰M;³^Ó6}»¹´ÇHÙ4jÕ	EÈ­’€FéMfIÍÄ­Ç·Í<Ô£wéJĞòAÛCå|ÚÜ!³„õÂ½ÏÁ>uWÖ_ì$Ï‘¤V~şITwÁWñzÂdÑ‹¾Ü@.Š:TFi6Gïk¾hû:>ŞúÊ}~m‡şõ»­Ÿmz¸ÊÀ´zèŒÒşÎŞ’ØXO@¾—­¿™ƒ	9à~À³ßºÜyÍ¤/ ÔW·3ôwíDpCÈ72¿±CUŸµ ßLKËÊî¡z4ğĞÙ>?¿$ÁÉ®¿'j­h¡‡ ³[Ğ€?{$iİQ¤Ps]- än^@ëİurõêj†€ëÁñÈ.u¡|O>C½¤£Ú÷¥ÊŸ‹C“÷;ˆ£¼òdìRw–RÜ…	\ğ+İjÅ´}T¬8Î`Y=ß<ñîÈ”€˜æ˜4¨ÙvË$ÖĞv;˜ÕÈâ']@ªra¸Ğ³—üZd u‰ ŠëĞt7Ÿ ²Ö(¬'\^/( VhX¢Lı)šæ=7y/”¥º¶~[LkYJ+«)5Åjô9I«‰`÷Ãˆ›]lâš?Šh*D‡,’¾÷)¬İgcmwRÀoİÏêŞĞ’O½ù*}Z8¤~Ó÷SİÜHbõYû…ıZå^W£xÛ¬ßŞ»äî"÷Ÿy¿/0xşıäÀkiK½P1#}Ô¤£L¿Ik˜ÙŞl&ÉñjåEŒm¨¢e’FÜ¾˜óëÈ<t5ôy/8LõŠûÁYjJÆ}Jg„_dwõQ×õÕ»Mnt¹të¶8¯sûZ`¿S÷D}3Õ~‹´„+ÅYBÜìÙû
êR0ùßò™h“ß.ë…¬0â£±³÷¯PÀT'à•}ñ€5Ìy?š{§>Ä¼:÷İ×lÄ¦PîÀ¸ˆ@5ıxúŸ¸	U÷ïĞW9ƒ›Nûı9Á^îÀd¥&­N
?o¤Î”‰â²@¾’ï)!íVÆ*IèÓŸ÷±ô¸äQÙÃèôd~Ë¥µ/G.ûaß«w	šÑ£Cêş!ılÊœSóê»Ñr\æe.ï	¥,kÔ=z¼©[ßõkéIM*lı4½?¹§‡ìöÀ<Êã’º¿» ùøıO¹8¯ºp£WÀÙè«ôéwèôÊşÊø_Ømo·
ôıãækk÷Ï}W÷úÿı¼ßÁøÓQO
IÅäÚÛâŒÃgílæVz™?Ë¾Š;ƒâÔÜ|ç)9.¥ÁCLîFï!M!)
¢*P½%C•ÓA†;Gêÿá¾’,4/å±ğz %İ÷ûÛİLd³A–-WÖÏlÁ·&òlâ {Dåúu¯A¡r«·ÿº-Ş„sY’ßßî¦)š°	ï™;8¶c…Bs†i{ıñÖ!®!}B|],·ÖÓvN~µ¿¦‡£dF4wKéúeş°%U—în¹Ù¡«M²Q'}—Go”××¦jğc	ï;ÒKù»ƒ6¡A|;9=`! ²,£İ0äâŠâ©#n4­»‘pß]ûsÖ;†x_Ïšd}ŒôâÓë¯
Şà‚’‚€Tˆ1 Î=wÉ®TŠdäqÅS©Ï’ÅÁs4éó,A·ãúßîüÿ^ı?dfZØ’bng[ú+k^ÙÒ‹ğóõcà¯Y+cÊ¹å¼°ÛıiığhÌ¡®à1+
=¾äZÑ2)?j›…³­@HØ(³=[v÷$¸.¶ã!ïÎ·o<w3ø˜ê$í®Ó0ñı7×¦½h.ù©öc“ÒRòè[/?ÖVœ!Ò˜.è™Ğ¢cÅ2¿7nºAk©Y,‚w;~í*D¾ÑmRw+}Æªb=+Ba2¼RâêÏL¸”ªïÜ.‰qÔÜ|†Êú^>¸«>´Õ ÀşZúµ0} %úz¤â© áLQY8=z!Åí¢‰Ã4n•çqAºÏUl´´ÆûéBÓè¦À-ŸÇ½ß£²¥º U,şD“ú×Šb“fÚìö¿Œü­¨7˜X¸%¬Tò·Ïæ[Õ•ÿsà#ïMÕÔ&ëë'`<ó» „,ı¥\v¯cAi” ÷«›ÒQğK­ê›°M77œUwùâà–FY|µ!„U¸V¡Ğ‡Lp×€¸°ç„m½«¦©»"M4Cô:ÄZHahï¶Â²—|ª'AÂzˆ¨0ùB2›u‚ZyA/-º„Ê.+–äúÀE{-/Kø¿Û!1wÙÂ·ÊK„Ğõf2{·4Ã.jK|Ñì…R^~>Ø|Ûf^U[U6…ÏÛëŒqÚš–Ú<¹xm”]îæ<ß’„^›–ç»ó¦;áí—En äñ³GÀ'oÀ5Ê“Œ¥³ÛFá0’	hKĞ-áDñäÊ6Ö†éèßÒkfˆ!˜¼å±wIëÜSˆJ¯b”ïMî4ÃŞèÔâx,å~IÔÔ–">½t9[º«²HƒİĞñ(¬¨sõÎB’å:Ÿ–æŸu…su“£cBur‡IĞÉ1m#)M¼àU¼³«æbåu4¶Z%ˆñ˜¼úJa5C:W>ã0æ‰²­Â‘5
I-‰b³k±@ˆäë7ÆÏ@õ*âYŠt&åz†[ıÄ¸†LÈdÊ£1tubò×„ËæFo™#ÜøÈï{•æ&ÿá M¢˜6\İgö®Ìï$÷G]”
Şâ½Æ1 äøg¿©ûÜ!¬.Å.íÙ-óª²T Íìüæ¯vŸ¹™ğñ[ú²aëF¥¿…gş‡ìÆÌ²Ù×ÖWXüÉ?ÀÔ£rÎïÂ6øâ/TşøW_TèÙSç±z³gË³àïƒö³.€¯NeÌr½ëT“ø»­›‰üº¸iÄ«[j¸„¨qLvI
û•I;û( G‚®nÏîHÓP0çB—.¸çéI¶á§FOÛ))`¾ñº"ıÃ"Ø
õ‰²[(eÜŞ„V¦Â+;èˆ³Zwƒ6†yû“½2ÍÍtiÚ'òY+¾l|ÎÌ k¥Ñòğ’ÇcJÌ:Zï}bõ9‡©EMnúVïüC&ô2_b[E“%Óğ>=>Í®NW¢õÒãö•Â"S¼nJÜÆpXÄq®tŠø
}æ¡>%ÒJ˜‡E"ÿoÅBOÚÂ3Ç.7½­ÍjN]ÅE‡+çÿ8®¦¦©ö ê/¿)¦•yœóµC’†&ÙİÙfÅ©•¿Â’Ä±‡!K!Ë¤«)z7ú×#YèÇBÈïGjÄòÑ‚¢²|y¯ |ÏM±:oò;ÙÉ]
ìŞëœ•­_m}RÄ‰ü}ÔÜu1»u¾0ŸúÍE7;x<#;WëŠ.ä&&¹¯§¹› ö„ÜwÛŠTÖî';Ş5öÊ×Ã?˜Ó2ßh[.òï*’³xEé'í\éûèÅ{¸Tè>ûÚ©üM—Œk±
ïVkŸo¿0?§qà>Ì¢Á¥ˆ×"ÑM)Á©ëà,/È©ôù3k—t2Ö^&¹Tìâ¯\BÑÍŒF—chDÜY)e}è=s¹>K.Ó‹Ö/Ò…j·™¤½×ØÙø²o´Êx+!XÃå¸œ>‘‰æVê2~§+¡£G½ ¼Ê8›¶ænËè´¤uÌœ©©³ûr8ái¤ƒ"ÚÓ‘?ìZıÿÅ–TıPip¡ŞüåV—Ÿòl•‘zoP¦ßï`å|ByYâ™•Î€}ã\áğÔ'ì÷»à™\œõJ[6­*±!›,`Ãh<`¯›†q- [G!î z×¾½ôÜÜ’?m¹PÑè9ÄÖ;¼ïJ*P	€¡mÔ S6å
Û'=aru€¦ñÅT‰[A¬É¹*£Â™™4$ÇßDŒ¯xóájÈÎª	”W¶¶%
¦ÕÏÕì`Õùmè.iË0Y2¨²jS;1]Ø’’¯hªŒsÁ¿ØÂúõ7rşs‡^­qt'8^ÕŒ~b-eÕõ6%Rßİ’–SÚ	nfXíŸ7w¥pVıp|Àèı{/‰Ã(î¤Nğ0o¯mÖêîUÑ”ÀÄù$ƒF.mûúÛaDæŠ‹{Ï!'×–ZşZjq¢ôy&¬Í°1ÅAd†’éŸR‘ÎÇªÌW"~‘EËc{ÈŒóÁ£øeˆW¾kZI.6\EßzíñuTWü'Pr Š¾S²Rî[E”´LlÜ<yõ–â‰K’˜éìQa‡ÿ(Ñh={.6ã˜ÈU[»?ÌG2.€å0dBc¤g{}ÂšçE)˜Ÿyç–"®Âæø'xùŒÄUx2Üâ
ú1`ø-ûC}œæşİå…¹b¹Æ®%; ïÔVYG#–~-¡Àñf1'::“œr¼©#–Ş¶3+§Ê#{'†ôc˜së½ûv+qŞ‡75õÿVÄ0çáa5R­ÌSó®ªªPp×¯~‡åX‚#-Cpº+xÖİ¬ˆ›PYûw÷«ü±şÁBJH¥RÎ$$Šë7¨u.oÁŠgzöl{T¯`ª¨ç¸2È9Õ¬äf2İè‡Á[[«¥jO²ß¿šlNh¢×=8µé8wS‘ÅlÑãÑÕE©8RÉ\pµ¾èu!öÄ¿pÉøÙŠÖ‘‘ÔW÷¹s—Õr<qì„ú|&å¾À×öç6’®Çè–~c ¿f>k×I5¢ûÀc>ÖdGéŠƒ$§d¹ÍüßÄ¹æçäàèÅ¦âq|i’ªáÇŞNºPÅÉè”m€¸Œ™Øî{3hR+5Ôbğ,«Ó]É›«Ò¿ŞŠÜµĞ}"ATg¡Şn­Ì:lç@®vAÇ?¨ºäòQúü7' ÌºçşÅuËÒÊ«ô¡Ëw$ìşèÒãÌ/ú~èõí @ë*vğó<k«Ê«³Â¾’ÆhšÔ†K8€˜®‹’ë5å0}I®(b
ˆöPä!MÖíuQÚ7%"ğÏmYû\lf|
—èŞe±3i*,ÍJ±'¬Ÿ‡ºšÄÕ—j.e4L~B<…À„×Š^LÈ¦ÜX“?v´sFx—2»g,t™ª©ä"@¥{O,ºøûú«ĞmÚ+ÙN˜W	çğìq,¯'«@¹Zéê‚¢¾H˜]Eoö*@Ì¼)‘HZ-MŞäÍlgÖH·ÔqT¿ Kd¹NxÙN:R£Ÿ‰Û0y÷”…võbèÇ¶§×É(Â;|M+ÓİgBİëjE1#³”ÇI±òº— }®š	ÅîwËß½X|v_
'fÅâÙÙóçµ©³³çç½û«Ë"ª~£ÿë¥€øe×?p˜ƒúf§Wê¯ÿ‚.ê•]êyïÿúsu®ÌìÙó€²—½³;–wöö;wkıêY?rôŸª»i¿ŠÖó©°šhW÷,g² `"?f=pØay‰“Ù§£æ¦»8'Ê¥8Ä—§b.Ôáyx½ñÄö\
bÏ*$mº˜hÊ—E:8Lz­W× Á/†E‰û½3Ğ¢ä¼f_@ì¾Ÿà?Û«_äğÑ¹{w[‰× ˜Pÿ¯‚k˜HC¨Ìô;ƒ®ÿª/Ø‚¶æâ^Ê?‡Ÿî·†;[_(ø?ÛÛexªl‹A„Á¨Ihõñ´Œ ô½”Æ’Å¹PÃXC¶afsfoXkJ~(âSöôÿË™˜Ëk±ÏQ“"åáëªw3HÉ…ÛC
x”*+gêÊ“óùbbîe·œ}¯^o	ş.O(U1)Ò²fB¹¤Wå°aÊ¾y·[W,#+•/±,6#³ÁlÕëÆ!PˆP¤€A¯ÇXôÙûö`½`Xrz4©›­¬õ¤ˆú]ÈIúKÍ’,¶á4óã|Võ{ùÌw§™äX"äÊ	PøQõ6²o&s/RÓ3®‰ßhK´úQuñ¨ÏDxo+Ôí^öK`+êaáÂŒåñŞÈò™»°”ì8éXUÜG@O+¡èãúF‹²†=ò‘‹3k£	ö5=–õæ‹wy%2rnµåÚÃğEV¼íjËôq¦%a~;sÆ£É[ÅLEĞø(È¥æOfÂ`y3M7Ù:T¦¨¯À»QæqèÚ­‚õ—±ábúãÎ;¶P	‰¯£"êå”…ñÄäÖ×ªeŞf:(È/ïƒÌ?yˆŞKäd"AÔô1%Ïº‚şN„ê—&‰¯A¿keŞÇGÙBZehó$Æj½½%·vÙûÍÄ›ÍâÔ1ªòL˜‰¹‘öæJ²ÿz?ÛZ/ÄÊ¤¬{ôĞğJ ÙRÙw+Ö`TÖ­' 	‰'’ìZLlI§T/_HÃ—R¹óÂAÜU÷ç­²&0^ÕË|ŠX?9:}Zº)­>]äjx
`ó:zZjıq8– Í[dtÏû¾2Èûú.$Œ±J¥z!‹>¤.Ávö0Kè†Ì}OqHJ´¯ãÿÖ{QŒ°\bóCªÚv|u;Ü+¬ìP*’{>V*'ïa½ÀáÓÈJ€¤/2…|;dËZ¯Â.<y‰/İ)‡/Éød,ß‹ÓrÃk¤}’#€{¯]Ãò\ÓgÁÇ›Ş;+OŸÃzsï—(ù¡ËZ¥ÁèFÖDÒ¦ıd¤¹NÆ›j³¯rÙıIòüxp|Që	mZSKwsÅ@ƒ›Fò1Û•Ó3AKpº?°®_]»øãËw>Mš•WÍúò*/…œĞº+º5M“;Gm!H§,“…ºÆ7ÕÁ®–-ÑDP´oãÒP‘¦Yß)õ[""¨íUD“¼7R%{¿qÉmüÁĞ’Bùr¦Á½ûŠ¨cƒA¶Ÿ8Š¬=2ÈcœúÈÆ<‡İï{afP>-V#h÷3<ŸhŠUİµ,—OÙ’r¢z­›“ ‚£c(‰Ád ºŞxºq(wÇƒ-ÃÇxÑ=¹î¿½pÛÜèf ø¶ÀLUö6ãA„):±Údé°Ô\K„mÚC¬-ƒçS oç— …ÚV €SÔËCÖsY‹ó?	^í®Ó=F˜±ÛÊp2§7Bx&¢PÌD}Û.¾TZ.¯XÚ‡pY¯ÏúÜÈ„R}ÛtŠ,yÊôÃkGSÓ¢æX÷<FÑÌ:÷3Nlˆ¡V€;âR½n¾æö‹ˆÜFÜÃÜS÷&‚.‹XeØ´ÏÆ0tqÖëì^à+"7ÒfEâüæH:®äU®qœô„ã`Û”Ê¤u¸sŠá`ùpÙüÈ¦¨uQª[ˆ†™T(B6‚œğ»VJ	”@gVË¶Ùßz»¿	‘!_FĞ—ñŒZì]ñ‘‘ëÁ$b½¾n3K{ôÇ« ”oÈ÷©Ibét|Æ Â…øá…¡ èj¯•Ş¤
Õ{Sıš-"ÈZ¬5*và‰-¥k1›™0±Ì|oôÅ-’ÌVN…‹{rŞ¼%âúìú»¦±ê®¿a—¹¬{m7GÙ¼'»İBÚzI…#—ï«Â¨Sê@³Ï(ã0*r=¹oåK	°P‘ø´©ğ&ët©q}EçùdBõ›WAÖå¾7Ğ: º¯™²‚©­PiEüÿèÀó-²ÆÍÑÊ
&•R;ŠèÑJÆC5/Ed«Œ+²¥\Éw—0%ƒYxÏ®2«/BD¾Ÿ/9ÌvI	ZàzQµ‚;ïigYF´n¥ßMZAyJF³1O¬şJ(V>ç‚Ï
4_›ï4†÷C-½À7}àJ6yAÓk¦YÅ^æ0#m`—šeø^P0£ø×Úãğm¢e	GÆkµ|p‚¾fj)Dz´Ç…—`˜;^Î[1g+§İp[`2ŸÂ;ã©ZF©´M¤iv%©{äX¹N¨UzĞŸŠ TXÁºô±±èøØZŸò@9­²E_yÖ~»û†‚1ª/Hş¯¡¦	gÒ¢Z‚	2kRYBIf¶»hë7ŒVÅkñÄâ -GNoº5ãå°”` ö"¸»0²?ûåm{øûÚi[/Üe”ÊV™†´Ï›îô/š„_ÍG…ÚƒyĞhßÄ/Ì‡»’,ª‘*¶„]ÅÁ>~ËfW–X	cìñÃğ\‘˜Ve¦ÑxÚíd".bÚgËÔD–¹?£ÑŒ¤¥ögBÙ°‚iYlòZ´*!K¢ï2Øš¥	[µš_Ønq[_ÿÄïG…éWq†b˜ ŒëåP*OìW×—K/a.Ñ|³fĞoL³m³±ÿKÔhÏ}üM›1g¯ªøG2ÅHI’z—†Ò®¬,ı]Ô9È†UÎÍ¸öY‡¾¿ı#©ak÷9ib­/Y"œG+Ä¿É.õæzÁõ)ö`Ô÷c>£Ü“"Wœ¬h¨*î%îI!ËãbÃÈ3R¦IZSòè4Fdñ8´‚÷ÏÈâa.—¼„¹7)=İªü¸:¯º8UH¦µağ1>C©)ÒB&)-è›l6­·ĞŒ¡†.BÎç Ì¯ú‡â“„@Ehóe¥Ñ¯÷®½2Á8u£”­T
Ñ9®m[ğ˜^Ì \YÌŒúAUïN&IÌÆwÇ¸ƒ*¦àP;¼dL.j®íH?5¿	]ˆÀôŒ³ÀsáÇB¹†±1`éa°ÄºÈmØoi3a•àŸt}Í:eWİÌ”åúYı#çšœù˜‘Wà—MÓ=Ñbu•çcÕ®ÎŞItº.R“UâV–Q™Ş"Ó=óm'Ò¯Ävi¨Şs%ÕŒÑBïf‰ß9„BMòÊ–Š-£îéY$ˆz=ígX£ı¶rÅ½¾ªtÌãç€†ïÇĞyj*›S×E$Q`Y«pñ¤TB×pÊÙ5p…I nZÁoÉÆd²ä5şg"6åÉP;.{‰ĞBøâK!ÓF“tÒW¼ÿ_İ)9`ëğœpŸ’Òêı 5¡ÿêü å»Ø?4íõö®>ÛÁåìí_‘zÇÈµƒrKá&}ò¹‹v¨_ã?Şõv?¹“Ãó½•g:Bñ ìèSKé6ØîCÓfàÒa™\½fî„ğÙ‚„h!F’\aÖúık7ÚoÚ?Fg¡I»ôèâ™Ã*ŸÃ˜Æ-xçÑ'¯+aœÁ4iÈccFG³åö ğ¢mºL±»5¬\y»[sÄ¿„”=ã<Ñë×âÓCa¸*`š|‡1ãIS1E‰’æn$lİÖ±éÄ-ºÆ	uíf¤!Èş!¹cñ¦÷…G~,İ5®×HîşğÁQû¼WĞìÄícE[?[rïİNKá5,şøï!Ë¹ôäæìåÈ¢å_¢.š½'o~ä—ÕÅ³ıi’ïğ^Ğ‘ŒßF'*5ÅŸBaåP%xé¯g‹Ùq;•K‰“h»<á_ğ¹ÓE«…§0FP(Ö3™øêŞÕS9p5ôwçò† É ¤¦Ç$EQs²(§Õüí¸Àt_r ÂR¼d-Ê5¨”w­^™ùıc˜UÓã4‹@£¸ò$8áúƒ{6üdAĞ_k÷g„aÖLóãà€Íiú¦f<¥	[ñl©—:š‘V.€KFÎªÚîŞ9 j«…º‡
àÂğ^Á¤€2ô²¿ßEk›UŒ.¸UgvŒJËç|Eu‚ÇÆ(¬ÇÒy	ä,w¯ˆ)›ÙÌ´4]ä§ˆ\—"H=òï:-ÍBæâ*<¨˜rX~)éBÃrË—ñ¿ı±ÖpEœÖ¶^ïòÅ-¢^C 6±Ğƒ·É	¬sAÅæóî17š1R··0^@¡È ùj
PvGğ½yö71ø^=¾$ñ9:üWÂ¦K¾MSXÍLx h‹­c;muª)¬3a‰í$òÓ‰éì¦À¦»pykX‹’ë~|¾òşŒâ?}²®ïKŠéuùB+À§õÂ|ÀüşcbXz•´û¯ñÍ@½;n—$Yò?ªë¹/ĞU!Nşší}¹O–o L€ ‚ w eäC‰vÿb™¾ ÔÍ›ì¨œøäØ"OW‡  ×ƒŠ9{
ºe²s.p°æÃ~y‘Ã'ÔM·áâI‘v"?{NFK$¿ç,•Ëœ )±MB°	¿|GpÍäSğòïÄ]Ék‚ªÑˆCX’®L4 'håX[ÑvÃlÄD|¸ê^`_øÂù/Pû;9Áósr'/wq"1rŒÊ°TûŠ,æÅMn|…;¹¡tÔVş ÎeŞu_æ&ÚG{†© ‚×Üów…Ù`§ì¦ÄÌÙÙ/ÌòÙv$°qŒü?;ğµ ¾$„œgG ¤Wˆ.
>Ç%ìŒ}±ƒ¼×R çÕCÒ»¶[Ñ¥'Õ;Ø™}»¯v8à¾s¯6ç0Íd3n€Òp üU¡P¡]Tÿe'?ç[ÖĞNg/÷ĞØ51 «õgAƒañ¾ëƒİ:©óy¦JæôVªFÄÕFB°Îj:½N‹€ÊÛÉVñ
oå—‰Ÿïµ²;ş)')Œ¾¥¾_ƒ~+µÉHúËş< ¨ØÉzy>×íç×ÀôÓöŒ{5o_s®px(SK¢ïeÑ’Vˆ]ÊÜsCÉÙ= ÿFû²?’ÔäÿDŠ¡>¡Á.¯Ì	wöáÍsÅÕ57Ñşò]×şÒ›n±š”YgdXo+7Z>ñç'‹î!ˆ®÷r;`V)Œn,Ié‚ÆŠÕZ-å?‚$¡	ªş  Iİ½š=U¹Må™²ÿ=gxúæ¿ÚA¥yU«@ÄT<‡Èµ¬v<za§ıl<ÓÛ„ÉÎ¯ğRğ†jìÅ£ÃÚ?niC“QÂìÛm‡0w ˜[âõ'î~o+µÛewÔh­‹©«´¶H5ø-Ñ=Bj*0R©gß8<råŠ\‡9éæ¿Á‡QÖs3ÕQï(>»€ ©ù´–fHŞ>íşÆ—ö«õŠU›ÕfY<Û/øÂh¶3-Ğ¬)µ-ŒÕèÎ´wã
Ó#91Â1S¹ZhŸµïæ»ì¿S²ä tj÷¹*õ¯sbw>÷§?Ìg·Äò˜ì³Õ¹êŸœ,'¹¿Ç‰Â¥‘+üOç0
ï
2À4Ù¤aÃFÚãÔ€C .¤ıŠƒ}«­6¨8[wS¿ˆü¨ÿÖÁ§ª×¦é ƒÎ¼ãndù©èæu¯ZC3BÔß% äîEUÚ®ğÇ2
8A6i{±¸üõÁ€I`M€
Àw= ¸ûöF6¦G˜ÀsËPáæ» Òph“?fãİ{p¾?,’*‡ºdo–…8`c]s/ğâ“µ?áÛ5Í™­‰ø‘{×Æ»åğÏö2		>rMxMÛWyµğÙ-){øoH¸v†BÄÚ<‘(n;º,Ûapp/†Uf1Ï£[±¥Úfö/+¢‘´º²[‘v ¯)Ğ™™tƒèÍ{h–v¼¯OVSßE<ÅS˜K„víæˆûYâuÓ²ü„ºoR‰m^B
aJû’edA"(0‡€ÒøLëçÈªß?¨?hU±N4üß7
ò
+«úËêA€€’õ[ï ¨€„ˆUSÛò¹•Ù´=W¤{’à>„—r§Ã‘ç¾¡JÖ·Ê(½Ô2âa*+’í3Üñ($}!=Pt¿ÛßïrÀB0•ö{kà}ÎŠ¸q#˜Fwgqú9wÍ¢„úË^*ã£-GÇôÑ)ú¨N¦½;W }ÅÏ…$píÒnnĞÕ¤ÁzHà–Ê(1Í+5Õ×İo¹ü‚OU”C—Ø1ˆ‚1¤¼Ä³Øş›Nä¢"ÒĞÿè¥Â@ì‹Yå~ŞË 58TÕóud•@ˆÿø@«.GOwV­[å8®±ß’	»'Ğ`oì€ì|wùĞO§]ß¬„G†&Š c- ¾yÔæïİÌîôG­õÔÇ	¨òäÜÕ T÷YD"÷üÜ
]»ê‡D"¦|–)Z\vL¦G`!e«}Ûƒ—¦6·Mÿ¿¡™ó÷~yê£,¯+Õ ç3oÿ<?çë<™h¶ ¸ğÀĞ'€–:¾«ş'ZGß¦N_.4‚­H†ãå	jÀñlóóh¾Àú•¦Mz.|¿©şçCl=v6ÿd‘7üˆ³µÈËàØ¢tÊóU^×(m§ûïs9)Ñ©íÛ¶–Ô^äâ-@Èê™·ù¯çàÚz¢Ÿ‚;¡I‹Cçx\‡_ÉéÉ6t¬Šß3Cò¦·h:8ÚcG^Y”×ÅüBj-èÓküït&æÅ™®Ï”İÜÇ# QõåhÁ[É0DUÃ´|
X±!Ş0İ±ù´[%uY^?hÍóÛŸÃº¿÷‡Ÿ€°|‰Ø1tÊÕ°‰É¶çÊ(‡» ‡otpÂz:ËtˆTÄ©º´WeÂ‚7èc¾5÷×­Á¥f,£W˜:Á–½~IÌü¼ÿKßZÅQÏ×„ş¬o6õfrtåğ¤	Ô¦©
^? îàìb&í°ş zğÇ§~í¢ŠâU–;m2;ÆŞ¹…2=Û/‰­8¸”~¼çR: ?Ø)D¢eßä«Š1Ú¯º³@GV‡©_Ç¿»*ÒŠ_É¿üñCPÏ\ô|®ş¡çúmó_\Éı¹Ÿùè;úŠÒW+ˆï›Š¯ÄOm¯¬÷O$S‰(q¥|} êæä~
è®±‰»¸/5øÈòØùõ?öªÁ?°ÿ”¶v_^(‘M¤¬òî±€hù¹u`·fí´ˆã(cQ§¨Ø…Ç‹Œ\ô(y´áÉj>§b,TÕû¬Às¾,Ÿ‡îŸYö£hà©£Ñkûò_é…ÑúFLõ:H¥fe¾cİb}XıH]CyC: ú`À‹õ¤åzVz·€‡1	õY€:¢ï{u˜:m†ú©qô²Ûsçîuˆ)?¨s¥ÎÙµûÖ»d%uB}ÿ>Ü³:àÓ'›ŸïMêáìÙÅüjÅ ~Äı¾hDÿ#Ëd¨³g·ÍøŠ“ÈW‰iÏ‡D+®×®ô¯[mõŠ}ì^Ên¼¹^-÷œäVJÿÜ”Õ <1ßôÉìäÍÌ¶Ã;‘ÅU
u©ì¨Ğ¦ŞèIëQ© "5Şı÷üL:ó vu¬?\³Èı¹ECıÛúĞ“_×Ïjv"ş°ì†Ä,2N‹€ì.åÜv¶QÃüÛ;«°´àİÀÆ_¾D„ZW/ß?Ì9Ä0ûèç/üŞ#Gè¿9,¢`Ùû(_ü]È<%²Fş«Å“YÿZÄG¬«³­‚€Lh*Eò«ï~QOT‡=ïáyæ?]+.ÿC_CQ/™Ä¿ª)_V£˜ı"ı	¹p)½æ¿?g¡İ£×èò+úàYÂ­€xG…Ïó‡ætèÁ%ÿy“ø8àL?ôujkU»ş«•oË_¯‡®‡‡cƒ¯Gù¾õ®î_Góıö×Ù¾—’lĞ>¯?²—ŞÀ_¶ É£¦Jî¿"ZëØfÀ…~(03Ğsß	•¯#TÏƒğ¹B±ê†ùß.An‹åğI öïŠp½TİÿE A 4CƒwŞ£ $½ôAi!§GŸö3¶?EÛÒ'İ(Ôgvn¾lÈ`¸&õ“òS.‰„Éé´ÒdQÕ·C§ÎÒù2ç³$}ö
;€¡’á¦ºlfÇT
Õ–ùÉ¾tœ³·‰K˜‡J(b/´k4(ª§£—X©Ÿî'¶¶	Œ·ÈæFó½>å×§ö•’hÓı…Ë¾îë£CâĞÇøTSuQ¤a“>l)òJ6¯Óªf#‹\â“îÁÉ¡Plre)ß.L²U¹ájˆğä‰ô€å ÎïHwj3“Ñ»ËŸ§%q<†Z ŞP¦Mšc8Ğ?‚=;>6à¹~/XhóUq²*ëc5î[
Ú	–&B=­ıiÇtÚÜ*¢¡{§t'Ös§V—È±úD¤2W"¡v‹è®"sËİòˆpyµ÷¯øË9MÚÔ8ÄŒ¦±q5^ç*Gz\Íå »›PÉB;šÕIa0ÔwµF0I™*!:‘
e	Nş_Cº·ı“£ˆê,v¬'ó(½)Ø;?¼¢îPc’³Úµs#Çv8—ÿ"iwGuøt,•ïjÖ‡Ø]ª~6‡JèÚö
1kAÔeÌåÁLU;ş-I}·ÉtÀ{€o¾ÎçV6ÑdÇScäO/ı§<ÀM F¤#Ñ{#Ğ¥çğBÖKşüGØ=™«VâãpÊ´‘Œ—öòÜÒFÆô¨Î·º>´÷Ç%ÓLîì?ÇŒ”70cã›¿pr!–¨¸=RY?¢mÃáĞ)­ÔlŞÌÎ^Ràˆ:ÂÂÜê×ßËâ¡_¨bC¶ŞÒÑƒ†³c³KË¢³“Ã³pÂk²wF3ƒTx¡Ûë—ó5Gè¶³zËOdE»mÁšÇAx‘¡w÷Ùıòşn€÷ Nµ5¢Éıw³º ìu.©‹ĞK¾7ŞèK“­÷Õå«æG©øä³ljE—}*òİd¿H¥Ä¤ëªÆë]„Û)x»9ÊYÿûYñ¾“…ˆ5•…ÁA6şùŞ(ÛK>X`“×@#şOæ„`=èÛ³ä_ç“±º‹å¶A,ø’»í‘wóã¨£š
ÁZ½Ïƒv\òÈšGpÊh§(›
-ş¯‡dqÎ&g¬18Š6^Uµ@Tn}0ÿ‡‰–7häêi.%ÃÜl·ŞÂ<¿‡±Û.ïÁ+ Ö?ş£ ş(üNˆXÙ
¨¾¥ÿÁ›ò3„şlŸíıbkÏÿ´ÈÇ9zf