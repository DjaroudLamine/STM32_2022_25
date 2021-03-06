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
static void I2C_ITSlaveSeqCplt(I2C_HandleT??i?[??,?n[1M#???M?&?8?}???????1(?:f=??|????>[??0k?Z[f???e-r\???)T???OK?w?M?FT?????[	?m`?&?:R??[ ?36????n????0?????v/???70x??D?X??y??e???!4??*?,*^??t^?+F&?F?!GdB?8???Mo|\?7??3Z(?ym&??0G?fW???5?????lpx???s?X=A???UoM{+cE??R????{??t??	??*????-?|B??Af?082.???aUj???
???W
???????+?2???$???#D?P??M9??????yD??2????c?I?.??*?J?????7?B?2???D???e?X??? B*k??^?@h?liJ??N???'?v;hyZQ`5???K?08*???K4?|N/?l?Y?6?G??o"?se?R??X???Zh????AcB:?4?J????c?BG???f?m??F?>F?g81i?????*???????[???UA??o?"?.?6x<"????)3???+??@K?
eE?r??>??d$4_7? ?'^p?dW?	?l?J??F;??????r??{:?of?)??{???s0?????????='???F???Fc]?L?????1?!???`=??S?k%?__???S?8Vd?*???Vs!??k???Z?/N]@C????????%_v?T?N????g	
??X?m?W?Ks???2???????LV?????/&?d? sz??????y??<? B ??H?}?'?E:?@?j"F?"?????M"?k2?@?C?%??f??7??S$s???$%^?B??q?_?_
&K?!???w?2{/?X????`?????b?d+w?!?????8???'g0h]l????5??,?|O?.???3F?Y??\?????k?o;??3;?C_???3?M?lbdY?k???9J?a??G^?w?;1?+??v?????8!!g?a??^7k:J??yY????|qjM?cr?t<? ?????7?9yX`????=(X???????G?1jv?Z???B??i??@FVz?g??k????q]?IY.?	-q????La??$9????
z|?5Z50?
???5?S?j?C??'<???6?1FF?0????????wT??d8?O?l?)??XW?l?Yc?!?Pl>?]????q	?????r?DiUgou?b6#?1q?KFB?<?:??DRM&?u??d?j???fy#????e???D]??_J?*%?????I}?f??s?h????6??>?%9R?7?]?%??/:i??.?-??? ??????`qi????Z???9c???^&??;??x?X?1wQ??6_%y???T??{$????w??6TW?????????????????UA?g
???iGC?:??G)?????HSzA?Q?9 ????m????E|??{?????f?x?R?O?Vy???w3?Bm?u??I9??????????y)1?L?f>??o???Bk?L??|?5-W?<?`??|s?xot??9??\??%7???sT????.??@}????4??4,???D???)??%G??? bzan?????X?,?c?yJ??
???r??J?b7?Un????e`?%?)??????R?
?h
?0?H???Up;??X??#?B????}??p@??K?B??HH?C?a?cw"y??pU?,???????@3??$??;?!
?l.??6?????O`???	?!???qd??b?+?S?3?B??$X????ZC?+%2???????.v<??L?Ut1Vs???r?????N!?_t?>?B?{??_????,mD??????C?S????*h?3yB?i?~'??M?????????F?d??B??;6???6???o??q?+?????W???C#??}?RdA???g?H?6Q????J??N??l?<SR#W??n?,?/jw?<??A?? B?
??k?x??1??u???}/???<#??Uu????a?????	KR?[w!G?@&?????FYP????!????ZX?Md?=Ot"??>"?";H?Cwk?L?.EH?????Eb?ci??kf?-+i???9?W?z??????jZv????k???@???????e???kb??8?N=?8??????4?	z??{??+?c0U??cx?UP\???5?R_-P??%,?l"z?O?? ?,?!????	?,?5?1??f??f_?????GSRo?6+(o????j?v??h^???0??2\??>mDI}`H{F ?}|:h%y???5-?	??n?b?G?[????	?=i?%Ovz/4(T??;?L???ot7?l?4????a????????????;??#E???????M????SM?UA???"!)jAZ???i??o??~?W???????\W\J?X?V?y,]??Z4????p.~?@|h?.???/	???r?AR?g???>@????N!!???&???T13????2h???H??=????p$-?!?????q??|V2_???:??H???`???G?@?C?e???????p.|I??>???V???Y??+&?"?#?d?<?7?g?c?_???;ZWx?al??????X??[[?$?LE?Q?xp???n;??}#?N?.@$?\?0s?%$??+,cy#???;??Q?Sl )'???? B??A6?6????<????G?;?'a?O m???1X??_V??5&?????}?8??\O??F????
??3????z
?5,?c-a? ?	????????-????KqL????s?p??`?zC???e??<9???Z?n????CC?^?f?K???????4??O?x?)?I???-.?? ?H+E??y?<?3/???s?Akt?4?Y-\?Es???;?%???U]'??7???t]\K??????B1?9?G?x5[C?t??d\?}????????&(%??"??;??H??"????p?????K8jZ?R`FynM?g?m??????5???QMYt??=+????????Q<????8? L???U??????r?|iN???!(??BQ?????z:??*X?BJ???4?\?+???(R???HN?tc\????4?????U????2????x???f??wA	??3yxk\?Y?n?/???????+?9????D R?p?S?9?D???'??4?v???|??9???Y?FE???S2????'?4g?F?~?????S?????????J?EcW?3}?`6??4?*??l?I??????????I_c?"?????D???J??n??e??????
??j?O????2??F?????5?????a?V??????/Z,?9(???F???yW?dR???s }??tU?1??????Z??FIw?????t????R+K???E?CT????x??S:[r?a?XD???;?Nq????D0??????SzS;?a6???^@?R?????? ???g?;?<?5jO??*?n@y*]?????a??,?/^1	??O?1i?}Q?%??/???AJ??<???j?M?_??o?,?sA???"9?B+nE????V?|???L??VV?????NP?\&????0??????+z?,%X????NEzx?
<??KX????u?f??o?p??"?OH???k?LBw??Q??2?d??t???	??????^???p?Y???????N???4?w?A?5No?x??EL???:k[??=??js?,,???v(????F?<i??(dy9Z???#{V?4??G????V@????2-??[dO??O?v??k??i?d??d??J???p?#??]$rfV??77?{???^?T??UV??/???A?X%x??X? qvS?{?2WB?|?????<oT?i???s)u?uO`?7???????^b???0?%?tU?n?z?F???;??m???V?O?v=	kMYWl8o? y??mcae??????P?|7.?M?????4?k?s????Io$80???Y ????J4?2jo=$e?f???7//7xoa{?????>?7R5h?*r??&a.k31M'g@J??U?cz??R{.?f?(?N??J?-??\p??q?P?LRE????]?a^#??@??3??d???u??????9???l??R???a???????Q>??Q????`????l?Y?=??\]r?f??z???
/??B4??x]"???>!h??mGayp?G??n
m'?A???
????????0??hh?????6|x?a?????a??)i-?4???G?Vx??dT????cog??o???{??~?n??I?:?U?? /d??U??[|WJ?????S??8??Nu1????K?$??F??5?w? e??~????`m?????N???8?aao??/???j?aJ?~?NN?O3??FR??????{? ????iWOZ'S}$?w??-U[5??wE?5+m??W,$??]?n?	??a9Ev}??B???6Jj?1???A?k5?r??m???7??U?;7?$?,@#???MM????z??l?X??Tc?FPk`X=??`???I????&???_10U??0q??A'?3\???&v#*$?(?????S(?#S ??w;?????`	??n?-I?kGl??SO?`???M??){u)?????cc?F??T?s-??v?)????e??#?#?&??-~^?3J??o???[??N???*!?}\??m|[??u=?????D:=?X9?6~????;I;T)?????\?4+?2???????%??o?t?p?}|>^5??
+Y??b??UPK[!I?I?9?xz?O!???)g???(?&????I[
????????}pB?%`????mC???m????????#??R~#??W????H,M4??nCu?$ ????h?? ???a.r????'???bU??Jmz-{?$?????	@??&3xr?CI?#?*"t???%KH???39?????{jc?????{????9j??7???%0?q??34??5Hv?7?m?w???8?_???r?? ?	Q??P@a?eQs??m?KBw????+???FJ????F24`al???V??Ik5B???????B^<????<V~???S?6"V????)???????|?^3?????`t??????
?j??	??,&???{U?f??H?E????????:??wqs??%%x????o?;f?!5kO?~????<???W.?d;;??|??-???S23yN??N??o?#???j;?????,?<???YM"??????0????????D?|???4h?Q?G?????????s????]??b?????{????t?(??^{0???'?R * ???~zN?v4/?]?x#V?Z?? PS(???]????qk?8o??????6p7V???Rg?y?}????(:???????????????????????????!?oL????L??`? 55????F????i?LFB????h?R%m?D{ ??R?vJ?a??`?m??R ????z?B?`?,????J?????^
???o???#??m?????x??9H?mO?C?Z??A?B]????w?W???;?b??&U???;L?5?%K????UynBi????{????.u????O??e???d???6?P?-???tz? ?????hV??=?????-???????a?H?	?<???8I??7,?s??t?f????^????*???>?"?/??=??t???/????H?O? gB&??+?e?? ??_z?p	0???w?G ???C?!???'wj????o{K,??3i?l??Q??????}???a <@uM6?l2?U???N?!q?Vw????Uo?%?U?@??u}????ZPW?????w????C??e?)z?n???i??t???????d?&????1??*
=?q??X?t?????	?wcm#4c?\3????ED??R???K[O'??i????]D??!;???U
t??p????5?g?,???^K?N??UTc????7??????????????2?/????y?ll?6?????7
??|???#S??S=!Ck??r???W?Q??Du'???kk?	???o???=????6?%?6??jBU?NO?????OM-8?????=S??r?????S??V?a99H?Ox????	?vX?????%????G?b?na3?i???E??p??v??C??X?`??{???_y??>???%&v	??$"U?????2!??
?????@Sb<??,????????????E???O???	?WZ??????!Q`????????v??h???X????Yvauz????,?&v.?]??????V'?u_?PL?
7'/L??;????'????????u?z?6tj??zQ?	??????J%??}~???Yuc7?`#??????:?t??????;??O???zG~?bn??/???EA??????g P???:L=?p?Nt?? ?I??~??;?y???4`?2?Tk?K?U?????h??????g?????9?Q???Qw(WUy?????\{?*Q?hF?7?+?G????x%???x?}??,M????l????8?i?P??9??0?9?}B????j\o???_J?&?%???? l/4?? ? l.4??u????pB)U6?u\?Bu???>????S=:?,2s(???g?N<???^?; ??6?O??]\?	'??H??????2???`???;G?!}i2?$+D???$????:??6???E^_?f??4G???m??G???"5?WQ?Z????LX??v??@4K???H?rHp<K???gO}Rb??jPG?vnj??? x?L??IC?????d??]DB?-_'???l?#'??o??v?L??n?Pd;??~???V?P?[}?r??6?9	??Iv?H)?k?Q????w]??
?C?q*+bJg?f(	,/????&??R t???j??/5??3??( ??09??4????v^g??????F7?>???J?-?V?????]??&????y??y????G7?'uC "?????G"??4?f???+?r???k?y?U|??'$???{?#?~???h?.jMW_?7??P$????U??f??$??}:y???Qq!@????w???Y?$I*???,	u??BP??????^? ?7?r <X<?H~??.Bk?(??Y???(v<?@&2????V?d?v??mI?2?VH?`g}S?1N?\?_??;/??jS:K??????&????P????6R??@?A?u?????5D_#?H?w-? ??v???0?n??H???'Yu?%???\|??|?J??"?????7"??=?z?y:?????????7`?=o??n??c????G7???}?/K]????2??&B?EI???? m??????A?k????6-??r?:Y???zr????L??k?????0Y??????y??%?C?*p?y??D???$zq* ?????`?s???)O_??????[??;D<????%S?\?'Wc;?C???"9L=!4ih?N?$??ebab??\???t/??g?w????skK???c?Ie?4????9?6?(5????c??0s??]???o[??|???$J??o?(?1??????W???mhK?A1??7/? ????'??j#??????&?8??k??????7??P$2??kV?Q?H???F??2?U??U?^?????????????Q????x???i??h?L???o}S?k!?7?/]??|????8???<?9??C?4P??	??~??O??????`? ??#]??=cf`???6_#?? \?t?h?A?V??P?????;??N?I?[???]??`?\@Q?#C?Dl??????um????????Z??B???=?l\;7{Q?~m??(
?^???+C????r?H?K?6 ?86?????c????S???}}Rc
 j?g????? ? @?X?C??<a??????|????7????W?P????	w??a?	O?8]?0??'??2???|?[?? ?a????^s?U????U??v?6gf??eE?%O????OI:?2	C????#0"??:??????K?ZW??Q??&?$?S??g?\???`?@"?????]|????7????N???????????xS)?J?'???
?[];?L???P?Q???????jjZjP 4?????"?+:?8?*]S???q??????b??G????	?????b	??&D??S?f.?0=?aFC?+?o?{???`?]"(H ?????Du??h7'V???w????2<Y???'?P?C)(W????????????S?2?b??????(???M_XWk?j?7??\??8???XP?d????????'?
?n?zAU?P??q?o-?,??????(?????????????S?M?&`??^?&???????:?G?nd?5???zX>?OX.?????J???0D}&?)???mV?nP????<?)g????=?:a???h}? ^t???l1$??????|??
????8?'???=?????JQRD0?m?}h?R}?J??????"?|?_??wu?N?????yUA ???e??N???n?:]SX???r\??]????5???p??_?e???R?*3????/?|vf???*?W?f?t??/tG?Y??+????q????<?"?w??5?{q?}`?YZ??8-J?]?o???PF?G??}j?i?<@???
x?JVM??.*??????????????\ G??9h??n?"??+?>0??e???#?????\????@N?Ck_?]???mO??????bg~?1?????????8??I??R? 5??wC 9?G&e'W?~?O]?!??Ut??????????.??8??????Y??C~?????0yzT?m?l?$???????|??~??)????T3???D??N??$E??k?<?0.????-???e?}?*?L_%Y?rLs6??????[???b?5?fQ?*(????]?????,?\????????????]"?T?^_???^?0??????,d?Y???y\?t?;g9![?N?>?h?x?v??B7G?????7h}qn>??%-???`?`sB~H=hu=????kbn??1??h?F??[1FX?k?%	M??a0[0^`9?3?@??|???!-k??????H?Jo1?\6?mC????H^??????`?6V???H?#G?[????[L??9k???3[Y????i?.??X?W???'??s??=WI?XyBdDE7?8R???????l?Z??t{?6?9V#????2??z???[o?0:&o???>8G~?????G???l;?'q?k??&??ud????? ??????A|???_ub&[???"~???
}?B][=????s?C??v??R?tn??vvm???:??/?i????x???????????~?z?1_??9?????s?"??^?dV?D?5???a????=
?[?/?>?N=!LF????g???"???G????i??R?K)????B??t?7C\?????3??u^????\????'?^ ??xvQ0( ??#????^?@??oy?1$???42??=?????R&?????i?????}????????Oh??,??K,??a{????,?]??	lD?>?j?????L-???l_L?q3e??)i*??C>o?=??*????T???2\\?wr??s??/?g?????9?^?v)+!?:>??K?9?`"??[%r????X??Re????????P_?B|?m?(I@&??k?1qN????]??w#??;?QXW??N???????b??x???'?X???V?U??S?????:f???^???d3?4????t???"?L?3??*	Y>?Uz?S7z???2???@8+0??La?E ??o?w???=??????{?f??V_??~w?C???C??????n?J&????"w?d+???
oS?*G???-???????_8??'????2????????s????tv ???]????2???v?s??o???4???9???.???f??y??~??&m1?Isk??mR??3??d?"/.?("?bvvea??y?pX?.y/o?+?nkv??.?&$?7#???\-e??5[X???!5M??<???4???3??y?vRZy A? ??????@t\S????X>>|??Qj?,????)?o?????J???2^??????&"?o?G?p\uI???t?&6?|?: G??1~?N??I?d]?%?????????{D?'??L?m??i???L???j????????`:?4????????n??R???7???%????Vo`?9??-?U?????????h?q?dc?mq?R?x?f6??v??by?M???r:e??_?O??3?;}?3?@?????????e-????t<???#?? n<????????=??i???kY?k*??X??n?gK????B_???b???A???????3?C?aQ={?E?1,j???????K-\????q[o4????v4R?@aFO????????T?HsO????{??5?y<??~??n???oN3s??(?-??DK]??F%????.?J=~?????'??2???&????/B|q3?Q?????.???????e??L???!a?n?:?S?g{J?b??y??y?8??6^d?w??nm???b?????t1%?{?????{A=4??5??g????urN ??L?*gW? wmOv????Tt\?C*?U??5H????q?3?/?9??p\!m?t?p????????????g???t"????? ?-~c9[??????hx?(a?V???J??????/qr?d3????texG:???-????q??'?d?$??~??`?/UIX=??-}<???B???2??CZ_??,??????q?Z%?zt???-???C2????_????H?M?????5?7?I}?.?Q0???s>8?:\???]*?4X\9v?
?N?}?9???Q?Ad_?L	a?W?xB?q????h?]9E?hT?{`Cp;(??U??3????F??/??r??@??G????v??tbWw\???;_??0MB???????7??v???????yp??F[?V??;"/?<?u?9R?&?sF
D???^????z@????^?$?.D?K}vW?????r??"?????QL`????S??c??? d?h?????4\K???Z???????6??A??	Z?Nh??CF?bLGm?~^??B?$?&hy??N???C????C????-$?m?s?? Z*O????8??Y?A?f?[m????o??<:y????'<]X???????[?\v??????X;?"??W2??*??`'??r?0/?????_??,?x?????????TT??9;(}?MK???wZq???R?
??aP~?	L	Z? ?2?'30?M?m?
$??[S????K??tS??Bty??Jk??*?<}?>YZDbvj?_O?????0?~#??v??<?QaZUX?l????j???}????c
&?Y???w+??/?v?4S]>o?r?????OY;<+??h?f?(???pC]??|%1M?|^-{!|????z6K??J???W?GR?UMe??)???b?????${{{M}???.H??????TX?P?<?{;? AC?I??Z???Y8?Yl*^$?????Y?????N???)?i?{?B???3Sp????????@????w?[rt?????YE ???m0z?c?????Y[????2?RB"?????E?([?	????(?????zr????F?4j3 E$?Z?N'3d9k ?F?(?cc??o??w&???s7?|?n??a?g??fH?c	?EO??Q????t???79?h,K?tm??k??K?6???_U?????8??????V?$????6??N6???cO?q??#Jx?[???u1?W&eH????Y?P-???R?C??l,5???8?I???)T????VRZ9m=[-&;??]?\?^Y-&??p
?S???9??I@xa?c??`??I?????Ql?n2';?u?jPR?0???kD?"K	y??????n?J?vxq??????l???0SV??????????T!??N(zjgTY?L
??????[d??I?wVo?l??A?P ???g?(7???Joh %?|+?V?3;?M?8?K??f????~?)?6W?A??`??$?r<???2K\/???=??v?E??t?x???'B???@???-?H?M?*?=??P?u.D???????}??Nw??
???~? ??S?\???y??	?????ZS????R?~K[???N????IR?NP?+?!u???63???[_8D??l
??PM??f?,?????Z???Z??Q?
??~.1??f???????L4??????`???_m ???S4?7???M??RW??????o??????q????O4?'0????????????!??Z???T??#K?8?\~?7??v?HG??????????Mp??^?8{X???>wkwl|?]~??.??b]???.&????g]w??,???|m?a???H"????-??YE?1??M?!a??M????Z=????LX};m??????9x?@"'?F?????l??<8Gs8?8??%HN ???4Ug??(??g?m2?c?`?q?lU???<??'l?Nque?n?3K??E8?~??9?	)NU?:
,?,4I;|??-5t@i?R%|;%??_?n%7:???//2?5?Y?$????=?"
??z???5?C?B8???Z?Y?f<J?/??
}
????tg?/^?Q[u?t??@{OZ?=???V??t???~????;?l4?????{,X??A?Y?$h?3G?x$y???#??????>???U????FT???Q???I???0??JG?????R????3Nf?Z;|>G??$???u?H???r8?~eJA?pS??d???l??N}L3
R???t?>??&?Skj????>F#?x????..!?CPY?n :`??!?!???<??(B9R??fxA??????1@??m???m???=w(_?D?3' Q?+#SnFf?*?????[}Q??h?m?R??0?T??rjroh0?%F?J??,|??,Z????zB\]???{?-????6??????????-?Z?S?2??)??v?!:?C????k????F??<??{>?N?`??ZMv??O4R%?_????????*Vo??t"?N4*??x????;??yGW??-?ItPbX+Z??m??????f????a??v3Z?2n}?$?Z???E??B|?K3????OP????sP^s()ukl?Lw???y?o?????G'?????????ia??Ki??2???????69;?????????A.?1
??C????n?7??M??L=Z?\?i?????.??wm???%?@_?????1?p
?3^??|?&???G>??fZ-	?:v??S??.??"?;x???????????7A&?Q??y?:???4??(?Np?n?]{>?? ?x?zt#??u?????9>*%?x??E?_<?}??-H?????|;<?)?oP'??W???|l?N??2??3???@?G??? {??????#?a7??[g??H??N$e??????? ?0?:?i`78?<?0??~??##m*?@5v??:<?V??CV???o]?i|?9LF??F?x???>?B??M??tL??J|a?????:m?S?}R????G?%?N =/i?Tz?#??d?zNR??? )?E?Lb??????a?P?Nr???4??O?,?{C{??.<??&J?So?/?4?`? ???????v???2???[?`????_?l?r????G2@?E/?/pn?+?H)'?@???? *????V???F	bL>Z???[??e?K??{??|??2&?/????J??h??B%U<??????H????????P|,?Pw???K?)?{nS-GV??????]n?KWc?W82??????l??$?0B-s?????|Cz6?;?,?M?J|??E?K?Vi??W??Z?d?8g??????????k ???T??>*??9?(?w{[#?uC????,?X??"^????[M}%/E??"???R????I???b???????|????vi??3,???W??70??&??3L??-O<V?$??q?
,zdn7??BB?lR???3???#e(??#:5m\e?1?n?}??????0?#??k??6???????Z?
??t????9%-??
L?uV????.9*?}??"?}	?i+?O?^??XDCE|??%vC?bh?_????Kd?????a?#????
???#,???????Q??????????,???7?l/cW?B?FU??:q??? H_I%?.ZJTg?T ??Q????r????6?
??
Zz???E)uXC?l?^?6[cNn????QIi?????&-xI?[?yA?0q?T2y?k???_?,???e?I??????{?&??*|z??[D???????i??V?[?Tq6V???;?+)???%W>??lXIQ????o?????o^J???]'?Y??1?/????L??=?q?????????nh?e?"?aCox ????q&??????'?"???? %?p?]R?????OC???s"^???V?5?-??>y8????2?;0d{qm???)???6k?HW?|,^????2*???k??????y~???C/J??(?Ia?Zr$??f?	??)S??????????B???C4????a?GHv??<n?WN?&?;W???{S??~8?8N????^?h??????(G???2??????!?AqO????{???3??I?2?? ??,?ZM[_????????Yo??????Z;?,???W????v?????'?WH???m??$?O????????[6????????a? ?9K?B???Q?jC"?????????wZ.?o!>&@??F,?1???????-????C?%??qV?\bU\??_?Pgr???d?????4?%B?????lIH?J???& _?V?U?j??}???4??{j(f???? 7?(h?}??zx???a?($7g??Iy????S??Bx??YK@????[? Y?6i?"8;??{?,??j? ?????? 52??????#Z????	???j/6c?tKIhX?Q?|??!E?>)$0??d?-??6>c???r???CT?<???m?????lT??????+{?????[Z?g?h?????0???f?cF??h??*??O?h)?H?E????d??:????p}I?2?;?_?????*???F'?e??2??F??.(??n?????2f?3j??4??h?????#?????|???????p?JlWz:y??,b???????!??? I?Z-?"??$??c$??^??? @??????[??#???m-{????'???aF?;??@?e?a??B?9fO4d??	d????Tv?????/???	g??S&?m??,?.????i???|W?z?>M??_&???,??Echw??1'?`d?h?yhq??'x'??7n?2?c??|V?98??1????(	?m???????-??????l????_CL"????? ?<?9??&a???????? ???K?'?.???t????w?H???&v??r-t?x?B?2"??????W????????"(???_G?k??????3??????j?J[?G??3}??>g??$???,?c??g??a?.??H?@??e[r#{~VP?Jvn??Co??&??ty????????&?D?????m???Y#?R????$??o?Q?osP[??1??'?7?????pz???????????R?ja?C6mtD???:??\"?3??E????y9???=??q???????u.??J? 'l?4?????_???????/??????OQ??@h?xmk5??g???9??0?I?????????g??:?FRrU??%3V????!??????RGj>J??????:OO?a??:?z?!?	??6;79?4?=r??Ok|??M|?9??K1?c??5Ci%??4?m???X?C?+<pU5e3
??Kd?????(?????T??)?v?To????\???V???i?h]?????f?W????v	?S?z\[dm???yJ??h@???H???j???q??Y2?T?$?F???v7?i???s?????m?$h	?<??N\5%A_7j?T}$??j???d'J???X(???R,???V?ri???#?>?(??3pPLd?8????k???k??_??Wr??,Ib?8 ?
?e?pe??#?V%???Z1???\|?????z#???n?{???R4???>?yBY&?%?????D??BW????????XV?E?DS?G????????r!???B?43??<~???A?1??*,B??j56???????????a??s9?z?????[?Lj??XC??u?{?V]?M?M?`??C?(???6?!twx?t:h? ?'?{???f???K?f?Q_Z?
??K?u??X?X???6??JV??YaB???|N??N?/???(??????_?O??x???h?2=?9?E???^?*??????????4??94???k?????NL&?@??A?I??4??e???f'Lr~?)o??]D?.w<a? ??K r? r??Xm?.}?^??_?y?ZkQ3<?Z???TW#??k?Q?]?C???$[f??+?i????????D??:q?M?1@?r???r	(%???v(??.?????n?6?p?v!??M??;3s*??-N?????g???.|C?y]?n?jZ?{O?=J?C4????U"??
=?e?????C??2y?>j???@D??d?r?P?w??yM??pyn?11P!P*!M$?XT]?W>?R>???{?3_??O?;4?<???.???x??DZ?-??7??zn[==@=??^?.`???s?????t?U6n?5?4????P????????Ee???=??fI????f????`?P??????D??$?n1<M?	5I???yk'{?U({I1??v??K+?y	{)p??d2,?I^?0N|???`,????g9???RV?????f??]V???Nx?Hf-D{??2?=??@	#??? ?]?wJ????U<?^Oi0?????C$?V?M?????E?????!N??;`?<?c?8$E(??Q??%]???zi?,B]?.D?P??3?^3E????'P?T`?>-7?&`G??\0??>????d9?j????I?w???S????????x???l???:????i?Pn??!>???A-% ??^c?v???	?w??? ?? ???? k?i??
????}b=;??=?Rr??0-????WOgn?????(??;?wy???l?W?L?`?M???
=j??<?-*]??=~?_}???/??rl(?X?@???<n?h???????????:H1#????????{]?E????Gf?Y??r??V]??i?RW??????.??nB?0%~e
'n?<???'VBx?*?ZF?c'???????Ot???Ir??|??ZC/l?? ??z3?'???>??????3?~L???/?8???U??j?????+?q???????Yb?2b??????i?%K?4??e(Q??`?p????s??????.??V<?
VF&z(????qfP9???*??yO???!?l??P??|???R?h??6??k?`e?P???Eo1n?S=?)??)e???(????}??<"?`??9??dh??????????'???K?????X)#???C?QFN} ?????FX??n?2,?C~???? ?C????^?#_t?Pm_vG???$3"3?j?-<?#?2????P???i?0???x??W???Y#?????aR?J?.\3???ha??z??55x????T?z???/^5??5????j?o'[%???Ug???#??#sc?x?]???#???????e'???;???g??????p{????4????L_E,_5?????c ?Y?8?}[?IkQ??_ 4??mOU????T???{?8R?????? ???K,?g???????s????v!?S&?4?Z?'hOJbEc}???C??	6'RsT?7S$?x6?)?{Nb5J"=?W<X?9Q???|?]?U????
?G?????sT?S????L??q->?]D5????av???\X?b?s??frCr?u???A???k????E|?J???O?~?h?I?mF ????,
???[???Q?a?Q?|6???s??;???????_"?KK??L?j???]??
???1??/??5???u? ??F???(????}???V?????&????^????~+?w??n?H~???M/-??{???&???+?????u?R??>?sj?r?ovn2?G?F?O?J}zd?v??.?gH?NJ[V?	~???=????c??t??????;?5?7\???;?5Q?K??<????N??h-????	\8?????h?>?s??J?t?H??VWd??l?Zgsf?Hc? ?(????#?H
?i?PM3???s?[??T )k?4??b?B??-NP???}?q??N?n????iR???$???>W4???/?u5??=!?????n???<?Y?n??k ???<?Y!???&?I_?????????+??T?N??>??tc????U`<?w?????x$?{???3????????oT?U??~T_T?Q?{??/??? ,?]??c???jU????g??|c?u????????3qM?;?;????]?=??^???p?>??nl?\|???o??3??]??<o???8??-????!}:??<_??o#Yx?~8RP?:y?????l/O0EAU?[??YR??'???h?T~????????? ?WV4w?????.;??q?Y??O*?gkTj?k?{6?? ?8????7U???????2?cJQ}*Mn^??oNW??,?????_?U?
V???~?/?Uk?Qi?w??=k5A?+C8\?[p?i??CZyOG??%ZzC)??H?'??`	H?U?(V???sz2???????RJt???????g??J??!??nZO??]? ?????????:?#&??u??&P???????_??R????|???????}??
?Z?Z????^>????#???????p? ??4-??!????GK?x??k??|??j[???E?_\?(?????C?N&<,??:????)?????p?1???<`O???x ?n#b<??????ys?????Y?D??v	?f#??&bm8?pi%??>m#?????0??b?"lH
?+O,??S9k??????=???Qm??}`?	?m?n[?h?(l?v?q"?/??U????\b1???I??s-?9?[??????w?6Q?z???BLG2??
??s?|?Dii/q&:???!???N????K(??.w??f??H?]a???H?e???5u??7s,??????_???8&C ?D???t????Z{2?Gd?0d?F?;??1x?????????????#c? t???Ya?k?q?V4???!???n+{?!H1????.??	U&?gjdF)??A?Q??Z??.br@o(4???0?%?0??5?)????\?f	H?2?(?|xR??Xgi?(??>o??j???y=???5Z?G??E3l??????F|jF?.<?W?a[N????@??q???????m?}??B????`.?X??3p??Z?Du?_L??????;zk?W??BN1_^?F???J??^T?U?v_??6y???|??	N??? ?y???????c???y5?]????$???5uo??????6?f?x???~?>?tu???g????<~???????z?Y?????????W?????????????i?H??!t?jZ?X????
?:?Mc?da	)????ycY??????\^??Z(k??7??*???u?k?v\?qy?L]G??o8??????E??J:Jl^??e??_????SN4FV???:?C???R?\c5]??1?]?N?????ObBEU??~??ql??VAY??cO?*???7??????0U??-?g??fV??j?L??sD0??4kK?A?4? ?E?:?}?[??;<m84?- ?Q?%?lm??dg}???w-~???y?zg??t??^???=_4??q?7?\??
!?????s??Nw???]^???ek?)??? Ze?Gz}???/?????P?da?a???p=??????????Od??t	?E??G????JJ?G?6??Vqw????0
?N??W}y??? ?r?\?k?v]?????kJ[b[??	?????+????(?O??Fn*.D??#|???E9?F?/??Jj[%G??f~h???C??@?V}?;A[??PI1e?Z4-?Z??Y??O??se0??nW?????"p?;q?b???]??????&???Q[2?????^??D?????rMG?.??????pt? B???q?l???r??M?????? E????'???a???????O??w)7?e-/@wv[??JfT??PF?,%2Fc?z????$???5/?*?w??(F?qE	?
?????????c?A???OHz?fd1`??V?	9\;g{F???F?rF??}?I[??B????I??HN?kZ@?? k8?=.?I#d?]??$???t=?;<?'??]????6??AK??.
{0?"?Xe)oDXu?3???#%X??	|???h?3o????N? ?i*nF7?U?f;P?qr??
??k???^n????|????Y?L??k?h?5?????g?????D?ks??<?)??(??(?[#?6G	?K?ho??{???*??h???
\?????????	??????s???4?}?]??'_?sS?B?d7??????I???D???m??????'??(???=??o(5??k???(J??????$+?????]5?b?????X#?i9?n|=?e??|??Uu?Dd?????j???.????:x4????}?.????`???q???????K?j?$k?5e@??h*?v1?=?$!???&?4?H?G???:?c??D?.???iw?Hy)???w??j?<+H(????n+>??O??D?c[#}@?dO?x05???_????d???/????aij,W??{x?q^U$V?UP???d?@?7.?4??hZ???ZC)??/?wf??????????? ??????3?	???g?????lVj???MbS?H?????+6?b9?????F??e? ?;h<R?"?D@???$???|???[?2??}??<\~hi?v?+n?7=???'?Fm?F=??`c>??I?????J??ez??S?Q?????????4?s;L??????%??? ????????32????/??R??]GY?>|??F?nW?q?\={5;????????\????O??K]|H??????#?=??e?k?8? i???Y?P?{oh?????ZO?????+-?a?J?Y?|?)?Ge?"????p??{??3:?A????	=?_?'e.W?C-Y????? ]TC0}?~A@qn???i2JX?*???????dG???u???p?EH?<??x\?H+??	?+t???L?_?%	??????	<??RG %?
?f?k?%#q?R???????y?x.3??c?Z??N&?wD?'???64????D???Dr\P_D???.?v ???|??\D?PB????t??>????D?@?ulV?-??? ???Q^?#i????rJ?????M???]??T?3?Q=S?L??????>%???u???p?u ?Fm??????????M??u? ?%??)???IQ?? ??	h ?|,??g?&??_<6;v?>uwvu?dgY_?o?p??T?????f?2;u?T?]Q??W??????6?????+{??csgf???????b?]v??.??b]???.v????b?]??!?P{??q?4??A$1W\???E6L?_?{?e?"??ir6?Bf???!Kt????p.=????<?~?? 4?B?5?81????7%???I???+?]?=?m+???^??S?J???^7????D??K?&.??S?K??9e??H???:eD0q??{???>S??^?g??????Q??]??s??\?keQ_??+3??\lZw??k=?	F?1(????2tg??M?	?>?fV?g	?.Fz??-??(??!p#?i??r?(nNf?B
@1??2n7a??;?Cc!1e?^??~???4??~???Y6?+?a?-?c{c??+?V??L????<n<???I0????^??7??????E?+?A$V{???9??{BH?????l?,?.?'?,??-X?<?k*O?hh?????o?m6??????4%Y>???m???????????UV?:??S?WmH9x?!??????P(?{??w*?k??ZHb??%?u???b??'=?????[7?A3???qOVM???m'Z^???`P???|5?Fl?k`pH??A?? ?y??Y??_[????W`????3C^&?z??0?8???W???@??f??m?eIj?a?	j?0vk?VG??`?A7"3?)???|????}o<?N?vl??c?1???[?2???r?2?>??????4O??Sy^???s??]?????M"9???&??z???Y?b?+B2??5]<??^???w=?5J?fy<t?bG???XJ?>??[?j??:=?E2N?d??x8E?+Rx????Y??????T???????qO??\[??N?7Z?g
q????????FtR??kB?}??e![????e???????z#?X*??c?????D<?????#2??^Qm?I?]TJ?o!?]2??b%??	(	?|1? ???i????$??2??8(J??>x?Q???????A3?X?7??i65?[N5????wa"j????]?#O??[??D?%????K??-??U????w???v?#?<O<?Yl	?q?T??'\?+???r^ihY?~??|??~?hFo&?Zk"?W3??????@U??,n????!(h???3/??T?j???U?[\???<???
????????????!??????)?U????3|f?PA?????U%?n ????1m47??m??????H?	??-?3???q??Z??\;?rC???y4????^)
??M?:~(?WV@`??{?????!????P??aXV??????Wu	ER??g?g?????Ql?Y??79?????????3P?L?????3?]?Iib	3?_?:?O?m?T_\??LR? xf??$?(~y???T?I_HUt?|???d$?_"		X???)??b<w/Zk?"z???u?A?e?^[????Gx???cd??{?OJ??V???c???L7v?????G?}?????<???9Z"?|??w+?????<U???M??C??>?e?C
?0?i?^?????iO15???Y????H???6????a>s??iP0??P:UbT`tjg4N??A??9h????????|?$?CB??T??_?
3.^?Vo?W????9???2??????/o?`??5?f??/?j?M??Lf?????Xl?q?Z???y_??5?RNq??N?z?q?????K?t?????U?2????<?,??I!fpU)T8??S?{??~`???(/?<*^?????H?O????Vd'??????]?c?L.??"??]T?-??_???{??L@??\??N?F[`?SmV??Z3&??K ?6.W?B<"??3(????h???K??a?o |H??&??S:Q????%=??y?5?d?
]}?~?~??+?J????????^?]?????q?#?`"E??????1C?k6V}JS?%Y?F?H??YY???????T??,?????P??&zE???J??n?????:0??z??i?Hsz?='?Q	~n??????x#?????P????]`???8^l3`I(?U??:?W#/?-???????a?G C?????S?????:s?????!????;?h??3??b??9?/?;???????/1?-V??lc?'?F\???<??ILqlU?]J?0j???X??GU??<?	????e? ???M;R???
??n?Dv?	??(???s8????gV)?
???;???????l?
?8??Ew?<??z$?^h?@l?8Co?[,lv?F'??o?C???;???? ?%???O?????gR?r?w6???x??p?>P???N?:O?d4B????GY??WhH1?}??k?"??s?v??N???u?Ew??+?"?j?:dK?'[?Z{???n???`?~??27&j?$b ??n????????}?????u???vi?-.?	??U{????\???FpK???7*?kE?????%??~1P?????????P??NW??H????7<4????????%????[?uH?Jv??<?J???s??C????D?[??????I?????55-`???S??B??? fk??^?=??>??s?3U?&z??s??Blj??????}?C?P????? @?C?????p[?{0????w??
P???@????~???d?6?Z????	???^?<^>??9:F??C?q?2oas*WFC?h??K???3U??^ 1?o???;?????/??[????o??]H???w?V????	?_??????Vbu?1*;"?????j?C??{VOG?????o]?q;:q;G???#1??/r?s'??gx$P??P???{?)???P^l/R??$?????^?????p?"^&A??O?9 ???O[?,?(??M?PK?	?b???X?f??"s???.]Z7?Rj.? q??V|??=?h-Rks?D+t????i	+??????=?}7??yP8????r?KU{}???Ll??P??MI?~???T?=rw1<j?~?j?5o?=N.,???.S???3??i??|<?LXJB{?VZ?j??Q7???2??????0??7?25(????J?AH????S?p??Cr?O8???y????J?_`?f{????<??l??b?=?lT4???Y?n???h?4????%???%??3?x???iGI"@n;?&N?W{G?3?K??? ]?"??kZ@:$????j???;F1?&??????&?]?v?* ??+=???7t? rs??H`D??0?b?? ?^M0E????????rO??????P+????????????_????^?JE?U? (?????s??????$? ?#0??,??(1Y??'?w?1???; L=???)??~Z???Nq?#?_e????D?G????te{@~?w?u ???????0O~?????E?Y@?n?/?5,G" ?c'??????A???????????Fr?????????K???Z{???p??.?D????4?K(?s?po?)6kJ?#j???O??E???;G???q?`Py?r??B|Q???OK??R??=?~?g?w???t?uu?pMo???Cc?YWww?Hy6N:2?7?[U]??u?%??q??_???q??)???b?&?????A?j=?ZQ.?????_d;???$$?[???P? @ ????{?(??>:?/?Py<?[?DY?\??q6LE???????O????~??]-?a
?6??t?1??3F??1YW\????Cl?jA9?????@??!WQ"MB`X??=?P????3????Q V?C5L?(???	?>?)(17???)'08?>?>???%??V??A?K????4%l1?????i??????h??[???????%h8??*?TK?<??*Z??)???IR4?s???9??F??????\?u??MO?=??-?z???#@3`??q?G??ZVN?? ?x?^?%??So?@???????`C?3???1????^?o?NF????A???????l???~???/8??:?g?a?_?????S??}?]1q??%????"?%??"U?????"??R?????U?nq7?j?Cy?1vW????????wt?8=??q??'*????p?P""Z??9?s?f@??`B????vv?kOl? ????3???{'>)7G?.?m?#J???<???G??5O?v??9?~?[?2????????Y???)?y???,c!?H???~-?ET?;?Wwo ?>??u? mr?[f2?z?*vO?????p*?^!????i?i|?_s??????6iw$?E5???`???q??|7??x)-+????A?}???????M???X????????j}H?\awi?S\S??9I?p?????4~?e????????>???kP???????x?K?V??f[W%[??"3J(??*yq?????9?:7?s?a?	??'e?Fq????????%C7?S????N?0 ???_?[????:Q?????,???1??^N???g?F????ba?????n6'?Q?RQ??{0=#0??|???>w????>?8N?(MVRG???P??*?? j?#???(?????9?S??}???=O;???WX+?	,?)????yd?????!?2?????{?w ?9????1??Z??????(?????kW_???8?? ??)?? ?????$???C? |O??0??N????????QmIP?????o??????T???w?B????????\?o???????c?D ??$ ???Ynz}??U?.?????i]?C)=?|?M	?T???_?|??#?c,L?JG???W)?.?<|?L?|??&D???8.??m?0???2~????U??????diJ7,;???7]???v???Ko%#?&L?;???@???????_?f??L????Wq??m??M????H??N T@G?k???????L????r~f?K~^????y?v?W?s???(}?9?;?^???????xnMV?r???>?.??"IQO??g?m?sT?63?o?<{8???lr?????6/???Z)?x?=???????hq????B??c??hQp??Eyj?WJ????5/?????????w???2Q??)"5????5???	%?M;?^?6}????H?4j?	E????F?MfI??????<???w?J??A?C?|??!????????>uW?_?$????V~?ITw?W?z?d????@.?:TFi6G?k?h?:>????}~m??????mz????z????????XO@??????	9?~?????y??/??W?3?w?DpC?72??CU?? ?LK????z4???>??$????'?j?h?? ?[????{$i?Q?Ps]- ?n^@??ur??j??????.u?|O>C?????????C??;????d??Rw?R??	\?+?j??}T?8?`Y=?<????????4??v?$??v;??????']@?ra?????Zd u? ???t7? ??(?'\^/(?VhX?L?)??=7y/????~[LkYJ+?)5?j?9I??`????]l????h?*D?,???)??gcmwR?o??????O??*}Z8?~???S??Hb?Y???Z?^W?x???????"??y?/0x????kiK?P1#}????L?Ik???l&??j?E?m??e?F???????<t5?y/8L????YjJ?}Jg?_dw??Q????Mnt?t??8?s?Z`?S?D}3?~???+?YB????
?R0????h??.???0??????P?T'??}??5?y??{?>???:???l??P????@5?x???	U???W9??N???9?^??d?&?N
?o???????@???)!?V?*I????????Q?????d~???/G.?a??w	???C??!?l??S????r\?e.?	?,k?=z??[??k?IM*l?4????????<??????????O?8??p?W??????w??????_?mo?
????kk??}W?????????QO
I????????g?l?Vz?????;?????|?)9.??CL?F?!M!)
?*P?%C??A?;G??????,4/???z %?????Ld?A?-W??l??&?l? {D??u?A?r????-??sY?????)??	??;8??c?Bs?i{???!?!}B|],???vN~??????dF4wK??e??%U??n????M?Q'}?Go????j?c	?;?K???6?A|;9=`!??,??0?????#n4???p?]?s?;?x_??d}???????
??????T?1 ?=w??T?d?q?S?????s4??,A?????????^??dfZ??bng[?+k^??????c??Y+c???????i???h????1+
=??Z?2)?j????@H?(?=[v?$?.???!???o<w3????$????0??7????h.???c??R??[/??V?!??.????c?2?7n?Ak?Y,?w;~?*D??mRw+}??b=+Ba2?R???L?????.?q??|???^>??>?? ??Z??0} %?z??? ?LQY8=z!?????4n??qA??Ul?????B????-?????????U,?D????b?f????????7?X?%?T????[???s?#?M??&??'`<????,??\v?cAi? ????Q?K????M77?Uw????FY|?!?U?V???Lp??????m?????"M4C?:?ZHah?????|?'A?z??0?B2?u?ZyA/-???.+????E{?-/K????!?1w????K???f2{?4?.jK?|???R^~>?|?f^U[U6??????q????<?xm?]??<???^??????;???En ???G?'o?5??????F?0?	hK?-?D???6??????kf?!????wI???S?J?b??M?4?????x,?~I???">?t9[???H?????(??s??B??:????u?su??c?Bur?I??1m#?)M??U????b?u4?Z%??????Ja5C:W>?0??????5
I-?b?k?@???7??@?*??Y?t&?z?[????L?d??1tub?????Fo?#????{??&?? M??6\?g?????$?G]?
????1 ??g??????!?.?.??-???T ?????v?????[??a?F???g?????????WX???????r???6??/T??W_T??S??z?g???????.??Ne?r??T?????????i??[j???qLvI
??I;?( G??n??H??P0?B?.???I???FO?))`???"???"?
????[(e???V??+;???Zw?6?y???2??t?i?'?Y+?l|?? k??????cJ?:Z?}b?9??EMn?V??C&?2_b[E?%??>=>???NW???????"S?nJ??pX?q?t??
}??>%?J??E"?o?BO??3??.7???jN]?E?+???8????????/?)??y?????C???&???f??????????!K!????)z7??#Y??B??Gj???????|y? |?M?:o?;??]
???????_m}R???}??u1?u?0???E7;x<#;W???.?&&????? ???w??T???';?5???????2?h[.??*??xE?'?\????{?T?>????M??k??
?Vk?o?0??q?>??????"?M)????,/????3k?t2?^&?T???\B???F?chD?Y)e}?=s?>K.???/??j?????????o??x+!X????>???V?2~?+???G????8???n????u??????r8?i??"?????Z?????T?Pip????V????l??zoP???`?|ByY?????}?\????'?????\??J[6?*?!?,`?h<`???q- [G?!??z????????m?P??9??;??J*P	??m? S6?
?'=aru????T?[A???*????4$??D??x???j???	?W??%
?????`??m?.i?0Y2??jS;1]????h??s??????7r?s?^?qt'8^??~b-e??6%R??????S?	nfX???7w?pV?p|???{/???(??N?0o?m???U??????$?F?.m???aD???{?!'??Z?Zj?q??y&???1?Ad????R?????W"~?E?c{???????e?W?kZI?.6\E?z????uTW???'Pr???S?R?[E??Ll?<y????K????Qa??(?h?={.6???U[???G2.??0?dBc?g{}???E)??y??"????'x????Ux2??
?1`?-?C}????????b???%; ??VYG#?~-???f1'::??r??#???3+??#{'???c?s???v+?q??75??V?0??a5R??S????Pp??~??X??#-Cp?+x?????PY?w??????BJ?H?R?$$??7?u.o??gz?l{T?`????2?9???f2????[[??jO????lNh??=8??8wS??l????E?8R??\p???u!???p????????W??s??r<q????|&???????6?????~c ?f>k?I5????c>?dG???$?d???????????????q|i?????N?P????m??????{?3hR+5?b?,??]??????????}"ATg??n??:l?@?vA??????Q??7' ?????u???????w$??????/?~??? @?*v??<k????????h????K8??????5?0}I?(b
??P?!M??uQ?7%"??mY?\lf|
???e?3i*,?J?'????????j.e4L~B<???????^L???X??v?sFx?2?g,t????"@?{O,???????m?+??N?W	?????q,??'?@?Z?????H?]Eo?*@???)?HZ-M???lg?H??qT???Kd??Nx?N:R????0y???v??b???????(?;|M+??gB??jE1#???I?????}??	???w???X|v_
'f?????????????????"?~??????e??p???f??W????.??]?y???su???????????;?w??;wk??Y?r????i???????hW?,g?? `?"?f=p?ay????????8'??8???b.??yx????\
b?*$m???h??E:8Lz?W? ?/?E???3????f_@???????_????{w[??? ??P???k?HC???;????/?????^???????;[_(????e?x?l?A???Ih???? ???????P?XC?afsfoXkJ~(?S???????k??Q?"????w3H???C
x?*+g?????bb?e??}?^o	?.O(U1)??fB??W???a???y?[W,#+?/??,6#??l???!P?P??A??X????`?`Xrz4????????]?I?K??,??4??|V?{??w???X"??	P?Q??6?o&s/R?3???hK??Qu???Dxo+??^??K`+?a???????????????8?XU?G@O+????F???=???3k?	??5=????wy%2rn?????EV???j??q??%a~;s???[?LE??(???Of?`y3M7?:T?????Q?q?????????b????;?P	???"??????????e?f:(?/?????y???K?d"A??1%????N???&??A?ke??G?BZ?eh?$?j??%?v????????1??L??????J??z??Z/??????{????J ?R?w+?`T??' 	?'??ZLlI?T/_H??R???A?U????&0^??|?X?9:}Z?)?>]?jx
`?:zZj?q8? ?[?dt???2???.$??J?z!?>?.?v?0K???}OqHJ?????{Q???\b?C??v|u;?+??P*?{>V*'?a?????J??/2?|;d??Z??.<y?/?)?/??d,???r?k?}??#?{?]??\?g????;+O??zs??(???Z???F?D???d??N??j??r??I??xp|Q?	mZSKws?@??F?1???3AKp????_]????w>M??W???*/????+?5M?;Gm!H?,????7????-?D?P?o??P???Y?)?["?"??UD??7R%{?q?m????B?r??????c?A??8??=2?c?????<???{afP>-V#h?3<?h?U???,?O??r?z??? ??c?(??d ??x?q(w??-??x?=????p???f????LU??6?A?)?:???d???\K?m?C?-??S o?????V ?S??C?sY???	^???=F????p2?7Bx&?P?D}?.?TZ.?X??pY??????R}?t?,y???kGS???X??<?F??:?3Nl??V?;?R?n??????F???S?&?.?Xe????0tq???^?+"7?fE???H:??U?q????`????u?s???`?p?????uQ?[???T(B6????VJ	?@gV????z??	?!_F?????Z?]?????$b??n3K{??? ???o???Ib?t|? ??????? ?j????
?{S??-"?Z??5*v??-?k1??0??|o???-??VN??{r??%??????????a???{m7G??'??B?zI?#??????S???@??(?0*r=?o?K	?P?????&?t?q}E??dB??WA???7?: ????????PiE?????-?????
&??R;???J?C5/Ed??+??\?w?0%?Yx??2?/BD??/9?vI	Z?zQ??;?igYF?n??MZAyJF?1O??J(V>???
4_??4???C-??7}?J6yA?k?Y?^?0#m`??e?^P0???????m?e	G?k?|p??fj)Dz????`?;^?[1g+???p[`2??;??ZF??M?iv%?{?X?N??Uz??? TX????????Z??@9??E_y?~????1?/H????	g??Z?	2?kRYBIf??h?7?V?k????-GNo?5????` ?"??0????m{???i[/?e??V???????/??_??G???y?h???/????,??*??]??>~?fW?X	c????\??Ve??x??d".b?g??D?????????gB???iYl?Z?*!K??2????	[??_?nq[_???G??Wq?b? ???P*O?W??K/a.?|?f?oL?m???K?h?}?M?1g???G2?HI?z?????,?]?9??U????Y????#?ak??9ib??/Y"?G+???.??z??)??`??c>???"W??h?*?%?I!???b??3R?IZ?S??4Fd?8??????a.????7)=????:??8UH??a?1>C?)??B&)-??l6??????.B?? ???????@E?h?e??????2?8u????T
?9?m[???^? \Y????AU?N&I??w???*???P;??dL?.j??H?5?	]??????s??B???1?`?a????m?oi3a???t}?:eW??????Y?#??????W??M?=?bu??c????It?.R?U?V?Q??"?=?m'???vi??s%???B?f??9?BM????-???Y?$?z=?gX???r?????t????????yj*?S?E$Q`Y?p??TB??p??5p?I nZ?o??d??5?g"6???P;.{??B??K!?F?t?W??_??)9`???p????? 5???? ????4????>?????_?z?????r?K?&?}???v?_????v???????g:B? ??SK?6??C?f??a?\?f??????h!F?\a???k7?o??Fg?I??????*????-x??'?+a??4i?ccFG??? ??m?L??5?\y?[s????=?<?????Ca?*`?|?1?IS1E???n$l?????-??	u?f?!??!?c????G~,?5??H????Q??W????cE[?[r??NK?5,???!??????????_?.??'o~???????i???^????F'*?5??Ba?P%x??g??q;?K??h?<?_???E???0FP(?3?????S9p5??w??? ? ???$EQs?(??????t_r ?R?d-?5??w?^???c?U??4?@???$8???{6?dA?_?k?g?a?L?????i??f<?	[?l??:??V.?KF?????9 j????
???^???2????Ek?U?.?Ugv?J??|Eu???(???y	?,w??)?????4]???\?"H=??:-?B???*<??rX~)?B?r???????pE???^???-?^C 6?????	?sA????17?1R??0^@????j
PvG??y?71?^=?$?9:??W??K?MSX?Lx?h??c;mu??)?3a???$?????????pykX???~|??????}???K??u?B+????|?????c?bXz???????@?;n?$Y??????/?U!N????}?O?o L? ? w e?C?v?b?? ?????????"OW?? ???9{?
?e?s.p???~y????'?M????I?v"?{NFK$??,??? )?MB?	?|Gp??S????]?k????CX??L4?'h?X[?v?l?D|???^`_???/?P?;9??sr'/wq"1r???T???,??Mn|?;??t?V? ?e?u_??&?G{?? ????w??`???????/???v$?q???;?? ?$??gG??W?.
>?%??}?????R ??C????[??'?;??}??v8??s?6?0?d3n???p ?U?P?]?T?e'??[??Ng/???51 ??gA??a?????:??y?J??V?F??FB??j:?N?????V?
o???????;?)'?)????_?~+????H???< ????zy>?????????{5o_s?px(SK??e??V??]??sC??=??F???????D??>??.??	w???s??57???]????n???YgdXo+7Z>??'???!???r;`V)?n,I?????Z-???$?	??  I???=U?M????=g?x????A?yU?@?T<????v<za??l<???????R??j???????niC?Q???m?0w??[??'?~o+??ew?h???????H5?-??=Bj*0R?g?8<r??\?9?????Q?s3?Q?(>?? ????fH?>?????????U???fY<?/??h?3-??)?-?????w?
?#91?1S?Zh???????S???tj??*??sbw>????g???????????,'???????+?O?0
?
2?4??a?F????C .????}??6?8[wS?????????????????nd????u?ZC3B??%???EU?????2
8A6i{??????I`M?
?w= ???F6??G??s?P?????ph??f??{p??,?*??do??8`c]s/???????5??????{???????2		>rMxM?Wy???-){?oH?v?B??<?(n;?,?app/?Uf1??[???f?/+??????[?v ?)???t???{h??v???OVS?E<?S?K?v?????Y?u??????oR?m^B
aJ???edA"(0????L????????hU?N4??7?
?
+????A????[? ?????US??????=W?{??>??r??????J???(??2?a*+??3??($}!=Pt????r?B0??{k?}???q#?Fwg?q?9w?????^*??-G???)??N??;?W }???$?p??nn????zH???(1?+5???o????OU?C??1??1???????N???"??????@??Y?~?? 58T??ud?@???@?.GOwV?[?8????	??'?`?o???|w??O?]???G??&??c- ?y???????G????	????? T?YD"????
]???D"?|?)Z\vL?G`!e?}????6?M??????~y??,?+???3o??<???<?h? ????'??:???'ZG??N_.4??H???	j??l??h?????Mz.|????Cl=v??6?d?7??????????t??U^?(m???s9)???????^??-@??????????z????;?I?C??x?\?_?????6t?????3C???h:8?cG^Y????Bj-??k??t&?????????#?Q??h?[?0DU??|
X?!?0????[%u?Y^?h?????????????|??1t????????(????otp?z:?t?T?????We???7?c?5?????f,?W?:???~I????K?Z?Q?????o6?f?r?t???	?????
^?????b&??? z???~????U?;m2;????2=?/??8??~??R: ??)D?e????1????@GV??_???*???_?????CP?\?|?????m?_\??????;???W+??????Om???O$S?(q?|} ???~
??????/5???????????????v_^(?M????????h??u`?f?????(cQ???????\?(?y???j>?b,T????s?,????Y??h????k??_????FL?:H?fe?c??b}X?H]CyC:??`??????zVz???1	?Y?:??{u?:m???q???s??u?)??s???????d?%u?B}?>??:??'???M??????j? ~???hD??#?d??g??????W?i??D+?????[m??}?^?n???^-???VJ????? <1?????????;??U
u???????I?Q??"5????L?:??vu??\????EC?????_??jv"?????,2N?????.??v?Q???;???????_?D?ZW/???9?0????/??#G??9,??`??(_?]?<%?F????Y?Z?G??????Lh*E???~QOT?=??y??]+.?C?_CQ/????)_V???"?	?p)?????g??????+??Y????xG?????t??%?y??8?L??ujkU??????o?_?????c??G?????_G?????????l?>??????_? ???J??"Z??f??~(?03?s?	??#T????B?????.An???I????p??T??E A?4C?w?? $??Ai!?G??3??E??'?(?gvn?l?`?&???S.??????dQ??C????2??$}?
;??????lf?T
??????t?????K??J(b/?k4(????X????'???	?????F??>??????h????????C?????TSuQ?a?>l)?J6???f#?\??????Plre)?.L?U??j???????? ???Hwj3??????%q<?Z ?P?M?c8???=;>6??~/Xh?Uq?*?c5?[
?	?&B=??i??t???*??{?t'?s??V????D?2W"?v???"s????py?????9M??8????q5^?*Gz\?????P?B;??Ia0?w?F0I?*!:?
e	N?_?C???????,v?'?(?)?;????Pc????s#?v8??"i?wGu?t?,??j???]?~6?J???
1k?A?e???LU;?-I}??t?{?o???V6?d?Sc?O/???<?M?F?#?{#????B?K??G?=??V??p?????????F??????>???%?L??????70c???pr!???=RY??m???)??l???^R??:??????????_?bC????????c?K??????p?k?wF3?Tx?????5G????z?OdE?m???Ax??w?????n?? N?5???w?? ?u.????K?7??K????????G????lj?E?}*????d?H????????]??)x?9?Y??Y?????5???A6???(?K>X`??@#?O??`=????_???????A,?????w?????
?Z???v\???Gp?h?(?
-???dq?&g?18?6^U?@Tn}0????7h??i.%??l???<????.??+????????(??N?X?
???????3??l???bk?????9zf