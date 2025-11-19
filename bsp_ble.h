/*
 * SofiOS project V1.0.0
 * Copyright (C) 2023 Sofiatech Tunisia. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file bsp_ble.h
 * @brief ble driver
 */

/** @defgroup ble Relay
  * @brief BLE driver.
<pre>
  * Example use:
</pre>
<pre>

  void bsp_test_example(void)
  {
    // Suppose that the function to be tested takes 2 arguments : Arg1 and Arg2
    _ARGV stArgs;
    Argument1_Type Arg1;
    Argument2_Type Arg2;

    _INIT_ARGV(stArgs);
    _APPEND_ARGV(stArgs, &Arg1);
    _APPEND_ARGV(stArgs, &Arg2);
    bsp_ble_ctrl(BSP_CTRL_ID, stArgs.c, stArgs.v);

    // Note : BSP_CTRL_ID is a a value of @ref bsp_ctrl_t enumeration below.
  }
</pre>
  * @ingroup bsp
  * @{
  */

#ifndef _BSP_BLE_H_
#define _BSP_BLE_H_

/*-----------------------------------------------------------------------------------------------*/
/* Includes                                                                                      */
/*-----------------------------------------------------------------------------------------------*/
#include "stdint.h"

/*-----------------------------------------------------------------------------------------------*/
/* Exported defines                                                                              */
/*-----------------------------------------------------------------------------------------------*/
#define MAX_ARGV_ITEMS_COUNT                     3

/*-----------------------------------------------------------------------------------------------*/
/* Macros                                                                                        */
/*-----------------------------------------------------------------------------------------------*/

/* Initialize arguments struct */
#define _INIT_ARGV(argv)                         {                                          \
                                                   (argv).c = 0;                            \
                                                 }

/* Add arguments */
#define _APPEND_ARGV(argv, arg)                  {                                          \
                                                   if(MAX_ARGV_ITEMS_COUNT > (argv).c)      \
                                                   {                                        \
                                                     (argv).v[(argv).c++] = arg;            \
                                                   }                                        \
                                                 }

/*-----------------------------------------------------------------------------------------------*/
/* Types                                                                                         */
/*-----------------------------------------------------------------------------------------------*/
/** @defgroup bsp_ble_exported_types Exported types
  * @{
  */

  /* Arguments structure */
typedef struct
{
  uint8_t c;
  void *v[MAX_ARGV_ITEMS_COUNT];
}_ARGV;

  /**  BSP defs */
typedef enum
{
  BSP_COM_BLE_ADV_START = 0,                         /**< Start BLE advertising                              */
  BSP_COM_BLE_ADV_STOP,                              /**< Stop BLE advertising                               */
  BSP_COM_BLE_ADD_SERVICE,                           /**< Add a ble service                                  */
  BSP_COM_BLE_ADD_CARACTERISTIC,                     /**< Add a ble caracteristic for a specific service     */
  BSP_COM_BLE_READ_CARACTERISTIC,                    /**< Read a ble caracteristic value                     */
  BSP_COM_BLE_WRITE_CARACTERISTIC,                   /**< Write a ble caracteristic value                    */
  BSP_COM_BLE_ADD_NOTIFICATION,                      /**< Add notification                                   */
  BSP_COM_BLE_ADD_INDICATE    ,                      /**< Add indication                                     */
  BSP_COM_BLE_SET_ADV_PARAMS,                        /**< Set BLE advertising parameters                     */
  BSP_COM_BLE_GET_ADV_PARAMS,                        /**< Get BLE advertising parameters                     */
  BSP_COM_BLE_UPDATE_CONN_PARAMS,                    /**< Update connection parameters                       */
  BSP_COM_BLE_PERIPHERAL_DISCONNECT,                 /**< Disconnect from central                            */
  BSP_BLE_SET_ROLE,                                  /**< Set BLE role (central/periphral)                   */
  BSP_BLE_GET_ROLE,                                  /**< Get BLE role (central/periphral)                   */
  BSP_COM_BLE_SET_SCAN_PARAMS,                       /**< Set BLE scan parameters                            */
  BSP_COM_BLE_GET_SCAN_PARAMS,                       /**< Get BLE scan parameters                            */
  BSP_COM_BLE_SCAN_START,                            /**< Start BLE scanning                                 */
  BSP_COM_BLE_SCAN_STOP,                             /**< Stop BLE scanning                                  */
  BSP_COM_BLE_CENTRAL_WRITE,                         /**< write a value from central                         */
  BSP_COM_BLE_CENTRAL_READ,                          /**< read a  value from central                         */
  BSP_COM_BLE_CENTRAL_DISCONNECT,                    /**< Disconnect from peripheral                         */
  BSP_COM_BLE_MAX,
}bsp_ctrl_t;

/**  BSP error codes */
typedef enum
{
  BSP_OK                = 0,                         /**< No error, everything OK                */
  BSP_ERR               = -1,                        /**< Generic error                          */
  BSP_ERR_TIMEOUT       = -2,                        /**< Timeout                                */
  BSP_ERR_VAL           = -3,                        /**< Illegal value                          */
  BSP_ERR_ARG           = -4,                        /**< Illegal argument                       */
  BSP_ERR_UNHANDLED     = -5,                        /**< Unhandled feature/option               */
  BSP_ERR_MCU           = -6,                        /**< MCU error                              */
  BSP_ERR_OS            = -7,                        /**< Operating system error                 */
  BSP_ERR_NO_DATA       = -8,                        /**< No data available                      */
  BSP_ERR_RESET         = -9,                        /**< Device is in reset state               */
}bsp_err_t;

/**
  * @}
  */


/*-----------------------------------------------------------------------------------------------*/
/* Functions                                                                                     */
/*-----------------------------------------------------------------------------------------------*/
/* Initialize the bsp ble */
bsp_err_t bsp_ble_init(void);
/* Exit the bsp ble */
bsp_err_t bsp_ble_exit(void);
/* Control request from ble */
bsp_err_t bsp_ble_control(bsp_ctrl_t eCtrlId, uint8_t u8Argc, void** pArgv);

#endif /* _BSP_BLE_H_ */

  