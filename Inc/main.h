/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_DelayUntil/Inc/main.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "stm32f7xx_hal.h"

/* EVAL includes component */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

/* FreeRTOS */
#include "cmsis_os.h"

/* uGUI */
#include "ugui.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/ 
/* Exported macro ------------------------------------------------------------*/
#define uGUI_UPDATE_MS		50		/* uGUI update time in milliseconds */
#define uGUI_MAX_OBJECTS	20

/* Exported variables --------------------------------------------------------*/
/* Touch screen */
TS_StateTypeDef TS_State;

/* uGUI structure */
UG_GUI gui;

TickType_t LastTouchedTickCount;

// FreeRTOS
TaskHandle_t xHandle_StartThread;
TaskHandle_t xHandle_uGUIUpdateThread;
TaskHandle_t xHandle_GraphDrawThread;


/* uGUI : Main Menu Window */
UG_WINDOW	wnd_MainMenu;
UG_OBJECT 	obj_buff_wnd_MainMenu[uGUI_MAX_OBJECTS];
UG_BUTTON 	btn_MainMenu_LED;
UG_BUTTON 	btn_MainMenu_switchGraphWindow;
UG_TEXTBOX	txt_MainMenu_Hello;

/* uGUI : Graph Window */
UG_WINDOW	wnd_Graph;
UG_OBJECT 	obj_buff_wnd_Graph[uGUI_MAX_OBJECTS];
UG_BUTTON 	btn_Graph_switchMainMenu;
UG_TEXTBOX	txt_Graph_sin;
UG_TEXTBOX	txt_Graph_cos;

/* Exported functions ------------------------------------------------------- */
/* uGUI porting function */
void pset(UG_S16, UG_S16, UG_COLOR);

/* uGUI : Main Menu Window */
void createMainMenuWindow(void);
void MainMenuWindow_callback(UG_MESSAGE*);

/* uGUI : Graph Window */
void createGraphWindow(void);
void GraphWindow_callback(UG_MESSAGE*);

/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

