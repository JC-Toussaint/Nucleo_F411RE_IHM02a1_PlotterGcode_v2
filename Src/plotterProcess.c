/**
 ******************************************************************************
 * @file       example.c
 * @date       01/10/2014 12:00:00
 * @brief      Example functions for the X-NUCLEO-IHM02A1
 ******************************************************************************
 *
 * COPYRIGHT(c) 2014 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "main.h"

#include "params.h"
#include "xnucleoihm02a1_interface.h"
#include "xnucleoihm02a1.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern uint8_t TIM2_AWAITING_UPDATE_EVENT;

typedef struct {
	char name[10];
	uint16_t nbArgs;
	void (*pCmdFunc)();
} CMD_T;

__IO uint16_t cmdIndex=0;
//#define DEBUG
//#define NUCLEO_USE_USART

#define MAXARGS 6 // 6 maximum number of arguments

const float SCALE=800.0;
const float TOL  =1e-1;

void CMD_G1(    uint16_t nbArgs, char *argc, float *args );
void CMD_G2(    uint16_t nbArgs, char *argc, float *args );
void CMD_G3(    uint16_t nbArgs, char *argc, float *args );
void CMD_G4(    uint16_t nbArgs, char *argc, float *args );
void CMD_Null(   uint16_t nbArgs, char *argc, float *args );
void CMD_Error(  uint16_t nbArgs, char *argc, float *args );

const CMD_T CmdTable[] = {
		{"G0",      6, CMD_G1}, // redirected to G1
		{"G1",      6, CMD_G1},
		{"G2",      4, CMD_G2},
		{"G3",      4, CMD_G3},
		{"G4",      1, CMD_Null},
		{"G21",     0, CMD_Null},
		{"G90",     0, CMD_Null},
		{"G91",     0, CMD_Error},
		{"M3",      0, CMD_Null},
		{"M4",      0, CMD_Null},
		{"M5",      0, CMD_Null},
		{"M98",     0, CMD_Null},
		{"",        0, NULL}
};

void UART_ReadString(char* str);
void CMD_Parser(char* str);

static uint32_t PCLK1TIM(void);
static void Wait(double Ts);
static void TimerStart(double Ts);
static void TimerStop(void);

static void __GoTo(int32_t  X, int32_t  Y);
static void __GoHome(void);
static void __GoToEndStop(void);
static void __MoveBy (int32_t DX_target, int32_t DY_target);
static void __MoveTo(int32_t X, int32_t Y);
static void __DrawBy(int32_t DX, int32_t DY);
static void __DrawTo(int32_t X, int32_t Y);
static void __ArcG2(int32_t Xstart, int32_t Ystart, int32_t Xend, int32_t Yend, int32_t DX, int32_t DY);
static void __ArcG3(int32_t Xstart, int32_t Ystart, int32_t Xend, int32_t Yend, int32_t DX, int32_t DY);
static void __Arc(int32_t Xstart, int32_t Ystart, int32_t Xend, int32_t Yend, int32_t DX, int32_t DY);
static void __Circle(int32_t X, int32_t Y,  int32_t Radius);
static void __Ellipse(int32_t X, int32_t Y, int32_t a, int32_t b);

void __PenUp(void);
void __PenDown(void);

__IO uint8_t STARTUP_STATUS  =0;
__IO uint8_t END_STOP_X_EVENT=0;
__IO uint8_t END_STOP_Y_EVENT=0;

StepperMotorBoardHandle_t *StepperMotorBoardHandle;
MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;

/**
 * @addtogroup MicrosteppingMotor_Example
 * @{
 */

/**
 * @addtogroup Example
 * @{
 */

/**
 * @defgroup   ExamplePrivateFunctions
 * @brief      Example Private Functions.
 * @{
 */

uint16_t BSP_ST1S14_PGOOD(void);
uint32_t usrPow(uint8_t base, uint8_t exponent);

/**
 * @}
 */ /* End of ExamplePrivateFunctions */

/**
 * @addtogroup ExamplePrivateFunctions
 * @brief      Example Private Functions.
 * @{
 */

/**
 * @addtogroup ExampleExportedFunctions
 * @brief      Example Exported Functions.
 * @{
 */

/**
 * @brief  Example no.1 for X-NUCLEO-IHM02A1.
 * @note	Perform a complete motor axis revolution as MPR_1 equal movements,
 *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
 *			At the end of each movement there is a delay of DELAY_1 ms.
 *     	After each motor has performed a complete revolution there is a
 *			delay of DELAY_2 ms.
 *			Now all motors for each X-NUCLEO-IHM02A1 will start at the same
 *			time.
 *			They are going to run at INIT_SPEED for DELAY_3 ms.
 *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardStop
 *			at the same time.
 *			Perform a complete motor axis revolution as MPR_2 equal movements,
 *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
 *			At the end of each movement there is a delay of DELAY_1 ms.
 *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardHiZ
 *			at the same time.
 */
void plotterProcess(void)
{
#define MPR_1     4			  //!< Motor Movements Per Revolution 1st option
#define MPR_2     8			  //!< Motor Movements Per Revolution 2nd option
#define DELAY_1   1000		//!< Delay time 1st option
#define DELAY_2   2500		//!< Delay time 2nd option
#define DELAY_3   10000   //!< Delay time 3rd option

//#ifdef NUCLEO_USE_USART
//	USART_Transmit(&huart2, (uint8_t *)"Initial values for registers:\r\n");
//	USART_PrintAllRegisterValues();
//#endif

	/* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
	MotorParameterDataGlobal = GetMotorParameterInitData();

	StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(0);
	MotorParameterDataSingle = MotorParameterDataGlobal;
	StepperMotorBoardHandle->Config(MotorParameterDataSingle);



//#ifdef NUCLEO_USE_USART
//	USART_Transmit(&huart2, (uint8_t *)"Custom values for registers:\r\n");
//	USART_PrintAllRegisterValues();
//#endif

	/****************************************************************************/

#define NOTESTALONE
#ifdef TESTALONE
	printf("!!!! ATTENTION EndStop removed !!!!\n");
	STARTUP_STATUS=0;
	printf("INIT CPLT\n");
#else
	__GoToEndStop();
#endif

	char buffer[256];
	buffer[0] = '\0'; // empty string
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		UART_ReadString(buffer);
		CMD_Parser(buffer);
	}

}

/**
 * @}
 */ /* End of ExampleExportedFunctions */

/**
 * @brief  This function return the ADC conversion result about the ST1S14 PGOOD.
 * @retval PGOOD The number into the range [0, 4095] as [0, 3.3]V.
 * @note   It will just return 0 if USE_ST1S14_PGOOD is not defined.
 */
uint16_t BSP_ST1S14_PGOOD(void)
{
#ifdef USE_ST1S14_PGOOD
	HAL_ADC_Start(&HADC);
	HAL_ADC_PollForConversion(&HADC, 100);

	return HAL_ADC_GetValue(&HADC);
#else
	return 0;
#endif
}

/**
 * @brief  Calculates the power of a number.
 * @param  base      the base
 * @param  exponent  the exponent
 * @retval power     the result as (base^exponent)
 * @note   There is not OVF control.
 */
uint32_t usrPow(uint8_t base, uint8_t exponent)
{
	uint8_t i;
	uint32_t power = 1;

	for (i=0; i<exponent; i++)
		power *= base;

	return power;
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case GPIO_PIN_13:
		BSP_EmergencyStop();
		break;
	case L6470_nBUSY_SYNC_Pin:
		BSP_L6470_BusySynchEventManager();
		break;
	case L6470_nFLAG_Pin:
		BSP_L6470_FlagEventManager();
		break;

		// PC2 Pin = GPIO_EXTI_TRAVEL_STOP_X_Pin
	case GPIO_EXTI_TRAVEL_STOP_X_Pin: // PC2
		if (!STARTUP_STATUS) Error_Handler();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		END_STOP_X_EVENT=1;
		break;
		// PC3 Pin = GPIO_EXTI_TRAVEL_STOP_Y_Pin
	case GPIO_EXTI_TRAVEL_STOP_Y_Pin:
		if (!STARTUP_STATUS) Error_Handler();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		END_STOP_Y_EVENT=1;
		break;
	}

}

static void __GoToEndStop(void){
	STARTUP_STATUS = 1;
	END_STOP_X_EVENT=0;
	END_STOP_Y_EVENT=0;
	uint8_t device;
	uint32_t Speed = Step_s_2_Speed(100.0);

	__PenUp();

	BSP_L6470_ResetPos(0, L6470_ID(0));
	BSP_L6470_ResetPos(0, L6470_ID(1));

	if (HAL_GPIO_ReadPin(GPIO_EXTI_TRAVEL_STOP_X_GPIO_Port, GPIO_EXTI_TRAVEL_STOP_X_Pin)==GPIO_PIN_RESET)
		__MoveBy(1000, 0); // security shift

	if (HAL_GPIO_ReadPin(GPIO_EXTI_TRAVEL_STOP_Y_GPIO_Port, GPIO_EXTI_TRAVEL_STOP_Y_Pin)==GPIO_PIN_RESET)
		__MoveBy(0, 1000); // security shift

	device = L6470_ID(0);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoUntil(device, L6470_ACT_RST_ID, L6470_DIR_REV_ID, Speed);

	device = L6470_ID(1);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoUntil(device, L6470_ACT_RST_ID, L6470_DIR_REV_ID, Speed);

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	while (END_STOP_X_EVENT == 0){ ; }

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	device = L6470_ID(0);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoUntil(device, L6470_ACT_RST_ID, L6470_DIR_FWD_ID, Speed);

	device = L6470_ID(1);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoUntil(device, L6470_ACT_RST_ID, L6470_DIR_REV_ID, Speed);

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	while (END_STOP_Y_EVENT == 0){ ; }

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	__MoveBy(1000, 1000); // security shift
	BSP_L6470_ResetPos(0, L6470_ID(0));
	BSP_L6470_ResetPos(0, L6470_ID(1));

#if DEBUG
	int DL0=AbsPos_2_Position(L6470_GetParam(L6470_ID(0), L6470_ABS_POS_ID));
	int DL1=AbsPos_2_Position(L6470_GetParam(L6470_ID(1), L6470_ABS_POS_ID));
	printf("DL0 : %d[step]  DL1 : %d[step] Abs Posx : %d[step]  Abs Posy : %d[step] \n", DL0, DL1, DL0+DL1, -DL0+DL1);
#endif

	__PenUp();

	STARTUP_STATUS=0;
	printf("INIT CPLT\n");
}

static void __GoHome(void){
	uint8_t device;
	__PenUp();

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the GoHome command */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoHome(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}

	//HAL_Delay(DELAY_3);

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardHiZ command
		 * immediately disables the power bridges (high impedance state). */
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	/* Switch on the user LED */
	BSP_LED_On(LED2);
}


/**
 * @brief  GoTo command
 * @param  signed int X, Y in [-2**21, 2**21-1]=[-2097152, 2097151]
 * @retval None
 */
//#define PREPAREMOVE
#define HARDSTOP

static void __GoTo(int32_t X, int32_t Y){
	uint8_t device;

	int32_t DL0=X-Y;
	int32_t DL1=X+Y;

	//	uint32_t Speed = Step_s_2_Speed(100.0);
	//	BSP_L6470_SetParam(0, L6470_ID(0), L6470_MAX_SPEED_ID, Speed);
	//	BSP_L6470_SetParam(0, L6470_ID(1), L6470_MAX_SPEED_ID, Speed);

//	__PenUp();
	/* Prepare the stepper driver to be ready to perform the GoTo command which produces
	 *  a motion to ABS_POS absolute position through *** the shortest path *** */
	device=L6470_ID(0);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(+DL0));

	/* Prepare the stepper driver to be ready to perform the GoTo command which produces
	 *  a motion to ABS_POS absolute position through *** the shortest path *** */
	device=L6470_ID(1);
	StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(+DL1));

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0){
			//uint32_t Speed = L6470_GetParam(device, L6470_SPEED_ID);
			//printf("device %d V=%lu Speed %f\n", device, BSP_ST1S14_PGOOD(), Speed_2_Step_s(Speed));
			//printf("%f\n", Speed_2_Step_s(Speed));
		}
	}

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
#ifdef HARDSTOP
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);
#else
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
#endif
	}

	StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

	for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
	{
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
	}
//	__PenDown();
}

static int32_t gcd(int32_t a, int32_t b){
	while (b != 0){
		int32_t r=a % b;
		a=b;
		b=r;
	}
	return a;
}

static void __MoveBy(int32_t DX_target, int32_t DY_target){
	uint8_t device;
	eL6470_DirId_t DIR0, DIR1;
	int32_t DX, DY;

	int32_t DL0_target=DX_target-DY_target;
	int32_t DL1_target=DX_target+DY_target;

	int32_t res=1;
	int32_t GCD=gcd(DL0_target/res, DL1_target/res)*res;
	if (GCD == 0){
		return;
	}

	uint32_t NMaxStep = 500;
	uint32_t NStep0 = abs(DL0_target/GCD);
	uint32_t NStep1 = abs(DL1_target/GCD);

	NStep0 = (uint32_t) (NStep0);
	NStep1 = (uint32_t) (NStep1);
	uint32_t NCHUNK  = abs((float) GCD);

#if DEBUG
	printf("0: NStep0 : %lu  NStep1 : %lu NCHUNK : %lu\n", NStep0, NStep1, NCHUNK);
#endif
	while ((NStep0<NMaxStep) && (NStep1<NMaxStep) && (NCHUNK>2)){
		NStep0 <<=1;
		NStep1 <<=1;
		NCHUNK >>=1;
	}

	while ((NStep0>NMaxStep) || (NStep1>NMaxStep)){
		NStep0 >>=1;
		NStep1 >>=1;
		NCHUNK <<=1;
	}
#if DEBUG
	printf("1: NStep0 : %lu  NStep1 : %lu NCHUNK : %lu\n", NStep0, NStep1, NCHUNK);
#endif

	float d2l0_target = (float) DL0_target / (float) NCHUNK;
	float d2l1_target = (float) DL1_target / (float) NCHUNK;

	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	device=L6470_ID(0);
	if (DL0_target > 0)
		DIR0 = L6470_DIR_FWD_ID;
	else
		DIR0 = L6470_DIR_REV_ID;

	device=L6470_ID(1);
	if (DL1_target > 0)
		DIR1 = L6470_DIR_FWD_ID;
	else
		DIR1 = L6470_DIR_REV_ID;

	float dl0_target=0;
	float dl1_target=0;

	for (uint32_t nchunk=0; nchunk<NCHUNK; nchunk++ ){
		/* Prepare the stepper driver to be ready to perform the Move command which produces
		 * a motion of NSTEEPX microsteps along the DIRX direction*/
		device=L6470_ID(0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIR0 , NStep0);
		device=L6470_ID(1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareMove(device, DIR1 , NStep1);

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}


		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		device=L6470_ID(0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
		device=L6470_ID(1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}

		int32_t DL0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID))-POS0;
		int32_t DL1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID))-POS1;
		dl0_target+= d2l0_target;
		dl1_target+= d2l1_target;

		DX=(+DL0+DL1)/2;
		DY=(-DL0+DL1)/2;

#if DEBUG
		printf("nchunk %ld  DX : %ld[step]  DY : %ld[step] NStep0 : %ld DL0 : %ld %f [step]  NStep1 : %ld DL1 : %ld %f[step]\n", nchunk, DX, DY,
				NStep0, DL0, dl0_target, NStep1, DL1, dl1_target);
#endif

		/* Prepare the stepper driver to be ready to perform the GoTo command which produces
		 *  a motion to ABS_POS absolute position through *** the shortest path *** */
		device=L6470_ID(0);
		int32_t L0 = (int32_t)(+dl0_target+POS0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(L0));

		/* Prepare the stepper driver to be ready to perform the GoTo command which produces
		 *  a motion to ABS_POS absolute position through *** the shortest path *** */
		device=L6470_ID(1);
		int32_t L1 = (int32_t)(+dl1_target+POS1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareGoTo(device, Position_2_AbsPos(L1));

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}

		/* Prepare the stepper driver to be ready to perform the HardStop command
		 * which causes an immediate motor stop with infinite deceleration */
		device=L6470_ID(0);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);
		device=L6470_ID(1);
		StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareSoftStop(device);

		StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();

		for (uint8_t device = L6470_ID(0); device <= L6470_ID(1); device++)
		{
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(0, device, BUSY_ID) == 0);
		}
	}
}

static void __MoveTo(int32_t X, int32_t Y){
	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	int X0=(+POS0+POS1)/2;
	int Y0=(-POS0+POS1)/2;

	__MoveBy(X-X0, Y-Y0);
}

static void __DrawBy(int32_t DX, int32_t DY){
	__PenDown();
	__MoveBy(DX, DY);
	__PenUp();
}

static void __DrawTo(int32_t X, int32_t Y){
	__PenDown();
	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	int X0=(+POS0+POS1)/2;
	int Y0=(-POS0+POS1)/2;

	__MoveBy(X-X0, Y-Y0);
	__PenUp();
}

static void __ArcG2(int32_t Xstart, int32_t Ystart, int32_t Xend, int32_t Yend, int32_t DX, int32_t DY){
	float eps=TOL;

//	__PenUp();
	__GoTo(Xstart, Ystart);
//	__PenDown();

	uint32_t N=200;
	float dthetaMax=2*M_PI/(float) N;
	int32_t Xc=Xstart+DX;
	int32_t Yc=Ystart+DY;
	float Radius =hypot((float) DX, (float) DY);
	float Radius2=hypot((float) Xend-Xc, (float) Yend-Yc);
	if (fabs(Radius-Radius2)>eps*(Radius+Radius2)){
		printf("Radius %f %f\n", Radius, Radius2);
		Error_Handler();
	}

	float thetaStart=atan2((float) -DY, (float) -DX);
	float thetaEnd  =atan2((float) (Yend-Yc), (float) (Xend-Xc));
	if (thetaEnd>=thetaStart) thetaEnd-=2*M_PI;

	float sign=-1.0;
	N=(uint32_t)(sign*(thetaEnd-thetaStart)/dthetaMax);

	dthetaMax = sign*dthetaMax;
	float thetap=thetaStart;
	for (uint32_t n=0; n<=N; n++){
		float theta=thetaStart+ (float) n *dthetaMax;
		float dct=cos(theta)-cos(thetap);
		float dst=sin(theta)-sin(thetap);
		int32_t DX = (int32_t) (Radius*dct);
		int32_t DY = (int32_t) (Radius*dst);
		__MoveBy(DX, DY);
		thetap=theta;
	}
	__GoTo(Xend, Yend);
//	__PenUp();
}

static void __ArcG3(int32_t Xstart, int32_t Ystart, int32_t Xend, int32_t Yend, int32_t DX, int32_t DY){
	float eps=TOL;

//	__PenUp();
	__GoTo(Xstart, Ystart);
//	__PenDown();

	uint32_t N=200;
	float dthetaMax=2*M_PI/(float) N;
	int32_t Xc=Xstart+DX;
	int32_t Yc=Ystart+DY;
	float Radius =hypot((float) DX, (float) DY);
	float Radius2=hypot((float) Xend-Xc, (float) Yend-Yc);
	if (fabs(Radius-Radius2)>eps*(Radius+Radius2))
		Error_Handler();

	float thetaStart=atan2((float) -DY, (float) -DX);
	float thetaEnd  =atan2((float) (Yend-Yc), (float) (Xend-Xc));
	if (thetaEnd<=thetaStart) thetaStart-=2*M_PI;

	float sign=+1.0;
	N=(uint32_t)(sign*(thetaEnd-thetaStart)/dthetaMax);

	dthetaMax = sign*dthetaMax;
	float thetap=thetaStart;
	for (uint32_t n=0; n<=N; n++){
		float theta=thetaStart+ (float) n *dthetaMax;
		float dct=cos(theta)-cos(thetap);
		float dst=sin(theta)-sin(thetap);
		int32_t DX = (int32_t) (Radius*dct);
		int32_t DY = (int32_t) (Radius*dst);
		__MoveBy(DX, DY);
		thetap=theta;
	}
	__GoTo(Xend, Yend);
//	__PenUp();
}

static void __Arc(int32_t Xstart, int32_t Ystart, int32_t Xend, int32_t Yend, int32_t DX, int32_t DY){
	float eps=5e-4;
	uint32_t N=200;
	float dthetaMax=2*M_PI/(float) N;
	int32_t Xc=Xstart+DX;
	int32_t Yc=Ystart+DY;
	float Radius =hypot((float) DX, (float) DY);
	float Radius2=hypot((float) Xend-Xc, (float) Yend-Yc);
	if (fabs(Radius-Radius2)>eps*(Radius+Radius2))
		Error_Handler();

	__PenUp();
	__GoTo(Xstart, Ystart);
	__PenDown();

	float thetaStart=atan2((float) -DY, (float) -DX);
	float thetaEnd  =atan2((float) (Yend-Yc), (float) (Xend-Xc));
	if (thetaEnd-thetaStart>+M_PI) thetaEnd-=2*M_PI;
	if (thetaEnd-thetaStart<-M_PI) thetaEnd+=2*M_PI;

	float sign=(thetaEnd<thetaStart? -1.0: +1.0);
	N=(uint32_t)(sign*(thetaEnd-thetaStart)/dthetaMax);

	dthetaMax = sign*dthetaMax;
	float thetap=thetaStart;
	for (uint32_t n=0; n<=N; n++){
		float theta=thetaStart+ (float) n *dthetaMax;
		float dct=cos(theta)-cos(thetap);
		float dst=sin(theta)-sin(thetap);
		int32_t DX = (int32_t) (Radius*dct);
		int32_t DY = (int32_t) (Radius*dst);
		__MoveBy(DX, DY);
		thetap=theta;
	}
	__GoTo(Xend, Yend);
	__PenUp();
}


static void __Circle(int32_t X, int32_t Y, int32_t Radius){
	uint32_t N=200;
	__PenUp();
	__GoTo(X+Radius, Y);
	__PenDown();
	float thetap=0;
	for (uint32_t n=0; n<=N; n++){
		float theta=2.*M_PI*(float) n/(float)N;
		float dct=cos(theta)-cos(thetap);
		float dst=sin(theta)-sin(thetap);
		int32_t DX = (int32_t) (+(float)Radius*dct);
		int32_t DY = (int32_t) (+(float)Radius*dst);
		__MoveBy(DX, DY);
		thetap=theta;
	}
	__PenUp();
}

static void __Ellipse(int32_t X, int32_t Y, int32_t a, int32_t b){
	uint32_t N=200;
	__PenUp();
	__GoTo(X+a, Y);
	__PenDown();
	float thetap=0;
	for (uint32_t n=0; n<=N; n++){
		float theta=2.*M_PI*(float) n/(float)N;
		float dct=cos(theta)-cos(thetap);
		float dst=sin(theta)-sin(thetap);
		int32_t DX = (int32_t) (+(float)a*dct);
		int32_t DY = (int32_t) (+(float)b*dst);
		__MoveBy(DX, DY);
		thetap=theta;
	}
	__PenUp();
}

void __PenUp(void){
	BSP_ServoMotorOn();
	BSP_ServoMotorSetAngle(22);
	HAL_Delay(200);
	BSP_ServoMotorOff();
}

void __PenDown(void){
	BSP_ServoMotorOn();
	BSP_ServoMotorSetAngle(0);
	HAL_Delay(100);
	BSP_ServoMotorOff();
}

/**
 * @brief  Reads a string terminatec by '\n' character
 * @param char array
 * @retval None
 */
void UART_ReadString(char* str){
	char readBuf[1];
	str[0]='\0'; // empty string
	for (;;){
		HAL_UART_Receive(&huart2, (uint8_t*)readBuf, 1, HAL_MAX_DELAY);
		if (readBuf[0]=='\n') return;
		strncat(str, readBuf, 1); // do not use the unsafe strcat
	}
}

/**
 * @brief  Parses a command string
 * @param char array
 * @retval None
 */
void CMD_Parser(char* buffer){
	// get the first token
	char cmd[256], str[256];
	const char token[3] = " ,";

	buffer[strcspn(buffer, "\r\n")] = 0; // works for LF, CR, CRLF, LFCR, ...
	strcpy(cmd, buffer);
	char *sCmd = strtok(cmd, token);

	sprintf(str, "Cmd Name #%s#\n", sCmd);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);

	/* Command Callback identification */
	uint8_t MATCH=0;
	for(cmdIndex=0; CmdTable[cmdIndex].pCmdFunc!=NULL; cmdIndex++) {
		if (strcmp(CmdTable[cmdIndex].name, sCmd) == 0 ){
			MATCH=1;
			break;
		}
	}
	if (!MATCH) return;

	uint16_t nbArgs=CmdTable[cmdIndex].nbArgs;

#if DEBUG
	sprintf(str, "Function Call %d Parameters Number %d\n", cmdIndex, nbArgs);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
#endif

	float funcFloatArgs[MAXARGS];
	char  funcCharArgs [MAXARGS+1];
	funcCharArgs[MAXARGS]='\0';
	for (uint16_t n=0; n<nbArgs; n++){
		sCmd = strtok(NULL, token);
		funcCharArgs [n]=sCmd[0];     // One char only for a command
		funcFloatArgs[n]=atof(sCmd+1);
//		printf("==>%c%f\n", funcCharArgs[n], funcFloatArgs[n]);
	}

#if DEBUG
	sprintf(str, "Function Pointer %p\n", CmdTable[cmdIndex].pCmdFunc);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
#endif

	if(CmdTable[cmdIndex].pCmdFunc!=NULL){
#if DEBUG
		sprintf(str, "Function Call %d\n", cmdIndex);
		HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
#endif
		CmdTable[cmdIndex].pCmdFunc(nbArgs, funcCharArgs, funcFloatArgs);
	}
	else{
		Error_Handler();
	}
}

static uint32_t PCLK1TIM(void){ // F4 Example
	/* Get PCLK1 frequency */
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

	/* Get PCLK1 prescaler */
	if((RCC->CFGR & RCC_CFGR_PPRE1) == 0){
		/* PCLK1 prescaler equal to 1 => TIMCLK = PCLK1 */
		return (pclk1);
	}
	else{
		/* PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1 */
		return(2 * pclk1);
	}
}

/**
 * @brief  Wait command
 * @param  Ts duration in seconds
 * @retval None

 * PSC=8399    PCLK1TIM_FREQ=84 MHz
 * FREQ = 84MHz/(PSC+1)= 10000 Hz --> T=1e-4 s
 * Ts = 10s
 * CNT = FREQ * Ts -1 = 1e5 -1 = 99 999
 */

static void Wait(double Ts){
	uint32_t PCLK1TIM_FREQ=PCLK1TIM();
	uint32_t PSC=TIM2->PSC;
	uint32_t FREQ = PCLK1TIM_FREQ/(PSC+1);
	uint32_t CNT = (uint32_t)(FREQ* Ts)-1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, CNT);

	/*
	 * Setting the URS bit inside the TIMx->CR1 register: this will cause
that the UEV event is generated only when the counter reaches the overflow.
It is possible to configure the timer so that the ARR register is buffered,
by setting the TIM_CR1_ARPE bit in the TIMx->CR1 control register.
This will cause that the content of the shadow register is updated automatically
	 */
	TIM2->CR1 |= TIM_CR1_ARPE; //Enable preloading

	HAL_TIM_Base_Start_IT(&htim2);
	TIM2_AWAITING_UPDATE_EVENT=1;
	while (TIM2_AWAITING_UPDATE_EVENT){ ; }
	HAL_TIM_Base_Stop_IT(&htim2);
}

static void TimerStart(double Ts){
	uint32_t PCLK1TIM_FREQ=HAL_RCC_GetPCLK1Freq();
	uint32_t PSC=TIM2->PSC;
	uint32_t FREQ = PCLK1TIM_FREQ/(PSC+1);
	uint32_t CNT = (uint32_t)(FREQ* Ts)-1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, CNT);

	/*
	 * Setting the URS bit inside the TIMx->CR1 register: this will cause
that the UEV event is generated only when the counter reaches the overflow.
It is possible to configure the timer so that the ARR register is buffered,
by setting the TIM_CR1_ARPE bit in the TIMx->CR1 control register.
This will cause that the content of the shadow register is updated automatically
	 */
	TIM2->CR1 |= TIM_CR1_ARPE; //Enable preloading

	HAL_TIM_Base_Start_IT(&htim2);
	TIM2_AWAITING_UPDATE_EVENT=1;
}

static void TimerStop(void){
	HAL_TIM_Base_Stop_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2){
		TIM2_AWAITING_UPDATE_EVENT=0;
	}
}



void CMD_help(uint16_t nbArgs, int32_t *args ){
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	printf("CPLT\n");
}

void CMD_gohome(uint16_t nbArgs, int32_t *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%ld\n", args[n]);
	}
#endif
	__GoHome();
	printf("CPLT\n");
}

void CMD_G1(uint16_t nbArgs, char *argc, float *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%c %f\n", argc[n], args[n]);
	}
#endif
	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	int X=(+POS0+POS1)/2;
	int Y=(-POS0+POS1)/2;
	for (uint16_t n=0; n<nbArgs; n++){
		switch (argc[n]){
		case 'X':
			X=(int32_t) (SCALE*args[n]);
			break;
		case 'Y':
			Y=(int32_t) (SCALE*args[n]);
			break;
		}
	}

	printf("X%d Y%d\n", X, Y);
	__PenUp();
	__GoTo(X, Y);
	__PenDown();
	printf("CPLT\n");
}

void CMD_arc(uint16_t nbArgs, int32_t *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%ld\n", args[n]);
	}
#endif
	__Arc(args[0], args[1], args[2], args[3], args[4], args[5]);
	printf("CPLT\n");
}

void CMD_G2(uint16_t nbArgs, char* argc, float *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%c %f\n", argc[n], args[n]);
	}
#endif
	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	int Xstart=(+POS0+POS1)/2;
	int Ystart=(-POS0+POS1)/2;
	int Xend=Xstart;
	int Yend=Ystart;
	int I=0;
	int J=0;
	for (uint16_t n=0; n<nbArgs; n++){
		switch (argc[n]){
		case 'X':
			Xend=(int32_t) (SCALE*args[n]);
			break;
		case 'Y':
			Yend=(int32_t) (SCALE*args[n]);
			break;
		case 'I':
			I=(int32_t) (SCALE*args[n]);
			break;
		case 'J':
			J=(int32_t) (SCALE*args[n]);
			break;
		}
	}
	__ArcG2(Xstart, Ystart, Xend, Yend, I, J);
	printf("CPLT\n");
}

void CMD_G3(uint16_t nbArgs, char* argc, float *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%c %f\n", argc[n], args[n]);
	}
#endif
	int POS0=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(0), L6470_ABS_POS_ID));
	int POS1=AbsPos_2_Position(BSP_L6470_GetParam(0, L6470_ID(1), L6470_ABS_POS_ID));

	int Xstart=(+POS0+POS1)/2;
	int Ystart=(-POS0+POS1)/2;
	int Xend=Xstart;
	int Yend=Ystart;
	int I=0;
	int J=0;
	for (uint16_t n=0; n<nbArgs; n++){
		switch (argc[n]){
		case 'X':
			Xend=(int32_t) (SCALE*args[n]);
			break;
		case 'Y':
			Yend=(int32_t) (SCALE*args[n]);
			break;
		case 'I':
			I=(int32_t) (SCALE*args[n]);
			break;
		case 'J':
			J=(int32_t) (SCALE*args[n]);
			break;
		}
	}
	__ArcG3(Xstart, Ystart, Xend, Yend, I, J);
	printf("CPLT\n");
}

void CMD_G4(uint16_t nbArgs, char* argc, float *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%c %f\n", argc[n], args[n]);
	}
#endif
	double duration=0;
	for (uint16_t n=0; n<nbArgs; n++){
		switch (argc[n]){
		case 'P':
			duration=(double) (args[n])/1000.0;
			break;
		case 'S':
			duration=(double) (args[n]);
			break;
		}
	}
	Wait(duration);
	printf("CPLT\n");
}

void CMD_Null(uint16_t nbArgs, char *argc, float *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%c %f\n", argc[n], args[n]);
	}
#endif
printf("CPLT\n");
}

void CMD_Error(uint16_t nbArgs, char *argc, float *args ){
#ifdef NUCLEO_USE_USART
	printf("Cmd %s\n", CmdTable[cmdIndex].name);
	for (uint16_t n=0; n<nbArgs; n++){
		printf("%c %f\n", argc[n], args[n]);
	}
#endif
	Error_Handler();
}

/**
 * @}
 */ /* End of ExamplePrivateFunctions */

/**
 * @}
 */ /* End of Example */

/**
 * @}
 */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
