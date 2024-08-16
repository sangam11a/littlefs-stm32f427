/*
 * MAIN_CONFIG.h
 *
 *  Created on: Aug 23, 2022
 *      Author: jaddu
 */

/****************************************************************************************
 * SPI2 -> Extern ADC -> 5.25 MBits/s													*
 * SPI3 -> MAIN_FM    -> 21.0 MBits/s													*
 * SPI4 -> SMSN_FM	   -> 21.0 MBits/s													*
 * 																						*
 * UART1  -> COM_TCV_STREAM -> 115200 bps -> Interrupt									*
 * UART3  -> RESET_STREAM   -> 115200 bps -> Interrupt									*
 * UART6  -> MSN1_STREAM    -> 9600 bps												*
 * UART7  -> MSN2_STREAM	 -> 115200 bps												*
 * UART8  -> MSN3_STREAM	 -> 115200 bps												*
 * 																						*
 * ADC1   --> DMA Mode DMA2 (Half word data width) (480 Cycles sampling time)			*
 * Rank 1: RESET_3V3_CS (C_2)
 * Rank 2: BATT_CS_I  (C_3)																*
 * Rank 3: SolarP1_CS_I (C_4)															*
 * Rank 4: SolarP2_CS_I (C_5)															*
 * Rank 5: SolarP3_CS_I (C_6)															*
 * Rank 6: SolarP4_CS_I (C_7)															*
 * Rank 7: Vrefint_Channel																*
 * Rank 8: Internal_Temperature_Sensor_channel											*
 * 																						*
 * ADC2   --> DMA Mode DMA2 (Half word data width) (480 Cycles sampling time)			*
 * Rank 1: _3v31_CS_I (C_10)															*
 * Rank 2: _3v32_CS_I (C_11)															*
 * Rank 3: _5v_CS_I (C_12)																*
 * Rank 4: UNREG1_CS_I (C_13)															*
 * Rank 5: UNREG2_CS_I (C_14)															*
 * Rank 6: RAW_CS_I	(C_15)																*
 * Rank 7: SolarP5_CS_I (C_8)															*
 * Rank 8: SolarP_Total_CS_I (C_9)														*
 * TIM6 -> Base clock 42MHz  (use for RESET watchdog)									*
 * 	Pre-scaler 42																		*
 * 	Counter period 1000																	*
 * 	interrupt every 1 ms 																*
 * 																						*
 * TIM7 -> Base clock 42MHz																*
 * 	Pre-scaler 42																		*
 * 	Counter period (DEFAULT)															*
 * 																						*
 * 	use for microsecond Delay															*
 * 																						*
 * TIM9 -> Base Clock 84 MHz  (use for HK data collection timing)						*
 * 	Pre-scaler 8400																		*
 * 	Counter Period 10																	*
 * 																						*
 * 	Interrupt Every 1 ms																*
 ****************************************************************************************/
#pragma once

#ifndef INC_MAIN_CONFIG_H_
#define INC_MAIN_CONFIG_H_
#include "main.h"

#define DEBUG_MODE
//#define TRACE_MODE
#define ACTIVATE_INT_FLASH

//#define CDC_USB_DEBUG
#define UART_DEBUG
//#define BAUDRATE_4800_BPS
//#define BAUDRATE_2400_BPS
#define BAUDRATE_1200_BPS

//@note: choose between S_COM_MODE and ADDNICS_COM_MODE before uploading/generating code for MUNAL
//depending on the COM board configuration
//#define S_COM_MODE
#define S_COM_PLAN_F_MODE
//#define ADDNICS_COM_MODE

#define COM_TCV_STREAM 			huart1
#define DEBUG_STREAM			huart7

extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

#define E_ADC					hspi2
#define MAIN_FM					hspi3
#define SMSN_FM					hspi4
//
//#define	I_ADC1					hadc1
//#define I_ADC2					hadc3

//#define RESET_TIMER				htim6
//#define uS_Delay_TIMER			htim7
//#define HK_TIMER				htim9

#define HEADER 					0xBA
#define FOOTER					0xFE

#define RESET_CMD_LEN			6

#ifdef S_COM_MODE
#define AX25_HEADER_FOOTER		0x42
#define	AX25_CONTROL			0x3E
#define	AX25_PID				0xF0

#define COM_CHECKSUM_PASS		0x50
#define COM_CHECKSUM_FAIL		0x46
#define COM_ACK					0xACAD
#define COM_RX_DATA_BUFFER_LEN	33		// length of buffer for receiving data from Transciever Board


#define COM_TX_DATA_BUFFER_LEN	100		// size of one packed of data transmitted
#define PACKET_DATA_LEN			80		// no of bytes of data in one packet of data transmitted to GS

#endif

#ifdef ADDNICS_COM_MODE

#define COM_TX_DATA_BUFFER_LEN	100
#define PACKET_DATA_LEN			90

#define AX25_HEADER_FOOTER		0x42
#define	AX25_CONTROL			0x3E
#define	AX25_PID				0xF0

#define COM_CHECKSUM_PASS		0x50
#define COM_CHECKSUM_FAIL		0x46

#define COM_ACK					0xACAD
#define COM_RX_DATA_BUFFER_LEN	22		// length of buffer for receiving data from Transceiver Board
#endif

#define CMD_LEN					13		// length of pc command in bytes

#ifdef S_COM_PLAN_F_MODE

#define COM_TX_DATA_BUFFER_LEN	100
#define PACKET_DATA_LEN			90

#define AX25_HEADER_FOOTER		0x42
#define AX25_CONTROL			0x3E
#define AX25_PID				0xF0

#define COM_CHECKSUM_PASS		0x50
#define COM_CHECKSUM_FAIL		0x46

#define COM_ACK					0xACAD
#define COM_RX_DATA_BUFFER_LEN	16

#endif	//S_COM_PLAN_F_MODE

#define SAT_ID					0x4D		//FM_A

#define HK_DATA_WAIT_DELAY		10000

#define WDG_REFRESH_INTERVAL	60		// refresh Watch-dog every 60 Sec

#ifdef BAUDRATE_4800_BPS
#define	HK_INTERVAL				90		// collect housekeeping data every 90 sec, if Baudrate of com board is set to 4800bps
#endif//BAUDRATE_4800_BPS

#ifdef BAUDRATE_2400_BPS
#define HK_INTERVAL				90		// collect housekeeping data every 180 sec if baudrate of com board is set to 2400bps
#endif //BAUDRATE_2400_BPS

#ifdef BAUDRATE_1200_BPS
#define HK_INTERVAL				90
#endif	//BAUDRATE_1200_BPS

#define BEACON_RTR_INTERVAL		90		// retransmit Beacon every 120 sec
#define FM_SPI_SPEED			21		// in MBits/s
#define	UART_MAX_TIMEOUT		5000	// maximum timeout for UART in ms

#define	DATA_BUFFER_LEN			256		// length of data buffer to receive data onto (size of one page of flash memory)
#define RESET_DATA_TX_LEN		26		//length of data buffer to receive from Reset STM
#define RESET_DATA_RX_LEN		18		//length of ack data received from Reset

#define RESET_DATA_LEN			RESET_DATA_RX_LEN
#define RESET_DATA_ACK_LEN		RESET_DATA_RX_LEN

#define FLAG_DATA_LEN			8		//length of data buffer to store the flag data(flag data size)
#define HK_DATA_LEN				80		//size of HK data
#define BEACON_DATA_LEN			41		//size of Beacon data to transmit
#define SAT_HEALTH_LEN			40

#define MSN_TX_BUFFER_LEN		13		//length of the data to be sent to the CAM MSN MCU
#define ACK_LEN					9
#define SAT_LOG_LEN				13

#define I_ADC1_DATA_LEN			7
#define I_ADC2_DATA_LEN			7
#define E_ADC_0_DATA_LEN		8
#define E_ADC_1_DATA_LEN		12
#define ADC_LOOP_MAX			200

#define SOLAR_STATUS_DATA_LEN	20

#define ANT_DEP_TIME			30 		// antenna deployment time [after satellite deploys from POD [in Mins]

#define MSN_RUN_TIME			300000 		// MSN operation timeout in mSec

/*
 * formula for Current sensor value conversion C = V /(2*R*G) for LMP8640 sensor
 */
#define RES_LMP8640				0.025
#define GAIN_LMP8640			10

/*
 * formula for Current Sensor value conversion C=(V-1.65) / SENS for TMCS1108 sensor
 */
#define SENS_TMCS				264

#define SECTOR_SIZE				65536
#define ADDRESS_DATA_SIZE		8*4+4

#define RSV_TABLE_SIZE			50

//first sector is for address data address
#define ADDRESS_DATA_LOC_ADDRESS			SECTOR_SIZE*1

#define FLAG_DATA_START_ADDRESS				SECTOR_SIZE*4
#define HK_DATA_START_ADDRESS				SECTOR_SIZE*10		//we are starting the sector number from
#define SAT_LOG_START_ADDRESS				SECTOR_SIZE*211
#define RSV_TABLE_START_ADDRESS				SECTOR_SIZE*612
#define PACDS_MSN_DATA_START_ADDRESS		SECTOR_SIZE*623
#define SPDM_MSN_DATA_START_ADDRESS			SECTOR_SIZE*724
#define	CAM_MSN_DATA_START_ADDRESS			SECTOR_SIZE*765
#define CAM_MSN_IMG_INFO_START_ADDRESS		SECTOR_SIZE*1616

#define ADDRESS_DATA_START_ADDRESS			SECTOR_SIZE*2039

#define FLAG_DATA_END_ADDRESS				SECTOR_SIZE*8
#define HK_DATA_END_ADDRESS					SECTOR_SIZE*209
#define SAT_LOG_END_ADDRESS					SECTOR_SIZE*610
#define RSV_TABLE_END_ADDRESS				SECTOR_SIZE*621
#define PACDS_MSN_DATA_END_ADDRESS			SECTOR_SIZE*722
#define SPDM_MSN_DATA_END_ADDRESS			SECTOR_SIZE*763
#define	CAM_MSN_DATA_END_ADDRESS			SECTOR_SIZE*1614
#define CAM_MSN_IMG_INFO_END_ADDRESS		SECTOR_SIZE*1617
#define ADDRESS_DATA_END_ADDRESS			SECTOR_SIZE*2043

// satellite HouseKeeping Data Collection structure
typedef struct __attribute__((__packed__)) _SAT_HEALTH_TypeDef {
	uint8_t HEAD;				// Header for Starting Data
	uint16_t TIM_DAYS;// No of days since the deployment of satellite from Rocket
	uint8_t TIM_HOUR;			// Hour since last global reset
	uint8_t TIM_MIN;			// Minutes
	uint8_t TIM_SEC;			// Seconds

	uint8_t ANT_DEP_STAT;// Antenna deployment status flag; indicates the no of time antenna deployment was attempted, will attempt every 24 hrs unless UPLINK_STAT_FLAG is set, 5 times every attempt max
	uint8_t UL_STAT;// Command Up-link Status flag, will be set once,[0xAE - Up-link successful]
	uint8_t OPER_MODE;// Current Operational_Power mode of the satellite [0x5A / 0x6A / 0x7A]
	uint8_t MSN_FLAG;// FLag determining Status of MSN [if any MSN in operation FLAG will be 0xAE]
	uint8_t RSV_FLAG;// Flag Determining Status of RSV_CMD [if RSV executing, flag will be 0xAE]
	uint8_t MSN3_STAT;		// If SPDM is running flag will be 0xAE else 0x00
	uint8_t MSN2_STAT;		// If PACDS is running flag will be 0xAE else 0x00
	uint8_t MSN1_STAT;			// If CAM is running flag will be 0xAE else 0x00
	uint8_t KILL_SWITCH;

	uint8_t RSV_CMD;			// no of reservation commands stored in table
	uint16_t BATT_VOLT;			// Current Voltage of Battery in mV
	int16_t BATT_TEMP;		// Temperature of Battery [signed integer format]
	int16_t BAT_C;		// Current drawn from Battery

	// Solar panel current in mA
	int16_t SOL_P1_C;
	int16_t SOL_P2_C;
	int16_t SOL_P3_C;
	int16_t SOL_P4_C;
	int16_t SOL_P5_C;
	int16_t SOL_T_C;
	int16_t RST_3V3_C;
	int16_t RAW_C;
	uint16_t _3V3_1_C;
	uint16_t _3V3_2_C;
	uint16_t _5V_C;
	uint16_t UNREG1_C;
	uint16_t UNREG2_C;

	// solar panel voltages in mV
	uint16_t SOL_P1_V;
	uint16_t SOL_P2_V;
	uint16_t SOL_P3_V;
	uint16_t SOL_P4_V;
	uint16_t SOL_P5_V;
	uint16_t SOL_T_V;
	uint16_t RAW_V;

	// all temperature multiplied by 100
	int16_t Y_TEMP;		//+Y Panel
	int16_t Y1_TEMP;	//-Y Panel
	int16_t X_TEMP;		//+X Panel
	int16_t X1_TEMP;	//-X Panel
	int16_t Z_TEMP;		//+Z Panel
	int16_t Z1_TEMP;	//-Z Panel
	int16_t BPB_TEMP;
	int16_t OBC_TEMP;
	int16_t COM_TEMP;

	int16_t ACCL_X;
	int16_t ACCL_Y;
	int16_t ACCL_Z;
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	int16_t MAG_X;
	int16_t MAG_Y;
	int16_t MAG_Z;

	uint16_t OBC_ABNL_RST;
	uint16_t RST_ABNL_RST;
	uint8_t RST_TIME;			// Time after last OBC Reset [in Hrs]
	uint16_t CHK_SUM;			// Checksum for data consistency
} SAT_HEALTH_TypeDef;

// beacon data structure
typedef struct __attribute__((__packed__)) _SAT_BEACON_TYPE_A {
	/*1-byte*/uint8_t HEAD;
	/*4-bit*/uint8_t TYPE :4;
	/*12-bit*/unsigned int TIM_DAY :12;
	/*1-byte*/uint8_t TIM_HOUR;

	/*2-byte*/uint16_t BAT_V;  		// in mV scale
	/*2-byte*/int16_t BAT_C;		// in mA scale
	/*2-byte*/int16_t BAT_T;// stored as integer value in Degree Centigrade scale multiplied by 10. [42.4 as 424]
	/*1-byte*/int8_t RAW_C;	// stored up-to 2 SF value multiplied by 10 to the Ampere scale [ 1.2A as 12]
	/*2-byte*/uint16_t SOL_TOT_V;	// in mV scale
	/*2-byte*/int16_t SOL_TOT_C;	// in mA scale

	/*1-byte*/int8_t BPB_T;
	/*1-byte*/int8_t OBC_T;
	/*1-byte*/int8_t Y1_T;
	/*1-byte*/int8_t Y_T;
	/*1-byte*/int8_t Z1_T;
	/*1-byte*/int8_t Z_T;
	/*1-byte*/int8_t X1_T;
	/*1-byte*/int8_t X_T;// stored in integer format same as BATT_TEMP, in Centigrade scale

	/*1-bit*/uint8_t SOL_P1_STAT :1;
	/*1-bit*/uint8_t SOL_P2_STAT :1;
	/*1-bit*/uint8_t SOL_P3_STAT :1;
	/*1-bit*/uint8_t SOL_P4_STAT :1;
	/*1-bit*/uint8_t SOL_P5_STAT :1;
	/*1-bit*/uint8_t MSN1_STAT :1;
	/*1-bit*/uint8_t MSN2_STAT :1;
	/*1-bit*/uint8_t MSN3_STAT :1;

	/*3-bit*/uint8_t ANT_STAT :3;
	/*1-bit*/uint8_t KILL_1_STAT :1;
	/*1-bit*/uint8_t KILL_2_STAT :1;
	/*3-bit*/uint8_t UL_STAT :3;

	/*1-byte*/uint8_t OPER_MODE;

	/*2-byte*/uint16_t OBC_RESET_COUNT;
	/*2-byte*/uint16_t RESET_RESET_COUNT;
	/*1-byte*/uint8_t LAST_RESET;
	/*1-byte*/uint8_t CHK_CRC;
} SAT_BEACON_TYPE_A;

typedef struct __attribute__ ((__packed__)) _SAT_BEACON_TYPE_B {
	/*1-byte*/uint8_t HEAD;
	/*4-bit*/uint8_t TYPE :4;
	/*12-bit*/unsigned int TIM_DAY :12;

	/*1-byte*/uint8_t SOL_P1_V; // voltage upto 1dp, without decimal
	/*1-byte*/uint8_t SOL_P2_V;
	/*1-byte*/uint8_t SOL_P3_V;
	/*1-byte*/uint8_t SOL_P4_V;
	/*1-byte*/uint8_t SOL_P5_V;

	/*1-byte*/int8_t SOL_P1_C;
	/*1-byte*/int8_t SOL_P2_C;
	/*1-byte*/int8_t SOL_P3_C;
	/*1-byte*/int8_t SOL_P4_C;
	/*1-byte*/int8_t SOL_P5_C;

	/*2-byte*/int16_t GYRO_X;
	/*2-byte*/int16_t GYRO_Y;
	/*2-byte*/int16_t GYRO_Z;
	/*2-byte*/int16_t ACCL_X;
	/*2-byte*/int16_t ACCL_Y;
	/*2-byte*/int16_t ACCL_Z;

	/*2-byte*/int16_t MAG_X;
	/*2-byte*/int16_t MAG_Y;
	/*2-byte*/int16_t MAG_Z;
	/*1-byte*/uint8_t CHK_CRC;
} SAT_BEACON_TYPE_B;

// reset communication data set definition
typedef struct __attribute__((__packed__)) _RSTRx_DATASET_TypeDef {
	uint8_t HEAD;						// HEADER of incoming data packet [0xBA]
	uint16_t TIM_DAYS;// No of days since the deployment of satellite from Rocket
	uint8_t TIM_HOUR;					// Hour since last global reset
	uint8_t TIM_MIN;					// Minutes
	uint8_t TIM_SEC;					// Seconds
	uint8_t GBL_RST_STAT;// GLOBAL_RESET TIME indicator [0x5C GLOBAL_RESET Time]
	uint16_t OBC_ABN_RST;				// no of abnormal resets of OBC STM
	uint16_t RST_ABN_RST;				// no of abnormal resets of Reset STM
	uint8_t ANT_STAT_FLAG;// Antenna deployment status flag; indicates the no of time antenna deployment was attempted, will attempt every 24 hrs unless UPLINK_STAT_FLAG is set, 5 times every attempt max
	uint8_t UL_STAT;// Command Up-link Status flag, will be set once,[0xAE - Uplink successful]
	uint8_t OPER_MODE;// Current Operational_Power mode of the satellite [0x5A / 0x6A / 0x7A]
	uint8_t ACK_BYTE;// Acknowledgement byte of Previous Data [0xAC - correct Data / 0xEE - Error Data]
	uint16_t CHK_SUM;					// Checksum Data
	uint8_t FOOT;					// FOOTER of the incoming data packet [0xFE]
} RSTRx_DATASET_TypeDef;

// structure defining the transmission packet type to RESET
typedef struct __attribute__((__packed__)) _RSTTx_DATASET_TypeDef {
	uint8_t HEAD;	//1 HEADER of incoming data packet [0xBA]
	uint16_t TIM_DAYS;//3 No of days since the deployment of satellite from Rocket
	uint8_t TIM_HOUR;	//4 Hour since last global reset
	uint8_t TIM_MIN;	//5 Minutes
	uint8_t TIM_SEC;	//6 Seconds
	uint8_t CMD;		//7 Commands for Reset
	uint8_t ANT_DEP_STAT;//8 Antenna deployment status flag; indicates the no of time antenna deployment was attempted, will attempt every 24 hrs unless UPLINK_STAT_FLAG is set, 5 times every attempt max
	uint8_t UL_STAT;//9 Command Up-link Status flag, will be set once,[0xAE - Uplink successful]
	uint8_t OPER_MODE;//10 Current Operational_Power mode of the satellite [0x5A / 0x6A / 0x7A]
	uint8_t MSN_FLAG;//11 FLag determining Status of MSN [if any msn in operation FLAG will be 0xAE]
	uint8_t RSV_FLAG;//12 Flag Determining Status of RSV_CMD [if RSV executing, flag will be 0xAE]
	uint8_t MSN3_STAT;	//13 If SPDM is running flag will be 0xAE else 0x00
	uint8_t MSN2_STAT;	//14 If PACDS is running flag will be 0xAE else 0x00
	uint8_t MSN1_STAT;	//15 If CAM is running flag will be 0xAE else 0x00
	uint8_t BATT_VOLT;//16 Current Voltage of Battery [voltage will be multiplied by 10]
	int8_t BATT_TEMP;	//17 Temperature of Battery [signed integer format]
	uint8_t SOL_1_STAT;	//18 State of solar panel 1 [if active byte will be 0xAE]
	uint8_t SOL_2_STAT;	//19 State of solar panel 2 [if active byte will be 0xAE]
	uint8_t SOL_3_STAT;	//20 State of solar panel 3 [if active byte will be 0xAE]
	uint8_t SOL_4_STAT;	//21 State of solar panel 4 [if active byte will be 0xAE]
	uint8_t SOL_5_STAT;	//22 State of solar panel 5 [if active byte will be 0xAE]
	uint8_t RSV_CMD;		//23 no of reservation commands stored in table
	uint16_t CHK_SUM;		//25 Checksum Data
	uint8_t FOOT;			//26 FOOTER of the incoming data packet [0xFE]
} RSTTx_DATASET_TypeDef;

//enumeration of commands to send to RESET_MCU
typedef enum _RST_CMD {
	RESET_FLAG_TYPE = 0x21, TURN_ON_3V3_2 = 0x32,		//	MSN Power BUS
	TURN_OFF_3V3_2 = 0x35,
	TURN_OFF_5V = 0x45,		// CAMERA Power BUS
	TURN_ON_5V = 0x50,
	TURN_ON_UNREG_2 = 0x1A,		// COM Power BUS
	TURN_OFF_UNREG_2 = 0x2A,
	TURN_ON_UNREG_1 = 0x3A,		// BURNER POWER BUS
	TURN_OFF_UNREG_1 = 0x4A,
	OPER_MODE_1 = 0x5A,		// NORMAL_MODE
	OPER_MODE_2 = 0x6A,		// LOW_PWR_MODE
	OPER_MODE_3 = 0x7A,		// SAFE_MODE
	OPER_MODE_4 = 0x8A,		// SAT_KILL_MODE
	RST_SAT = 0xAF,		// RESET_SATELLITE
	RST_FLAG_RST = 0xEF,	//RESET_ALL_FLAGS
} RST_CMD;

typedef enum _OPERA_MODES {
	NRML_MODE = 0x5A,
	LOW_PWR_MODE = 0x6A,
	SAFE_MODE = 0x7A,
	SAT_KILL_MODE = 0x8A,
} OPERA_MODES;

typedef enum _SYS_STATE {
	ACTIVE = 0xAE, INACTIVE = 0x00,
} SYS_STATE;

typedef struct _SOLAR_PWR_GEN {
	uint16_t SOLAR_VOLT[5];		// voltage of solar panels [1-5]
	uint16_t SOLAR_CURRENT[5];
} SOLAR_PWR_GEN;

// structure defining the essential critical flags during configuration of power mode
typedef struct __attribute__((__packed__)) __CRITIC_FLAG {
	uint8_t MSN1_EN_FLAG :1;
	uint8_t MSN2_EN_FLAG :1;
	uint8_t MSN3_EN_FLAG :1;
	uint8_t RSV_MSN_EXE_FLAG :1;
	uint8_t COM_EN_FLAG :1;
	uint8_t LOG_EN_FLAG :1;
	uint8_t TEL_TX_FLAG :1;
	uint8_t FM_EN_FLAG :1;
} CRITIC_FLAG;

typedef enum _STATE {
	ON = 0x01, OFF = 0x00,
} STATE;

/*
 * @brief	enumeration definition for ACK STATES
 */
typedef enum _ACK_STATE {
	ACK_SUCCESS = 0xAC, ACK_FAILURE = 0xEE,
} ACK_STATE;

/*
 * @brief enumeration definition for ANTENNA deployment states
 */
typedef enum _ANT_STATE {
	UNDEPLOYED = 0x00, DEPLOY_NOW = 0XAE, DEPLOYED = 0xDE,
} ANT_STATE;

/*
 * @brief enumeration definition for Uplink status
 */
typedef enum _UL_STATE {
	UL_NOT_RX = 0x00, UL_RX = 0xAE,
} UL_STATE;

/*
 * @brief	enumeration definition for status of Reservation commands
 */
typedef enum _RSV_CMD_STATE {
	RSV_NOT_RUNNING = 0x00, RSV_RUNNING = 0xAE,
} RSV_CMD_STATE;
#endif /* INC_MAIN_CONFIG_H_ */
