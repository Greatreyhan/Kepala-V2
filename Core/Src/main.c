/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

//*************************** CONFIGURATIONS **********************************//
#define DETEKSI_KORBAN_PID
#define WALL_FOLLOWER_PID
#define USE_STABILIZE_PID
#define DETEKSI_SF_PID

//*************************** PARAMETERS **********************************//
#define SPEED_WALKING						10
#define SAMPLE_TIME_S 					0.01f
// Luas pembacaan diperoleh dari (luas arena - luas robot bagian luar) = 45 - 33 = 12
#define LEBAR_PEMBACAAN 				12.00
// Panjang ping kaki diperoleh dari pengukuran jarak dari ping ke kaki bagian dalam
#define PANJANG_PING_KAKI 			9.00
// Mengetahui Jarak terhadap tembok yang ada di depan (perlu diperhitungkan terkait dengan daerah minimal yang diperlukan untuk melakukan rotasi
#define BATAS_DEPAN 						9.00

//*************************** DEBUG MODE **********************************//
//#define TEST_HUSKY
//#define TEST_DYNA
//#define TEST_PING
//#define TEST_COMMUNICATION
//#define MAIN_PROGRAM
#define MAIN_PROGRAM_STACK
//#define TEST_BENCH

//*************************** VARIABEL JALAN ********************************//
//#define STEP_0
//#define STEP_1
//#define STEP_2
//#define STEP_3
//#define STEP_4
//#define STEP_5
#define STEP_6
//#define STEP_7
//#define STEP_8
//#define STEP_9
//#define STEP_10
//#define STEP_11
//#define STEP_12
//#define STEP_13

//*************************** MODE KALKULASI ********************************//			
#define USE_PID								

//*************************** ON/OFF MODUL ********************************//
#define HUSKY_ON 					
#define DYNA_ON 					
#define PING_ON						
#define COMMUNICATION_ON	

//*************************** FILE EKSTERNAL ******************************//
#ifdef PING_ON
#include "Ping_driver.h"
#include "DWT_Delay.h"
#endif

#ifdef COMMUNICATION_ON
#include "Komunikasi.h"
#endif

#ifdef HUSKY_ON
#include "Huskylens_driver.h"
#endif

#ifdef DYNA_ON
#include "Dynamixel.h"
#endif

#ifdef USE_PID
#include "PID_driver.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Filter untuk sensor PING
//#define FILTER_KAKI
#define FILTER_AVG

// State untuk deteksi Wall
#define STATE_KANAN 						0x01
#define STATE_KIRI 							0x02
#define STATE_DOUBLE						0x03
#define STATE_DEPAN 						0x04
#define STATE_BELAKANG 					0x05

// State untuk belok
#define BELOK_KANAN 						0x01
#define BELOK_KIRI 							0x02

// State untuk mode jalan
#define MODE_MENDAKI 						0x01
#define MODE_MELEWATI_KELERENG 	0x02
#define MODE_MENURUN 						0x03
#define MODE_MENCARI_KORBAN 		0x04

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum{
	KOSONG_KANAN = 0x01U,
	KOSONG_KIRI = 0x02U
}home_typedef_t;

typedef enum{
	DIRECTION_KANAN = 0x01U,
	DIRECTION_KIRI = 0x02U,
	DIRECTION_DEPAN = 0x03U,
	DIRECTION_BELAKANG = 0x04U
}follower_direction_t;

huskylens_area_identification_t running_arena = HOME;
home_typedef_t home_direction = 0x00;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//****************************** CONFIG  HUSKYLENS *******************************//
#ifdef HUSKY_ON
huskylens_info_t huskAll;
huskylens_status_t status;
huskylens_arrow_t arrows;
huskylens_block_t blocks;
huskylens_all_byid_t id;
double husky_distance = 0;
uint16_t algorithm_type = ALGORITHM_NOT_FOUND;
#endif

//****************************** CONFIG  PING *******************************//
#ifdef PING_ON
Ping_t FR;
Ping_t BR;
Ping_t FL;
Ping_t BL;
Ping_t FF;
Ping_t BB;
Ping_t CP;

double kanan = 0,kiri = 0, depan = 0, belakang = 0, capit = 0;
double FRV,BRV,FLV,BLV,FFV,BBV, CPV;

#endif

//****************************** CONFIG  DYNAMIXEL *******************************//
#ifdef DYNA_ON
dynamixel_t ax;
uint16_t dyna_sudut = 0;
#endif

//***************************** CONFIG PID **********************************//
#ifdef USE_PID
// PID pencarian korban
PIDController pid_pk;
float setpoint_pk = 160;

// PID deteksi korban
PIDController pid_dk;
float setpoint_dk = 160;

// PID Wall Follower
PIDController pid_wf;
float setpoint_wf = 4;
float setpoint_wf_y = 8;

// PID Stabilizer
PIDController pid_st;
float setpoint_st = 0;

// PID Jalan Korban Follower
PIDController pid_kf;
float setpoint_kf = 512;

// PID Rotasi Korban
PIDController pid_rk;
float setpoint_rk = 512;

// PID AXIS
PIDController pid_axis_x; PIDController pid_axis_y;
#endif

//**************************** CONFIG COMMUNICATION ********************************//
#ifdef COMMUNICATION_ON
feedback_t feeding;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// reset
	feeding.jalan = 0x00;
	feeding.ping = 0x00;
	feeding.req = 0x00;
	feeding.rotasi = 0x00;
	feeding.standby = 0x00;
	feeding.statis = 0x00;
	feeding.translasi = 0x00;
	feeding.capit = 0x00;
	feeding.serok = 0x00;
	rx_feedback(&feeding);
}
#endif
void pid_run(uint8_t state_jalan, double input, follower_direction_t dir);
void pid_run_batu(uint8_t state_jalan, double input, follower_direction_t dir);

//****************************** PROTOTYPE ACTION  ************************************//
void scp_wall_follower(uint8_t state, follower_direction_t direction_play);
void scp_wall_follower_batu(uint8_t state, follower_direction_t direction_play);
void scp_belok(uint8_t direction, uint16_t time);
void scp_wall_stabilizer(uint8_t state);
bool scp_deteksi_korban(uint8_t id, follower_direction_t dir);
void scp_deteksi_arena(uint8_t id);
void scp_mode_jalan(mode_jalan_t mode);
void scp_pengangkatan_korban(void);
void scp_penurunan_korban(void);
void scp_kepala_move(dynamixel_kepala_direction_t dir);
bool scp_korban_follower(uint8_t id, follower_direction_t dir);
bool scp_korban_follower_rotasi(uint8_t id, follower_direction_t dir);
void scp_wall_follower_belakang(void);
void scp_ambil_korban(void);
void scp_turun_korban(void);
bool scp_korban_follower_statis(uint8_t state);
bool scp_korban_follower_kepala(void);
bool scp_axis_stabilizer(int16_t x_axis, int16_t y_axis, uint8_t state_x, uint8_t state_y );
void ping_clear(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	//*********************** COMMUNICATION CONFIG **************************//
	#ifdef COMMUNICATION_ON
	komunikasi_init(&huart2);
	rx_start();
	tx_move_steady();
	tx_capit(HOME_CAPIT, 0x00, 20);
	HAL_Delay(2000);
	while(!feeding.statis){
		tx_statis(80,70,-80); 
		HAL_Delay(10);
	}
	
	#endif
	
	//********************** Config For Huskylens *************************/
	#ifdef HUSKY_ON
	if(husky_setup(&hi2c2) == HUSKY_OK){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
	
	huskAll = husky_getAllArrowBlock();
	#endif
	
	//********************** Config For PING *************************/
	#ifdef PING_ON
	DWT_Delay_Init();
	
	// PING 1 -> Depan Kanan
	FR.PING_PORT = GPIOE;
	FR.PING_PIN = GPIO_PIN_5;
	// IR1 2 -> Belakang Kanan
	BR.PING_PORT = GPIOB;
	BR.PING_PIN = GPIO_PIN_0;
	// PING 3 -> Belakang Kiri
	BL.PING_PORT = GPIOB;
	BL.PING_PIN = GPIO_PIN_9;
	// PING 4 -> Capit
	CP.PING_PORT = GPIOB;
	CP.PING_PIN = GPIO_PIN_8;
	// PING 6 -> Depan Kiri
	FL.PING_PORT = GPIOB;
	FL.PING_PIN = GPIO_PIN_4;
	// PING 7 -> Depan
	FF.PING_PORT = GPIOB;
	FF.PING_PIN = GPIO_PIN_3;
	// PING 5 -> Belakang
	BB.PING_PORT = GPIOB;
	BB.PING_PIN = GPIO_PIN_5;
	#endif
	
	//*********************** DYNAMIXEL CONFIG **************************//
	#ifdef DYNA_ON
	dyna_init(&huart4, &ax, 0x11);
	dyna_calibrate(&ax);
	#endif
	
	//*********************** PID CONFIG **************************//
	#ifdef USE_PID
	// PID untuk Pencarian korban 
	pid_pk.Kp = 1.1; 				pid_pk.Ki = 0.2; 				pid_pk.Kd = 0; 					pid_pk.tau = 0.02;
	pid_pk.limMax = 20; 		pid_pk.limMin = -20; 		pid_pk.limMaxInt = 5; 	pid_pk.limMinInt = -5;
	pid_pk.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_pk);
	
	// PID untuk Deteksi Korban
	pid_dk.Kp = 1; 				pid_dk.Ki = 0; 				pid_dk.Kd = 0.1; 					pid_dk.tau = 0.02;
	pid_dk.limMax = 20; 		pid_dk.limMin = -20; 		pid_dk.limMaxInt = 5; 	pid_dk.limMinInt = -5;
	pid_dk.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_dk);
	
	// PID untuk Wall Follower
	pid_wf.Kp = 2; 				pid_wf.Ki = 0; 				pid_wf.Kd = 0.1; 					pid_wf.tau = 0.02;
	pid_wf.limMax = 10; 	pid_wf.limMin = -10; 	pid_wf.limMaxInt = 5; 	pid_wf.limMinInt = -5;
	pid_wf.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_wf);
	
	// PID untuk Stabilizer
	pid_st.Kp = 2; 				pid_st.Ki = 0; 				pid_st.Kd = 0.1; 					pid_st.tau = 0.02;
	pid_st.limMax = 20; 	pid_st.limMin = -20; 	pid_st.limMaxInt = 5; 	pid_st.limMinInt = -5;
	pid_st.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_st);
	
	// PID untuk Jalan Follower Korban
	pid_kf.Kp = 2; 				pid_kf.Ki = 0; 				pid_kf.Kd = 1; 					pid_kf.tau = 0.02;
	pid_kf.limMax = 20; 	pid_kf.limMin = -20; 	pid_kf.limMaxInt = 5; 	pid_kf.limMinInt = -5;
	pid_kf.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_kf);
	
	// PID untuk Rotasi Korban
	pid_rk.Kp = 2; 				pid_rk.Ki = 0.5; 				pid_rk.Kd = 0; 					pid_rk.tau = 0.02;
	pid_rk.limMax = 20; 	pid_rk.limMin = -20; 	pid_rk.limMaxInt = 5; 	pid_rk.limMinInt = -5;
	pid_rk.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_rk);
	
	// PID untuk Axis X
	pid_axis_x.Kp = 2; 				pid_axis_x.Ki = 0.1; 				pid_axis_x.Kd = 0; 				pid_axis_x.tau = 0.02;
	pid_axis_x.limMax = 20; 	pid_axis_x.limMin = -20; 	pid_axis_x.limMaxInt = 5; 	pid_axis_x.limMinInt = -5;
	pid_axis_x.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_axis_x);
	
	// PID untuk Axis Y
	pid_axis_y.Kp = 2; 				pid_axis_y.Ki = 0.1; 				pid_axis_y.Kd = 0; 				pid_axis_y.tau = 0.02;
	pid_axis_y.limMax = 20; 	pid_axis_y.limMin = -20; 		pid_axis_y.limMaxInt = 5; 	pid_axis_y.limMinInt = -5;
	pid_axis_y.T_sample = SAMPLE_TIME_S;
	PIDController_Init(&pid_axis_y);
	#endif
	
	//**********************======================*************************//
	//**********************= ONE STOP ALGORITHM =*************************//
	//**********************======================*************************//
	#ifdef MAIN_PROGRAM_STACK
	
	//----- TEST STEP
	#ifdef STEP_0
	while(1){
		scp_korban_follower_statis()
	}
	#endif
	
	//----- Home -----
	//1. Identifikasi Arah Pergerakan
	#ifdef STEP_1
	while(1){
		// Pembacaan Sensor PING
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		
		if((FLV < 15) && (FLV > 1)){
			home_direction = KOSONG_KANAN;
			scp_kepala_move(KEPALA_BELAKANG);
			break;
		}
		else if((FRV < 15) && (FRV > 1)){
			home_direction = KOSONG_KIRI;
			scp_kepala_move(KEPALA_DEPAN);
			break;
		}
	}
	#endif
	
	//2. Bergerak sesuai konfigurasi Start
	#ifdef STEP_2
	while(1){
		if(scp_deteksi_korban(1, DIRECTION_DEPAN)) break;
		else if(home_direction == KOSONG_KANAN) scp_wall_follower(STATE_DEPAN, DIRECTION_KANAN);
		else if(home_direction == KOSONG_KIRI) scp_wall_follower(STATE_BELAKANG, DIRECTION_KIRI);
	}
	#endif
	
	//3. Pencarian korban K1
	#ifdef STEP_3
	bool status = false;
	while(1){
		status = scp_korban_follower_statis();
		if(status == true){
			break;
		}
	}
	#endif
	
	// 4. Pengangkatan Korban
	#ifdef STEP_4
		while(1){
			while(1){
				BBV = ping_read(BB);
				if(BBV > 1){
					if(BBV <= 16) tx_move_jalan(0, 10, 30, SPEED_WALKING, JALAN_NORMAL, 10);
					else{
						tx_move_steady();
						break;
					}
				}
			}
			scp_korban_follower_rotasi(1, DIRECTION_KANAN);
			scp_kepala_move(KEPALA_BELAKANG);
			tx_capit(HOME_CAPIT, 0x00, 15);
			tx_capit(AMBIL_KORBAN, 0x00, 15); 
			HAL_Delay(3000);
			dyna_calibrate(&ax);
			HAL_Delay(500);
			tx_move_steady();
			tx_statis(80,70,-100); 
			tx_move_steady();
			break;
		}
	#endif
	
	// 5. Mundur Sesuai dengan batas belakang
	#ifdef STEP_5
	while(1){
			HAL_Delay(10);
			BBV = ping_read(BB);
			if(BBV > 1){
				if(BBV >= 13) tx_move_jalan(0, -10, 30, SPEED_WALKING, JALAN_NORMAL, 10);
				else{
					tx_move_steady();
					break;
				}
			}
	}
	#endif
	
	// 6. Linear moving untuk melewati obstacle jalan pecah dan menurun
	#ifdef STEP_6
	while(!feeding.statis){
		tx_statis(80,70,-100); 
		HAL_Delay(10);
	}
	while(1){
		ping_clear();
		FFV = ping_read(FF);
		FLV = ping_read(FL);
		BLV = ping_read(BL);
		BBV = ping_read(BB);
		if(FFV > 20 && FLV < 10 && BLV < 10 && BBV < 12){
			tx_move_steady();
			break;
		}
		else {
			scp_wall_follower_belakang();
		}
	}
	
	while(1){
			ping_clear();
			HAL_Delay(1);
			BBV = ping_read(BB);
			if(BBV <= 15) tx_move_jalan(0, 10, 60, SPEED_WALKING, JALAN_NORMAL, 10);
			else{
				scp_kepala_move(KEPALA_KANAN_DEPAN);
				HAL_Delay(500);
				tx_capit(HOME_CAPIT, 0x00, 15);
				tx_capit(AMBIL_KORBAN, 0x00, 15); 
				HAL_Delay(5000);
				scp_kepala_move(KEPALA_KANAN);
				tx_move_steady();
				break;
			}
	}
	
	scp_wall_stabilizer(STATE_KIRI);
	
	while(1){
		ping_clear();
		FFV = ping_read(FF);
		FLV = ping_read(FL);
		BLV = ping_read(BL);
		BBV = ping_read(BB);
		if(FFV < 50){
			tx_move_steady();
			break;
		}
		else {
			scp_wall_follower_batu(STATE_KIRI, DIRECTION_DEPAN);
		}
	}
	
	while(!scp_korban_follower_statis(STATE_DEPAN)){
		HAL_Delay(10);
	}
	

	#endif
	
	// 7. Arena Batu Bawah
	#ifdef STEP_7
	while(1){
		FFV = ping_read(FF);
		FLV = ping_read(FL);
		BLV = ping_read(BL);
		if((FLV > 1)&&(BLV > 1) && (FFV > 1)){
			if((FLV >= 10) && (BLV >=10) && (FFV < 20)){
				scp_wall_follower_belakang();	
			}
			else{
				tx_move_steady();
				break;
			}
		}
	}
	#endif
	
	#ifdef STEP_8
	while(1){
		scp_kepala_move(KEPALA_KANAN_DEPAN);
		HAL_Delay(3000);
		tx_move_jalan(0, 0, 30, SPEED_WALKING, JALAN_NORMAL, 10);
		scp_korban_follower_rotasi(1, DIRECTION_KANAN);
		break;
	}
	#endif 
	
	// 8. Safe Zone
	#ifdef STEP_9
	while(1){
		FFV = ping_read(FF);
		FLV = ping_read(FL);
		BLV = ping_read(BL);
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BBV = ping_read(BB);
		if((FFV > 1) && (BBV > 1)){
			if(BBV <= 27) tx_move_jalan(0, 10, 60, SPEED_WALKING, JALAN_NORMAL, 10);
				else{
					tx_capit(PENYELAMATAN_KORBAN, 0x00, 15); 
					HAL_Delay(1000);
					tx_move_steady();
					break;
				}
		}
	}
	#endif
	
	// 9. Penurunan Korban
	#ifdef STEP_10
	while(1){
		FFV = ping_read(FF);
		BBV = ping_read(BB);
		if((FFV > 1) && (BBV > 1)){
			if(BBV >= 18) tx_move_jalan(0, -10, 30, SPEED_WALKING, JALAN_NORMAL, 10);
			else{
				// Penurunan Korban
				tx_move_steady();
				break;
			}
		}
	}		
	#endif
	
	// 10. Melewati jalanan Berbatu 
	#ifdef STEP_11
	while(1){
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
		BBV = ping_read(BB);
		if( (FRV > 1) && (BRV > 1) && (FFV > 1) && (BBV > 1)){
			if((FRV >= 20) && (BRV >= 20)){
				scp_wall_follower_batu(STATE_KIRI, DIRECTION_DEPAN);
			}
			else{
				scp_kepala_move(KEPALA_KANAN);
				tx_move_steady();
				break;
			}
		}	
	}
	#endif
	
	// 11. Penyesuaian Korban di Area Kelereng
	#ifdef STEP_12
	while(1){
		if(scp_deteksi_korban(1, DIRECTION_KANAN)){
			tx_move_steady();
			break;
		}
		else scp_wall_follower_batu(STATE_KIRI, DIRECTION_DEPAN);
	}
	#endif
	
	// 10. Rotasi Robot Pada K2
	#ifdef STEP_13
	while(1){
		if(scp_korban_follower_rotasi(1, DIRECTION_KANAN) == true){
			tx_move_steady();
			break;
		}		
	}
	#endif
	
	// 11. Pengangkatan Korban K2
	#ifdef STEP_14
	while(1){
		scp_kepala_move(KEPALA_BELAKANG);
		tx_capit(HOME_CAPIT, 0x00, 15);
		tx_capit(AMBIL_KORBAN, 0x00, 15); 
		HAL_Delay(3000);
		dyna_calibrate(&ax);
		HAL_Delay(500);
		tx_move_steady();
		tx_statis(80,70,-100); 
		tx_move_steady();
		break;
	}
	#endif
	
	// 12. Menuju Safety Zone 2
	#ifdef STEP_15
	while(1){
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
		BBV = ping_read(BB);
		if( (FLV > 1) && (BLV > 1) && (FFV > 1) && (BBV > 1)){
			if((FLV >= 10) && (BLV >= 10)){
				scp_wall_follower_belakang();
			}
			else{
				tx_move_steady();
				break;
			}
		}	
	}
	#endif
	
	// 13. Penyesuaian Posisi Badan -> rotasi berdasarkan ping bb, bl, br, ff
	#ifdef STEP_16
		BLV = ping_read(BL);
		BRV = ping_read(BR);
		FFV = ping_read(FF);
		BBV = ping_read(BB);
	#endif
	
	// 14. Peletakkan Korban
	#ifdef STEP_17
	#endif
	
	// 15. Penyesuaian Badan
	#ifdef STEP_18
	#endif
	
	//
	#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#ifdef MAIN_PROGRAM
		
		#endif
		#ifdef TEST_BENCH
		scp_axis_stabilizer(10, 10, STATE_KIRI, STATE_BELAKANG );
		#endif
		
		//************************* HUSKY TEST ***********************//
		#ifdef TEST_HUSKY
		#ifdef HUSKY_ON
		blocks = husky_getBlocks();
		#endif
		#endif
		
		
		//************************* DYNAMIXEL TEST ***********************//
		#ifdef TEST_DYNA
		#ifdef DYNA_ON
		dyna_set_moving_speed(&ax, 0x0300, MOVING_CCW);
		dyna_set_goal_position(&ax, 0x0000);
		HAL_Delay(1000);
		dyna_set_goal_position(&ax, 0x0267);
		dyna_read_posisition(&ax);
		HAL_Delay(1000);
		dyna_set_goal_position(&ax, 0x0000);
		#endif
		#endif
		
		//************************* PING TEST ***********************//
		#ifdef TEST_PING
		#ifdef PING_ON
		BBV = ping_read(BB);
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
		#endif
		#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 1000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//************************* PID CALCULATION *******************//
#ifdef WALL_FOLLOWER_PID
void pid_run_batu(uint8_t state_jalan, double input, follower_direction_t dir){
		if((state_jalan == STATE_KANAN)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S) {
				PIDController_Update(&pid_wf, setpoint_wf, input);
				if(dir == DIRECTION_DEPAN) tx_move_jalan((pid_wf.out*(-1)), 15, 40, SPEED_WALKING, JALAN_NORMAL,1);
				else if(dir == DIRECTION_BELAKANG) tx_move_jalan((pid_wf.out*(-1)), -15, 40, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
		else if((state_jalan == STATE_KIRI)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S){
				PIDController_Update(&pid_wf, setpoint_wf, input);
				if(dir == DIRECTION_DEPAN) tx_move_jalan(pid_wf.out, 30, 60, SPEED_WALKING, JALAN_NORMAL,1);
				else if(dir == DIRECTION_BELAKANG) tx_move_jalan(pid_wf.out, -30, 60, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
		else if((state_jalan == STATE_DEPAN)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S){
				PIDController_Update(&pid_wf, setpoint_wf_y, input);
				if(dir == DIRECTION_KANAN) tx_move_jalan(15, pid_wf.out*(-1), 40, SPEED_WALKING, JALAN_NORMAL,1);
				else if (dir == DIRECTION_KIRI) tx_move_jalan(-15, pid_wf.out*(-1), 40, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
		else if((state_jalan == STATE_BELAKANG)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S){
				PIDController_Update(&pid_wf, setpoint_wf_y, input);
				if(dir == DIRECTION_KANAN) tx_move_jalan(20, pid_wf.out, 50, SPEED_WALKING, JALAN_NORMAL,1);
				else if(dir == DIRECTION_KIRI) tx_move_jalan(-20, pid_wf.out, 50, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
}

void pid_run(uint8_t state_jalan, double input, follower_direction_t dir){
		if((state_jalan == STATE_KANAN)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S) {
				PIDController_Update(&pid_wf, setpoint_wf, input);
				if(dir == DIRECTION_DEPAN) tx_move_jalan((pid_wf.out*(-1)), 15, 40, SPEED_WALKING, JALAN_NORMAL,1);
				else if(dir == DIRECTION_BELAKANG) tx_move_jalan((pid_wf.out*(-1)), -15, 40, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
		else if((state_jalan == STATE_KIRI)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S){
				PIDController_Update(&pid_wf, setpoint_wf, input);
				if(dir == DIRECTION_DEPAN) tx_move_jalan(pid_wf.out, 15, 40, SPEED_WALKING, JALAN_NORMAL,1);
				else if(dir == DIRECTION_BELAKANG) tx_move_jalan(pid_wf.out, -15, 40, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
		else if((state_jalan == STATE_DEPAN)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S){
				PIDController_Update(&pid_wf, setpoint_wf_y, input);
				if(dir == DIRECTION_KANAN) tx_move_jalan(15, pid_wf.out*(-1), 40, SPEED_WALKING, JALAN_NORMAL,1);
				else if (dir == DIRECTION_KIRI) tx_move_jalan(-15, pid_wf.out*(-1), 40, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
		else if((state_jalan == STATE_BELAKANG)&&(input > 0.0)){
			for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S){
				PIDController_Update(&pid_wf, setpoint_wf_y, input);
				if(dir == DIRECTION_KANAN) tx_move_jalan(20, pid_wf.out, 30, SPEED_WALKING, JALAN_NORMAL,1);
				else if(dir == DIRECTION_KIRI) tx_move_jalan(-20, pid_wf.out, 30, SPEED_WALKING, JALAN_NORMAL,1);
			}
		}
}
#endif

//***************************************************************************************************/
//************************************ IMPLEMENTASI ACTION ******************************************/

void scp_wall_follower(uint8_t state, follower_direction_t direction_play){
	
	// READ VALUE PING 
		BBV = ping_read(BB);
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
	
	//************************* Filter for PING **********************//
	
		// Belok
		if(((FFV <= 18) && (FFV > 0) && (FFV != 1)) && !(state == STATE_DEPAN) && !(state == STATE_BELAKANG)){
			
			// Belok Kanan
			if((FRV >= LEBAR_PEMBACAAN) && (BRV >= LEBAR_PEMBACAAN) && (FFV <= 18) && (state == STATE_KANAN)){
				scp_belok(BELOK_KANAN, 1000);
			}
			
			// Belok Kiri
			else if((FLV >= LEBAR_PEMBACAAN) && (BLV >= LEBAR_PEMBACAAN) && (FFV <= 18) && (state == STATE_KIRI)){
				scp_belok(BELOK_KIRI, 1000);
			}
			
//			tx_move_steady();
		}
		
		// Kiri
		else if((FLV > 0) && (BLV > 0) && (FLV != 1) && (BLV != 1) && (state == STATE_KIRI)){
			
			// Filter Tembok Rata
			if((FLV <= LEBAR_PEMBACAAN) && (BLV <= LEBAR_PEMBACAAN)){
				kiri = (FLV + BLV)/2;
				pid_run(STATE_KIRI, kiri, direction_play);
			}
		
		}
		
		// Kanan
		else if((FRV > 0) && (BRV > 0) && (FRV != 1) && (BRV != 1) && (state == STATE_KANAN)){
			
			// Filter Tembok Rata
			if((FRV <= LEBAR_PEMBACAAN) && (BRV <= LEBAR_PEMBACAAN)){
				kanan = (FRV + BRV)/2;
				pid_run(STATE_KANAN, kanan, direction_play);
			}
		}
		
		// Depan
		else if((state == STATE_DEPAN) && (FFV >= 0) && (FFV != 1)){
			depan = (FFV);
			pid_run(STATE_DEPAN, depan, direction_play);
		}
		
		// Belakang
		else if((state == STATE_BELAKANG) && (BBV >= 0) && (BBV != 1)){
			belakang = (BBV);
			pid_run(STATE_BELAKANG, belakang, direction_play);
		}
		
}

void scp_wall_follower_batu(uint8_t state, follower_direction_t direction_play){
	
	// READ VALUE PING 
		BBV = ping_read(BB);
		FRV = ping_read(FR);
		BRV = ping_read(BR);
		BLV = ping_read(BL);
		FLV = ping_read(FL);
		FFV = ping_read(FF);
	
	//************************* Filter for PING **********************//
		
		// Kiri
		if((FLV > 0) && (BLV > 0) && (FLV != 1) && (BLV != 1) && (state == STATE_KIRI)){
			
			// Filter Tembok Rata
			if((FLV <= LEBAR_PEMBACAAN) && (BLV <= LEBAR_PEMBACAAN)){
				kiri = (FLV + BLV)/2;
				pid_run_batu(STATE_KIRI, kiri, direction_play);
			}
		
		}
		
		// Kanan
		else if((FRV > 0) && (BRV > 0) && (FRV != 1) && (BRV != 1) && (state == STATE_KANAN)){
			
			// Filter Tembok Rata
			if((FRV <= LEBAR_PEMBACAAN) && (BRV <= LEBAR_PEMBACAAN)){
				kanan = (FRV + BRV)/2;
				pid_run(STATE_KANAN, kanan, direction_play);
			}
		}
		
		// Depan
		else if((state == STATE_DEPAN) && (FFV >= 0) && (FFV != 1)){
			depan = (FFV);
			pid_run(STATE_DEPAN, depan, direction_play);
		}
		
		// Belakang
		else if((state == STATE_BELAKANG) && (BBV >= 0) && (BBV != 1)){
			belakang = (BBV);
			pid_run(STATE_BELAKANG, belakang, direction_play);
		}
		
}

void scp_belok(uint8_t direction, uint16_t time){

		// Rotasi Clock Wise
		if(direction == BELOK_KANAN) tx_move_rotasi(0, 0, -30, 30, 1, 10, 2);
	
		// Rotasi Counter Clock Wise
		if(direction == BELOK_KIRI)  tx_move_rotasi(0, 0, 30, 30, 1, 10, 2);
				
		// Delay selama rotasi -> Menyesuaikan parameter rotasi
		HAL_Delay(time);
}

void scp_wall_stabilizer(uint8_t state){
	while(1){
		double front_v = 0, back_v = 0;
		if(state == STATE_KANAN){
			FRV = ping_read(FR);
			BRV = ping_read(BR);
			if((FRV >1) && (BRV > 1)){
				front_v = FRV;
				back_v = BRV;
				if(front_v-back_v <= 3) break;
				PIDController_Update(&pid_st, setpoint_st, (front_v-back_v));
				tx_move_rotasi(0, 0, pid_st.out, 50, 1, 13, 1);
			}
			else{
			continue;
			}
		}
		else if(state == STATE_KIRI){
			FLV = ping_read(FL);
			BLV = ping_read(BL);
			if((FLV >1) && (BLV > 1)){
				front_v = FLV;
				back_v = BLV;
				if(front_v-back_v <= 3) break;
				PIDController_Update(&pid_st, setpoint_st, (front_v-back_v));
				tx_move_rotasi(0, 0, pid_st.out*(-1), 50, 1, 13, 1);
			}
			else{
			continue;
			}
		}
	}
}

bool scp_deteksi_korban(uint8_t id, follower_direction_t dir){

	// Set ke Algoritma color detection
	if(algorithm_type != ALGORITHM_COLOR_RECOGNITION){
		status = husky_setAlgorithm(ALGORITHM_COLOR_RECOGNITION);
		algorithm_type = ALGORITHM_COLOR_RECOGNITION;
	}	
		blocks = husky_getBlocks();
		dyna_sudut = dyna_read_posisition(&ax);
		if(blocks.id == id){	
			return true;
		}
		return false;
}

bool scp_korban_follower_statis(uint8_t state){
	// Set ke Algoritma color detection
	if(algorithm_type != ALGORITHM_COLOR_RECOGNITION){
		status = husky_setAlgorithm(ALGORITHM_COLOR_RECOGNITION);
		algorithm_type = ALGORITHM_COLOR_RECOGNITION;
	}	
	
		blocks = husky_getBlocks();
		dyna_sudut = dyna_read_posisition(&ax);
		if(blocks.id == 1){
			if(state == STATE_BELAKANG){
				BBV = ping_read(BB);
					PIDController_Update(&pid_pk, setpoint_pk, blocks.X_center);
						if((blocks.X_center >= 160) && (blocks.X_center <= 170)){
							return true;
						}
						else{
							tx_move_jalan((pid_pk.out*(-1)), 0, 30, 15, JALAN_NORMAL, 1);
						}
						return false;
			}
			else if(state == STATE_DEPAN){
				FLV = ping_read(FL);
				PIDController_Update(&pid_pk, setpoint_pk, blocks.X_center);
						if((blocks.X_center >= 160) && (blocks.X_center <= 170)){
							return true;
						}
						else{
							tx_move_jalan(0, (pid_pk.out*(-1)), 30, 15, JALAN_NORMAL, 1);
						}
						return false;
			}
		}
	
}

bool scp_axis_stabilizer(int16_t x_axis, int16_t y_axis, uint8_t state_x, uint8_t state_y ){
		double Kanan_avg = 0;
		double Kiri_avg = 0;
		double Kanan_Depan = ping_read(FR);
		double Kanan_Belakang = ping_read(BR);
		double Kiri_Depan = ping_read(FL);
		double Kiri_Belakang = ping_read(BL);
		double Depan = ping_read(FF);
		double Belakang = ping_read(BB);
	if(Kanan_Depan > 1 && Kanan_Belakang > 1){
		Kanan_avg = (Kanan_Depan + Kanan_Belakang)/2;
	}
	if(Kiri_Depan > 1 && Kiri_Belakang > 1){
		Kiri_avg = (Kiri_Depan + Kiri_Belakang)/2;
	}
	if(state_x == STATE_KANAN && state_y == STATE_DEPAN){
		PIDController_Update(&pid_axis_x, x_axis, Kanan_avg);
		PIDController_Update(&pid_axis_y, y_axis, Depan);
		tx_move_jalan((pid_axis_x.out)*(-1), (pid_axis_y.out)*(-1), 30, 15, JALAN_NORMAL, 1);
	}
	else if(state_x == STATE_KIRI && state_y == STATE_BELAKANG){
		PIDController_Update(&pid_axis_x, x_axis, Kiri_avg);
		PIDController_Update(&pid_axis_y, y_axis, Belakang);
		tx_move_jalan((pid_axis_x.out), (pid_axis_y.out), 30, 15, JALAN_NORMAL, 1);
	}
}

bool scp_korban_follower_kepala(void){

	// Set ke Algoritma color detection
	if(algorithm_type != ALGORITHM_COLOR_RECOGNITION){
		status = husky_setAlgorithm(ALGORITHM_COLOR_RECOGNITION);
		algorithm_type = ALGORITHM_COLOR_RECOGNITION;
	}	
		blocks = husky_getBlocks();
		dyna_sudut = dyna_read_posisition(&ax);
		if(blocks.id == 1){

					PIDController_Update(&pid_dk, setpoint_dk, blocks.X_center);
					if(pid_dk.out >= 0){
						dyna_set_moving_speed(&ax, 50, MOVING_CW);
						dyna_set_goal_position(&ax, dyna_sudut+pid_dk.out);
					}
					else if(pid_dk.out < 0){
						dyna_set_moving_speed(&ax, 50, MOVING_CCW);
						dyna_set_goal_position(&ax, dyna_sudut+pid_dk.out);
					}
					
					if((blocks.X_center >= 150) && (blocks.X_center <= 160)){
						dyna_set_moving_speed(&ax, 1023, MOVING_CW);
						dyna_calibrate(&ax);
						HAL_Delay(500);
						dyna_set_goal_position(&ax, 785);
						return true;
					}
				return false;
		}
	}

	
bool scp_korban_follower(uint8_t id, follower_direction_t dir){

	// Set ke Algoritma color detection
	if(algorithm_type != ALGORITHM_COLOR_RECOGNITION){
		status = husky_setAlgorithm(ALGORITHM_COLOR_RECOGNITION);
		algorithm_type = ALGORITHM_COLOR_RECOGNITION;
	}	
		blocks = husky_getBlocks();
		dyna_sudut = dyna_read_posisition(&ax);
		if(blocks.id == id){
				FFV = ping_read(FF);
				FRV = ping_read(FR);
				BRV = ping_read(BR);
				for (float t = 0.01f; t <= 5; t += SAMPLE_TIME_S) {

					PIDController_Update(&pid_pk, setpoint_pk, blocks.X_center);
					PIDController_Update(&pid_kf, setpoint_kf, dyna_sudut);
					if(pid_pk.out >= 0){
						dyna_set_moving_speed(&ax, 50, MOVING_CW);
						dyna_set_goal_position(&ax, dyna_sudut+pid_pk.out);
					}
					else if(pid_pk.out < 0){
						dyna_set_moving_speed(&ax, 50, MOVING_CCW);
						dyna_set_goal_position(&ax, dyna_sudut+pid_pk.out);
					}
					if(dir == DIRECTION_DEPAN){
						if((blocks.X_center >= 140) && (blocks.X_center <= 170)){
							if(FFV <= 14){
								tx_move_jalan(pid_kf.out, 5, 30, 15, JALAN_NORMAL, 2);
							} 
							else{
								tx_move_jalan(pid_kf.out, 0, 30, 15, JALAN_NORMAL, 2);
							}
							if((dyna_sudut >= 510) && (dyna_sudut >= 530) && (FRV >= 40) && (BRV >= 40)) return true;
							else return false;
						}
						else{
							tx_move_steady();
						}
						return false;
					}
//					else if(dir == DIRECTION_KANAN){
//						if((blocks.X_center >= 140) && (blocks.X_center <= 170)){
//								tx_move_steady();
//								return true;
//						}
//						else{
//							tx_move_jalan(0, pid_kf.out, 30, 15, JALAN_NORMAL, 2);
//						}
//						return false;
//					}
				}
				return false;
		}
	}

	bool scp_korban_follower_rotasi(uint8_t id, follower_direction_t dir){

	// Set ke Algoritma color detection
	if(algorithm_type != ALGORITHM_COLOR_RECOGNITION){
		status = husky_setAlgorithm(ALGORITHM_COLOR_RECOGNITION);
		algorithm_type = ALGORITHM_COLOR_RECOGNITION;
	}	
	while(1){
		blocks = husky_getBlocks();
		dyna_sudut = dyna_read_posisition(&ax);
		if(blocks.id == id){

					PIDController_Update(&pid_pk, setpoint_pk, blocks.X_center);
					PIDController_Update(&pid_rk, setpoint_rk, dyna_sudut);
					if(pid_pk.out >= 0){
						dyna_set_moving_speed(&ax, 50, MOVING_CW);
						dyna_set_goal_position(&ax, dyna_sudut+pid_pk.out);
					}
					else if(pid_pk.out < 0){
						dyna_set_moving_speed(&ax, 50, MOVING_CCW);
						dyna_set_goal_position(&ax, dyna_sudut+pid_pk.out);
					}
					
					if((dyna_sudut >= 502) && (dyna_sudut <= 522) && (blocks.X_center >= 140) && (blocks.X_center <= 170)){
						tx_move_steady();
						break;
					}
					else{
						tx_move_rotasi(0, 0, pid_rk.out*(-1), 60, 1, 15, 1);
					}
				
		}
	}
}

void scp_deteksi_arena(uint8_t id){
	
	// Set ke Algoritma color detection
	if(algorithm_type != ALGORITHM_OBJECT_CLASSIFICATION){
		status = husky_setAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
		algorithm_type = ALGORITHM_OBJECT_CLASSIFICATION;
	}
	
}

void scp_mode_jalan(mode_jalan_t mode){
	tx_move_jalan(0, 15, 0, 5, mode, 2);
}

void scp_kepala_move(dynamixel_kepala_direction_t dir){
	dyna_set_moving_speed(&ax, 1023, MOVING_CW);
	dyna_calibrate(&ax);
	HAL_Delay(500);
	if(dir == KEPALA_DEPAN) dyna_calibrate(&ax);
	else if(dir == KEPALA_KANAN) dyna_set_goal_position(&ax, 261);
	else if(dir == KEPALA_BELAKANG) dyna_endless_turn(&ax, 400, 1023, MOVING_CW);
	else if(dir == KEPALA_KIRI) dyna_set_goal_position(&ax, 785);
	else if(dir == KEPALA_KANAN_DEPAN) dyna_set_goal_position(&ax, 300);
}

void pid_run_belakang(uint8_t state_jalan, double input, follower_direction_t dir){
		if((state_jalan == STATE_BELAKANG)&&(input > 0.0)){
			PIDController_Update(&pid_wf, setpoint_wf_y, input);
			if(dir == DIRECTION_KANAN) tx_move_jalan(20, pid_wf.out, 65, 15, JALAN_NORMAL,1);
			else if(dir == DIRECTION_KIRI) tx_move_jalan(-20, pid_wf.out, 65, 15, JALAN_NORMAL,1);
		}
}

void scp_wall_follower_belakang(void){
	BBV = ping_read(BB);
	if((BBV > 1) ){
			belakang = (BBV);
			pid_run_belakang(STATE_BELAKANG, belakang, DIRECTION_KIRI);
		}
	HAL_Delay(5);
}

void scp_ambil_korban(void){
		tx_capit(HOME_CAPIT, 0x00, 15);
		HAL_Delay(3000);
		tx_capit(AMBIL_KORBAN, 0x00, 15); 
		HAL_Delay(3000);
		dyna_calibrate(&ax);
		scp_kepala_move(KEPALA_BELAKANG);
		HAL_Delay(500);
}

void scp_turun_korban(void){
		dyna_calibrate(&ax);
		tx_capit(HOME_CAPIT, 0x00, 15);
		HAL_Delay(3000);
		tx_capit(PENYELAMATAN_KORBAN, 0x00, 15); 
		HAL_Delay(3000);
		dyna_calibrate(&ax);
}

void ping_clear(void){
	FFV = 0; FRV = 0; BRV = 0; FLV = 0; BLV = 0; BBV = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
