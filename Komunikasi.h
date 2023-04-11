/*
 * Komunikasi.h
 *
 *  Created on: Mar 2, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef KOMUNIKASI_H_
#define KOMUNIKASI_H_

#include "main.h"
#include <stdbool.h>

typedef enum{
	JALAN_TANGGA = 0x01U,
	JALAN_KELERENG = 0x02U,
	JALAN_BATU = 0x03U,
	JALAN_NORMAL = 0x04U
}mode_jalan_t;

typedef struct{
	bool ping;
	bool standby;
	bool jalan;
	bool translasi;
	bool rotasi;
	bool req;
	bool statis;
}feedback_t;

typedef enum{
	PING = 0x01U,
	MOVE_STEADY = 0x02U,
	MOVE_JALAN = 0x03U,
	MOVE_TRANSLASI = 0x04U,
	MOVE_ROTASI = 0x05U,
	SEND_REQ = 0x06U,
	GET_STATIS = 0x07U,
}type_jalan_t;

typedef struct{
	int16_t pos_x;
	int16_t pos_y;
	int16_t pos_z;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int8_t time;
	int8_t walkpoint;
	int8_t mode;
	type_jalan_t type;
	int8_t speed;
	mode_jalan_t mode_jalan;
}com_get_t;

void komunikasi_init(UART_HandleTypeDef* uart_handler);
bool tx_ping(void);
static uint8_t checksum_generator(uint8_t* arr, uint8_t size);
bool tx_move_steady(void);
bool tx_move_jalan(int16_t pos_x, int16_t pos_y, int16_t pos_z, int8_t speed, mode_jalan_t mode);
bool tx_move_translasi(int16_t pos_x, int16_t pos_y, int16_t pos_z, int8_t time, int8_t walkpoint);
bool tx_move_rotasi(int16_t roll, int16_t pitch, int16_t yaw, int16_t pos_z, int8_t mode, int8_t speed);
void rx_start(void);
void rx_feedback(feedback_t* fed);
void rx_start_get(void);
void rx_get(com_get_t* get);
bool tx_statis(int16_t pos_x, int16_t pos_y, int16_t pos_z);
#endif