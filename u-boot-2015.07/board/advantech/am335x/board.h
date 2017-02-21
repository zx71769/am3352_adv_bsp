/*
 * board.h
 *
 * TI AM335x boards information header
 *
 * Copyright (C) 2011, Texas Instruments, 
 * Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#define board_is_bone() 0
#define board_is_bone_lt() 0
#define board_is_bbg1() 0
#define board_is_evm_sk() 0
#define board_is_idk() 0
#define board_is_gp_evm() 0
#define board_is_evm_15_or_later() 0
#define board_is_icev2() 0
#define board_is_adv_am3352() 1

/*
 * We have three pin mux functions that must exist.  We must be able to enable
 * uart0, for initial output and i2c0 to read the main EEPROM.  We then have a
 * main pinmux function that can be overridden to enable all other pinmux that
 * is required on the board.
 */
void enable_uart0_pin_mux(void);
void enable_uart1_pin_mux(void);
void enable_uart2_pin_mux(void);
void enable_uart3_pin_mux(void);
void enable_uart4_pin_mux(void);
void enable_uart5_pin_mux(void);
void enable_i2c0_pin_mux(void);
void enable_board_pin_mux(void);
#endif
