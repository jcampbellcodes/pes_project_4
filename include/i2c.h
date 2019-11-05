/*
 * @file i2c.h
 * @brief Project 4
 *
 * @details Handles I2C transactions.
 *
 * @author Jack Campbell
 * @tools  PC Compiler: GNU gcc 8.3.0
 *         PC Linker: GNU ld 2.32
 *         PC Debugger: GNU gdb 8.2.91.20190405-git
 *         ARM Compiler: GNU gcc version 8.2.1 20181213
 *         ARM Linker: GNU ld 2.31.51.20181213
 *         ARM Debugger: GNU gdb 8.2.50.20181213-git
 *
 *  Attribution: Much of this code taken from from Embedded Systems Fundamentals
 *  with Arm Cortex-M based Microcontrollers, Dean, 2017, ARM Education Media
 *  (The word doc supplied by Prof. Montgomery)
 */

#ifndef PESI2C_H
#define PESI2C_H

#include <stdint.h>
#include <stdbool.h>
/**
 * @brief Initialize I2C registers.
 */
void i2c_init(void);

/**
 * @brief Write a single byte over I2C.
 * @param dev The address of the peripheral to write to.
 * @param reg The address of the register to write to.
 * @param data The byte to write to that register.
 */
void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data);

/**
 * @brief Write multiple bytes over I2C.
 * @param inSlaveAddr The address of the peripheral to write to.
 * @param inRegAddr The address of the register to write to.
 * @param inData The data to write.
 * @param inDataSize The size of the data to write.
 */
void i2c_write_bytes(uint8_t inSlaveAddr, uint8_t inRegAddr, uint8_t* inData, int8_t inDataSize);

/**
 * @brief Read a single byte over I2C.
 * @param dev The address of the peripheral to read from.
 * @param reg The register to read from.
 * @return The byte read.
 */
uint8_t i2c_read_byte(uint8_t dev, uint8_t reg);

/**
 * @brief Read multiple bytes over I2C.
 * @param dev_adx The address of the peripheral to read from.
 * @param reg_adx The address of the register to read from.
 * @param outData The data read.
 * @param data_count The size of the output array for data read.
 */
void i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t* outData, int8_t data_count);

/**
 * Return whether the requested device gave back an ACK when requested.
 */
bool i2c_connected(uint8_t dev_adx);

#endif
