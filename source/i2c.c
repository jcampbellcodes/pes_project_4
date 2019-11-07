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

#include "i2c.h"
#include "MKL25Z4.h"
#include "logger.h"
#include "delay.h"

/**
 * @brief Send a start bit using the control register.
 */
#define I2C_M_START I2C1->C1 |= I2C_C1_MST_MASK

/**
 * @brief Send a stop bit using the control register.
 */
#define I2C_M_STOP I2C1->C1 &= ~I2C_C1_MST_MASK

/**
 * @brief Send a repeated start bit using the control register.
 */
#define I2C_M_RSTART I2C1->C1 |= I2C_C1_RSTA_MASK

/**
 * @brief Use control register to change to transmit mode.
 */
#define I2C_TX I2C1->C1 |= I2C_C1_TX_MASK

/**
 * @brief Use the control register to change to receiver mode.
 */
#define I2C_RX I2C1->C1 &= ~I2C_C1_TX_MASK

/**
 * @brief Spin until the interrupt flag has been set.
 */
#define I2C_WAIT while((I2C1->S & I2C_S_IICIF_MASK)==0) {} \
	                 I2C1->S |= I2C_S_IICIF_MASK;

/**
 * @brief Send a no-acknowledge bit after this operation.
 */
#define NACK I2C1->C1 |= I2C_C1_TXAK_MASK

/**
 * @brief Send an acknowledge bit after this operation.
 */
#define ACK I2C1->C1 &= ~I2C_C1_TXAK_MASK

/**
 * Checks for arbitration lost or no ACK received from device.
 */
static bool communicationErrorOccured()
{
	// check if arbitration was lost
	if((I2C1->S & I2C_S_ARBL_MASK))
	{
		LOG_STRING(LOG_MODULE_I2C,
				LOG_SEVERITY_STATUS,
				"Arbitration lost. Aborting read.");
		I2C1->S |= I2C_S_ARBL_MASK;
		I2C_M_STOP; //send stop
		return true;
	}

	if(!(I2C1->S & I2C_S_RXAK_MASK))
	{
		LOG_STRING(LOG_MODULE_I2C,
				LOG_SEVERITY_STATUS,
				"No ACK received. Aborting read.");
		I2C_M_STOP; //send stop
		return true;
	}
	return false;
}

void i2c_init(void)
{
	/* Enable clock for I2C1 module */
	SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;

	/* Enable clock for Port E */
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	/* Port E MUX configuration */
	PORTE->PCR[24] |=  PORT_PCR_MUX(5);
	PORTE->PCR[25] |=  PORT_PCR_MUX(5);

	/* Configure Divider Register */
	I2C1->F |= I2C_F_ICR(0x11) | I2C_F_MULT(0);

	/* Enable I2C module and interrupt */
	I2C1->C1 |= I2C_C1_IICEN_MASK;
}

void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data)
{
	// poll until not busy
	while((I2C1->S & I2C_S_BUSY_MASK)!=0) {}

	I2C_TX; // set to transmit mode
	I2C_M_START; //send start
	I2C1->D = dev; // send dev address (write)
	I2C_WAIT // wait for ack

	if(communicationErrorOccured()){ return; }

	I2C1->D = reg; // send register address
	I2C_WAIT

	if(communicationErrorOccured()){ return; }

	I2C1->D = data; //send data
	I2C_WAIT
	if(communicationErrorOccured()){ return; }

	I2C_M_STOP;
}

void i2c_write_bytes(uint8_t inSlaveAddr, uint8_t inRegAddr, uint8_t* inData, int8_t inDataSize)
{
	int8_t num_bytes_written = 0;
	I2C_TX; //set to transmit mode
	I2C_M_START; //send start
	I2C1->D = inSlaveAddr;
	I2C_WAIT

	I2C1->D = inRegAddr; //send register address
	I2C_WAIT

	do {
		ACK;
		I2C1->D = inData[num_bytes_written++];
		I2C_WAIT
	} while (num_bytes_written < inDataSize-2);

	NACK; // to HW to send NACK after read
	inData[num_bytes_written++] = I2C1->D; // read data
	I2C_WAIT
	I2C_M_STOP; // send stop
}

uint8_t i2c_read_byte(uint8_t dev, uint8_t reg)
{
	uint8_t data;

	// poll until not busy
	while((I2C1->S & I2C_S_BUSY_MASK)!=0) {}

	I2C_TX; // set to transmit mode
	I2C_M_START; //send start
	I2C1->D = dev; // send dev address (write)
	I2C_WAIT
	if(communicationErrorOccured()){ return 0x0; }

	I2C1->D = reg; // send register address
	I2C_WAIT
	if(communicationErrorOccured()){ return 0x0; }

	I2C_M_RSTART; // repeated start
	I2C1->D = (dev|0x1); // send dev address (read)
	I2C_WAIT
	if(communicationErrorOccured()){ return 0x0; }

	I2C_RX; // set to receive mode

	NACK; // set NACK after read
	data = I2C1->D; // dummy read
	I2C_WAIT

	I2C_M_STOP; //send stop
	data = I2C1->D; // read data

	return data;
}

void i2c_read_bytes(uint8_t inSlaveAddr, uint8_t inRegAddr, uint8_t* outData, int8_t outDataSize)
{
	// poll until not busy
	while((I2C1->S & I2C_S_BUSY_MASK)!=0) {}

	I2C_TX; //set to transmit mode
	I2C_M_START; //send start
	I2C1->D = inSlaveAddr;
	I2C_WAIT

	if(communicationErrorOccured()){ return; }

	I2C1->D = inRegAddr; //send register address
	I2C_WAIT

	if(communicationErrorOccured()){ return; }

	I2C_M_RSTART; //repeated start

	I2C1->D = inSlaveAddr|0x01; //send dev address (read)
	I2C_WAIT

	if(!(I2C1->S & I2C_S_RXAK_MASK))
	{
		LOG_STRING(LOG_MODULE_I2C,
				LOG_SEVERITY_STATUS,
				"No ACK received. Aborting read.");
		return;
	}

	I2C_RX; //set to receive mode
	ACK; // tell hardware to send ack after read
	outData[0] = I2C1->D; // dummy read to start I2C read
	I2C_WAIT

	ACK;
	outData[0] = I2C1->D;
	I2C_WAIT

	NACK; // to HW to send nack after read
	outData[1] = I2C1->D; // read data
	I2C_WAIT

	if(communicationErrorOccured()){ return; }

	I2C_M_STOP; // send stop
}

bool i2c_connected(uint8_t inSlaveAddr)
{
	I2C_TX; // set to transmit mode
	I2C_M_START; //send start
	I2C1->D = inSlaveAddr; // send dev address (write)
	I2C_WAIT

	bool disconnected = communicationErrorOccured();

	I2C_M_STOP; // send stop

	return !disconnected;
}

