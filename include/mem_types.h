/*
 * @file mem_types.h
 * @brief Project 3
 *
 * Defines to be used by memory functions.
 *
 * @author Jack Campbell
 * @tools  PC Compiler: GNU gcc 8.3.0
 *         PC Linker: GNU ld 2.32
 *         PC Debugger: GNU gdb 8.2.91.20190405-git
 *         ARM Compiler: GNU gcc version 8.2.1 20181213
 *         ARM Linker: GNU ld 2.31.51.20181213
 *         ARM Debugger: GNU gdb 8.2.50.20181213-git
 */

#ifndef INCLUDE_MEM_TYPES_H_
#define INCLUDE_MEM_TYPES_H_

/**
 * @brief Return code for memory functions.
 */
typedef enum mem_status
{
	SUCCESS = 0,
	FAILED
} mem_status;


#endif /* INCLUDE_MEM_TYPES_H_ */
