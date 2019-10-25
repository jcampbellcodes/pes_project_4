/*
 * @file gen_pattern.h
 * @brief Project 3
 *
 * Functions for generating a random pattern.
 *
 * @author Jack Campbell
 * @tools  PC Compiler: GNU gcc 8.3.0
 *         PC Linker: GNU ld 2.32
 *         PC Debugger: GNU gdb 8.2.91.20190405-git
 *         ARM Compiler: GNU gcc version 8.2.1 20181213
 *         ARM Linker: GNU ld 2.31.51.20181213
 *         ARM Debugger: GNU gdb 8.2.50.20181213-git
 */

#ifndef GENPATTERNH
#define GENPATTERNH

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Pattern generation function will accept a number of bytes and a seed value and return a byte array.
 * @param outPattern The region of memory to write the pattern to.
 * @param inLength The number of bytes to make the pattern.
 * @param inSeed The seed to use in pattern generation.
 */
void gen_pattern(uint8_t * outPattern, size_t inLength, uint8_t inSeed);


#endif
