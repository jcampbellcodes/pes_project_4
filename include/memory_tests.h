/*
 * @file memory_tests.h
 * @brief Project 3
 *
 * A series of functions for manipulating and inspecting memory.
 *
 * @tools  PC Compiler: GNU gcc 8.3.0
 *         PC Linker: GNU ld 2.32
 *         PC Debugger: GNU gdb 8.2.91.20190405-git
 *         ARM Compiler: GNU gcc version 8.2.1 20181213
 *         ARM Linker: GNU ld 2.31.51.20181213
 *         ARM Debugger: GNU gdb 8.2.50.20181213-git
 */
#ifndef PES_PROJECT_3_MEM_TESTS_H
#define PES_PROJECT_3_MEM_TESTS_H

#include <stdint.h>
#include <stddef.h>
#include "mem_types.h"

/**
 * @brief Allocate a block of memory for the
 * test (using malloc()) – the argument for this
 * function is a number of bytes to allocate.
 * A maximum memory block size should be checked,
 * with an error thrown for an invalid allocation
 * request (execution can continue after the error) –
 * the routine should return a pointer to the allocated
 * memory.
 * @param length Number of bytes to allocate.
 * @return Pointer to allocated memory.
 */
uint32_t * allocate_words(size_t length);

/**
 * @brief Free a previously allocated block of
 * memory (using free()).  If free_memory
 * is called with no memory having been
 * allocated, a warning message should be
 * issued, but execution can continue.
 * @param src Pointer to the memory to free.
 * @return Status of the free. SUCCESS unless
 * the src pointer is invalid.
 */
mem_status free_words(uint32_t * src);

/**
 * @brief contents of a memory region will be “displayed” by
 * returning the contents of memory at the location –
 * arguments to the function include a memory address
 * and a number of bytes to display.
 * @param loc Address of the memory to display
 * @param length Number of bytes to diplay
 * @return Byte array that is length long.
 */
uint8_t * display_memory(uint32_t * loc, size_t length);

/**
 * @brief The user specifies an address and a byte value
 * to write.  The memory at that location is modified
 * accordingly.
 * @param loc Address to write to
 * @param value Value to write at the address.
 * @return Whether the operation succeeded.
 */
mem_status write_memory(uint32_t * loc, uint8_t value);


/**
 * @brief The user specifies an address and a number of bytes.
 * All bits in this region should be inverted using XOR.
 * @param loc Address to start the invert operation
 * @param length How many bytes to invert
 * @return Whether the operation succeeded.
 */
mem_status invert_block(uint32_t * loc, size_t length);


/**
 * @brief The user specifies an address, a number of bytes,
 * and a seed value.  The seed value and the number
 * of bytes will be provided to a pattern generator
 * utility function, which will return a byte array,
 * where each byte has a random value based on the seed.
 * The bytes returned will then be written into memory
 * starting at the specified address. (Note: the data type
 * for your seed value may vary.)
 * @param loc The address to write the pattern to.
 * @param length How many bytes the pattern should be.
 * @param seed Seed to use in random number generator.
 * @return Whether the operation succeeded.
 */
mem_status write_pattern(uint32_t * loc, size_t length, uint8_t seed);


/**
 * @brief The user specifies an address, a number of bytes, and
 * a seed value.  Using the seed value and the pattern
 * generator utility,  generate a byte array with random
 * values based on the seed.  Check whether the newly generated
 * pattern in the byte array returned matches the existing byte
 * pattern in memory at the specified address.  The function
 * should return a list of any memory location addresses
 * (compare bytes) where memory did not match the pattern.
 * (Note: the data type for your seed value may vary.)
 * @param loc The address of the region to verify against
 * @param length How many bytes long is the region
 * @param seed The seed used in pattern generation
 * @return A list of any bytes that did not match.
 */
uint32_t * verify_pattern(uint32_t * loc, size_t length, uint8_t seed);


/**
 * @brief this function will take an offset from a specified
 * location and calculate a fully addressed memory location
 * for use in the functions above.  In this way, memory can
 * be more easily addressed by an offset in the region intended
 * for use.
 * @param base The base of the address to get
 * @param offset How many bytes to offset the address.
 * @return The offset address. May not be aligned.
 */
uint32_t * get_address(uint32_t * base, uint32_t offset);



#endif //PES_PROJECT_2_DELAY_H
