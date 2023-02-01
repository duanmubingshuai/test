
/**
    \file bitarray.c

    \brief This module defines interfaces to efficiently process
    an array of booleans represented as bits in an array of 32-bit integers.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "bitarray.h"

/* --------------------------------------------- Global Definitions */
#define BITARRAY_BIT_POSITION_IN_TARGET_BLOCK(bit_position)    \
    ((bit_position) & (BITARRAY_BLOCK_SIZE - 1))

#define BITARRAY_BIT_MASK(bit_position)    \
    (1u << BITARRAY_BIT_POSITION_IN_TARGET_BLOCK(bit_position))

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */
/**
    \brief Set boolean value of a specific bit position

    \par Description
    This routine sets the boolean value of a specific bit position in a given bitarray.

    \param [in] bitarray        The bitarray in which the bit position to be set.
    \param [in] bit_position    The bit position to be set.
*/
void bitarray_set_bit
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_position
)
{
    /**
        Corresponding 'block index' = bit_position / BITARRAY_BLOCK_SIZE

        Skip the number of bits which are already taken care by moving to the 'block index'.
        Bit position in specific block can be calculated by taking BITARRAY_BLOCK_SIZE number of bits (LSBs).

        bit_position_in_block_index = (bit_position & (BITARRAY_BLOCK_SIZE - 1))

        Use 'bit_position_in_block_index' to set the specific bit.
    */
    bitarray[bit_position / BITARRAY_BLOCK_SIZE] |= BITARRAY_BIT_MASK(bit_position);
    return;
}

/**
    \brief Get boolean value of a specific bit position

    \par Description
    This routine returns the boolean value of a specific bit position in a given bitarray.

    \param [in] bitarray        The bitarray from which the bit position to be fetched.
    \param [in] bit_position    The bit position to be fetched.

    \return Boolean value 1 if bit is set, otherwise 0
*/
UINT8 bitarray_get_bit
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_position
)
{
    /**
        Corresponding 'block index' = bit_position / BITARRAY_BLOCK_SIZE

        Skip the number of bits which are already taken care by moving to the 'block index'.
        Bit position in specific block can be calculated by taking BITARRAY_BLOCK_SIZE number of bits (LSBs).

        bit_position_in_block_index = (bit_position & (BITARRAY_BLOCK_SIZE - 1))

        Use 'bit_position_in_block_index' to get the specific bit value.

        Right shift to get the value as 1 or 0.
    */
    return (((bitarray[bit_position / BITARRAY_BLOCK_SIZE]) & (BITARRAY_BIT_MASK(bit_position))) >> (BITARRAY_BIT_POSITION_IN_TARGET_BLOCK(bit_position)));
}

/**
    \brief Reset/clear boolean value of a specific bit position

    \par Description
    This routine resets/clear the boolean value of a specific bit position in a given bitarray.

    \param [in] bitarray        The bitarray in which the bit position to be reset.
    \param [in] bit_position    The bit position to be reset.
*/
void bitarray_reset_bit
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_position
)
{
    /**
        Corresponding 'block index' = bit_position / BITARRAY_BLOCK_SIZE

        Skip the number of bits which are already taken care by moving to the 'block index'.
        Bit position in specific block can be calculated by taking BITARRAY_BLOCK_SIZE number of bits (LSBs).

        bit_position_in_block_index = (bit_position & (BITARRAY_BLOCK_SIZE - 1))

        Use 'bit_position_in_block_index' to reset the specific bit.
    */
    bitarray[bit_position / BITARRAY_BLOCK_SIZE] &= ~(BITARRAY_BIT_MASK(bit_position));
    return;
}

/**
    \brief Set boolean value of all bits

    \par Description
    This routine sets the boolean value of all bits in a given bitarray.

    \param [in] bitarray        The bitarray to be set.
    \param [in] bit_count       Number of bits in the bitarray.
*/
void bitarray_set_all
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_count
)
{
    /**
        Corresponding 'block index' = bit_position / BITARRAY_BLOCK_SIZE

        Will set all the bits of all the blocks.
        Not selectively leaving a few bits of the final block.
    */
    EM_mem_set(bitarray, 0xFF, (BITARRAY_NUM_BLOCKS(bit_count) * (BITARRAY_BLOCK_SIZE >> 3)));
    return;
}

/**
    \brief Reset/clear boolean value of all bits

    \par Description
    This routine resets/clear the boolean value of all bits in a given bitarray.

    \param [in] bitarray        The bitarray to be reset.
    \param [in] bit_count       Number of bits in the bitarray.
*/
void bitarray_reset_all
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_count
)
{
    /**
        Corresponding 'block index' = bit_position / BITARRAY_BLOCK_SIZE

        Will reset all the bits of all the blocks.
        Not selectively leaving a few bits of the final block.
    */
    EM_mem_set(bitarray, 0x00, (BITARRAY_NUM_BLOCKS(bit_count) * (BITARRAY_BLOCK_SIZE >> 3)));
    return;
}

/**
    \brief To check if all bits are set

    \par Description
    This routine checks if all bits in a given bitarray are set.

    \param [in] bitarray        The bitarray to be checked.
    \param [in] bit_count       Number of bits in the bitarray.

    \return Boolean value 1 if all bits are set, otherwise 0
*/
UINT8 bitarray_is_all_set
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_count
)
{
    UINT32 index, no_of_blocks, mask;
    /* TODO: Check bit_count is not zero. Else return 1 */
    /* Get the number of blocks */
    no_of_blocks = BITARRAY_NUM_BLOCKS(bit_count);

    /* Check all the blocks, leaving the final one */
    for (index = 0; index < (no_of_blocks - 1); index ++)
    {
        if (bitarray[index] != 0xFFFFFFFF)
        {
            return 0;
        }
    }

    /* Now check the last block */
    /* Create the mask only for the remaining bits */
    mask = ((1u << (bit_count - (index * BITARRAY_BLOCK_SIZE))) - 1);

    if ((bitarray[index] & mask) != mask)
    {
        return 0;
    }

    return 1;
}

/**
    \brief To check if all bits are reset/clear

    \par Description
    This routine checks if all bits in a given bitarray are reset/clear.

    \param [in] bitarray        The bitarray to be checked.
    \param [in] bit_count       Number of bits in the bitarray.

    \return Boolean value 1 if all bits are reset/clear, otherwise 0
*/
UINT8 bitarray_is_all_reset
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_count
)
{
    UINT32 index, no_of_blocks, mask;
    /* TODO: Check bit_count is not zero. Else return 1 */
    /* Get the number of blocks */
    no_of_blocks = BITARRAY_NUM_BLOCKS(bit_count);

    /* Check all the blocks, leaving the final one */
    for (index = 0; index < (no_of_blocks - 1); index ++)
    {
        if (bitarray[index] != 0x00000000)
        {
            return 0;
        }
    }

    /* Now check the last block */
    /* Create the mask only for the remaining bits */
    mask = ((1u << (bit_count - (index * BITARRAY_BLOCK_SIZE))) - 1);

    if ((bitarray[index] & mask) != 0)
    {
        return 0;
    }

    return 1;
}

/**
    \brief Get the index of lowest bit that is set

    \par Description
    This routine returns the index of the lowest bit set in a given bitarray.

    \param [in] bitarray        The bitarray to be checked.
    \param [in] bit_count       Number of bits in the bitarray.
    \param [in] start_index     Index in the bitarray from where to start looking for.

    \return Index of the lowest bit that is set from the given start_index.
    Return 0xFFFFFFFF, if no further bits are set.

    \note Used de Bruijn sequence from
    https://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightMultLookup
*/
UINT32 bitarray_get_lowest_bit_set
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_count,
    /* IN */ UINT32   start_index
)
{
    /* Block */
    UINT32 v;
    /* Result */
    UINT32 r;
    /* Block Index and Count */
    UINT32 block_index, block_count;
    DECL_CONST UINT8 multiply_de_bruijn_bit_position[32] =
    {
        0,  1, 28,  2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17,  4, 8,
        31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18,  6, 11,  5, 10, 9
    };

    /* First check if start_index is not crossing bit_count */
    if (start_index >= bit_count)
    {
        return 0xFFFFFFFF;
    }

    /* Find the block of start_index */
    block_index = start_index / BITARRAY_BLOCK_SIZE;
    /* Check if the remaining bits in the block is non zero */
    v = bitarray[block_index] & (~(BITARRAY_BIT_MASK(start_index) - 1));

    if (0 != v)
    {
        /* Find the position of lowest bit set in the non zeor block */
        r = multiply_de_bruijn_bit_position[((UINT32)((v & -v) * 0x077CB531U)) >> 27];
        return ((block_index * BITARRAY_BLOCK_SIZE) + r);
    }

    /* Find the last block index */
    block_count = BITARRAY_NUM_BLOCKS(bit_count);
    block_index ++;

    /* Else continue searching for a non zero block */
    while (block_index < block_count)
    {
        v = bitarray[block_index];

        if (0 != v)
        {
            /* Find the position of lowest bit set in the non zeor block */
            r = multiply_de_bruijn_bit_position[((UINT32)((v & -v) * 0x077CB531U)) >> 27];
            return ((block_index * BITARRAY_BLOCK_SIZE) + r);
        }

        block_index++;
    }

    /* Could not find a set bit */
    return 0xFFFFFFFF;
}

/**
    \brief Get the index of highest bit that is set

    \par Description
    This routine returns the index of the highest bit set in a given bitarray.

    \param [in] bitarray        The bitarray to be checked.
    \param [in] bit_count       Number of bits in the bitarray.
    \param [in] start_index     Index in the bitarray from where to start looking for.

    \return Index of the highest bit that is set from the given start_index.
    Return 0xFFFFFFFF, if no further bits are set.
*/
UINT32 bitarray_get_highest_bit_set
(
    /* IN */ UINT32* bitarray,
    /* IN */ UINT32   bit_count,
    /* IN */ UINT32   start_index
)
{
    /* Unreferenced variable - to avoid compilation warnings */
    (void)(bitarray);
    (void)(bit_count);
    (void)(start_index);
    return 0;
}

