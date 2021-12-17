
/**
    \file nvsto.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "nvsto_internal.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/** NVSTO Partition table for the number of banks */
DECL_STATIC NVSTO_PARTITION nvsto_partition[NVS_NUM_BANKS][NVSTO_NUM_PARTITIONS];

/** NVSTO Partition count for the banks */
DECL_STATIC UINT8 nvsto_partition_count[NVS_NUM_BANKS];

/** NVSTO Partition offset for the banks */
DECL_STATIC UINT16 nvsto_partition_offset[NVS_NUM_BANKS];

/** NVSTO Bank size available at the platform */
DECL_STATIC UINT16 nvsto_size[NVS_NUM_BANKS];

/* --------------------------------------------- Functions */
/**
    \brief

    \Description


    \param void

    \return void
*/
void nvsto_init (UINT32 base1,UINT32 base2)
{
    nvs_flash_base1 = base1;
    nvs_flash_base2 = base2;
    /* Initialize the storage partitions */
    EM_mem_set (&nvsto_partition[NVS_BANK_PERSISTENT], 0, sizeof (NVSTO_PARTITION));
    /* Initialize the partition counts */
    nvsto_partition_count[NVS_BANK_PERSISTENT] = 0;
    /* Initialize the partition offsets */
    nvsto_partition_offset[NVS_BANK_PERSISTENT] = 0;
    /* Initialize the platform */
    nvsto_size[NVS_BANK_PERSISTENT] = nvs_init (NVS_BANK_PERSISTENT);
}

/**
    \brief

    \Description


    \param void

    \return void
*/
void nvsto_shutdown (void)
{
    /* Shutdoen the platform */
    nvs_shutdown (NVS_BANK_PERSISTENT);
}

/**
    \fn nvsto_register

    \brief

    \Description


    \param storage
    \param size
    \param handle

    \return void
*/
INT8 nvsto_register
(
    /* IN */  UINT8     storage,
    /* IN */  UINT16    size,
    /* OUT */ UINT8*    handle
)
{
    UINT8 current;
    NVSTO_PARTITION* part;

    /* Check if the list count in the given storage type is not full */
    if (NVSTO_NUM_PARTITIONS == nvsto_partition_count[storage])
    {
        *handle = NVSTO_NUM_PARTITIONS;
        /* TODO: Log */
        return (INT8)-1;
    }

    /*
        For a non-zero bank size at the platform, check if the registration would
        exceed the size
    */
    if ((nvsto_size[storage] != 0) &&
            (nvsto_size[storage] < (nvsto_partition_offset[storage] + size)))
    {
        *handle = NVSTO_NUM_PARTITIONS;
        /* TODO: Log */
        return (INT8)-1;
    }

    /* Get the current index */
    current = nvsto_partition_count[storage];
    part = &nvsto_partition[storage][current];
    /* Register the partition size */
    part->size = size;
    /* Update the base offset */
    part->base = nvsto_partition_offset[storage];
    /* Update the global offset */
    nvsto_partition_offset[storage] += size;
    /* Update the current index */
    nvsto_partition_count[storage] ++;
    /* Return the bank index reserved */
    *handle = current;
    return 0;
}


/**
    \fn nvsto_open

    \brief

    \Description


    \param storage
    \param handle
    \param access

    \return void
*/
INT16 nvsto_open
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle,
    /* IN */ UINT8    access
)
{
    NVSTO_PARTITION* part;
    /**
        Validate if the offset + length is in
        range of the bank id size
    */
    part = &nvsto_partition[storage][handle];
    return nvs_open (storage, access, part->base);
}


/**
    \fn nvsto_close

    \brief

    \Description


    \param storage
    \param handle

    \return void
*/
INT16 nvsto_close
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle
)
{
    /* TODO: Not using part */
    return nvs_close (storage);
}


/**
    \fn nvsto_write

    \brief

    \Description


    \param storage
    \param handle
    \param buffer
    \param length

    \return Number of bytes written
*/
INT16 nvsto_write
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle,
    /* IN */ void*    buffer,
    /* IN */ UINT16   length
)
{
    NVSTO_PARTITION* part;
    /**
        Validate if the offset + length is in
        range of the bank id size
    */
    part = &nvsto_partition[storage][handle];

    if (part->size < length)
    {
        /* TODO: Log */
        return -1;
    }

    /* Write the data */
    return nvs_write (storage, buffer, length);
}

/**
    \fn nvsto_read

    \brief

    \Description


    \param storage
    \param handle
    \param buffer
    \param length

    \return Number of bytes read
*/
INT16 nvsto_read
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle,
    /* IN */ void*    buffer,
    /* IN */ UINT16   length
)
{
    NVSTO_PARTITION* part;
    /**
        Validate if the offset + length is in
        range of the bank id size
    */
    part = &nvsto_partition[storage][handle];

    if (part->size < length)
    {
        /* TODO: Log */
        return -1;
    }

    /* Read the data */
    return nvs_read (storage, buffer, length);
}

/**
    \fn nvsto_read_crc16

    \brief

    \Description


    \param storage
    \param handle
    \param buffer
    \param length

    \return Number of bytes read
*/
INT16 nvsto_read_crc16
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle,
    /* IN */ UINT16*    buffer,
    /* IN */ UINT16   length
)
{
    /* TODO: Not using part */
    /* Read the data */
    return nvs_read_crc16 (storage, buffer, length);
}


/**
    \fn nvsto_seek

    \brief

    \Description


    \param storage
    \param handle
    \param offset

    \return void
*/
INT16 nvsto_seek
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle,
    /* IN */ UINT32   offset
)
{
    NVSTO_PARTITION* part;
    /**
        Validate if the offset + length is in
        range of the bank id size
    */
    part = &nvsto_partition[storage][handle];

    if (part->size < offset)
    {
        /* TODO: Log */
        return -1;
    }

    return nvs_seek (storage, (part->base + offset));
}

/**
    \fn nvsto_erase

    \brief

    \Description


    \param storage
    \param handle

    \return void
*/
INT16 nvsto_erase
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle
)
{
    /* TODO: Not using part */
    return nvs_erase (storage);
}

/**
    \fn nvsto_erase

    \brief

    \Description


    \param storage
    \param handle

    \return void
*/
INT16 nvsto_write_header
(
    /* IN */ UINT8    storage,
    /* IN */ UINT8    handle,
    /* IN */ UINT32    value
)
{
    /* TODO: Not using part */
    return nvs_write_header (storage,value);
}

