/**
    \file cli.c

    This file implements command line interface (CLI) framework.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "cliface.h"

//#ifdef HAVE_CLI
CLI_COMMAND* g_cli_cmd_list;
UINT8 g_cli_cmd_len;

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Function */
/**
    \fn CLI_init

    \brief Initialize CLI

    \Description
    This routine intializes CLI.

    \return EM_SUCCESS or an error code indicating reason for failure

*/
EM_RESULT CLI_init
(
    CLI_COMMAND* list,
    UINT8   len
)
{
    g_cli_cmd_list = list;
    g_cli_cmd_len = len;
    return 0;
}

/**
    \brief Process a command line instruction

    \Description
    This routine processes a command line instruction.

    \param [in] buffer        Buffer containing a command
    \param [in] buffer_len    Length of command in buffer
    \param [in] cmd_list      Command List
    \param [in] cmd_count     Number of command in the list

    \return EM_SUCCESS or an error code indicating reason for failure
*/
EM_RESULT CLI_process_line
(
    /* IN */ UCHAR*        buffer,
    /* IN */ UINT32        buffer_len
//    /* IN */ CLI_COMMAND* cmd_list,
//    /* IN */ UINT32        cmd_count
)
{
    UINT32  argc;
    UCHAR*   argv[CLI_MAX_ARGS];
    UCHAR*   cmd;
    UINT32   index;
    /* TBD: Parameter Validation */
    CLI_NULL_CHECK(buffer);

    /* Skip initial white spaces */
    for (; CLI_IS_WHITE_SPACE(*buffer) && (0 != buffer_len); buffer++, buffer_len--);

    /* Check this is not an empty command line */
    if (0 == buffer_len)
    {
        CLI_ERR(
            "[CLI] Empty command line\n");
        return EM_FAILURE;
    }

    /**
        Got the initial command.
        Parse the remaining command line to get the arguments.
    */
    argc = 0;

    for (cmd = buffer + 1; cmd < (buffer + buffer_len); cmd++)
    {
        /**
            If command argument separator is detected, replace with '\0'
            to create each as separate strings.
        */
        if (CLI_IS_CMD_SEPARATOR(*cmd))
        {
            *cmd = '\0';
        }
        /* Check if this is start of a new argument */
        else if ('\0' == (*(cmd - 1)))
        {
            argv[argc++] = cmd;
        }
        else
        {
            /* Nothing to do */
        }
    }

    CLI_TRC(
        "[CLI] Command %s, Number of arguments %d\n", buffer, argc);
    {
        UCHAR ai;

        for (ai = 0; ai < argc; ai++)
        {
            CLI_TRC(
                "Arg [%02X] %s\n", ai, argv[ai]);
        }
    }
    /* Identified command name */
    cmd = buffer;

    /* Search command and call associated callback */
    for (index = 0; index < g_cli_cmd_len; index++)
    {
        if (0 == CLI_STR_COMPARE(buffer, g_cli_cmd_list[index].cmd))
        {
            g_cli_cmd_list[index].cmd_hdlr(argc, argv);
            break;
        }
    }

    return EM_SUCCESS;
}

/* TODO: Create a separe utility module or move to a common utility module */
/* Supporting Macros */
#define IS_SPACE(c) ((' ' == (c)) || ('\t' == (c)))
#define IS_DIGIT(c) (('0' <= (c)) && ('9' >= (c)))
#define IS_UPPER(c) (('A' <= (c)) && ('F' >= (c)))
#define IS_LOWER(c) (('a' <= (c)) && ('f' >= (c)))
#define IS_ALPHA(c) IS_LOWER(c) || IS_UPPER(c)

/* Convert string to Integer */
INT32 CLI_strtoi
(
    /* IN */ UCHAR* data,
    /* IN */ UINT16 data_length,
    /* IN */ UINT8 base
)
{
    INT32  value;
    UINT16 index;
    INT8   sign_adj;
    UINT8  c;
    c = 0;

    /* Skip Whitespaces */
    for (index = 0; index < data_length; index++)
    {
        c = data[index];

        if (IS_SPACE(c))
        {
            continue;
        }
        else
        {
            break;
        }
    }

    value = 0;
    sign_adj = 1;

    /* Check Sign */
    if ('-' == c)
    {
        sign_adj = (INT8)-1;
        index++;
    }

    /* Not handling spaces after '-' or '0x' etc. */
    for (; index < data_length; index++)
    {
        c = data[index];

        /* Check if Digit */
        if (IS_DIGIT(c))
        {
            value *= base;
            value += (c - '0');
        }
        else if (IS_LOWER(c))
        {
            value *= base;
            value += (c - 'a' + 10);
        }
        else if (IS_UPPER(c))
        {
            value *= base;
            value += (c - 'A' + 10);
        }
        else
        {
            break;
        }
    }

    return (sign_adj * value);
}

/* Convert string to Integer Array */
EM_RESULT CLI_strtoarray
(
    /* IN */  UCHAR*   data,
    /* IN */  UINT16   data_length,
    /* OUT */ UINT8*   output_array,
    /* IN */  UINT16   output_array_len
)
{
    INT32  index;
    UINT8  c0, c1;
    UINT8  base;
    UINT16 output_index;
    /* HEX */
    base = 16;
    c0 = 0;
    c1 = 0;
    /* Fill with Zeros */
    EM_mem_set(output_array, 0, output_array_len);

    /* Check the length */
    if (data_length > (2 * output_array_len))
    {
        return EM_FAILURE;
    }

    /* Process from end */
    output_index = output_array_len - 1;

    for (index = data_length - 1; index >= 0; index -= 2)
    {
        if (0 != index)
        {
            c1 = data[index];
            c0 = data[index - 1];
        }
        else
        {
            c1 = data[index];
            c0 = '0';
        }

        /* Check if Digit */
        if (IS_DIGIT(c0))
        {
            c0 = (c0 - '0');
        }
        else if (IS_LOWER(c0))
        {
            c0 = (c0 - 'a' + 10);
        }
        else if (IS_UPPER(c0))
        {
            c0 = (c0 - 'A' + 10);
        }
        else
        {
            return EM_FAILURE;
        }

        /* Check if Digit */
        if (IS_DIGIT(c1))
        {
            c1 = (c1 - '0');
        }
        else if (IS_LOWER(c1))
        {
            c1 = (c1 - 'a' + 10);
        }
        else if (IS_UPPER(c1))
        {
            c1 = (c1 - 'A' + 10);
        }
        else
        {
            return EM_FAILURE;
        }

        output_array[output_index] = c0 * base + c1;
        output_index--;
    }

    return EM_SUCCESS;
}

/* Convert string to Integer Array in Little Endian Packing */
EM_RESULT CLI_strtoarray_le
(
    /* IN */  UCHAR*   data,
    /* IN */  UINT16   data_length,
    /* OUT */ UINT8*   output_array,
    /* IN */  UINT16   output_array_len
)
{
    INT32  index;
    UINT8  c0, c1;
    UINT8  base;
    UINT16 output_index;
    /* HEX */
    base = 16;
    c0 = 0;
    c1 = 0;
    /* Fill with Zeros */
    EM_mem_set(output_array, 0, output_array_len);

    /* Check the length */
    if (data_length > (2 * output_array_len))
    {
        return EM_FAILURE;
    }

    /* Process from end */
    output_index = 0;

    for (index = data_length - 1; index >= 0; index -= 2)
    {
        if (0 != index)
        {
            c1 = data[index];
            c0 = data[index - 1];
        }
        else
        {
            c1 = data[index];
            c0 = '0';
        }

        /* Check if Digit */
        if (IS_DIGIT(c0))
        {
            c0 = (c0 - '0');
        }
        else if (IS_LOWER(c0))
        {
            c0 = (c0 - 'a' + 10);
        }
        else if (IS_UPPER(c0))
        {
            c0 = (c0 - 'A' + 10);
        }
        else
        {
            return EM_FAILURE;
        }

        /* Check if Digit */
        if (IS_DIGIT(c1))
        {
            c1 = (c1 - '0');
        }
        else if (IS_LOWER(c1))
        {
            c1 = (c1 - 'a' + 10);
        }
        else if (IS_UPPER(c1))
        {
            c1 = (c1 - 'A' + 10);
        }
        else
        {
            return EM_FAILURE;
        }

        output_array[output_index] = c0 * base + c1;
        output_index++;
    }

    return EM_SUCCESS;
}

//#endif /* HAVE_CLI */

