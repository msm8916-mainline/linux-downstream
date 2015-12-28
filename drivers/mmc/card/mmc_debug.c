/*
 *  linux/drivers/mmc/card/mmc_test.c
 *
 *  Copyright 2007-2008 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/mmc/host.h>

#include <linux/module.h>

#if defined(CONFIG_LGE_MMC_DYNAMIC_LOG)
#include <linux/mmc/debug_log.h>
#endif

#define     CMDS_PER_LINE       4
#define     QUEUE_SIZE          255                 // hold less 3K memory.

typedef struct {
    int     CMD;
    int     ARG;
    int     RESP;
} CMD_INFO;

static int      start_index = 0;                    // oldest mmc CMD on queue.
static int      index = 0;                          // currnet index;
static int      max_index = 0;
static CMD_INFO cmd_info[QUEUE_SIZE+1];

void print_mmc_cmd_info(struct mmc_host *host)
{
    int     i, j, count=0;

    // Print it carefully, it should not take all dmesg queue.
    pr_info("=============== mmc CMD DUMP (%s)=============\n", mmc_hostname(host));
    for (i=0; i < QUEUE_SIZE/CMDS_PER_LINE +1; i++)
    {
        // print 4 cmds on line to minimize dmesg size.
        for (j=0; j < CMDS_PER_LINE; j++)
        {
            if (count >= QUEUE_SIZE)
                break;
            pr_info ("%4d 0x%8x 0x%8x | ", cmd_info[start_index].CMD, cmd_info[start_index].ARG, cmd_info[start_index].RESP);
            start_index = (start_index >= QUEUE_SIZE) ? 0 : start_index+1;
            count++;
        }
        pr_info("\n");
    }

    // flush mmc_cmd_info
    start_index = 0;
    index = 0;

    // clear mmc_cmd_info buffer
    for (i=0; i <= QUEUE_SIZE; i++)
    {
        cmd_info[start_index].CMD = 0;
        cmd_info[start_index].ARG = 0;
        cmd_info[start_index].RESP = 0;
    }
}

EXPORT_SYMBOL(print_mmc_cmd_info);

// what if async cmd. who know resp is for proper cmd.
// It is better to save CMD, ARG, RESP all together when checking resp.
void enqueue_mmc_cmd_resp_info (int cmd, int arg, int resp)
{
    // Timing saving
    if (max_index == 0)
    {
        max_index = sizeof(cmd_info) / sizeof(CMD_INFO);
    }
    if (index == max_index)
    {
        pr_info("cmd_info overflow.\n");
        return;
    }

    cmd_info[index].CMD = cmd;
    cmd_info[index].ARG = arg;
    cmd_info[index].RESP = resp;

    // set next index and start_index.
    if (index >= QUEUE_SIZE)                    // first
    {
        index = 0;
        start_index = 1;
    }
    else
    {
        index++;
        if (index == QUEUE_SIZE)                // last
        {
            start_index = 0;
        }
        else if (index <= start_index)          // middle
            start_index++;
    }
}

EXPORT_SYMBOL(enqueue_mmc_cmd_resp_info);
