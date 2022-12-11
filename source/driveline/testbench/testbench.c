#include "testbench.h"

static void tiParseMessage(micro_t *m);
static int16_t tiParseTerm(char *rx_buf, uint8_t start, char *search_term, uint32_t *val_addr);

void tiInit(micro_t *m, q_handle_t *tx_queue){
    *m = (micro_t) {
        .tx_queue    = tx_queue,
        .data_stale  = true,
        .last_parse_time = 0xFFFF0000,
        .last_rx_time = 0xFFFF0000,
        .last_msg_time = 0xFFF0000,
        .last_serial_time = 0xFFF0000
    };

        return;
}

void tiSetParam(motor_t *m, micro_t *mi)
{
    char cmd[31]; // 30 byte + '\0'
    int arg1;
    int arg2;
    int arg3;
    int arg4;
    int arg5;

    arg1 = m->current_x10 * 10;
    arg2 = m->voltage_x10 * 10;
    arg3 = m->rpm * 10;
    arg4 = m->motor_temp * 100;
    arg5 = m->curr_power_x10 * 10;

    snprintf(cmd, 31, "%04d,%05d,%06d,%04d,%05d\r\n\0", arg1, arg2, arg3, arg4, arg5);
    qSendToBack(mi->tx_queue, cmd);
}

void tiPeriodic(micro_t* m) {
    tiParseMessage(m);
}

static void tiParseMessage(micro_t *m)
{
    char     tmp_rx_buf[TI_MAX_RX_LENGTH];
    int16_t  curr;
    uint8_t  i;
    uint32_t val_buf;

    curr = m->last_msg_loc;

    // Copy buffer so it doesn't change underneath us
    for (i = 0; i < TI_MAX_RX_LENGTH; ++i) {
        tmp_rx_buf[i] = m->rx_buf[i];
    }

    m->last_serial_time = sched.os_ticks;

    // Parse Front Left Torque
    if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "FL", &val_buf);
    if (curr >= 0) m->Tx_in[0] = (uint16_t) val_buf;

    // Parse Front Right Torque
    if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "FR", &val_buf);
    if (curr >= 0) m->Tx_in[1] = (uint16_t) val_buf;

    // Parse Rear Left Torque
    if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "RL", &val_buf);
    if (curr >= 0) m->Tx_in[2] = val_buf;

    // Parse Rear Right Torque
    if (curr >= 0) curr = tiParseTerm(tmp_rx_buf, curr, "RR", &val_buf);
    if (curr >= 0) m->Tx_in[3] = (uint8_t) val_buf;
}

static int16_t tiParseTerm(char *rx_buf, uint8_t start, char *search_term, uint32_t *val_addr)
{
    uint8_t search_length = strlen(search_term);
    bool match = false;
    uint8_t curr = 0xFF;

    for (uint8_t i = start; i < TI_MAX_RX_LENGTH + start; ++i) 
    {
        if (rx_buf[i % TI_MAX_RX_LENGTH] == search_term[0])
        {
            match = true;
            // possible match, check entire term matches
            for (uint8_t j = 0; j < search_length; ++j)
            {
                if (rx_buf[(i + j) % TI_MAX_RX_LENGTH] != search_term[j])
                {
                    // not a match, continue searching
                    match = false;
                    break;
                }
            }
            if (match)
            {
                curr = i % TI_MAX_RX_LENGTH;
                break;
            }
        }
    }
    if (!match) return -1;
    // destroy match to prevent re-reading same data
    // rx_buf[curr] = '\0';
    curr = (curr + search_length) % TI_MAX_RX_LENGTH;

    uint32_t val = 0;

    /* Extract value */
    for (uint8_t i = curr; i < TI_MAX_RX_LENGTH + curr; ++i)
    {
        char c = rx_buf[i % TI_MAX_RX_LENGTH];
        if ((c == ' ' && val == 0) || c == '.') continue;
        else if (c >= '0' && c <= '9')
        {
            val = (val * 10) + (c - '0');
        }
        else
        {
            curr = i % TI_MAX_RX_LENGTH;
            break;
        }
    }

    *val_addr = val;
    return curr;
}
