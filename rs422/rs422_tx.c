#include "rs422_tx.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "rs422_tx.pio.h"

static PIO pio;
static unsigned int sm;
static unsigned int offset;

void rs422_init(unsigned int baudrate, unsigned int pin)
{
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&rs422_tx_program, &pio, &sm, &offset, pin, 2, true);
    hard_assert(success);

    rs422_tx_program_init(pio, sm, offset, pin, baudrate);
}

void rs422_puts(const char *str, unsigned int len)
{
    rs422_tx_program_puts(pio, sm, str, len);
}