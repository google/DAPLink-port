#include "daplink_addr.h"

MEMORY
{
    m_cfgrom_bl           (RW)  : ORIGIN = DAPLINK_ROM_CONFIG_ADMIN_START, LENGTH = DAPLINK_ROM_CONFIG_ADMIN_SIZE
}

SECTIONS
{
    .cfgrom_bl : {KEEP(*(cfgrom_bl))} > m_cfgrom_bl
}

#include "daplink.ld"
