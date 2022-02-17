#ifndef UDB_FAULT_INFO_H
#define UDB_FAULT_INFO_H

#include <stdbool.h>

bool udb_is_fault_info_uncleared(void);
void udb_print_fault_info(void);
void udb_clear_fault_info(void);

#endif  // UDB_FAULT_INFO_H
