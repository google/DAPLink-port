#ifndef _UDB_ASSERT_H_INCLUDED_
#define _UDB_ASSERT_H_INCLUDED_

#include <stdint.h>
#include <stdbool.h>

void udb_backtrace(const char *file, uint16_t line);
void udb_assert(bool cond);

#endif // _UDB_ASSERT_H_INCLUDED_
