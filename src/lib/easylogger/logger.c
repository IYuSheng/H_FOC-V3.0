#include "logger.h"

void syslog_init(void)
{
    if (elog_init() != ELOG_NO_ERR) {
        return;
    }

    for (int level = ELOG_LVL_ASSERT; level <= ELOG_LVL_VERBOSE; level++) {
        elog_set_fmt(level, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    }
    elog_start();
}

void syslog_deinit(void)
{
    elog_stop();
    elog_deinit();
}
