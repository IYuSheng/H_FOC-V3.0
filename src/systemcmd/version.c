#define LOG_TAG "ver"
#include "version.h"

#include "git_info.h"
#include "logger.h"
#include "shell.h"

/**
 * @brief 打印与参考库风格一致的固件启动信息
 * @note 启动过程和 `version` 命令共用同一份版本信息。
 */
void print_version_info(void)
{
    log_i("");
    log_i("**************************************************");
    log_i("***********   H_MC_V3.0 Firmware   ***********");
    log_i("**************************************************");
    log_i("Software Version: %u.%u.%u", MC_SOFTWARE_VERSION_MAJOR,
          MC_SOFTWARE_VERSION_MINOR, MC_SOFTWARE_VERSION_PATCH);
    log_i("Git Branch: %s", GIT_BRANCH);
    log_i("Git Commit: %s", GIT_COMMIT);
    log_i("Git Tag: %s", GIT_TAG);
    log_i("Build Date: %s", MC_BUILD_DATE);
    log_i("Build Time: %s", MC_BUILD_TIME);
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    version, print_version_info, print version);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
