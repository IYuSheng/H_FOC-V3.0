/**
 * @file version.h
 * @brief 固件版本和启动信息打印接口。
 */

#ifndef VERSION_H_
#define VERSION_H_

#define MC_SOFTWARE_VERSION_MAJOR 3U
#define MC_SOFTWARE_VERSION_MINOR 0U
#define MC_SOFTWARE_VERSION_PATCH 0U

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 打印与参考库风格一致的固件启动信息
 */
void print_version_info(void);

#ifdef __cplusplus
}
#endif

#endif /* VERSION_H_ */
