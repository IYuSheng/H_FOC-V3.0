1.使用：
    #include LOG_TAG "user_main"
    #include "syslog.h"

    syslog_init（）；

    log_a("This is an assert log");

2.输出函数：
    log_a("This is an assert log");     //assert
    log_e("This is an error log");      //error
    log_w("This is a warning log");     //warning
    log_i("This is an info log");       //info
    log_d("This is a debug log");       //debug
    log_v("This is an verbose log");    //verbose
    elog_raw("This is a raw log");      //raw

3.级别：
    ASSERT>ERROR>WARN>INFO>DEBUG>VERBOSE

4.定义：
    每个文件开头定义LOG_TAG "文件名",例:
    #define LOG_TAG "user_main"

    注：若未定义LOG_TAG 默认为NO_TAG

4.默认：
    输出：
        A/user_main [006.144] This is an assert log
        ⬆    ⬆         ⬆        ⬆
        级别  TAG     时间戳   日志内容

    默认所有级别的信息都输出

    注：时间戳函数需实现，否则程序程序会进入Error_Handler.(返回NULL)
