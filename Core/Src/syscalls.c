/**
 * syscalls.c - 适配 STM32 的 C 库底层接口
 */
#include <sys/stat.h>
#include <sys/times.h>
#include "errno.h"
#include "stm32f1xx_hal.h" // 确保能识别 UART 句柄

#undef errno
extern int errno;

// 声明外部定义的串口句柄 (在 main.c 中)
extern UART_HandleTypeDef huart1;

/* 环境变量指针 (必须定义) */
char *__env[1] = { 0 };
char **environ = __env;

/* ================================================================================= */
/* 1. 非常重要：_write 函数                                                         */
/* 标准库的 printf 最终会调用这个函数。                                              */
/* 我们在这里把它重定向到串口发送。                                                  */
/* ================================================================================= */
int _write(int file, char *ptr, int len)
{
    // 将数据通过串口 1 发送出去
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

/* ================================================================================= */
/* 2. 非常重要：_sbrk 函数                                                          */
/* malloc 动态内存分配依赖此函数来向系统申请堆空间。                                   */
/* ================================================================================= */
extern int _end; /* Linker script 定义的堆起始地址 */

caddr_t _sbrk(int incr)
{
    static unsigned char *heap_end;
    unsigned char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = (unsigned char *)&_end;
    }

    prev_heap_end = heap_end;

    // 这里可以增加堆栈溢出检测逻辑
    // ...

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

/* ================================================================================= */
/* 3. 其他桩函数 (Stubs)                                                            */
/* 只要定义为空即可满足编译器要求                                                     */
/* ================================================================================= */

int _close(int file) { return -1; }

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) { return 1; }

int _lseek(int file, int ptr, int dir) { return 0; }

int _read(int file, char *ptr, int len) { return 0; }

void _exit(int status)
{
    _write(1, "exit", 4);
    while (1) { ; }
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

int _getpid(void) { return 1; }