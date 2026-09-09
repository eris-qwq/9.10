/**
 * @file syscalls.c
 * @brief 为裸机 newlib-nano 提供最小系统调用，工程不依赖 Linux 文件系统。
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

extern char _end;
extern char _estack;

/** @brief 裸机没有文件描述符关闭操作。 */
int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

/** @brief 把标准流报告为字符设备。 */
int _fstat(int file, struct stat *status)
{
    (void)file;
    status->st_mode = S_IFCHR;
    return 0;
}

/** @brief 裸机调试输出按终端设备处理。 */
int _isatty(int file)
{
    (void)file;
    return 1;
}

/** @brief 裸机字符流不支持定位。 */
off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    return 0;
}

/** @brief 当前固件没有标准输入，立即返回无数据。 */
int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    return 0;
}

/** @brief 当前固件不把 printf 接到硬件，丢弃字符但报告写入成功。 */
int _write(int file, const char *buffer, int length)
{
    (void)file;
    (void)buffer;
    return length;
}

/**
 * @brief 为 newlib 提供简单堆增长接口。
 * @note 业务任务使用 FreeRTOS heap_4；本接口只防止标准库缺少符号。
 */
void *_sbrk(ptrdiff_t increment)
{
    static char *heap_end;
    char *previous;

    if (heap_end == NULL)
    {
        heap_end = &_end;
    }
    previous = heap_end;
    if ((heap_end + increment) >= &_estack)
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    heap_end += increment;
    return previous;
}
