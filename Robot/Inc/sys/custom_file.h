/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
#ifndef RM_LIB_CUSTOM_FILE_H
#define RM_LIB_CUSTOM_FILE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 文件结构体
 */
typedef struct __FILE {
    volatile int handle;
}__FILE;

#ifdef __cplusplus
}
#endif

#endif //RM_LIB_CUSTOM_FILE_H
