/*
 * NCCL error codes - extracted from NCCL headers
 */

#ifndef NCCL_ERR_H
#define NCCL_ERR_H

typedef enum {
    ncclSuccess = 0,
    ncclUnhandledCudaError = 1,
    ncclSystemError = 2,
    ncclInternalError = 3,
    ncclInvalidArgument = 4,
    ncclInvalidUsage = 5,
    ncclRemoteError = 6,
    ncclInProgress = 7,
    ncclNumResults = 8
} ncclResult_t;

// Logging levels
#define NCCL_LOG_NONE 0
#define NCCL_LOG_VERSION 1
#define NCCL_LOG_WARN 2
#define NCCL_LOG_INFO 3
#define NCCL_LOG_ABORT 4
#define NCCL_LOG_TRACE 5

// Debug logger function type
typedef void (*ncclDebugLogger_t)(int level, unsigned long flags, 
    const char *file, int line, const char *fmt, ...);

// Pointer support flags
#define NCCL_PTR_HOST 0x1
#define NCCL_PTR_CUDA 0x2
#define NCCL_PTR_DMABUF 0x4

// Maximum handle size
#define NCCL_NET_HANDLE_MAXSIZE 128

// Net device types
#define NCCL_NET_DEVICE_HOST 0
#define NCCL_NET_DEVICE_INVALID_VERSION 0

// Maximum sizes
#define NCCL_MAX_NET_SIZE_BYTES (1ULL << 31)

#endif // NCCL_ERR_H
