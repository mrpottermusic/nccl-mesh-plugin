/*
 * NCCL Net Plugin API - main header
 */

#ifndef NCCL_NET_H
#define NCCL_NET_H

#include "err.h"
#include "net_v8.h"

// Maximum number of outstanding requests
#define NCCL_NET_MAX_REQUESTS 32

// Use v8 as current version
typedef ncclNet_v8_t ncclNet_t;
typedef ncclNetProperties_v8_t ncclNetProperties_t;

#endif // NCCL_NET_H
