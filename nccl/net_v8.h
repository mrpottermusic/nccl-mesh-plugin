/*
 * NCCL Net Plugin API v8 - extracted from NCCL headers
 */

#ifndef NCCL_NET_V8_H
#define NCCL_NET_V8_H

#include "err.h"
#include <stdint.h>
#include <stddef.h>

// Network device handle (opaque to NCCL)
typedef void* ncclNetDeviceHandle_t;

// Network properties structure (v8)
typedef struct {
    char* name;             // Used mostly for logging
    char* pciPath;          // Path to the PCI device in /sys
    uint64_t guid;          // Unique identifier for the NIC chip
    int ptrSupport;         // NCCL_PTR_HOST or NCCL_PTR_HOST|NCCL_PTR_CUDA
    int speed;              // Port speed in Mbps
    int port;               // Port number
    float latency;          // Network latency in microseconds
    int maxComms;           // Maximum number of comms we can create
    int maxRecvs;           // Maximum number of grouped receives
    int netDeviceType;      // Network device type
    int netDeviceVersion;   // Network device version
    uint64_t maxP2pBytes;   // Maximum P2P transfer size
} ncclNetProperties_v8_t;

// Net plugin structure v8
typedef struct {
    // Name of the network (mainly for logs)
    const char* name;
    
    // Initialize the network
    ncclResult_t (*init)(ncclDebugLogger_t logFunction);
    
    // Return the number of adapters
    ncclResult_t (*devices)(int* ndev);
    
    // Get various device properties
    ncclResult_t (*getProperties)(int dev, ncclNetProperties_v8_t* props);
    
    // Create a receiving object and provide a handle to connect to it
    // The handle can be up to NCCL_NET_HANDLE_MAXSIZE bytes and will be
    // exchanged between ranks to create a connection
    ncclResult_t (*listen)(int dev, void* handle, void** listenComm);
    
    // Connect to a handle and return a sending comm object for that peer
    // This call must not block for the connection to be established, and
    // instead should return ncclSuccess with sendComm == NULL if the
    // connection is not established yet
    ncclResult_t (*connect)(int dev, void* handle, void** sendComm,
                           ncclNetDeviceHandle_t** sendDevComm);
    
    // Finalize connection establishment after remote peer has called connect
    // This call must not block for the connection to be established, and
    // instead should return ncclSuccess with recvComm == NULL if the
    // connection is not established yet
    ncclResult_t (*accept)(void* listenComm, void** recvComm,
                          ncclNetDeviceHandle_t** recvDevComm);
    
    // Register/deregister memory for use with send/recv
    ncclResult_t (*regMr)(void* comm, void* data, size_t size, int type,
                         void** mhandle);
    ncclResult_t (*regMrDmaBuf)(void* comm, void* data, size_t size, int type,
                               uint64_t offset, int fd, void** mhandle);
    ncclResult_t (*deregMr)(void* comm, void* mhandle);
    
    // Asynchronous send to a peer
    // May return ncclInProgress if the operation cannot be posted immediately
    ncclResult_t (*isend)(void* sendComm, void* data, int size, int tag,
                         void* mhandle, void** request);
    
    // Asynchronous receive from a peer
    // May return ncclInProgress if the operation cannot be posted immediately
    ncclResult_t (*irecv)(void* recvComm, int n, void** data, int* sizes,
                         int* tags, void** mhandles, void** request);
    
    // Flush data received through irecv
    ncclResult_t (*iflush)(void* recvComm, int n, void** data, int* sizes,
                          void** mhandles, void** request);
    
    // Test whether a request has completed
    ncclResult_t (*test)(void* request, int* done, int* sizes);
    
    // Close and free send/recv comm objects
    ncclResult_t (*closeSend)(void* sendComm);
    ncclResult_t (*closeRecv)(void* recvComm);
    ncclResult_t (*closeListen)(void* listenComm);
    
    // Get device-side memory handle for registered memory
    ncclResult_t (*getDeviceMr)(void* comm, void* mhandle, void** dptr_mhandle);
    
    // Notify that irecv has been consumed
    ncclResult_t (*irecvConsumed)(void* recvComm, int n, void* request);
    
} ncclNet_v8_t;

#endif // NCCL_NET_V8_H
