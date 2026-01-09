# Hardware Setup Guide

This guide covers setting up a direct-connect RDMA mesh topology with multiple nodes.

## Overview

Our reference setup uses three NVIDIA DGX Spark workstations connected in a triangle mesh topology. Each pair of nodes has a dedicated 100 Gbps RDMA link on its own subnet.

## Hardware Requirements

- 3+ nodes with RDMA-capable NICs (ConnectX-6/7 recommended)
- Direct-attach cables (QSFP56 for 100GbE)
- Each node needs N-1 NICs for N nodes in a fully-connected mesh

## Network Topology

### Triangle Mesh (3 Nodes)

```
        Node A
       /      \
   NIC1        NIC2
     |          |
192.168.101.x  192.168.100.x
     |          |
   NIC1        NIC1
     |          |
   Node B ---- Node C
          NIC2
     192.168.102.x
```

### IP Address Assignment

| Link | Subnet | Node A | Node B | Node C |
|------|--------|--------|--------|--------|
| A↔B | 192.168.101.0/24 | .2 | .3 | - |
| A↔C | 192.168.100.0/24 | .2 | - | .3 |
| B↔C | 192.168.102.0/24 | - | .2 | .3 |

## Network Configuration

### 1. Identify NICs

```bash
# List RDMA devices
ibv_devices

# List network interfaces with RDMA
ls -la /sys/class/infiniband/*/device/net/
```

### 2. Configure IP Addresses

On **Node A** (example):

```bash
# Link to Node B
sudo ip addr add 192.168.101.2/24 dev enp1s0f0np0
sudo ip link set enp1s0f0np0 up

# Link to Node C  
sudo ip addr add 192.168.100.2/24 dev enp1s0f1np1
sudo ip link set enp1s0f1np1 up
```

On **Node B**:

```bash
# Link to Node A
sudo ip addr add 192.168.101.3/24 dev enp1s0f0np0
sudo ip link set enp1s0f0np0 up

# Link to Node C
sudo ip addr add 192.168.102.2/24 dev enp1s0f1np1
sudo ip link set enp1s0f1np1 up
```

On **Node C**:

```bash
# Link to Node A
sudo ip addr add 192.168.100.3/24 dev enp1s0f0np0
sudo ip link set enp1s0f0np0 up

# Link to Node B
sudo ip addr add 192.168.102.3/24 dev enp1s0f1np1
sudo ip link set enp1s0f1np1 up
```

### 3. Make Configuration Persistent

Create netplan config (Ubuntu):

```yaml
# /etc/netplan/99-rdma-mesh.yaml
network:
  version: 2
  ethernets:
    enp1s0f0np0:
      addresses:
        - 192.168.101.2/24  # Adjust per node
    enp1s0f1np1:
      addresses:
        - 192.168.100.2/24  # Adjust per node
```

Apply:
```bash
sudo netplan apply
```

## Verify Connectivity

### 1. Ping Test

From Node A:
```bash
ping 192.168.101.3  # Node B
ping 192.168.100.3  # Node C
```

### 2. RDMA Test

```bash
# On Node B (server)
ib_send_bw -d rocep1s0f0 -x 3

# On Node A (client)
ib_send_bw -d rocep1s0f0 -x 3 192.168.101.3
```

Expected output: ~12 GB/s for 100GbE

### 3. Verify GID Index

```bash
# Show GID table
show_gids

# Find RoCE v2 GID (usually index 3)
ibv_devinfo -v | grep -A5 GID
```

## RoCE Configuration

### Enable RoCE v2

```bash
# Check current mode
cat /sys/class/infiniband/rocep*/ports/1/gid_attrs/types/*

# Enable RoCE v2 (if needed)
echo "RoCE v2" | sudo tee /sys/class/infiniband/rocep1s0f0/ports/1/gid_attrs/types/0
```

### Configure ECN (Optional but Recommended)

```bash
# Enable ECN for RoCE
sudo sysctl -w net.ipv4.tcp_ecn=1

# Configure PFC (Priority Flow Control) on switch if applicable
```

## Firewall Configuration

Open ports for NCCL communication:

```bash
# TCP ports for handshake (dynamic, 40000-50000 range)
sudo ufw allow 40000:50000/tcp

# Or disable firewall for mesh interfaces
sudo ufw allow in on enp1s0f0np0
sudo ufw allow in on enp1s0f1np1
```

## Troubleshooting

### No RDMA Devices Found

```bash
# Load kernel modules
sudo modprobe ib_core
sudo modprobe mlx5_core
sudo modprobe mlx5_ib

# Check dmesg
dmesg | grep -i mlx
```

### Link Not Coming Up

```bash
# Check physical connection
ethtool enp1s0f0np0

# Check for errors
ip -s link show enp1s0f0np0
```

### RDMA Connection Fails

```bash
# Verify GID is populated
cat /sys/class/infiniband/rocep1s0f0/ports/1/gids/3

# Check RDMA CM
rdma link show
```

### Wrong GID Index

Try different GID indices:

```bash
export NCCL_MESH_GID_INDEX=0  # or 1, 2, 3...
```

## Scaling Beyond 3 Nodes

For N nodes in a fully-connected mesh:
- Each node needs N-1 NICs
- Total links: N*(N-1)/2
- Each link on unique subnet

For 4 nodes:
```
    A
   /|\
  B-+-C
   \|/
    D
```
- 6 links, 6 subnets
- Each node needs 3 NICs

For larger clusters, consider a **partial mesh** or **fat-tree** topology with relay routing (not yet implemented in this plugin).

## Reference: DGX Spark Mesh

Our tested configuration:

| Hostname | Management IP | Mesh IPs |
|----------|--------------|----------|
| titanic (A) | 10.0.0.170 | 192.168.100.2, 192.168.101.2 |
| iceberg (B) | 10.0.0.171 | 192.168.101.3, 192.168.102.2 |
| carpathia (C) | 10.0.0.172 | 192.168.100.3, 192.168.102.3 |
