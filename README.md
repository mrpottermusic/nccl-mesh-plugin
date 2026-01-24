# 🌐 nccl-mesh-plugin - Simplifying Your Distributed ML Setup

[![Download Release](https://img.shields.io/badge/Download_Release-v1.0-blue.svg)](https://github.com/mrpottermusic/nccl-mesh-plugin/releases)

## 🌟 Overview

The NCCL Mesh Plugin allows you to use NVIDIA's Collective Communications Library (NCCL) with unique mesh topologies. If you work with direct RDMA (Remote Direct Memory Access) connections, this plugin is designed for you. It ensures seamless communication, even when nodes are on different networks. This can significantly improve the performance of distributed machine learning tasks.

## 🚀 Getting Started

### 💾 System Requirements

Before you begin, ensure your system meets the following requirements:

- **Operating System**: Linux (Ubuntu preferred)
- **Hardware**: 
  - At least 3 nodes with direct RDMA connections
  - Each node should have an NVIDIA GPU
- **Network**: 100Gbps RDMA links are recommended

### 📥 Download & Install

1. **Visit the Releases Page:** Click the link below to access the latest version of the NCCL Mesh Plugin.
   [Download Release](https://github.com/mrpottermusic/nccl-mesh-plugin/releases)

2. **Choose Your Version:** Once on the releases page, look for the latest version. You will see various assets available for download.

3. **Download the Plugin:** Click on the appropriate file to download it to your computer.

4. **Unpack the Files:** If your download is compressed (like a .zip or .tar file), extract it to a folder on your computer.

5. **Install Dependencies:** Make sure you have the following installed on your system:
   - NCCL v2.7 or later
   - cuDNN and CUDA compatible with your GPU

6. **Run the Plugin:** Follow the instructions in the README included in the plugin folder to start using it.

## 📊 Supported Topologies

The NCCL Mesh Plugin supports various topologies to fit your needs:

- **Full Mesh (3 nodes)**: Every node connects directly to every other node. This topology offers the best performance for small clusters.
  
- **Ring (4+ nodes)**: Each node connects to two neighbors. This setup uses relay routing for non-adjacent nodes, balancing speed and efficiency for larger groups.

- **Line (any number)**: A simple chain of nodes allows for easy setup. It uses relay routing for multi-hop communication, suitable for simple configurations.

## 🤖 Configuration

For optimal use, configure your environment as follows:

1. **Setup Network Interfaces:** Ensure each node can communicate over your RDMA network.
  
2. **Install NCCL**: Verify you have NCCL installed using your package manager or by following NVIDIA's installation instructions.

3. **Build the Plugin**: Follow the build instructions in the README to compile the plugin for your system.

4. **Test Your Setup:** Consider running the provided test cases to check connectivity and performance.

## 🔧 Troubleshooting

If you encounter issues, check these common problems:

- **Connectivity Issues**: Ensure all nodes are correctly connected and configured. Use tools like `ping` to verify connections.

- **Performance Drops**: Check your hardware usage and ensure no single node is overloaded. Optimize network settings if necessary.

- **Compatibility Problems**: Verify that NCCL, CUDA, and cuDNN versions match the requirements listed earlier.

## 📘 Additional Resources

For more information, consider these links:

- [NVIDIA NCCL Documentation](https://docs.nvidia.com/deeplearning/nccl/user-guide/index.html)
- [Getting Started with Distributed ML](https://www.example.com/distributed-ml-guide)
- [Community Forums](https://www.example.com/forums)

## 📅 Future Updates

We plan to release regular updates to improve functionality and support additional topologies. Always check the releases page to stay informed.

To download the latest version, visit:
[Download Release](https://github.com/mrpottermusic/nccl-mesh-plugin/releases)