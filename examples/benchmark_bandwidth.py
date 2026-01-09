#!/usr/bin/env python3
"""
Bandwidth benchmark for NCCL Mesh Plugin

Usage:
    # On each node (adjust --rank):
    python benchmark_bandwidth.py --rank 0 --world-size 3 --master-ip 10.0.0.170
"""

import argparse
import time
import torch
import torch.distributed as dist


def benchmark_allreduce(size_mb: int, iterations: int, warmup: int = 5):
    """Benchmark all-reduce bandwidth"""
    
    # Create tensor
    num_elements = (size_mb * 1024 * 1024) // 4  # float32 = 4 bytes
    tensor = torch.ones(num_elements, device='cuda', dtype=torch.float32)
    
    # Warmup
    for _ in range(warmup):
        dist.all_reduce(tensor)
    torch.cuda.synchronize()
    
    # Benchmark
    start = time.perf_counter()
    for _ in range(iterations):
        dist.all_reduce(tensor)
    torch.cuda.synchronize()
    elapsed = time.perf_counter() - start
    
    # Calculate bandwidth
    # All-reduce transfers 2*(N-1)/N * size data in ring algorithm
    total_data_gb = (size_mb * iterations) / 1024
    bandwidth_gbs = total_data_gb / elapsed
    
    return bandwidth_gbs, elapsed


def main():
    parser = argparse.ArgumentParser(description='Benchmark NCCL bandwidth')
    parser.add_argument('--rank', type=int, required=True)
    parser.add_argument('--world-size', type=int, default=3)
    parser.add_argument('--master-ip', type=str, default='10.0.0.170')
    parser.add_argument('--master-port', type=int, default=29500)
    parser.add_argument('--iterations', type=int, default=20)
    args = parser.parse_args()

    # Initialize
    init_method = f'tcp://{args.master_ip}:{args.master_port}'
    dist.init_process_group('nccl', rank=args.rank, world_size=args.world_size,
                           init_method=init_method)
    
    if args.rank == 0:
        print(f'\n{"="*60}')
        print(f'NCCL Mesh Plugin Bandwidth Benchmark')
        print(f'World size: {args.world_size}')
        print(f'Iterations per size: {args.iterations}')
        print(f'{"="*60}\n')
        print(f'{"Size":<12} {"Bandwidth":<15} {"Time":<12}')
        print(f'{"-"*12} {"-"*15} {"-"*12}')

    # Test different sizes
    sizes_mb = [1, 4, 16, 64, 128, 256, 512]
    
    for size_mb in sizes_mb:
        bandwidth, elapsed = benchmark_allreduce(size_mb, args.iterations)
        
        if args.rank == 0:
            print(f'{size_mb:>6} MB    {bandwidth:>8.2f} GB/s    {elapsed:>6.3f} s')
        
        # Sync between sizes
        dist.barrier()

    if args.rank == 0:
        print(f'\n{"="*60}')
        print('Benchmark complete!')
        print(f'{"="*60}\n')

    dist.destroy_process_group()


if __name__ == '__main__':
    main()
