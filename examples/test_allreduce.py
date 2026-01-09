#!/usr/bin/env python3
"""
Basic all-reduce test for NCCL Mesh Plugin

Usage:
    # On rank 0:
    python test_allreduce.py --rank 0 --world-size 3 --master-ip 10.0.0.170
    
    # On rank 1:
    python test_allreduce.py --rank 1 --world-size 3 --master-ip 10.0.0.170
    
    # On rank 2:
    python test_allreduce.py --rank 2 --world-size 3 --master-ip 10.0.0.170
"""

import argparse
import torch
import torch.distributed as dist


def main():
    parser = argparse.ArgumentParser(description='Test NCCL all-reduce')
    parser.add_argument('--rank', type=int, required=True, help='Rank of this process')
    parser.add_argument('--world-size', type=int, default=3, help='Total number of processes')
    parser.add_argument('--master-ip', type=str, default='10.0.0.170', help='Master node IP')
    parser.add_argument('--master-port', type=int, default=29500, help='Master node port')
    args = parser.parse_args()

    # Initialize process group
    init_method = f'tcp://{args.master_ip}:{args.master_port}'
    print(f'Rank {args.rank}: Initializing with {init_method}')
    
    dist.init_process_group(
        backend='nccl',
        rank=args.rank,
        world_size=args.world_size,
        init_method=init_method
    )
    
    print(f'Rank {args.rank}: Process group initialized')

    # Create tensor on GPU
    tensor = torch.ones(1000, device='cuda')
    print(f'Rank {args.rank}: Created tensor with sum = {tensor.sum().item()}')

    # All-reduce (sum)
    dist.all_reduce(tensor, op=dist.ReduceOp.SUM)
    
    result = tensor[0].item()
    expected = float(args.world_size)
    
    print(f'Rank {args.rank}: After all-reduce, tensor[0] = {result}')
    
    if abs(result - expected) < 0.001:
        print(f'Rank {args.rank}: ✓ SUCCESS! Result matches expected value {expected}')
    else:
        print(f'Rank {args.rank}: ✗ FAILED! Expected {expected}, got {result}')

    # Cleanup
    dist.destroy_process_group()
    print(f'Rank {args.rank}: Done')


if __name__ == '__main__':
    main()
