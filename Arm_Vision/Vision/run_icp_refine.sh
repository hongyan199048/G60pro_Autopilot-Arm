#!/bin/bash
cd "$(dirname "$0")"
export LD_LIBRARY_PATH="$(pwd)/pyorbbecsdk/install/lib:$LD_LIBRARY_PATH"
/home/admin123/miniconda3/envs/yolo_env/bin/python icp_refine.py
