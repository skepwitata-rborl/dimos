# GraspGen Docker Module

Docker-based implementation of the GraspGen grasp generation model as a DimOS module.

## Quick Start

### 1. Get Model Checkpoints

```bash
python -c "from dimos.utils.data import get_data; get_data('graspgen')"
```

### 2. Build the Integration Base Image (Once)

```bash
docker build -t dimos/integration:latest -f dimos/docker/integration/Dockerfile .
```

### 3. Build GraspGen

```bash
docker build -t dimos-graspgen:latest -f dimos/grasping/docker_context/Dockerfile .
```

### 4. Use in a Blueprint

```python
from dimos.grasping import graspgen
from pathlib import Path

demo = autoconnect(
    camera_module,
    graspgen(
        docker_file_path=Path(__file__).parent / "grasping" / "docker_context" / "Dockerfile",
        docker_build_context=Path(__file__).parent.parent,
        gripper_type="robotiq_2f_140",
    ),
)
```

## DimOS Integration

This Dockerfile uses the simplified 4-line DimOS integration:

```dockerfile
# Application setup (90 lines of GraspGen-specific code)
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04
RUN conda create -n app python=3.10 -y
RUN conda run -n app pip install torch
# ... install GraspGen, CUDA extensions, etc. ...

# DimOS Integration (4 lines - auto-detects Python environment)
COPY --from=dimos/integration:latest /opt/dimos-integration /opt/dimos-integration
RUN /opt/dimos-integration/integrate.sh
ENTRYPOINT ["/dimos/entrypoint.sh"]
CMD ["dimos.core.docker_module", "run"]
```

The `integrate.sh` script **automatically detects** the conda `app` environment and installs DimOS into it.

## Advanced Options

### Custom Gripper Type

Supported grippers: `robotiq_2f_85`, `robotiq_2f_140`, `barrett_hand`, `franka_hand`

```python
graspgen(docker_file_path=..., gripper_type="robotiq_2f_85")
```

Or set default in Dockerfile:

```dockerfile
ENV DEFAULT_GRIPPER=robotiq_2f_85
```

### Custom PyTorch Version

Edit line ~70 in Dockerfile:

```dockerfile
RUN conda run -n app pip install torch==2.5.0 --index-url https://download.pytorch.org/whl/cu121
```

### GPU Memory Issues

Increase shared memory:

```python
graspgen(docker_file_path=..., docker_shm_size="8g")
```

## Troubleshooting

### "Checkpoints not found"

```bash
python -c "from dimos.utils.data import get_data; get_data('graspgen')"
```

### "dimos/integration:latest not found"

```bash
docker build -t dimos/integration:latest -f dimos/docker/integration/Dockerfile .
```

### Container dies on startup

Check GPU:

```bash
docker run --rm --gpus all nvidia/cuda:12.8.1-base-ubuntu22.04 nvidia-smi
```

## Documentation

- [DimOS Docker Integration Guide](../../docker/README.md)
- [GraspGen Module Source](../graspgen_module.py)
- [GraspGen Repository](https://github.com/NVlabs/GraspGen)
