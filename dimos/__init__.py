"""dimos — A fork of dimensionalOS/dimos.

A framework for building dimensional, agent-based robotic systems
with reactive pipelines, memory management, and multimodal capabilities.

Personal fork notes:
    - Forked for learning and experimentation with agent-based robotics.
    - Upstream: https://github.com/dimensionalOS/dimos
"""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("dimos")
except PackageNotFoundError:
    __version__ = "0.0.0.dev0"

__author__ = "dimos contributors"
__license__ = "Apache-2.0"

__all__ = [
    "__version__",
    "__author__",
    "__license__",
]
