# AGENTS.md — DimOS

Guide for AI agents working on the DimOS codebase.

## What is DimOS

An open-source universal operating system for generalist robotics. Modules communicate via typed streams over LCM, shared memory, or other transports. Blueprints compose modules into runnable robot stacks.

---

## Quick Start

```bash
# Install
uv sync --all-extras --no-extra dds

# List all runnable blueprints
dimos list

# --- Go2 quadruped ---
dimos --replay run unitree-go2                  # perception + mapping, replay data
dimos --replay run unitree-go2 --daemon         # same, backgrounded
dimos --replay run unitree-go2-agentic          # + LLM agent (GPT-4o) + skills
dimos --replay run unitree-go2-agentic-mcp      # + McpServer + McpClient (MCP tools live)
dimos run unitree-go2-agentic --robot-ip 192.168.123.161  # real Go2 hardware

# --- G1 humanoid ---
dimos --simulation run unitree-g1-agentic-sim   # G1 in MuJoCo sim + agent + skills
dimos --simulation run unitree-g1-basic-sim     # G1 sim, no agent
dimos run unitree-g1-agentic --robot-ip 192.168.123.161   # real G1 hardware
dimos run unitree-g1-full --robot-ip 192.168.123.161      # G1 hardware + keyboard teleop

# --- Inspect & control a running instance ---
dimos status
dimos log              # last 50 lines, human-readable
dimos log -f           # follow/tail in real time
dimos log -n 100       # last N lines
dimos log --json       # raw JSONL
dimos agent-send "say hello"

# Stop / restart
dimos stop             # graceful SIGTERM → SIGKILL escalation
dimos stop --force     # immediate SIGKILL
dimos restart          # stop + re-run with same original args
```

### Blueprint quick-reference

| Blueprint | Robot | Hardware | Agent | MCP server | Notes |
|-----------|-------|----------|-------|------------|-------|
| `unitree-go2-agentic-mcp` | Go2 | real | via McpClient | ✓ | **Only blueprint with McpServer live** |
| `unitree-g1-agentic-sim` | G1 | sim | GPT-4o (G1 prompt) | — | Full agentic sim, no real robot needed |
| `xarm-perception-agent` | xArm | real | GPT-4o | — | Manipulation + perception + agent |
| `xarm7-trajectory-sim` | xArm7 | sim | — | — | Trajectory planning sim |
| `arm-teleop-xarm7` | xArm7 | real | — | — | Quest VR teleop |
| `keyboard-teleop-xarm7` | xArm7 | real | — | — | Keyboard-driven arm teleop |
| `dual-xarm6-planner` | xArm6×2 | real | — | — | Dual-arm motion planner |

Run `dimos list` for the full list of all available blueprints.

---

## Tools available to you (MCP)

**MCP only works if the blueprint includes `McpServer`.** Currently the only shipped blueprint that does is `unitree-go2-agentic-mcp`. All other agentic blueprints use the in-process `Agent` module and do NOT expose an MCP endpoint. To add MCP to a blueprint, see [Adding McpServer to a blueprint](#adding-mcpserver-to-a-blueprint) below.

```bash
# Start the MCP-enabled blueprint first:
dimos --replay run unitree-go2-agentic-mcp --daemon

# Then use MCP tools:
dimos mcp list-tools                                              # all available skills as JSON
dimos mcp call move --arg x=0.5 --arg duration=2.0               # call by key=value args
dimos mcp call move --json-args '{"x": 0.5, "duration": 2.0}'    # call by JSON
dimos mcp call speak --arg text="Hello, I am your robot"
dimos mcp call navigate_with_text --arg query="kitchen"
dimos mcp call execute_arm_command --arg command_name="HighWave"
dimos mcp call execute_sport_command --arg command_name="Hello"
dimos mcp status      # PID, module list, skill list
dimos mcp modules     # module → skills mapping

# Send a natural language message to the running agent (works without McpServer too,
# goes direct via LCM to /human_input):
dimos agent-send "walk forward 2 meters then wave"
```

The MCP server runs at `http://localhost:9990/mcp` by default (`GlobalConfig.mcp_port`).

From Python (e.g. in tests or scripts):

```python
from dimos.agents.mcp.mcp_adapter import McpAdapter

adapter = McpAdapter.from_run_entry()   # auto-discovers running instance from registry
adapter.wait_for_ready(timeout=10)
tools = adapter.list_tools()            # list[dict] — name, description, inputSchema
result = adapter.call_tool_text("server_status")
adapter.call_tool("move", {"x": 0.5, "duration": 2.0})
```

## Repo Structure

```
dimos/
├── core/                    # Module system, blueprints, workers, daemon, transports
│   ├── module.py            # Module base class (In/Out streams, @rpc, @skill)
│   ├── blueprints.py        # Blueprint composition (autoconnect)
│   ├── worker.py            # Forkserver worker processes
│   ├── module_coordinator.py # Module lifecycle manager
│   ├── daemon.py            # Daemon mode (daemonize, signal handling)
│   ├── run_registry.py      # Per-run tracking + log paths
│   ├── global_config.py     # GlobalConfig (env vars, CLI flags, .env)
│   ├── stream.py            # In[T]/Out[T] typed streams
│   └── transport.py         # LCMTransport, SHMTransport, pLCMTransport
├── robot/
│   ├── cli/dimos.py         # CLI entry point (typer)
│   ├── all_blueprints.py    # Auto-generated blueprint registry (DO NOT EDIT MANUALLY)
│   └── unitree/             # Unitree robot implementations (Go2, G1, B1)
│       ├── unitree_skill_container.py  # Go2 skill container (@skill methods)
│       ├── go2/             # Go2 blueprints and connection
│       └── g1/              # G1 humanoid blueprints and connection
│           ├── connection.py       # G1ConnectionBase ABC + G1Connection (real hardware)
│           ├── sim.py              # G1SimConnection (MuJoCo simulation)
│           ├── skill_container.py  # UnitreeG1SkillContainer (@skill methods)
│           ├── system_prompt.py    # G1_SYSTEM_PROMPT (robot-specific LLM prompt)
│           └── blueprints/
│               ├── basic/          # Hardware-only stacks
│               ├── perceptive/     # + perception + mapping + memory
│               └── agentic/        # + agent + skills (top-level runnable)
├── agents/                  # AI agent system
│   ├── agent.py             # Agent module (LangGraph-based, consumes skills from modules)
│   ├── system_prompt.py     # Default Go2 system prompt (SYSTEM_PROMPT)
│   ├── annotation.py        # @skill decorator
│   ├── mcp/                 # MCP server (Model Context Protocol)
│   └── skills/              # Reusable skill containers
│       ├── navigation.py    # NavigationSkillContainer (navigate_with_text, tag_location)
│       ├── speak_skill.py   # SpeakSkill (OpenAI TTS → robot speakers)
│       ├── person_follow.py # PersonFollowSkillContainer
│       └── gps_nav_skill.py # GPS navigation skills
├── navigation/              # Path planning, frontier exploration, patrolling
├── perception/              # Object detection, tracking, temporal memory
├── visualization/rerun/     # Rerun bridge (native + web viewers)
├── web/websocket_vis/       # Command center web dashboard (port 7779)
├── msgs/                    # Message types (geometry_msgs, sensor_msgs, nav_msgs)
├── protocol/                # Transport implementations (LCM, DDS, SHM)
├── memory/timeseries/       # Time series data storage + replay
├── hardware/sensors/        # Camera, lidar, ZED modules
├── mapping/                 # Voxel mapping, costmaps, occupancy grids
└── utils/                   # Logging, data loading, CLI tools
docs/                        # Human-readable documentation
├── usage/                   # Modules, blueprints, transports, visualization
│   ├── modules.md           # ← Start here for module system
│   ├── blueprints.md        # Blueprint composition guide
│   ├── visualization.md     # Viewer backends (rerun, rerun-web, foxglove)
│   └── configuration.md     # GlobalConfig + Configurable pattern
├── development/             # Testing, Docker, profiling, LFS
│   ├── testing.md           # Fast/slow tests, pytest usage
│   ├── dimos_run.md         # CLI usage, GlobalConfig, adding blueprints
│   └── large_file_management.md  # LFS + get_data()
└── agents/                  # Agent system documentation
```

## Architecture (minimum you need to know)

### Modules
Autonomous subsystems. Communicate via `In[T]`/`Out[T]` typed streams. Run in forkserver worker processes.

```python
from dimos.core.module import Module, In, Out
from dimos.core import rpc
from dimos.msgs.sensor_msgs import Image

class MyModule(Module):
    color_image: In[Image]       # input stream
    processed: Out[Image]        # output stream

    @rpc
    def start(self) -> None:
        self.color_image.subscribe(self._process)

    def _process(self, img: Image) -> None:
        self.processed.publish(do_something(img))
```

### Blueprints
Compose modules with `autoconnect()`. Streams auto-connect by `(name, type)` matching.

```python
from dimos.core.blueprints import autoconnect

my_blueprint = (
    autoconnect(module_a(), module_b(), module_c())
    .global_config(n_workers=4, robot_model="unitree_go2")
)
```

### GlobalConfig
Singleton config. Values from: defaults → `.env` → env vars → blueprint → CLI flags. Env vars prefixed with `DIMOS_`.

Key fields: `robot_ip`, `simulation`, `replay`, `viewer` (`rerun`|`rerun-web`|`foxglove`|`none`), `n_workers`.

### Transports
- **LCMTransport**: Default. Multicast UDP. Good for most messages.
- **SHMTransport/pSHMTransport**: Shared memory. Use for high-bandwidth (images, point clouds).
- **pLCMTransport**: Pickled LCM. For complex Python objects.

### Daemon Mode
`dimos run <blueprint> --daemon` daemonizes the process. Per-run logs go to `~/.local/state/dimos/logs/<run-id>/`. `dimos status` shows running instances. `dimos stop` sends SIGTERM → SIGKILL.

---

## CLI Reference

All commands use `dimos [GLOBAL_FLAGS] <command> [OPTIONS]`.

### Global flags (apply to `run` and `show-config`)

Every `GlobalConfig` field is a CLI flag: `--robot-ip`, `--simulation/--no-simulation`, `--replay/--no-replay`, `--viewer {rerun|rerun-web|foxglove|none}`, `--mcp-port`, `--mcp-host`, `--n-workers`, etc. Flags override `.env` and env vars.

```bash
dimos --replay --viewer rerun-web run unitree-go2-basic
```

### Core commands

| Command | Description |
|---------|-------------|
| `dimos run <blueprint> [--daemon]` | Start a blueprint (foreground or daemonized) |
| `dimos status` | Show running instance (run ID, PID, blueprint, uptime, log path) |
| `dimos stop [--force]` | Stop running instance (SIGTERM, escalates to SIGKILL after 5s; `--force` = immediate SIGKILL) |
| `dimos restart [--force]` | Stop + re-exec with original `sys.argv` (inherits `--daemon` if used originally) |
| `dimos list` | List all non-demo blueprints |
| `dimos show-config` | Print resolved GlobalConfig values |
| `dimos log [OPTIONS]` | View per-run logs (see below) |

### `dimos log` options

| Flag | Default | Description |
|------|---------|-------------|
| `--follow / -f` | off | Tail log in real time (Ctrl-C to stop) |
| `--lines / -n N` | 50 | Show last N lines |
| `--all / -a` | off | Show full log (no truncation) |
| `--json` | off | Raw JSONL instead of human-readable format |
| `--run / -r <id>` | most recent | Specific run ID |

Human-readable format: `15:55:06 [inf] blueprints.py  Building the blueprint`

Log files are JSONL at `~/.local/state/dimos/logs/<run-id>/main.jsonl`. Run registry JSON is at `~/.local/state/dimos/runs/<run-id>.json`.

### `dimos mcp` subcommands

Require a running DimOS instance with `McpServer` in the blueprint.

| Command | Description |
|---------|-------------|
| `dimos mcp list-tools` | List all available MCP tools (skills) as JSON |
| `dimos mcp call <tool> [--arg key=val] [--json-args '{}']` | Call any MCP tool |
| `dimos mcp status` | Server status: PID, modules, skills |
| `dimos mcp modules` | Module → skills mapping |

```bash
# Example: call the move skill from the CLI
dimos mcp call move --arg x=0.5 --arg duration=2.0

# Or with JSON args
dimos mcp call move --json-args '{"x": 0.5, "duration": 2.0}'
```

### `dimos agent-send`

Send a message to the running agent's `human_input` stream via LCM (no MCP round-trip):

```bash
dimos agent-send "walk forward 2 meters"
```

Internally calls the `agent_send` MCP skill which publishes to the `/human_input` pLCM topic.

### Debug/diagnostic commands

| Command | Description |
|---------|-------------|
| `dimos lcmspy` | LCM message monitor (passes args through to lcmspy) |
| `dimos agentspy` | Agent message monitor |
| `dimos humancli` | Interactive human text interface for sending messages to the agent |
| `dimos top` | Requires --dtop flag on blueprint, Live resource TUI (CPU/memory per worker process) |
| `dimos topic echo <topic> [type]` | Subscribe and print messages on an LCM topic |
| `dimos topic send <topic> <expr>` | Publish a Python expression as a message |
| `dimos rerun-bridge [--viewer-mode] [--memory-limit]` | Launch Rerun visualization bridge standalone |

### Run Registry

Every `dimos run` writes a JSON entry to `~/.local/state/dimos/runs/<run-id>.json`:

```json
{
  "run_id": "20260310-155506-unitree-go2-basic",
  "pid": 12345,
  "blueprint": "unitree-go2-basic",
  "started_at": "2026-03-10T15:55:06+00:00",
  "log_dir": "/home/user/.local/state/dimos/logs/20260310-155506-unitree-go2-basic",
  "cli_args": ["unitree-go2-basic"],
  "config_overrides": {"replay": true},
  "grpc_port": 9877,
  "original_argv": ["dimos", "--replay", "run", "unitree-go2-basic", "--daemon"]
}
```

`dimos stop` and `dimos restart` use `original_argv` to replay the exact command. Stale entries (dead PIDs) are cleaned up automatically on each `dimos run`.

---

## MCP Server

The MCP (Model Context Protocol) server exposes all `@skill` methods as JSON-RPC tools. It runs inside `McpServer` as a module in the blueprint.

### Architecture

```
External LLM / Claude / Cursor
        ↓  HTTP POST /mcp (JSON-RPC 2.0)
McpServer (FastAPI + uvicorn, port 9990 default)
        ↓  RPC call
SkillContainer.some_skill()
        ↓  LCM / internal RPC
Other modules (navigation, connection, etc.)
```

`McpServer` is both a DimOS `Module` (runs in a forkserver worker) and a FastAPI HTTP server. It listens on `GlobalConfig.mcp_port` (default 9990) at `GlobalConfig.mcp_host` (default `127.0.0.1`).

On `on_system_modules()`, it collects all `@skill` methods from every deployed module and registers them as MCP tools discoverable via `tools/list`.

### Built-in introspection skills

These are `@skill` methods on `McpServer` itself — they show up in `tools/list` alongside robot skills:

| Skill | Description |
|-------|-------------|
| `server_status` | Returns JSON: PID, list of module names, list of skill names |
| `list_modules` | Returns JSON: `{module_name: [skill_names]}` mapping |
| `agent_send(message)` | Sends a message to the agent via pLCM on `/human_input` topic |

### MCP protocol

Standard JSON-RPC 2.0 over HTTP POST `/mcp`. Methods:
- `initialize` → server info + capabilities
- `tools/list` → all available tools with schemas
- `tools/call` → call a tool by name with arguments

Notifications (no `id`) return HTTP 204 (no body).

### McpAdapter (client)

`dimos/agents/mcp/mcp_adapter.py` — use this to talk to a running MCP server from code or tests.

```python
from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError

# Discover from running instance (reads run registry)
adapter = McpAdapter.from_run_entry()

# Or explicit URL
adapter = McpAdapter("http://localhost:9990/mcp")

# Wait until ready
adapter.wait_for_ready(timeout=10)

# List tools
tools = adapter.list_tools()  # list[dict]

# Call a tool
result = adapter.call_tool("move", {"x": 0.5, "duration": 2.0})
text = adapter.call_tool_text("server_status")  # returns first text content as str

# Wait until server goes down (e.g., after stop)
adapter.wait_for_down(timeout=10)
```

`McpError` is raised for JSON-RPC errors. `requests.ConnectionError` is raised if the server is unreachable — handle both in robust callers.

### McpClient module

`dimos/agents/mcp/mcp_client.py` — an alternative to `Agent` that fetches tools from a remote MCP server instead of directly from colocated modules. Useful for cross-process or cross-machine agent setups.

- `config.mcp_server_url`: default `http://localhost:9990/mcp`
- Fetches tools at startup via `tools/list` (retries for up to 60s)
- Same `human_input: In[str]` / `agent: Out[BaseMessage]` interface as `Agent`

### MCP logging (per-run JSONL)

Every MCP tool call is logged to `main.jsonl`:
```
15:36:31 [inf] mcp_server.py  MCP tool call   tool=move args={"x": 0.5}
15:36:31 [inf] mcp_server.py  MCP tool done   tool=move duration=0.3s response=...
```

Agent messages (human, AI, tool) are also logged:
```
15:36:32 [inf] utils.py  Agent message  msg_type=human content='walk forward'
15:36:33 [inf] utils.py  Agent message  msg_type=ai tool_calls=[{"name": "move", ...}]
```

View with `dimos log` or `dimos log --follow`.

### Adding McpServer to a blueprint

The MCP-enabled pattern uses **two modules** instead of one:

- `McpServer` — FastAPI server that exposes all `@skill` methods over HTTP on port 9990
- `McpClient` — replaces `Agent`; fetches tools from `McpServer` via `tools/list` then drives the LLM

**Do not use `agent()` and `McpServer` + `mcp_client()` together** — they both consume `human_input` and both drive the LLM. Pick one.

The reference implementation is `unitree-go2-agentic-mcp`:

```python
# dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_mcp.py
from dimos.agents.mcp.mcp_client import mcp_client
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

unitree_go2_agentic_mcp = autoconnect(
    unitree_go2_spatial,   # robot stack: connection, perception, navigation, mapping
    McpServer.blueprint(), # HTTP MCP server — exposes all @skill methods on port 9990
    mcp_client(),          # LLM agent — fetches tools from McpServer, drives conversation
    _common_agentic,       # skill containers: navigation, person_follow, unitree_skills, speak, web_input
)
```

Compare with the standard non-MCP agentic blueprint:

```python
# unitree_go2_agentic.py — agent() directly wires to skills in-process, no HTTP
unitree_go2_agentic = autoconnect(
    unitree_go2_spatial,
    agent(),          # in-process LLM agent — no McpServer, no external tool access
    _common_agentic,
)
```

To add MCP to a new blueprint, follow the same pattern: swap `agent()` for `McpServer.blueprint() + mcp_client()`, keep the skill containers unchanged.

---

## Agent System

This is a substantial system. Read this entire section before modifying anything agent-related.

### Overview

The agent system turns robot skills into LLM tools. An `Agent` module receives human text input, calls skills on connected skill-container modules via RPC, and publishes responses.

The full pipeline:

```
human speech → (STT) → human_input stream → Agent
Agent → LangGraph (LLM with tools) → skill RPC call → SkillContainer.some_skill()
SkillContainer → (internal logic, RPC to other modules) → returns str result
Agent → LLM sees result → SpeakSkill.speak() → TTS → robot speakers
```

### The `@skill` Decorator

Defined in `dimos/agents/annotation.py`. Sets `__rpc__ = True` and `__skill__ = True` on a method.

- `@rpc` alone: callable via RPC, not exposed to LLM
- `@skill`: implies `@rpc`, AND exposed to the LLM as a tool. Do not stack both.

When `Agent.on_system_modules()` fires, it calls `get_skills()` on every connected module. Internally it runs `langchain_core.tools.tool(method).args_schema.model_json_schema()` on each `@skill` method to build the JSON schema the LLM receives.

#### Schema generation rules

The LLM sees exactly this JSON for each skill (example output for a real skill):

```json
{
  "description": "Move the robot.\n\nArgs:\n    x: Forward velocity (m/s)\n    y: Left/right velocity (m/s)\n    duration: How long to move (seconds)",
  "properties": {
    "x":        { "title": "X", "type": "number" },
    "y":        { "title": "Y", "type": "number", "default": 0.0 },
    "duration": { "title": "Duration", "type": "number", "default": 0.0 }
  },
  "required": ["x"],
  "title": "move",
  "type": "object"
}
```

Rules that directly affect LLM behaviour:

| Rule | What happens if you break it |
|------|------------------------------|
| **Docstring is mandatory** | `get_skills()` raises `ValueError` at startup — the entire module fails to register, all its skills silently disappear |
| **Type-annotate every param** | Missing annotation → `{"title": "X"}` with no `"type"` — LLM has no type info, will guess |
| **Return `str`** | Non-str return is cast with `str()`. Return `None` and the agent hears "It has started. You will be updated later." |
| **Default values → optional** | Params with defaults are omitted from `"required"` — LLM may skip them |
| **Full docstring goes into `description` verbatim** | Keep `Args:` block concise — the whole block is in every tool-call prompt |

Supported param types: `str`, `int`, `float`, `bool`, `list[str]`, `list[float]`. Avoid complex nested types — the LLM struggles with them.

#### Minimal correct skill

```python
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module

class MySkillContainer(Module):
    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def move(self, x: float, duration: float = 2.0) -> str:
        """Move the robot forward or backward.

        Args:
            x: Forward velocity in m/s. Positive = forward, negative = backward.
            duration: How long to move in seconds.
        """
        # ... implementation
        return f"Moving at {x} m/s for {duration}s"

my_skill_container = MySkillContainer.blueprint
```

#### Dynamic docstrings for long enum lists

When the valid values list is too long to inline cleanly (e.g. arm gesture commands), set `__doc__` after the class definition. This is the pattern used in `UnitreeG1SkillContainer` and `UnitreeSkillContainer`:

```python
class MySkillContainer(Module):
    @skill
    def execute_command(self, command_name: str) -> str:
        """Placeholder — replaced below."""
        ...

_commands = "\n".join(f'- "{name}": {desc}' for name, desc in COMMANDS.items())

MySkillContainer.execute_command.__doc__ = f"""Execute a command.

Args:
    command_name: One of the following:
{_commands}
"""
```

The `tool()` call in `get_skills()` reads `__doc__` at runtime, so the replacement is picked up correctly.

### The `Agent` Module

Located at `dimos/agents/agent.py`. Key fields:

- `human_input: In[str]` — subscribes to human text. Each message triggers an LLM call.
- `agent: Out[BaseMessage]` — publishes LLM messages (for logging/visualization).
- `agent_idle: Out[bool]` — publishes True when queue is empty, False when processing.
- `config.system_prompt: str` — the system prompt passed to the LLM.
- `config.model: str` — defaults to `"gpt-4o"`. Use `"ollama:<model>"` for local models.

The agent runs a background thread (`_thread_loop`) that drains a `Queue[BaseMessage]`. Each message is processed by LangGraph's `state_graph.stream()`, which handles multi-turn tool calling automatically.

### System Prompts

Each robot has its own system prompt. They live next to the robot's code:

| Robot | File | Variable |
|-------|------|----------|
| Go2 (default) | `dimos/agents/system_prompt.py` | `SYSTEM_PROMPT` |
| G1 humanoid | `dimos/robot/unitree/g1/system_prompt.py` | `G1_SYSTEM_PROMPT` |

The `Agent` module defaults to `SYSTEM_PROMPT` (Go2/quadruped). To use a robot-specific prompt, pass it when constructing the agent blueprint:

```python
from dimos.agents.agent import agent
from dimos.robot.unitree.g1.system_prompt import G1_SYSTEM_PROMPT

_agentic_skills = autoconnect(
    agent(system_prompt=G1_SYSTEM_PROMPT),
    ...
)
```

**When adding a new robot**: create `dimos/robot/<brand>/<model>/system_prompt.py`, define `<MODEL>_SYSTEM_PROMPT`, and pass it to `agent(system_prompt=...)` in the agentic blueprint.

System prompt conventions (follow these):
- Lead with identity ("You are Daneel, ...") and safety rules
- Describe communication mode (speakers only — use `speak`)
- Document every available skill with parameters and examples
- Keep descriptions short and action-oriented

### RPC Wiring System

Skill containers often need to call methods on other modules (e.g., a navigation skill calling `NavigationInterface.set_goal`). This is done via the `rpc_calls` list and `get_rpc_calls()`.

**Declaring needed RPCs** (in a skill container):

```python
class MySkillContainer(Module):
    rpc_calls: list[str] = [
        "SomeModule.some_method",
        "AnotherModule.another_method",
    ]
```

**Using them** (inside a `@skill` method):

```python
    @skill
    def my_skill(self, arg: str) -> str:
        some_rpc = self.get_rpc_calls("SomeModule.some_method")
        result = some_rpc(arg)
        return str(result)

    # Multiple at once:
    @skill
    def multi_rpc_skill(self) -> str:
        rpc_a, rpc_b = self.get_rpc_calls("SomeModule.method_a", "AnotherModule.method_b")
        ...
```

**How wiring works**: During `Blueprint.build()`, `_connect_rpc_methods()` scans all deployed modules for their `rpc_calls` lists, finds matching `@rpc` methods in the blueprint by `"ClassName.method_name"`, and injects the callable. The lookup is:

1. Concrete class name: `"G1Connection.move"` → only matches `G1Connection`
2. Abstract base class name: `"G1ConnectionBase.move"` → matches any class where `G1ConnectionBase` is an ABC in the MRO

**Critical**: If a skill container refers to `"ConcreteClass.method"` but the blueprint deploys a different concrete class (e.g., `G1SimConnection` instead of `G1Connection`), the wiring silently fails and `get_rpc_calls()` raises `ValueError` at runtime. **Always reference the ABC name** when multiple implementations of the same interface may be deployed. See `G1ConnectionBase` below.

### Abstract Base Class Pattern for Connections

**Status: not yet on `dev` as of March 2026.** This pattern is introduced in PR #1518 (open). On `dev` right now, `G1SimConnection` extends `Module` directly and `UnitreeG1SkillContainer.rpc_calls` still references `"G1Connection.move"` — which silently fails when the sim blueprint deploys `G1SimConnection` instead. PR #1518 fixes this by adding `G1ConnectionBase`.

**Why it matters**: when a skill container lists `"G1Connection.move"` but the blueprint deploys `G1SimConnection`, the RPC wiring silently skips it. `get_rpc_calls()` then raises `ValueError` at runtime when the skill is first called — not at startup. The ABC pattern fixes this by giving both concrete classes a shared name the skill container can reference.

**The pattern** — when a robot has multiple connection types (real hardware vs simulation), define an ABC:

```python
# dimos/robot/unitree/g1/connection.py
from abc import ABC, abstractmethod
from dimos.core.core import rpc
from dimos.core.module import Module

class G1ConnectionBase(Module, ABC):
    @rpc
    @abstractmethod
    def move(self, twist: Twist, duration: float = 0.0) -> None: ...

    @rpc
    @abstractmethod
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]: ...

class G1Connection(G1ConnectionBase): ...      # real hardware
class G1SimConnection(G1ConnectionBase): ...   # MuJoCo simulation
```

Skill containers reference the base class name:

```python
class UnitreeG1SkillContainer(Module):
    rpc_calls: list[str] = [
        "G1ConnectionBase.move",          # resolves to whichever concrete class is deployed
        "G1ConnectionBase.publish_request",
    ]
```

Blueprint wiring scans the MRO of every deployed module, so `"G1ConnectionBase.move"` matches both `G1Connection` and `G1SimConnection`. If only one is in the blueprint it wires unambiguously; if both are present it raises an ambiguity error (use `.remappings()` to resolve).

Apply this pattern to any new robot that has both a real connection and a sim connection.

### Built-in Skill Containers

#### NavigationSkillContainer (`dimos/agents/skills/navigation.py`)

Inputs: `color_image: In[Image]`, `odom: In[PoseStamped]`

RPC deps: `SpatialMemory.{tag_location, query_tagged_location, query_by_text}`, `NavigationInterface.{set_goal, get_state, is_goal_reached, cancel_goal}`, `ObjectTracking.{track, stop_track, is_tracking}`

Skills:
- `navigate_with_text(query)` — 3-tier search: tagged locations → camera vision (VLM bbox) → semantic map
- `tag_location(location_name)` — saves current odometry pose under a name in SpatialMemory
- `stop_navigation()` — cancels active navigation goal

Note: `navigate_with_text` uses `QwenVlModel` internally for vision-based navigation. It is initialized in `__init__`, not `start()` — so it imports the model at module creation time.

#### SpeakSkill (`dimos/agents/skills/speak_skill.py`)

No input/output streams. No RPC deps.

Skills:
- `speak(text)` — converts text to audio via OpenAI TTS (voice: Onyx, speed: 1.2x), plays through `sounddevice`. Blocks until audio completes (timeout: `max(5, len(text) * 0.1)` seconds).

Always include `speak_skill()` in agentic blueprints. Users hear through speakers, not screens.

#### UnitreeSkillContainer — Go2 (`dimos/robot/unitree/unitree_skill_container.py`)

RPC deps: `NavigationInterface.{set_goal, get_state, is_goal_reached, cancel_goal}`, `GO2Connection.publish_request`

Skills:
- `relative_move(forward, left, degrees)` — move relative to current TF pose, blocks until done
- `wait(seconds)` — sleep
- `current_time()` — returns current datetime
- `execute_sport_command(command_name)` — sends WebRTC sport command (e.g., "FrontFlip", "Sit", "Dance1"). ~40 commands available.

**Go2 important**: always call `execute_sport_command("RecoveryStand")` after dynamic movements (flips, jumps, sit) before navigating.

#### UnitreeG1SkillContainer — G1 humanoid (`dimos/robot/unitree/g1/skill_container.py`)

RPC deps: `G1ConnectionBase.move`, `G1ConnectionBase.publish_request`

Skills:
- `move(x, y=0, yaw=0, duration=0)` — direct velocity control. `x` = forward m/s, `y` = left/right m/s, `yaw` = rotational rad/s
- `execute_arm_command(command_name)` — arm gestures via `rt/api/arm/request` (api_id 7106): "Handshake", "HighFive", "Hug", "HighWave", "Clap", "FaceWave", "LeftKiss", "ArmHeart", "RightHeart", "HandsUp", "XRay", "RightHandUp", "Reject", "CancelAction"
- `execute_mode_command(command_name)` — gait modes via `rt/api/sport/request` (api_id 7101): "WalkMode", "WalkControlWaist", "RunMode"

Both arm/mode commands use `difflib.get_close_matches` to suggest alternatives on typos.

### Agentic Blueprint Composition Pattern

Agentic stacks are built in layers. Each layer is an `autoconnect()` call stored as a module-level variable:

```
unitree_g1_basic_sim          (hardware: G1SimConnection)
    ↓ composed by
unitree_g1_sim                (+ perception + mapping + memory, n_workers=8)
    ↓ composed by
unitree_g1_agentic_sim        (+ agent + navigation_skill + speak_skill + g1_skills)
```

The `_agentic_skills` private blueprint (underscore prefix = not directly runnable) holds just the agent + skill containers. The top-level blueprint `autoconnect`s the perceptive stack with the agentic skills.

File: `dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`
```python
_agentic_skills = autoconnect(
    agent(system_prompt=G1_SYSTEM_PROMPT),
    navigation_skill(),
    speak_skill(),
    g1_skills(),
)
```

File: `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_agentic_sim.py`
```python
unitree_g1_agentic_sim = autoconnect(unitree_g1_sim, _agentic_skills)
```

Go2 uses the same pattern — `_common_agentic` in `dimos/robot/unitree/go2/blueprints/agentic/_common_agentic.py` contains `navigation_skill + person_follow_skill + unitree_skills + web_input + speak_skill`.

### How to Add a New Skill

1. Choose the right skill container. If it needs hardware RPC calls, put it in the robot-specific container. If it's generic (navigation, speech), add it in `dimos/agents/skills/`.

2. Decorate with `@skill`. Write a clear docstring — the LLM reads it. Include parameter descriptions and an example call.

3. If the skill needs to call another module's RPC, add the method to `rpc_calls` and call `self.get_rpc_calls("ClassName.method")` inside the skill.

4. Return a descriptive `str` always.

5. Update the system prompt for any robot that uses this skill — add it to the `# AVAILABLE SKILLS` section.

6. If creating a new skill container module, expose it as `my_container = MySkillContainer.blueprint` and add it to the relevant agentic blueprint.

### How to Add a New Robot Connection

If the robot has both real hardware and simulation variants:

1. Create `<RobotName>ConnectionBase(Module, ABC)` with `@rpc @abstractmethod` for every shared method.
2. Have both `<RobotName>Connection(ConnectionBase)` and `<RobotName>SimConnection(ConnectionBase)` extend it.
3. In the skill container, reference `"<RobotName>ConnectionBase.<method>"` (not the concrete class).
4. Export the base class in `__all__`.

---

## Testing

```bash
# Fast tests (default — pyproject.toml addopts excludes slow, tool, mujoco)
uv run pytest

# Include slow tests (what CI runs)
./bin/pytest-slow

# Single file
uv run pytest dimos/core/test_blueprints.py -v

# Mypy
uv run mypy dimos/
```

Use `uv run` to ensure the venv and deps are correct.

**`uv run pytest` runs fast tests only** — `addopts` in `pyproject.toml` includes `-m 'not (tool or slow or mujoco)'`.

**CI runs `./bin/pytest-slow`** which uses `-m 'not (tool or mujoco)'` — includes slow tests but excludes tool and mujoco.

## Pre-commit & Code Quality

Pre-commit runs automatically on `git commit`. Includes ruff format, ruff check, license headers, LFS checks, doclinks.

**Known issue**: `doclinks` hook fails with `Executable 'python' not found`. Bypass with `SKIP=doclinks git commit -m "..."`.

**Always activate the shared venv before committing:**
```bash
source .venv/bin/activate
```

## Code Style Rules

- **Imports at top of file.** No inline imports unless there's a circular dependency.
- **Use `requests` for HTTP**, not `urllib.request`. It's an explicit dependency.
- **Use `Any` not `object`** for JSON value types.
- **Prefix non-CI manual test scripts with `demo_`** so they're excluded from pytest collection.
- **Don't hardcode ports/URLs.** Use `GlobalConfig` constants.
- **Type annotations required.** Mypy strict mode. Use `type: ignore` sparingly and only with specific error codes.

## `all_blueprints.py` is auto-generated

`dimos/robot/all_blueprints.py` is generated by `test_all_blueprints_generation.py`. After adding/renaming blueprints, run:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

This regenerates the file locally. In CI (`CI=1`), it asserts the file is current instead of regenerating.

## Git Workflow

- **Branches**: `feat/`, `fix/`, `refactor/`, `docs/`, `test/`, `chore/`, `perf/`
- **PRs target `dev`**, never push to `main` or `dev` directly
- **Don't force-push** unless after a rebase with conflicts
- **Minimize pushes to origin** — every push triggers CI (~1 hour on self-hosted runners)
- **Batch commits locally**, verify with `git diff origin/dev..HEAD`, then push once

## LFS Data

Test replay data is stored in Git LFS under `data/.lfs/`. The `get_data()` function handles extraction:

```python
from dimos.utils.data import get_data
path = get_data("go2_sf_office")  # auto-downloads from LFS if needed
```

After checking out a branch with new LFS data: `git lfs pull`.

## Common Pitfalls

1. **`all_blueprints.py` out of date** — CI will fail with `test_all_blueprints_is_current`. Run the generation test locally.
2. **`pyproject.toml` changes trigger full CI** (~1 hour). Include `uv.lock` in the same push.
3. **Forkserver workers** — modules run in separate processes. Don't rely on shared mutable state.
4. **DimOS type system** — stamped types inherit from base types. `TwistStamped` IS a `Twist`. Check class hierarchy before claiming type mismatches.
5. **`ReplayConnection.stop()` bug** — crashes with `AttributeError: stop_timer`. Known issue, pre-existing.
6. **macOS** — `kern.ipc.maxsockbuf` caps at 6291456. LCM multicast needs explicit sysctl config. Some tests excluded on macOS CI.
7. **RPC wiring with concrete class names** — if a skill container lists `"G1Connection.move"` but the blueprint deploys `G1SimConnection`, wiring silently fails. Reference the ABC (`"G1ConnectionBase.move"`) when multiple implementations may be deployed. This was the bug fixed in PR #1518.
8. **Wrong system prompt for robot** — `Agent` defaults to the Go2 quadruped `SYSTEM_PROMPT`. Humanoid (G1) blueprints must explicitly pass `agent(system_prompt=G1_SYSTEM_PROMPT)`. A mismatched prompt causes the LLM to hallucinate wrong skills.
9. **Skill docstrings are the LLM's API docs** — the docstring is the only description the LLM gets. Missing or vague docstrings cause bad tool calls. Always include parameter descriptions and an example call.
10. **`@skill` without `@rpc`** — `@skill` implies `@rpc`, don't stack both decorators. The `annotation.skill` decorator sets `__rpc__ = True` itself.
11. **`get_rpc_calls()` raises at runtime** — not at blueprint build time. If a module is missing from the blueprint or the method name is wrong, you'll only find out when the skill is called. Test with `dimos --simulation run <blueprint>` before deploying.
12. **`speak_skill` missing from agentic blueprint** — without it, the agent has no way to communicate with users (they hear audio, not text). Always include `speak_skill()` in any agentic blueprint.

## Viewer Backends

- **`rerun` (default)**: Native [dimos-viewer](https://github.com/dimensionalOS/dimos-viewer) — custom Rerun fork with built-in teleop + click-to-navigate.
- **`rerun-web`**: Browser dashboard at `http://localhost:7779`. Includes command center + Rerun 3D viewer.
- **`foxglove`**: Foxglove bridge on `ws://localhost:8765`.

Set via CLI (`--viewer rerun-web`) or env var (`VIEWER=rerun-web`).

## Replay Mode

`dimos --replay run <blueprint>` uses recorded sensor data instead of a real robot. Data loops continuously. Replay data lives in `data/` (extracted from LFS tarballs in `data/.lfs/`).

## Further Reading

- **Module system**: `docs/usage/modules.md`
- **Blueprints**: `docs/usage/blueprints.md`
- **Transports**: `docs/usage/transports/`
- **Visualization**: `docs/usage/visualization.md`
- **Testing**: `docs/development/testing.md`
- **CLI / dimos run**: `docs/development/dimos_run.md`
- **LFS data**: `docs/development/large_file_management.md`
- **Agent system**: `docs/agents/`
- **G1 humanoid platform**: `docs/platforms/humanoid/g1/index.md`
- **Go2 quadruped platform**: `docs/platforms/quadruped/go2/index.md`
