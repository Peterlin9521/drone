# Onboard code (Orange Pi Zero 3W)

The Pi talks to the flight controller over MAVLink on UART2. mavlink-router shares that link on
TCP port 5678, so these scripts and Mission Planner can be connected at the same time.

## `cpu/` - detection on the CPU (ONNX Runtime, YOLOv8n)
| Script | What it does |
|---|---|
| `detect_phone.py` | Single-image phone detection with timing benchmark (pre-/post-processing + inference) |
| `yolo_stream.py` | Live detection (Open Images V7, 601 classes) streamed as MJPEG over HTTP |
| `phone_land.py` | Commands LAND when a phone is seen in N consecutive frames (`--live` to actually send) |
| `drone_ai.py` | Person follow-me in GUIDED mode plus phone-triggered landing |
| `track_person.py`, `follow_me.py`, `follow_track.py` | Person detection, centroid tracking with target lock, follow-me velocity control |

## `npu/` - detection on the A733 NPU
| File | What it does |
|---|---|
| `npu_lib.c` | C wrapper around the VIPLite driver: load the NBG model once, run per frame, return raw outputs |
| `npu_yolo.py` | Python (ctypes) wrapper: letterbox, planar CHW RGB input, dequantize, NMS. ~8.4 ms/frame |
| `npu_stream.py` | Live NPU detection at the camera's 30 fps |
| `follow_track_npu.py` | Follow-me tracker running on the NPU |
| `tools/` | Debugging scripts used while converting the model (ONNX golden reference, input-layout checks) |

Build: `gcc -O2 -shared -fPIC npu_lib.c -o libnpu.so -lVIPhal -lNBGlinker`

## `agent/` - LLM in the loop
| Script | What it does |
|---|---|
| `dronefly.py` | The LLM decides whether the drone keeps flying or lands; the decision is validated in Python before any MAVLink command is sent |
| `dronejoke_agent.py` | Two-step LLM pipeline (generate, then yes/no judgement) with the answer checked in code |
| `dronejoke.py` | Minimal LLM API call |

The API key is read from the `PORTKEY_API_KEY` environment variable and is never stored in the repo.

## `mavlink/` - flight utilities
| Script | What it does |
|---|---|
| `fly_cmd.py` | Command-line status / takeoff over MAVLink (`--live` to actually arm and fly) |
| `land_now.py` | Emergency LAND: waits for heartbeat, switches mode, confirms |
| `uc.py` | Start / stop the mavlink-router service |

## `config/`
mavlink-router configuration and Pi boot settings (UART2 overlay, console moved off the UART).
