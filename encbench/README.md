# Status encoding benchmark: FlatBuffers vs JSON deltas

Prototype measurements for the NML replacement discussion (LinuxCNC discussion #4571). Not meant for merging.

Compares, on real EmcStat frames captured from task while a sim runs:

- FlatBuffers, as encoded by the websocket server on this branch;
- JSON full status;
- JSON merge-patch deltas (RFC 7386) against the previous status, with joint/axis/spindle arrays keyed by index;
- the same delta as CBOR.

## Pieces

- `src/emc/task/ws_server.cc`: two env-driven hooks in `serialize_and_fanout()`.
  - `LINUXCNC_WS_CAPTURE=path` (and `LINUXCNC_WS_CAPTURE_MAX`, default 50000) writes every encoded EmcStat as a 4-byte length plus buffer.
  - `LINUXCNC_WS_TIMING=1` times the real FlatBuffers builder, the reflection JSON and a JSON delta from EMC_STAT on every frame and prints averages to stderr every 10000 frames. `LINUXCNC_WS_TIMING_DUMP=path` keeps snapshots and deltas of the first 5000 frames for a replay check.
- `src/emc/task/json_delta.hh`: EMC_STAT to JSON delta, written straight to a string.
- `gen_json.py`: from the .fbs, generates `to_json` and a field-by-field `diff_json` for the object-API types (nlohmann/json).
- `gen_apply.py`: from the .fbs, generates `apply_json`, a simdjson on-demand decoder that applies a delta straight into the object-API status struct.
- `bench.cc`: offline sizes and timings on a capture, with replay checks.
- `pybench.py`: Python decode timings (pure Python FlatBuffers vs json + merge).
- `run3/`, `run9/`: headless captures. `driver.py` is the DISPLAY: machine on, home, run a program, a few MDI moves, estop. `run3` is the axis 3-axis sim with 3D_Chips.ngc, `run9` the 9-axis sim with `nine.ngc` moving eight axes at once (B is a locking indexer and must move alone).

## Running

Debian packages: flatbuffers-compiler, libflatbuffers-dev, python3-flatbuffers, nlohmann-json3-dev, libsimdjson-dev.

    # build the branch (rip), then capture
    encbench/run3/run.sh          # writes run3/frames.bin
    encbench/run3/run_timing.sh   # in-process timings in run3/timing.log

    cd encbench
    F=../src/emc/flatbuf
    flatc --cpp --gen-object-api --gen-mutable --reflect-names -o gen $F/emc_ws.fbs $F/emc_stat.fbs $F/emc_common.fbs $F/emc_error.fbs $F/emc_cmd.fbs
    flatc --python --gen-object-api -o pygen $F/emc_stat.fbs $F/emc_common.fbs
    python3 gen_json.py $F/emc_common.fbs $F/emc_stat.fbs > tojson.hh
    python3 gen_apply.py $F/emc_common.fbs $F/emc_stat.fbs > applyjson.hh
    g++ -O2 -std=c++20 -I. -o bench bench.cc -lsimdjson
    ./bench run3/frames.bin run3/delta     # also writes run3/delta_*.jsonl for pybench.py
    python3 pybench.py

A 40000 frame capture is about 140 MB and the bench holds all frames unpacked in memory (about 2 GB).
