#!/usr/bin/env python3
"""Record the interaction pole's force/torque sensor to CSV.

The interaction_pole model in the mocap_interaction world carries a
force_torque sensor on its base joint, measuring the wrench the pole delivers
into the floor. PX4 never sees that model, so unlike the vehicle-side rod
sensor (which gz_bridge publishes as the interaction_wrench uORB topic and the
logger writes into the ulog) there is nothing to record it. This script does.

It shells out to `gz topic -e` and parses the text output rather than using the
gz Python bindings, deliberately. The bindings need the `protobuf` pip package
and, worse, a version-pinned import: `gz.msgs10` on Harmonic, `gz.msgs11`
elsewhere. Parsing the CLI works against whatever `gz` is on PATH, which is
already a hard requirement for running the simulation at all.

Sign convention, verified in simulation:

    force z   about -245 N standing, the pole's own 25 kg pressing on the floor
    force x   POSITIVE when a vehicle pushes the panel in +X

Taring is on by default and removes that standing offset, so a run that starts
before contact reads about zero until the vehicle arrives.

Examples
--------
Log until Ctrl-C, into a timestamped file:

    Tools/dtrg/log_interaction_force.py

Log 60 seconds to a named file, no taring, keeping the raw standing load:

    Tools/dtrg/log_interaction_force.py --duration 60 --no-tare -o push.csv

A pole placed somewhere other than the default world or name:

    Tools/dtrg/log_interaction_force.py --world my_world --model pole_2

Read it back:

    import pandas as pd
    df = pd.read_csv("push.csv", comment="#")

Gotchas
-------
1. Start this after the world is up. `gz topic -e` on a topic that does not
   exist yet blocks silently instead of failing, so the script checks the topic
   is advertised first and tells you if it is not. PX4 runs the Gazebo server
   with GZ_IP=127.0.0.1, without which a `gz` client discovers nothing; the
   script retries with that set, so it works whether or not you exported it.
2. Taring averages the first --tare-samples samples, so do not be in contact
   with the panel when the script starts, or the offset absorbs part of the
   push. The tare offsets are printed and written into the CSV header comment.
3. Sim time comes from the message header, so it follows Gazebo's clock and
   pauses when the simulation pauses. Wall time does not. Use sim_time_s when
   correlating with a ulog.
4. Components that are exactly zero are omitted from the gz text output; this
   parser defaults them to 0.0 rather than dropping the sample.
"""

import argparse
import datetime
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path
import time

DEFAULT_WORLD = "mocap_interaction"
DEFAULT_MODEL = "interaction_pole"
DEFAULT_JOINT = "pole_base"
DEFAULT_SENSOR = "reaction_force"

COLUMNS = ["sim_time_s", "wall_time_s", "fx", "fy", "fz", "tx", "ty", "tz"]


def build_topic(world, model, joint, sensor):
    return f"/world/{world}/model/{model}/joint/{joint}/sensor/{sensor}/forcetorque"


def resolve_env(topic):
    """Return an environment in which `gz` can see `topic`, or None.

    PX4 starts the Gazebo server with GZ_IP=127.0.0.1, and a `gz` client
    without that set discovers nothing at all: `gz topic -l` comes back empty
    and `gz topic -e` blocks forever on a topic that is plainly working. So try
    the environment as-is first, then again with GZ_IP pinned to loopback.
    """
    candidates = [os.environ.copy()]

    if "GZ_IP" not in os.environ:
        loopback = os.environ.copy()
        loopback["GZ_IP"] = "127.0.0.1"
        candidates.append(loopback)

    for env in candidates:
        try:
            out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                                 text=True, timeout=10, env=env).stdout
        except (subprocess.TimeoutExpired, OSError):
            continue

        if topic in out.split():
            return env

    return None


def parse_messages(lines):
    """Yield (sim_time_s, force, torque) per message from `gz topic -e` output.

    The text format nests one level: `force {` / `torque {` / `header { stamp {`.
    Zero components are omitted entirely, so every field defaults to 0.0. A
    message ends when brace depth returns to zero.
    """
    depth = 0
    section = None
    in_stamp = False
    sec = nsec = 0
    force = [0.0, 0.0, 0.0]
    torque = [0.0, 0.0, 0.0]
    started = False

    for raw in lines:
        line = raw.strip()
        if not line:
            continue

        if line.endswith("{"):
            name = line[:-1].strip()
            if depth == 0:
                started = True
                section = name
            elif depth == 1 and section == "header" and name == "stamp":
                in_stamp = True
            depth += 1
            continue

        if line == "}":
            depth -= 1
            if depth == 1 and in_stamp:
                in_stamp = False
            elif depth == 0 and section == "torque":
                # torque is the last top-level field, so the message is complete
                yield (sec + nsec * 1e-9, tuple(force), tuple(torque))
                started = False
                sec = nsec = 0
                force = [0.0, 0.0, 0.0]
                torque = [0.0, 0.0, 0.0]
            continue

        if not started or ":" not in line:
            continue

        key, _, value = line.partition(":")
        key = key.strip()
        value = value.strip()

        if in_stamp:
            if key == "sec":
                sec = int(value)
            elif key == "nsec":
                nsec = int(value)
            continue

        if key in ("x", "y", "z"):
            idx = "xyz".index(key)
            try:
                number = float(value)
            except ValueError:
                continue
            if section == "force":
                force[idx] = number
            elif section == "torque":
                torque[idx] = number


def main():
    parser = argparse.ArgumentParser(
        description="Record the interaction pole force/torque sensor to CSV.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Examples")[1] if "Examples" in __doc__ else None)
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--joint", default=DEFAULT_JOINT)
    parser.add_argument("--sensor", default=DEFAULT_SENSOR)
    parser.add_argument("--topic", help="full topic, overriding the parts above")
    parser.add_argument("-o", "--output", help="CSV path (default: timestamped)")
    parser.add_argument("--duration", type=float,
                        help="seconds to record (default: until Ctrl-C)")
    parser.add_argument("--tare-samples", type=int, default=250, metavar="N",
                        help="samples averaged for the zero offset (default: 250, ~1 s)")
    parser.add_argument("--no-tare", action="store_true",
                        help="keep the raw reading, including the pole's standing weight")
    parser.add_argument("--quiet", action="store_true", help="no live status line")
    args = parser.parse_args()

    if shutil.which("gz") is None:
        sys.exit("error: `gz` not found on PATH; source the PX4 gz environment first")

    topic = args.topic or build_topic(args.world, args.model, args.joint, args.sensor)

    env = resolve_env(topic)

    if env is None:
        sys.exit(f"error: topic not advertised: {topic}\n"
                 "       is the world running, and does the model carry a force sensor?\n"
                 "       `GZ_IP=127.0.0.1 gz topic -l | grep forcetorque` lists what is available")

    directory = Path("result")
    directory.mkdir(parents=True, exist_ok=True)
    output = args.output or datetime.datetime.now().strftime(
        "result/interaction_force_%Y%m%d_%H%M%S.csv")

    proc = subprocess.Popen(["gz", "topic", "-e", "-t", topic],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, bufsize=1, env=env)

    # Ctrl-C should close the CSV cleanly rather than traceback.
    stopping = {"now": False}

    def stop(_signum, _frame):
        stopping["now"] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    tare = [0.0] * 6
    tare_acc = []
    started = time.monotonic()
    count = 0

    print(f"topic  {topic}")
    print(f"output {output}")
    if args.no_tare:
        print("tare   disabled, raw reading including standing weight")
    else:
        print(f"tare   averaging first {args.tare_samples} samples; keep clear of the panel")

    try:
        with open(output, "w", buffering=1) as csv:
            csv.write(f"# topic: {topic}\n")
            csv.write("# force N, torque N.m, pole frame (world aligned at yaw 0)\n")
            csv.write("# fx positive = vehicle pushing the panel in +X\n")
            header_written = False

            for sim_t, force, torque in parse_messages(proc.stdout):
                if stopping["now"]:
                    break
                if args.duration and time.monotonic() - started >= args.duration:
                    break

                sample = list(force) + list(torque)

                if not args.no_tare and len(tare_acc) < args.tare_samples:
                    tare_acc.append(sample)
                    if len(tare_acc) == args.tare_samples:
                        tare = [sum(col) / len(col) for col in zip(*tare_acc)]
                        csv.write("# tare offsets subtracted: "
                                  + " ".join(f"{v:.4f}" for v in tare) + "\n")
                    continue

                if not header_written:
                    if args.no_tare:
                        csv.write("# tare: none\n")
                    csv.write(",".join(COLUMNS) + "\n")
                    header_written = True

                values = [s - t for s, t in zip(sample, tare)]
                csv.write(f"{sim_t:.6f},{time.time():.6f},"
                          + ",".join(f"{v:.6f}" for v in values) + "\n")
                count += 1

                if not args.quiet and count % 25 == 0:
                    print(f"\r{count:7d} samples  "
                          f"fx {values[0]:8.2f}  fy {values[1]:8.2f}  fz {values[2]:8.2f} N",
                          end="", flush=True)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    if not args.quiet:
        print()
    print(f"wrote {count} samples to {output}")
    if count == 0:
        print("note: no samples recorded; is the simulation running rather than paused?")


if __name__ == "__main__":
    main()
