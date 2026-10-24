#!/usr/bin/env python3
"""Attach the real Gazebo GUI to the existing school server on a local X11 desktop."""

import os
import re
import shutil
import signal
import subprocess
import uuid


def main():
    display = os.environ.get("DISPLAY", "")
    match = re.fullmatch(r":(\d+)(?:\.\d+)?", display)
    if match is None:
        raise RuntimeError("A local X11 DISPLAY is required (for example :0)")
    for command in ("docker", "xauth"):
        if not shutil.which(command):
            raise RuntimeError(f"{command} is required")
    subprocess.run(
        ["docker", "inspect", "nv-sim"], check=True, stdout=subprocess.DEVNULL
    )
    desktop = (
        subprocess.run(
            ["docker", "context", "show"], check=True, capture_output=True, text=True
        ).stdout.strip()
        == "desktop-linux"
    )
    auth = f"/tmp/nv-gazebo-{uuid.uuid4().hex}.xauth"
    relay = None
    try:
        cookies = subprocess.run(
            ["xauth", "nlist", display], check=True, capture_output=True
        ).stdout
        if not cookies.strip():
            raise RuntimeError("No X11 authorization cookie found for DISPLAY")
        # Match the container's display hostname without printing or weakening X server access.
        wildcard = (
            b"\n".join(b"ffff" + line[4:] for line in cookies.splitlines()) + b"\n"
        )
        subprocess.run(
            ["docker", "exec", "-i", "nv-sim", "xauth", "-f", auth, "nmerge", "-"],
            input=wildcard,
            check=True,
            capture_output=True,
        )
        if desktop:
            if not shutil.which("socat"):
                raise RuntimeError("Docker Desktop GUI requires socat on the host")
            number = int(match[1])
            relay = subprocess.Popen(
                [
                    "socat",
                    f"TCP-LISTEN:{6000 + number},bind=127.0.0.1,reuseaddr,fork",
                    f"UNIX-CONNECT:/tmp/.X11-unix/X{number}",
                ],
                start_new_session=True,
            )
            display = f"host.docker.internal:{number}"
        # Preserve Gazebo's installed GUI plugins, with a camera covering the campus.
        subprocess.run(
            [
                "docker",
                "exec",
                "nv-sim",
                "python3",
                "-c",
                "from pathlib import Path; p=Path('/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config'); s=p.read_text().replace('<width>1000</width>','<width>1400</width>',1).replace('<height>845</height>','<height>1000</height>',1); import re; s,n=re.subn(r'<camera_pose>.*?</camera_pose>', '<camera_pose>110 -70 95 0 0.6 2.25</camera_pose>',s,count=1); assert n==1; Path('/tmp/nv-school-gui.config').write_text(s)",
            ],
            check=True,
        )
        if relay is not None and relay.poll() is not None:
            raise RuntimeError(
                "X11 relay failed; close the previous GUI relay before retrying"
            )
        print(
            "Opening Gazebo Harmonic GUI; the existing school simulation keeps running.",
            flush=True,
        )
        subprocess.run(
            [
                "docker",
                "exec",
                "-e",
                f"DISPLAY={display}",
                "-e",
                f"XAUTHORITY={auth}",
                "-e",
                "QT_X11_NO_MITSHM=1",
                "-e",
                "LIBGL_ALWAYS_SOFTWARE=1",
                "-e",
                "GZ_SIM_RESOURCE_PATH=/tmp/nv-school-models:/opt/px4-gazebo/share/gz/models",
                "nv-sim",
                "/usr/local/bin/nv-entrypoint.sh",
                "gz",
                "sim",
                "-g",
                "--gui-config",
                "/tmp/nv-school-gui.config",
            ],
            check=True,
        )
    finally:
        if relay is not None:
            try:
                os.killpg(relay.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            relay.wait(timeout=5)
        subprocess.run(
            [
                "docker",
                "exec",
                "nv-sim",
                "python3",
                "-c",
                "from pathlib import Path; import sys; Path(sys.argv[1]).unlink(missing_ok=True)",
                auth,
            ],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error)) from error
