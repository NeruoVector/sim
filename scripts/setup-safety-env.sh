#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
command -v uv >/dev/null
uv tool install conan==2.33.0
conan profile detect --exist-ok
if ! docker info >/dev/null 2>&1; then
    if command -v systemctl >/dev/null && systemctl --user cat docker-desktop.service >/dev/null 2>&1; then
        systemctl --user start docker-desktop
    else
        echo 'Start a Docker daemon before retrying setup-safety-env.sh.' >&2
        exit 1
    fi
fi
for attempt in {1..30}; do
    if docker info >/dev/null 2>&1; then break; fi
    sleep 1
done
docker info >/dev/null
# No desktop socket is required for headless safety checks.
mkdir -p logs/x11
export SIM_X11_SOCKET="${SIM_X11_SOCKET:-$(pwd)/logs/x11}"
export HEADLESS=1
compose_args=(-f compose.yaml)
if [ "$(docker context show)" = desktop-linux ]; then
    compose_args+=(-f compose.desktop.yaml)
fi
docker compose "${compose_args[@]}" --profile sim up -d --no-build sim
make safety-build
