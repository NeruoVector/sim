#!/usr/bin/env bash
# 호스트 준비 스크립트: Docker 설치 확인 + X11 허용 + 이미지 빌드
#
# 이 머신(Debian 13)에는 Docker 가 기본 설치되어 있지 않습니다.
# sudo 비밀번호가 필요하므로 설치 단계는 직접 실행하세요.
set -euo pipefail

info() { printf '\033[1;34m[sim]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[sim]\033[0m %s\n' "$*"; }

cd "$(dirname "$0")/.."

# 1) Docker
if ! command -v docker >/dev/null 2>&1; then
    warn "Docker 가 설치되어 있지 않습니다. 아래를 실행한 뒤 다시 시도하세요:"
    cat <<'EOF'

  # Debian / Ubuntu
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl gnupg
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/debian/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/debian $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
      docker-buildx-plugin docker-compose-plugin
  sudo usermod -aG docker "$USER"   # 재로그인 필요

EOF
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    warn "Docker 데몬에 접근할 수 없습니다. 'sudo usermod -aG docker \$USER' 후 재로그인하세요."
    exit 1
fi

info "docker: $(docker --version)"

# 2) .env
if [ ! -f .env ]; then
    cp .env.example .env
    info ".env 생성됨 (.env.example 복사)"
fi

# 3) X11 (GUI 사용 시)
if [ -n "${DISPLAY:-}" ]; then
    if command -v xhost >/dev/null 2>&1; then
        xhost +local:root >/dev/null
        info "X11 접근 허용 (xhost +local:root)"
    fi
else
    warn "DISPLAY 가 없습니다. GUI 대신 헤드리스(HEADLESS=1)로 실행하세요."
fi

# 4) 이미지 빌드
info "시뮬레이션 이미지 빌드 중... (최초 1회 수 GB 다운로드)"
docker compose build

info "완료. 다음 명령으로 기동하세요:"
cat <<'EOF'

  make up          # 컨테이너 기동
  make agent       # (터미널 2) Micro XRCE-DDS Agent
  make px4         # (터미널 3) PX4 SITL + Gazebo GUI
  make foxglove    # (터미널 4) Foxglove bridge -> ws://localhost:8765

EOF
