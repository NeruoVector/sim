# 3D 모델 (Gazebo)

드론/환경/소품 3D 모델은 **별도 레포를 만들지 않고 이 `sim` 레포에서 관리**합니다.
큰 바이너리 파일은 Git LFS 로 저장되므로, 모델을 추가하기 전에 아래를 한 번 실행하세요.

```bash
# Debian/Ubuntu
sudo apt-get install git-lfs
# macOS
brew install git-lfs

git lfs install
```

> `git lfs install` 을 하지 않고 clone 하면 모델 파일이 포인터 텍스트로 보입니다.
> 이미 clone 한 뒤라면 `git lfs pull` 로 실제 파일을 받을 수 있습니다.
>
> GitHub 웹 UI 의 drag & drop 업로드는 LFS 필터를 거치지 않습니다.
> 반드시 git-lfs 가 설치된 로컬 clone 에서 `git add` 후 push 하세요.

## 디렉터리 구조

```
models/
├── drones/        # PX4 기체 외형 덮어쓰기 모델 (x500_base)
├── environments/  # 월드/환경 모델
└── props/         # 나무, 건물, 표지판 등 소품
```

Gazebo 모델은 보통 아래처럼 구성합니다.

```text
models/drones/neurovector-x1/
├── model.config
├── model.sdf
└── meshes/
    └── neurovector-x1.dae
```

`model.config` / `model.sdf` 는 텍스트이므로 LFS 대상이 아닙니다.
`.dae`, `.stl`, `.obj`, `.fbx`, `.glb`, `.blend`, `.step`, `.usd*` 와
`models/` 아래 텍스처/`.bin`/`.zip` 파일이 자동으로 LFS 에 들어갑니다.

## 포함된 샘플 모델

`models/props/` 에 Gazebo Fuel CC0 샘플 10개가 들어 있습니다.
각 디렉터리의 `ATTRIBUTION.md` 에 원본 URL 과 라이선스가 기록되어 있습니다.

```text
models/props/
├── construction-barrel/
├── construction-cone/
├── jersey-barrier/
├── fire-hydrant/
├── oak-tree/
├── pine-tree/
├── prius-hybrid/
├── office-building/
├── dumpster/
└── walking-actor/
```

학교 데모는 `environments/school-demo/meshes/school_map.stl` 을 정적 월드 메시로,
`drones/x500_base/meshes/drone.stl` 을 PX4 x500의 외형으로 사용합니다. PX4의
x500 충돌 형상과 비행 물리는 유지합니다.

`model://construction-barrel` 처럼 디렉터리 이름으로 참조할 수 있습니다.
샘플을 다시 받으려면 `python3 scripts/fetch-sample-models.py --force` 를 실행하세요.
모델 배치 예시는 [`../README.md`](../README.md) 의 "샘플(더미) 모델" 을 참고하세요.

## 모델 추가하기

```bash
# sim 레포 루트에서
cp -R /path/to/my-drone models/drones/my-drone

git add models/drones/my-drone
git status                 # .dae 등이 LFS 로 잡히는지 확인
git lfs ls-files           # LFS 로 추적 중인 파일 목록
git commit -m "feat: my-drone Gazebo 모델 추가"
git push
```

컨테이너에서는 `models/` 가 `/models` 로 마운트되고
`GZ_SIM_RESOURCE_PATH` 에 자동으로 추가됩니다. Gazebo 실행 후
`gz model --list` 또는 월드에서 모델 URI(`model://my-drone`)로 사용할 수 있습니다.

## 주의 사항

- **이미 커밋한 큰 파일**은 일반 Git 객체로 들어가 있습니다. 히스토리를 정리하려면
  `git lfs migrate import --include="models/**"` 를 사용해야 하며, 이는 커밋 해시를
  바꾸므로 팀과 합의 후 진행하세요.
- GitHub LFS 는 플랜별 저장 용량/대역폭 quota 가 있습니다. 텍스처는 가능하면
  2K 이하로 줄이고, 메쉬는 decimate 해서 올리는 것을 권장합니다.
- `model.config`, `model.sdf`, `*.urdf`, `*.xacro` 같은 텍스트 파일은 LFS 로
  보내지 마세요. diff 와 리뷰가 불가능해집니다.
- 실제 기체의 CAD/STL 원본은 `sim` 이 아니라 `drone` 레포가 더 적합합니다.
  이 디렉터리는 Gazebo/PX4 시뮬레이션용 에셋을 위한 공간입니다.
