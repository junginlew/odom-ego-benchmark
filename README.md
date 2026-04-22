# WorldModel 좌표계 연산량 비교 벤치마크

Odom(Odometry) 좌표계와 Egocentric 좌표계에서 포인트 클라우드 지도를 누적할 때 발생하는 연산 비용을 비교하는 벤치마크.

---

## 프로젝트 개요

로봇이나 자율주행 차량이 이동하면서 3D 지도를 쌓는 방식은 크게 두 가지이다.

- **Odom 방식**: 로봇의 위치 고정된 월드 좌표계에서 추적하고, 포인트 클라우드를 월드 기준으로 누적
- **Egocentric 방식**: 항상 현재 로봇을 중심으로 좌표계를 유지하고, 로봇이 이동할 때마다 기존 지도를 현재 시점 기준으로 재변환

두 방식은 지도의 표현 방식이 다르며, 프레임이 쌓일수록 연산 비용에 차이가 생긴다. 이 벤치마크는 그 차이를 실측한다.

---

## 벤치마킹 대상 연산

### Odom (Odometry 좌표계)

매 프레임마다 **새로 들어온 포인트만** 월드 좌표로 변환한다.

```
transformed = current_pose @ new_points_homo.T
```

- 변환 대상: 현재 프레임의 신규 포인트 (`N_new`개)
- 누적된 기존 지도는 건드리지 않음
- 시간 복잡도: **O(N_new)** — 프레임 수에 무관하게 일정

### Egocentric (로봇 중심 좌표계)

매 프레임마다 **누적된 전체 지도**를 현재 로봇 시점 기준으로 재변환한다.

```
relative_transform = inv(current_pose) @ prev_pose
ego_map = relative_transform @ ego_map_homo.T
```

- 변환 대상: 지금까지 쌓인 모든 포인트 (`N_total`개)
- 로봇이 이동할 때마다 전체 지도를 새로운 로봇 시점으로 옮겨야 함
- 시간 복잡도: **O(N_total)** — 프레임이 쌓일수록 선형 증가

### 공정한 비교를 위한 처리

LiDAR 좌표를 카메라(로봇 중심) 좌표로 변환하는 Tr 행렬 적용은 두 방식 모두에 공통으로 필요한 전처리이므로, **타이밍 측정 범위 밖**에서 수행한다.

```python
# 타이밍 밖: LiDAR → 카메라 좌표 (공통 전처리)
pts_cam = (loader.calib["Tr"] @ new_points_homo.T).T

# 타이밍 안: Odom — 새 포인트만 월드로 변환
transformed_new_points = (current_pose @ pts_cam.T).T[:, :3]

# 타이밍 안: Ego — 전체 지도를 현재 시점으로 재변환
relative_transform = inv(current_pose) @ prev_pose
ego_map = (relative_transform @ ego_map_homo.T).T[:, :3]
```

---

## 좌표계 설명

```
LiDAR 좌표 (로봇 중심)
    │
    │  Tr (calib.txt의 변환 행렬, 4×4)
    ▼
카메라 좌표 (RDF: X=오른쪽, Y=아래, Z=앞)
    │
    │  current_pose (GT pose, 4×4)         ← Odom에서 적용
    ▼
월드 좌표 (고정 좌표계)

카메라 좌표
    │
    │  inv(current_pose) @ prev_pose       ← Ego에서 적용
    ▼
현재 로봇 시점 기준 좌표
```

- `Tr`: calib.txt의 5번째 줄, LiDAR → 카메라 좌표 변환 (프레임 무관, 고정값)
- `current_pose`: 각 프레임의 Ground Truth 위치/자세 (poses/00.txt)
- 동차좌표(homogeneous): 4×4 행렬과 행렬곱을 위해 포인트에 `w=1` 열을 추가

---

## 데이터셋

### KITTI Odometry Dataset (실데이터)

- **출처**: 독일 카를스루에 시내 도로 주행 데이터 (카를스루에 공대 + Toyota Tech Institute Chicago, 2012)
- **촬영 장비**: Velodyne HDL-64E LiDAR + 전방 스테레오 컬러 카메라
- **사용 시퀀스**: Sequence 00 (총 4541 프레임, 약 3.7km 주행)
- **사용 데이터**:
  - `sequences/00/velodyne/`: LiDAR 포인트 클라우드 (`.bin`, 프레임당 ~12만 점)
  - `sequences/00/image_2/`: 전방 컬러 카메라 이미지 (`.png`, 1226×370)
  - `sequences/00/calib.txt`: 카메라-LiDAR 캘리브레이션 행렬 (P0~P3, Tr)
  - `poses/00.txt`: 프레임별 Ground Truth 위치/자세 (3×4 변환 행렬)

KITTI 데이터가 없는 경우 **SyntheticLoader**로 자동 대체된다.

### SyntheticLoader (합성 데이터 fallback)

- 매 프레임: 랜덤 포인트 5000개 생성
- 이동: 프레임당 0.5m 전진 + 2도 Z축 회전
- `calib["Tr"]`: 단위행렬 (LiDAR = 카메라 좌표로 가정)

---

## 시각화

[Rerun](https://rerun.io/)을 사용하여 두 좌표계의 누적 지도를 나란히 표시한다.

- **왼쪽 (World / Odom)**: 고정된 월드 좌표계 기준 지도 — 로봇이 이동해도 장애물 위치는 고정
- **오른쪽 (Robot / Ego)**: 현재 로봇 중심 기준 지도 — 로봇 이동 시 지도 전체가 반대 방향으로 이동

매 10 프레임마다 로그를 기록하며, 포인트 수가 많을 경우 최대 50,000점으로 랜덤 다운샘플링한다.

포인트 색상은 카메라 이미지에서 각 LiDAR 점의 2D 투영 위치의 RGB값을 추출하여 적용한다. 카메라 시야 밖의 점은 회색(128, 128, 128)으로 표시된다.

---

## 실행 방법

```bash
# 의존성 설치
pip install numpy opencv-python matplotlib rerun-sdk

# Jupyter에서 실행
jupyter notebook benchmark.ipynb
```

KITTI 데이터는 `/project/jeongin/kitti_data/dataset` 경로에 있어야 하며, 없을 경우 SyntheticLoader로 자동 전환된다.

---

## 결과 해석

벤치마크 결과 그래프 (`Accumulated Point Cloud Update Cost: Odom vs Egocentric`):

- **파란선 (Odom)**: 프레임당 연산 시간이 거의 일정 — 새 포인트만 변환하기 때문
- **빨간선 (Ego)**: 프레임이 쌓일수록 연산 시간이 선형으로 증가 — 전체 지도를 매번 재변환하기 때문

지도 표현 방식의 선택은 연산 비용뿐 아니라 활용 목적(절대 위치 참조 vs 로봇 중심 인지)에 따라 달라질 수 있다.

<img width="855" height="547" alt="ass4_output" src="https://github.com/user-attachments/assets/dab3d168-ab16-41df-b15c-c33321bf482c" />

