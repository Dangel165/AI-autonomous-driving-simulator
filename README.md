# AI-autonomous-driving-simulator

AI 자율주행 잔디깎기 시뮬레이터입니다. 잔디깎기의 경로를 직접 지정하고, 전방의 장애물을 피하는 로직을 시뮬레이터로 구현해 보았습니다.

## 📺 실행 가이드

### 1. 처음 실행 화면
시뮬레이터를 실행했을 때의 기본 메인 화면입니다.

<img width="1289" height="760" alt="화면 캡처 2026-01-06 013236" src="https://github.com/user-attachments/assets/1aa77b3f-8354-41ff-9f43-32b0c5e7db24" />

---

### 2. 경로 지정 (Path Setting)
실행 후 마우스 좌클릭을 하면 선이 생기며, 이를 통해 로봇이 이동할 경로를 자유롭게 지정할 수 있습니다.

<img width="1278" height="752" alt="화면 캡처 2026-01-06 013253" src="https://github.com/user-attachments/assets/ec3b2c94-60bd-44ab-a6ac-916d0622f404" />

---

### 3. 장애물 배치 (Obstacle Setting)
`K`키를 눌러 원하는 위치에 장애물을 배치하여 주행 환경을 설정합니다.

<img width="1272" height="750" alt="화면 캡처 2026-01-06 013311" src="https://github.com/user-attachments/assets/aeabaddf-d534-4d9a-9e43-86fb105ddf20" />

---

### 4. 시뮬레이션 시작
**시작 버튼**을 누르면 **A* 알고리즘**을 통해 최적의 경로를 계산하며 장애물을 실시간으로 회피합니다.

<img width="1285" height="749" alt="화면 캡처 2026-01-06 013321" src="https://github.com/user-attachments/assets/32c80421-05d0-40df-8af7-2af61cb5e3e0" />



---

## 🛠 기술 사양 (Technical Specifications)

| 구분 | 내용 | 비고 |
| :--- | :--- | :--- |
| **언어 (Language)** | `Python 3.x` | 핵심 로직 및 시뮬레이션 구현 |
| **라이브러리 (Library)** | `Pygame` | 실시간 그래픽 렌더링 및 이벤트 처리 |
| **알고리즘 (Algorithm)** | `A* Search Algorithm` | 최적 경로 탐색 및 장애물 우회 |
| **수학/자료구조 (Logic)** | `Heuristic (Manhattan)`, `Heapq` | 경로 우선순위 큐 관리 및 거리 계산 |
| **데이터 변환 (Data)** | `GPS Coordinate Mapping` | 픽셀-위경도 좌표 변환 로직 (Pixhawk 스타일) |

---

## ⌨️ 조작 방법 (Controls)

| 입력 | 기능 |
| :--- | :--- |
| **마우스 좌클릭** | 경로점(Waypoint) 추가 |
| **마우스 우클릭** | 장애물 제거 |
| **K 키** | 마우스 위치에 장애물 생성 |
| **SPACE / 시작버튼** | 시뮬레이션 시작 |
| **R 키** | 초기화 (Reset) |
