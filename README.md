# AI-autonomous-driving-simulator
AI 자율주행 잔디깍기 시뮬레이터 입니다 이제 잔디깍기의 경로를 지정하고 전방의 장애물을 피하는 로직을 시뮬레이터로 만들어 보았습니다

처음 실행화면 

<img width="1289" height="760" alt="화면 캡처 2026-01-06 013236" src="https://github.com/user-attachments/assets/1aa77b3f-8354-41ff-9f43-32b0c5e7db24" />

실행하고 마우스를 클릭하면 선이 생깁니다 그 선이 경로입니다 경로를 마음대로 지정해줍니다 

<img width="1278" height="752" alt="화면 캡처 2026-01-06 013253" src="https://github.com/user-attachments/assets/ec3b2c94-60bd-44ab-a6ac-916d0622f404" />

그다음에 K키를 눌러 장애물을 지정해줍니다 

<img width="1272" height="750" alt="화면 캡처 2026-01-06 013311" src="https://github.com/user-attachments/assets/aeabaddf-d534-4d9a-9e43-86fb105ddf20" />

그리고 시작버튼을 누르시면 A*알고리즘으로 최적의 경로를 찾으면서 장애물도 회피합니다

<img width="1285" height="749" alt="화면 캡처 2026-01-06 013321" src="https://github.com/user-attachments/assets/32c80421-05d0-40df-8af7-2af61cb5e3e0" />


구분,내용,비고
언어 (Language),Python 3.x,핵심 로직 및 시뮬레이션 구현
라이브러리 (Library),Pygame,실시간 그래픽 렌더링 및 이벤트 처리
알고리즘 (Algorithm),A* Search Algorithm,최적 경로 탐색 및 장애물 우회
수학/자료구조 (Logic),"Heuristic (Manhattan), Heapq",경로 우선순위 큐 관리 및 거리 계산
데이터 변환 (Data),GPS Coordinate Mapping,픽셀-위경도 좌표 변환 로직 (Pixhawk 스타일)
