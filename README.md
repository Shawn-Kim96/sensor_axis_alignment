# Sensor Axis Alignment

## Background
- 가정
    - 축정렬은 차량에 장착된 이후 수행한다.
- 핵심 원리
    - 차량이 수평 & 정지상태 일 때 가속도 센서로부터 측정된 벡터는 중력 방향과 일치한다.
    - 차량이 수평 & 직선 경로에서 가/감속할 때 나타나는 가속도 벡터가 차량의 lateral 축과 일치한다.
    - 중력방향과 lateral 축을 검출할 수 있으면 외적을 통해 longitudinal 축을 추출할 수 있다.
- 알고리즘의 난이도를 올리는 요소들
    - 차량이 언제 수평상태에 있는지 알기 힘들다.
    - 차량이 언제 수평 & 직선 도로에 있는지 알기 힘들다.
    - 배수를 위 한 도로 구배도 축정렬에 영향을 준다.
    - 센서가 차량의 무게 중심 위치에 장착된 것이 아니어서 레버암 효과가 발생
- 개발 요구 조건
    - 참값과 측정값을 동시에 획득할 수 있는 시뮬레이션

## Data Structure
```
sensor_axis_alignment
├── README.md
│
├── notebooks         
│   └── 1.0-kse-data-exploration-and-axis-alignment-threshold-check.ipynb                
│
├── data
│   ├── processed     
│   └── raw         
│                     
└── src                
    ├── data           
    │   ├── make_dataset.py
    │   └── sensor_data_gen_from_driving_simulator_data.py
    │
    ├── imu_alginment_test.py: imuy_alignment.py에 구현된 알고리즘의 성능을 확인하는 코드
    └── imu_alignment.py: imu 센서와 차량의 축을 정렬하는 알고리즘 
```
