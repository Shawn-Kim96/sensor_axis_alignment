# Sensor Axis Alignment

## Background
- **Assumptions**
    - Axis alignment is performed after the sensor is mounted on the vehicle.

- **Core Principle**
    - The vector measured by the accelerometer when the vehicle is level and stationary aligns with the direction of gravity.
    - The acceleration vector during acceleration/deceleration on a level and straight path aligns with the vehicle's lateral axis.
    - Once the direction of gravity and the lateral axis are identified, the longitudinal axis can be derived using the cross product.

- **Factors Increasing the Complexity of the Algorithm**
    - It is difficult to determine when the vehicle is in a level state.
    - It is challenging to know when the vehicle is on a level and straight path.
    - Road slope also affects axis alignment.
    - The sensor is not installed at the vehicle's center of mass, causing lever arm effects.

- **Development Requirements**
    - A simulation environment that can simultaneously capture ground truth and measured values.


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
    ├── imu_alginment_test.py: validating algorithm made in imu_alignment.py
    └── imu_alignment.py: main algorithm
```
