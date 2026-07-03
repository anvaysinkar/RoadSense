# RoadSense

### An Adaptive Edge-Computing Framework for Crowdsourced Vehicular Telemetry and Road Condition Monitoring

RoadSense is a decentralized edge-computing framework that transforms ordinary passenger vehicles into intelligent crowdsensing nodes for real-time road infrastructure monitoring. The system combines inertial sensing, geospatial localization, and OBD-II/CAN telemetry to detect, classify, and geospatially map road anomalies including potholes, speed breakers, and varying levels of road roughness.

Unlike smartphone-based crowdsensing systems, RoadSense employs a rigidly mounted embedded sensing platform with Adaptive Baseline Calibration and multivariate sensor fusion to improve detection accuracy while minimizing false positives caused by suspension dynamics and driver-induced vehicle motion.

---

## Features

* Adaptive Baseline Calibration (ABC) for vehicle-independent threshold normalization
* Real-time detection of potholes, speed breakers, and road roughness
* OBD-II/CAN sensor fusion with vehicle speed and engine RPM
* Dynamic threshold scaling based on vehicle kinematics
* Driver-induced event masking for false positive suppression
* GPS-tagged telemetry logging
* Edge processing with cloud-ready telemetry output
* Crowdsourced road infrastructure monitoring architecture

---

## Hardware

* ESP32 Development Board
* MPU6050 Inertial Measurement Unit (IMU)
* NEO-6M GPS Module
* MCP2515 CAN Controller
* TJA1050 CAN Transceiver
* OBD-II Interface
* USB Power Supply / Power Bank

---

## Software Architecture

```
IMU + GPS + OBD-II
        │
        ▼
Adaptive Baseline Calibration
        │
        ▼
Sensor Fusion
        │
        ▼
Dynamic Threshold Scaling
        │
        ▼
Kinematic Event Masking
        │
        ▼
Road Anomaly Classification
        │
        ▼
Telemetry Logging
        │
        ▼
Cloud Dashboard / GIS Visualization
```

---

## Detection Capabilities

### Road Roughness

* Continuous vibration monitoring
* Three-tier surface classification
* Green (Smooth)
* Yellow (Moderate)
* Red (Poor)

### Pothole Detection

* High-frequency impulse detection
* GPS event localization
* Severity estimation

### Speed Breaker Detection

* Kinematic sequence recognition
* Elevation profile analysis
* GPS event localization

---

## Logged Telemetry

Each detected event is recorded as:

```
Timestamp
Latitude
Longitude
Vehicle Speed
Engine RPM
Effective Delta
Classification Label
```

Example:

```
5637,18.496862,73.832570,33,546.49,GREEN
```

---

## Experimental Validation

The framework was evaluated through real-world vehicular deployment over a **20 km urban test route** under controlled traffic conditions.

### Test Platform

* Vehicle: Kia Seltos GT Line
* Deployment: Rigid chassis-mounted edge node
* Environment: Urban roads with potholes, speed breakers, and varying pavement conditions

---

## Performance

### Discrete Road Anomaly Detection

| Metric              |     Value |
| ------------------- | --------: |
| Aggregate Precision | **89.6%** |
| Aggregate Recall    | **89.6%** |
| Aggregate F1-Score  | **0.896** |

### Individual Class Performance

| Event         | Precision |    Recall |  F1-Score |
| ------------- | --------: | --------: | --------: |
| Speed Breaker | **92.3%** | **92.3%** | **0.923** |
| Pothole       | **87.5%** | **87.5%** | **0.875** |

### Kinematic Noise Suppression

* Driver-induced kinetic events rejected: **247 / 250**
* False-positive suppression: **98.8%**

---

## Repository Structure

```
RoadSense/
│
├── firmware/
├── datasets/
├── docs/
├── README.md
└── LICENSE
```

---

## Applications

* Smart Transportation Systems
* Intelligent Road Infrastructure Monitoring
* Municipal Road Asset Management
* Predictive Maintenance
* Connected Vehicle Research
* Intelligent Transportation Systems (ITS)
* Smart City Infrastructure

---

## Future Work

* Direct ECU integration
* Hardware miniaturization
* Fleet-scale crowdsourced deployment
* Cloud-based spatial clustering
* OEM integration
* Machine learning-based anomaly classification

---

## Citation

If you use this repository in your research, please cite:

**RoadSense: An Adaptive Edge-Computing Framework for Crowdsourced Vehicular Telemetry and Road Condition Monitoring**

---

## License

This project is released for academic and research purposes.
