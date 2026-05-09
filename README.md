<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:0F2027,50:203A43,100:2C5364&height=250&section=header&text=QNX&fontSize=55&fontColor=ffffff&animation=fadeIn"/>
</p>

<h3 align="center">
 
### Real-Time Fault Recovery for Autonomous ADAS using QNX RTOS

A fault-tolerant autonomous ADAS architecture built on **QNX Neutrino RTOS** using **Raspberry Pi 4**, designed to ensure uninterrupted execution of safety-critical services even during failures, overloads, and service crashes.

---

## 📌 Project Overview

Autonomous vehicles rely on multiple real-time services such as:

- Camera Processing  
- Sensor Monitoring  
- Decision Making  
- Safety Control  

Failure in any of these services can cause unsafe vehicle behavior.

This project implements a **QNX microkernel-based fault recovery system** where:

- Safety-critical services are isolated  
- Non-critical failures are contained  
- Failed services are automatically restarted  
- Deterministic execution is maintained  

---

## 🎯 Objective

Build a fault-tolerant ADAS system capable of:

- Detecting service crashes  
- Restarting failed modules  
- Maintaining uninterrupted critical execution  
- Handling CPU overload and stress conditions  

---

## 🏗️ System Architecture

### Hardware
- Raspberry Pi 4  
- HC-SR04 Ultrasonic Sensor  
- LSM6DSOX IMU Sensor  
- GPIO LED Safety Output  

### Software
- QNX SDP 8.0  
- QNX Momentics IDE  
- QNX System Profiler  

---

## ⚙️ QNX Architectural Design

### Microkernel Strategy

Implemented using **QNX Neutrino Microkernel** with isolated services communicating through:

```c
MsgSend();
MsgReceive();
MsgReply();
```

### Benefits
- Deterministic IPC  
- Fault isolation  
- Modular restart capability  

---

## 🧠 CPU Affinity Mapping

| CPU Core | Assigned Service |
|----------|------------------|
| CPU0 | camera_critical_sim |
| CPU1 | safety_controller, decision_engine |
| CPU2 | sensor_service |
| CPU3 | multimedia_sim |

---

## 🏁 Thread Priorities

| Service | Policy | Priority |
|---------|--------|----------|
| camera_critical_sim | FIFO | 63 |
| safety_controller | FIFO | 58 |
| sensor_service | FIFO | 54 |
| decision_engine | FIFO | 52 |
| multimedia_sim | RR | 12 |

---

## 🔄 Fault Recovery Workflow

### Watchdog Supervisor

A custom watchdog continuously monitors thread heartbeats.

### Recovery Process

1. Detect failure  
2. Activate fallback safety logic  
3. Restart failed service  
4. Continue critical execution  

---

## 🧪 Fault Injection Tests

Simulated attacks:

- SIGKILL forced thread crash  
- CPU overload stress test  
- Multimedia flooding  
- Sensor timeout injection  

### Result

Successful recovery without interrupting:
- safety_controller  
- sensor_service  
- camera_critical_sim  

---

## 📊 Performance Analysis

### CPU Usage

Profiler logs show:

- Stable CPU utilization under load  
- Peak utilization around **50%**  
- Controlled periodic spikes from:
  - camera threads  
  - screen messaging  
  - sensor posting  

---

## 🕒 Thread Timeline Analysis

Observed stable execution of:

- adas_supervisor  
- safety_service  
- decision_service  
- sensor_service  
- camera_service  

Safety-critical threads remained uninterrupted during faults.

---

## 📷 System Profiler Screenshots

### Summary
<img width="1600" height="900" alt="Summary" src="https://github.com/user-attachments/assets/1dbe5d7a-c3c8-4f90-bd87-0bcebef7c873" />


### CPU Usage
<img width="1257" height="804" alt="CPU_USAGE" src="https://github.com/user-attachments/assets/8e0c3619-6aeb-40de-b531-1867618faf78" />


### Thread Timeline
<img width="1199" height="621" alt="TIMELINE" src="https://github.com/user-attachments/assets/7202847d-504a-41a2-a336-bcf793f534d1" />


---

## 📈 Results

The system successfully achieved:

- Fault isolation  
- Deterministic scheduling  
- Watchdog recovery  
- Stable CPU execution  
- Real-time resilience  

---

## 🚀 Future Scope

Possible improvements:

- CAN Bus integration  
- Distributed watchdogs  
- Automotive ECU deployment  
- Advanced sensor fusion  
- Autonomous fail-safe recovery  

---

## 📚 References

1. QNX Neutrino RTOS Documentation  
2. QNX SDP 8.0 Documentation  
3. Raspberry Pi 4 Technical Manual  
4. Real-Time Scheduling Literature  

---

## 👥 Team

**Team Name:** One Percent  
**Location:** Mysuru, Karnataka  

---

## 📄 License

This project is intended for academic and hackathon demonstration purposes.
