[README.md](https://github.com/user-attachments/files/27959603/README.md)
# 🛰️ QoS-Aware Backhaul Path Selection in 5G NR Network Slicing

> **A Comparative Study of Dijkstra and A\* Heuristic Algorithms Under mMTC Congestion**

[![C++](https://img.shields.io/badge/C++-17-00599C?style=flat-square&logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![ns-3 / 5G-LENA](https://img.shields.io/badge/ns--3-5G--LENA-4CAF50?style=flat-square)](https://5g-lena.cttc.es/)
[![License](https://img.shields.io/badge/License-Academic-orange?style=flat-square)](#)
[![Status](https://img.shields.io/badge/Status-Research-blueviolet?style=flat-square)](#)

---

## 📌 Overview

This project implements and compares two path-selection algorithms — **Dijkstra** and a **modified A\*** — for QoS-aware backhaul routing in 5G NR networks with **Network Slicing** under **mMTC congestion**.

The simulation models three 5G slice types:

| Slice | Description | Priority |
|-------|-------------|----------|
| **uRLLC** | Ultra-Reliable Low-Latency | Delay + Reliability |
| **eMBB** | Enhanced Mobile Broadband | Bandwidth |
| **mMTC** | Massive Machine-Type Comm. | Reliability + Energy |

The core finding: **Dijkstra picks the shortest path. A\* picks the best QoS path.**

---

## 🧠 Algorithms

### Dijkstra (Baseline)
Selects the minimum-delay path using a greedy priority queue. Ignores congestion, packet loss, and available throughput rate (ATR).

```
dist[v] = dist[u] + delay(u, v)
```

### Modified A\* (Proposed)
Uses a composite QoS metric as the real cost `g(n)`, and a delay-based heuristic `h(n)` to guide the search.

```
f(n) = g(n) + h(n)

g(n) = K0 × (1/ATR) + K1 × Delay + K2 × PacketLoss

h(v) = h(u) + K1 × delay(u, v)
```

---

## 📐 Composite QoS Metric

Based on the paper's formulation (simplified with S0 = S1 = S2 = 1):

```
CM = K0 × (1 / ATR)  +  K1 × OWD  +  K2 × PLT
```

| Symbol | Meaning |
|--------|---------|
| `ATR` | Available Throughput Rate — min ATR along path |
| `OWD` | One-Way Delay — cumulative sum along path |
| `PLT` | Packet Loss — `1 − ∏(1 − PLuv)` along path |
| `K0, K1, K2` | Slice-specific QoS weights |

### Per-Slice QoS Weights

| Slice | K0 (ATR) | K1 (Delay) | K2 (Loss) | Rationale |
|-------|----------|------------|-----------|-----------|
| uRLLC | 2 | 6.0 | 150 | High reliability + low latency |
| eMBB | 10 | 1.5 | 25 | Bandwidth-first |
| mMTC | 1.0 | 0.5 | 60 | Loss-sensitive IoT |

---

## 🗺️ Backhaul Topology

```
gNB ──── BH1 ──── BH3 ──┐
  │        │              │
  │       BH4 ◄──────────┘
  │        │
  └─ BH2 ─ BH5 ─── BH6 ──── Core
```

**Congested Link:** `BH1 → BH4`

| Condition | Utilization | Loss | Jitter |
|-----------|-------------|------|--------|
| Normal | 20% | 0.1% | 0.3 ms |
| Congested | 85% | 8% | 4.0 ms |

---

## 📊 Final Results

### Routing Output (Final Configuration)

| Slice | Algorithm | Path | E2E Latency | Loss | Cost |
|-------|-----------|------|-------------|------|------|
| uRLLC | Dijkstra | gNB → BH1 → **BH4** → BH6 → Core | 21 ms | 10.73% | 118.13 |
| uRLLC | **A\*** | gNB → BH1 → BH3 → BH4 → BH6 → Core | 22 ms | **4.90%** | **100.30** |
| eMBB | Dijkstra | gNB → BH1 → **BH4** → BH6 → Core | 24 ms | 10.73% | 27.63 |
| eMBB | **A\*** | gNB → BH2 → BH5 → BH6 → Core | 26 ms | **5.88%** | **25.50** |
| mMTC | Dijkstra | gNB → BH1 → **BH4** → BH6 → Core | 21 ms | 10.73% | 24.49 |
| mMTC | **A\*** | gNB → BH1 → BH3 → BH4 → BH6 → Core | 22 ms | **4.90%** | **15.43** |

### A\* Improvement Over Dijkstra

| Slice | Loss Reduction | Cost Reduction | Latency Overhead |
|-------|---------------|----------------|------------------|
| uRLLC | ✅ 54.33% better | ✅ 15.09% better | +1 ms |
| eMBB | ✅ 45.20% better | ✅ 7.71% better | +2 ms |
| mMTC | ✅ 54.33% better | ✅ 36.99% better | +1 ms |

> **Key Insight:** A\* consistently avoids the congested `BH1 → BH4` link by trading 1–2 ms of latency for significantly lower packet loss across all slice types.

---

## 🔬 RAN Simulation (5G-LENA)

```
=== RAN RESULTS ===
uRLLC latency = 9.00 ms  | loss = 0.00 %
eMBB  latency = 12.00 ms | loss = 0.00 %
mMTC  latency = 9.00 ms  | loss = 0.00 %
```

RAN latency is added to backhaul delay to compute E2E latency.

---

## 📁 Project Structure

```
5gslicing/
├── main.cpp              # Entry point — runs RAN + backhaul pipeline
├── graph.cpp/.h          # Backhaul topology & edge management
├── dijkstra.cpp/.h       # Dijkstra routing algorithm
├── astar.cpp/.h          # Modified A* with QoS heuristic
├── heuristic.cpp/.h      # ComputeHeuristic() — backward Dijkstra on delay
├── qos_weights.h         # Per-slice K0, K1, K2 weight definitions
├── qos_cost.h            # QoSCost() composite metric function
└── ran_sim.cpp/.h        # 5G-LENA RAN simulation wrapper
```

---

## ⚙️ Build & Run

### Requirements
- C++17 compiler (`g++` or `clang++`)
- ns-3 with [5G-LENA module](https://5g-lena.cttc.es/) *(for RAN simulation)*

### Compile

```bash
g++ -std=c++17 -O2 -o 5gslicing main.cpp graph.cpp dijkstra.cpp astar.cpp heuristic.cpp ran_sim.cpp
```

### Run

```bash
./5gslicing
```

### Expected Output

```
[1] Running 5G-LENA RAN simulation...
[2] Building congested backhaul topology...
=== PER-SLICE BACKHAUL ROUTING ===
uRLLC | Dijkstra | path = gNB -> BH1 -> BH4 -> BH6 -> Core | E2E latency = 21.00 ms | loss = 10.73%
uRLLC | A*       | path = gNB -> BH1 -> BH3 -> BH4 -> BH6 -> Core | E2E latency = 22.00 ms | loss = 4.90%
...
```

---

## 💡 Key Design Decisions

### Why A\* outperforms Dijkstra under congestion
Dijkstra optimizes for a single metric (delay). Under mMTC congestion, the shortest path passes through a heavily loaded link with 8% loss and 85% utilization. A\* avoids this by using a multi-metric cost function that penalizes congested links even if they have low delay.

### Why the heuristic uses delay only
The heuristic `h(n) = K1 × delay` is computed via a backward pass (from Core to gNB). This is admissible and consistent — it never overestimates the true remaining cost — which guarantees A\* finds the optimal path.

### Congestion model rationale

| Metric | Value | Rationale |
|--------|-------|-----------|
| Utilization | 85% | Realistic heavy load (not failure) |
| Loss | 8% | Within degraded-but-operational range |
| Jitter | 4 ms | Matches 5G backhaul congestion benchmarks |

40% packet loss was rejected as it models network failure, not congestion.

---

## 📖 References

This work is based on and extends:

- **5G-LENA** ns-3 module for NR RAN simulation
- ITU-R IMT-2020 requirements for uRLLC / eMBB / mMTC
- Composite QoS metric formulation from the referenced paper (K0·S0·ATR + K1·S1·OWD + K2·S2·PLT)
- Bangkok Skytrain 5G benchmarks: avg latency ~13 ms, avg loss ~0.1%

---

## 👤 Author

**Rahaf Almohammadi**
**Elaf Qatan**
**Rawan Saqar**
MCs IOT and Robotics and Autunumance systems — 5G Network Slicing & QoS-Aware Routing  
[GitHub](https://github.com/Csrahmoh/5gslicing)

---

*This simulation demonstrates that QoS-aware routing with A\* significantly improves reliability under mMTC congestion, at the cost of minimal latency overhead — making it a strong candidate for real 5G network slicing deployments.*
