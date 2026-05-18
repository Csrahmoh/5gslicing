#pragma once
#include <vector>
#include <string>
#include <cstdint>


struct LinkStats {
    double delayMs;       // propagation delay (ms)   — Dijkstra edge weight
    double jitterMs;      // delay variation (ms)      — A* composite cost
    double lossRatio;     // packet loss 0.0-1.0       — monitor trigger
    double utilization;   // link load 0.0-1.0         — A* composite cost
    double bandwidthMbps; // physical capacity (Mbps)  — monitor
    double usedBwMbps;    // consumed bandwidth (Mbps) — monitor
};

// build each node and where is it going dst
struct Edge {
    uint32_t  dst;
    LinkStats stats;
};



    /* Simulate Iperf congestion exactly as Hefele Section VI-B */
    
    void InjectCongestion(uint32_t u, uint32_t v, bool on) {
        auto upd = [&](uint32_t a, uint32_t b) {
            for (auto& e : adj[a]) {
                if (e.dst != b) continue;
                e.stats.utilization = on ? 0.93 : 0.12;
                e.stats.lossRatio   = on ? 0.40 : 0.01;
                e.stats.jitterMs    = on ? 9.0  : 0.5;
                e.stats.usedBwMbps  = on
                    ? e.stats.bandwidthMbps * 0.93
                    : e.stats.bandwidthMbps * 0.12;
            }
        };
        upd(u, v); upd(v, u);
    }

    
    double GetDelay(uint32_t u, uint32_t v) const {
        for (const auto& e : adj[u])
            if (e.dst == v) return e.stats.delayMs;
        return 0.0;
    }

       const LinkStats* GetStats(uint32_t u, uint32_t v) const {
        for (const auto& e : adj[u])
            if (e.dst == v) return &e.stats;
        return nullptr;
    }
;

inline std::string PathStr(const std::vector<uint32_t>& p) {
    const std::vector<std::string> N =
    {"gNB","BH1","BH2","BH3","BH4","BH5","BH6","Core"};

    std::string s;
    for (size_t i = 0; i < p.size(); i++) {
        if (i) s += " -> ";
        s += (p[i] < N.size()) ? N[p[i]] : std::to_string(p[i]);
    }
    return s;
}


inline double PathDelay(const BackhaulGraph& G,
                         const std::vector<uint32_t>& p) {
    double d = 0;
    for (size_t i = 0; i + 1 < p.size(); i++)
        d += G.GetDelay(p[i], p[i+1]);
    return d;
}

#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>

struct LinkStats {
    double delayMs;
    double jitterMs;
    double lossRatio;
    double utilization;
    double bandwidthMbps;
    double usedBwMbps;
};

struct Edge {
    uint32_t dst;
    LinkStats stats;
};

class BackhaulGraph {
public:

    uint32_t numNodes;

    std::vector<std::vector<Edge>> adj;

    explicit BackhaulGraph(uint32_t n)
        : numNodes(n),
          adj(n)
    {
    }

    void AddEdge(uint32_t u,
                 uint32_t v,
                 const LinkStats& s)
    {
        adj[u].push_back({v, s});
        adj[v].push_back({u, s});
    }

    // ===== Congestion Injection =====

    void InjectCongestion(uint32_t u,
                          uint32_t v,
                          bool on)
    {
        auto upd = [&](uint32_t a, uint32_t b)
        {
            for (auto& e : adj[a]) {

                if (e.dst != b)
                    continue;

                e.stats.utilization =
                    on ? 0.93 : 0.12;

                e.stats.lossRatio =
                    on ? 0.40 : 0.01;

                e.stats.jitterMs =
                    on ? 9.0 : 0.5;

                e.stats.usedBwMbps =
                    on
                    ? e.stats.bandwidthMbps * 0.93
                    : e.stats.bandwidthMbps * 0.12;
            }
        };

        upd(u, v);
        upd(v, u);
    }

    // ===== Get Link Delay =====

    double GetDelay(uint32_t u,
                    uint32_t v) const
    {
        for (const auto& e : adj[u]) {

            if (e.dst == v)
                return e.stats.delayMs;
        }

        return 0.0;
    }

    // ===== Get Full Link Statistics =====

    const LinkStats* GetStats(uint32_t u,
                              uint32_t v) const
    {
        for (const auto& e : adj[u]) {

            if (e.dst == v)
                return &e.stats;
        }

        return nullptr;
    }
};

// ===== Convert Path to String =====

inline std::string PathStr(
    const std::vector<uint32_t>& p)
{
    const std::vector<std::string> N =
    {
        "gNB",
        "BH1",
        "BH2",
        "BH3",
        "BH4",
        "BH5",
        "BH6",
        "Core"
    };

    std::string s;

    for (size_t i = 0; i < p.size(); i++) {

        if (i)
            s += " -> ";

        s += (p[i] < N.size())
             ? N[p[i]]
             : std::to_string(p[i]);
    }

    return s;
}

// ===== End-to-End Delay =====

inline double PathDelay(
    const BackhaulGraph& G,
    const std::vector<uint32_t>& p)
{
    double d = 0.0;

    for (size_t i = 0; i + 1 < p.size(); i++) {

        d += G.GetDelay(p[i], p[i + 1]);
    }

    return d;
}
