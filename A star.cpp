#pragma once
#include "graph.h"
#include <vector>
#include <cstdint>
#include <string>
#include "astar.h"
#include "qos_weights.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <climits>

struct AStarResult {
    std::vector<uint32_t> path;
    double delay;
    double cost;
    uint32_t expandedNodes;
};

AStarResult RunAStar(const BackhaulGraph& G,
                     uint32_t src,
                     uint32_t dst,
                     const std::string& slice);

// ===== Heuristic Function =======
std::vector<double> ComputeHeuristic(const BackhaulGraph& G,
                                     uint32_t dst,
                                     const QosWeights& weights)
{
    const double INF = 1e18;

    std::vector<double> h(G.numNodes, INF);
    std::vector<bool> visited(G.numNodes, false);

    std::priority_queue<
        std::pair<double, uint32_t>,
        std::vector<std::pair<double, uint32_t>>,
        std::greater<>
    > pq;

    h[dst] = 0.0;
    pq.push({0.0, dst});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (visited[u])
            continue;

        visited[u] = true;

        for (const auto& e : G.adj[u]) {
            if (visited[e.dst])
                continue;

            double atr = e.stats.bandwidthMbps - e.stats.usedBwMbps;
            if (atr <= 0.0)
                continue;

            double edgeCost = weights.K1 * e.stats.delayMs;

            if (h[u] + edgeCost < h[e.dst]) {
                h[e.dst] = h[u] + edgeCost;
                pq.push({h[e.dst], e.dst});
            }
        }
    }

    return h;
}

// QoS-aware routing algorithm
AStarResult RunAStar(const BackhaulGraph& G,
                     uint32_t src,
                     uint32_t dst,
                     const std::string& slice)
{
    const double INF = 1e18;

    QosWeights weights = GetWeights(slice);
    std::vector<double> heuristic = ComputeHeuristic(G, dst, weights);

    std::vector<double> gScore(G.numNodes, INF);
    std::vector<uint32_t> parent(G.numNodes, UINT32_MAX);
    std::vector<bool> visited(G.numNodes, false);

    std::vector<double> minAtr(G.numNodes, INF);
    std::vector<double> cumulativeLoss(G.numNodes, 0.0);

    uint32_t expandedNodes = 0;

    std::priority_queue<
        std::pair<double, uint32_t>,
        std::vector<std::pair<double, uint32_t>>,
        std::greater<>
    > open;

    gScore[src] = 0.0;
    minAtr[src] = INF;
    cumulativeLoss[src] = 0.0;

    open.push({heuristic[src], src});

    while (!open.empty()) {
        auto [f, u] = open.top();
        open.pop();

        if (visited[u])
            continue;

        if (u == dst)
            break;

        visited[u] = true;
        expandedNodes++;

        for (const auto& e : G.adj[u]) {
            if (visited[e.dst])
                continue;

            double atr = e.stats.bandwidthMbps - e.stats.usedBwMbps;
            if (atr <= 0.0)
                continue;

            double newAtr = std::min(minAtr[u], atr);

            double newLoss =
                1.0 - (1.0 - cumulativeLoss[u]) *
                      (1.0 - e.stats.lossRatio);

            double neighborCost = QoSCost(newAtr,
                                          e.stats.delayMs,
                                          newLoss,
                                          weights);

            double tentativeG = gScore[u] + neighborCost;

            if (tentativeG < gScore[e.dst]) {
                gScore[e.dst] = tentativeG;
                parent[e.dst] = u;

                minAtr[e.dst] = newAtr;
                cumulativeLoss[e.dst] = newLoss;

                double totalCost = tentativeG + heuristic[e.dst];
                open.push({totalCost, e.dst});
            }
        }
    }


// ====== Printing Purposes =======
    std::vector<uint32_t> path;

    if (gScore[dst] >= INF / 2) {
        return {path, -1, -1, expandedNodes};
    }

    for (uint32_t v = dst; v != UINT32_MAX; v = parent[v]) {
        path.push_back(v);
    }

    std::reverse(path.begin(), path.end());

    AStarResult res;
    res.path = path;
    res.delay = PathDelay(G, path);
    res.cost = gScore[dst];
    res.expandedNodes = expandedNodes;

    return res;
}