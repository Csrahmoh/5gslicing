#pragma once
#include "graph.h"
#include <vector>
#include <cstdint>
#include <string>
#include "dijkstra.h"
#include "qos_weights.h"
#include <queue>
#include <limits>
#include <algorithm>
#include <climits>

struct DijkstraResult {
    std::vector<uint32_t> path;
    double delay;
    double cost;
};

DijkstraResult RunDijkstra(const BackhaulGraph& G,
                           uint32_t src,
                           uint32_t dst,
                           const std::string& slice);

                           

// Dijkstra decision = أقل مجموع delay
DijkstraResult RunDijkstra(const BackhaulGraph& G,
                           uint32_t src,
                           uint32_t dst,
                           const std::string& slice)
{
    const double INF = 1e18; // infinty , these is no path yet

    std::vector<double> dist(G.numNodes, INF);
    std::vector<uint32_t> parent(G.numNodes, UINT32_MAX); // memorize the path


// priority Queue
    std::priority_queue<
        std::pair<double, uint32_t>,
        std::vector<std::pair<double, uint32_t>>,
        std::greater<>
    > pq;


// initilazation
    dist[src] = 0.0;
    pq.push({0.0, src});

    /*
     * Dijkstra decision:
     * shortest path based on delay only
     */
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (u == dst)
            break;

        if (d > dist[u])
            continue;

        for (const auto& e : G.adj[u]) {
            double w = e.stats.delayMs;

            if (dist[u] + w < dist[e.dst]) {
                dist[e.dst] = dist[u] + w;
                parent[e.dst] = u;
                pq.push({dist[e.dst], e.dst});
            }
        }
    }


// printing only
    std::vector<uint32_t> path;

    if (dist[dst] >= INF / 2) {
        return {path, -1, -1};
    }

    for (uint32_t v = dst; v != UINT32_MAX; v = parent[v]) {
        path.push_back(v);
    }

    std::reverse(path.begin(), path.end());

    QosWeights weights = GetWeights(slice);

    double minAtr = INF;
    double cumulativeLoss = 0.0;
    double totalCost = 0.0;

    for (size_t i = 0; i + 1 < path.size(); i++) {
        const LinkStats* s = G.GetStats(path[i], path[i + 1]);

        if (s == nullptr)
            continue;

        double atr = s->bandwidthMbps - s->usedBwMbps;

        if (atr <= 0.0)
            atr = 0.000001;

        minAtr = std::min(minAtr, atr);

        cumulativeLoss =
            1.0 - (1.0 - cumulativeLoss) *
                  (1.0 - s->lossRatio);

        totalCost += QoSCost(minAtr,
                             s->delayMs,
                             cumulativeLoss,
                             weights);
    }

    DijkstraResult res;
    res.path = path;
    res.delay = dist[dst];
    res.cost = totalCost;

    return res;
}