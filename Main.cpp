#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include "ran.h"
#include "graph.h"
#include "dijkstra.h"
#include "astar.h"

double PathLoss(const BackhaulGraph& G,
                const std::vector<uint32_t>& path)
{
    double success = 1.0;

    for (size_t i = 0; i + 1 < path.size(); i++) {
        const LinkStats* s = G.GetStats(path[i], path[i + 1]);
        if (s == nullptr)
            continue;

        success *= (1.0 - s->lossRatio);
    }

    return 100.0 * (1.0 - success);
}

BackhaulGraph BuildBackhaulGraph()
{
    BackhaulGraph G(8);

    // 0=gNB, 1=BH1, 2=BH2, 3=BH3, 4=BH4, 5=BH5, 6=BH6, 7=Core

    G.AddEdge(0, 1, {2.0, 0.3, 0.01, 0.10, 1000.0, 100.0});
    G.AddEdge(0, 2, {3.0, 0.4, 0.01, 0.15, 1000.0, 150.0});

    G.AddEdge(1, 3, {4.0, 0.5, 0.01, 0.20, 100.0, 20.0});
    G.AddEdge(1, 4, {5.0, 0.6, 0.01, 0.25, 100.0, 25.0});

    G.AddEdge(2, 3, {3.0, 0.5, 0.01, 0.30, 100.0, 30.0});
    G.AddEdge(2, 5, {6.0, 0.7, 0.01, 0.20, 100.0, 20.0});

    G.AddEdge(3, 4, {2.0, 0.4, 0.01, 0.18, 100.0, 18.0});
    G.AddEdge(3, 5, {4.0, 0.5, 0.01, 0.15, 100.0, 15.0});

    G.AddEdge(4, 6, {3.0, 0.5, 0.01, 0.20, 100.0, 20.0});
    G.AddEdge(5, 6, {2.0, 0.4, 0.01, 0.15, 100.0, 15.0});

    G.AddEdge(6, 7, {2.0, 0.3, 0.01, 0.10, 1000.0, 100.0});

    // Congestion scenario
    G.InjectCongestion(1, 3, true);
    G.InjectCongestion(3, 4, true);

    return G;
}

void PrintResult(const std::string& slice,
                 double ranLatency,
                 double ranLoss,
                 const std::string& algorithm,
                 const std::vector<uint32_t>& path,
                 double backhaulDelay,
                 double backhaulLoss,
                 double cost,
                 uint32_t expandedNodes = 0)
{
    double e2eLatency = ranLatency + backhaulDelay;

    double ranSuccess = 1.0 - (ranLoss / 100.0);
    double bhSuccess  = 1.0 - (backhaulLoss / 100.0);
    double e2eLoss = 100.0 * (1.0 - (ranSuccess * bhSuccess));

    std::cout << slice
              << " | " << algorithm
              << " | path = " << PathStr(path)
              << " | backhaul delay = " << backhaulDelay << " ms"
              << " | E2E latency = " << e2eLatency << " ms"
              << " | loss = " << e2eLoss << " %"
              << " | cost = " << cost;

    if (algorithm == "A*")
        std::cout << " | expanded nodes = " << expandedNodes;

    std::cout << "\n";
}

int main()
{
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "\n[1] Running 5G-LENA RAN simulation...\n";

    SliceMetrics ran = RunRanSimulation();

    std::cout << "\n=== RAN RESULTS ===\n";
    std::cout << "uRLLC latency = " << ran.urllcLatency
              << " ms | loss = " << ran.urllcLoss << " %\n";
    std::cout << "eMBB  latency = " << ran.embbLatency
              << " ms | loss = " << ran.embbLoss << " %\n";
    std::cout << "mMTC  latency = " << ran.mmtcLatency
              << " ms | loss = " << ran.mmtcLoss << " %\n";

    std::cout << "\n[2] Building congested backhaul topology...\n";

    BackhaulGraph G = BuildBackhaulGraph();

    uint32_t src = 0; // gNB
    uint32_t dst = 7; // Core

    std::cout << "\n=== PER-SLICE BACKHAUL ROUTING ===\n";
    std::cout << "Metrics: ATR + Delay + Packet Loss\n\n";

    // uRLLC
    DijkstraResult urllcD = RunDijkstra(G, src, dst, "uRLLC");
    AStarResult    urllcA = RunAStar(G, src, dst, "uRLLC");

    PrintResult("uRLLC", ran.urllcLatency, ran.urllcLoss,
                "Dijkstra", urllcD.path, urllcD.delay,
                PathLoss(G, urllcD.path), urllcD.cost);

    PrintResult("uRLLC", ran.urllcLatency, ran.urllcLoss,
                "A*", urllcA.path, urllcA.delay,
                PathLoss(G, urllcA.path), urllcA.cost,
                urllcA.expandedNodes);

    std::cout << "\n";

    // eMBB
    DijkstraResult embbD = RunDijkstra(G, src, dst, "eMBB");
    AStarResult    embbA = RunAStar(G, src, dst, "eMBB");

    PrintResult("eMBB", ran.embbLatency, ran.embbLoss,
                "Dijkstra", embbD.path, embbD.delay,
                PathLoss(G, embbD.path), embbD.cost);

    PrintResult("eMBB", ran.embbLatency, ran.embbLoss,
                "A*", embbA.path, embbA.delay,
                PathLoss(G, embbA.path), embbA.cost,
                embbA.expandedNodes);

    std::cout << "\n";

    // mMTC
    DijkstraResult mmtcD = RunDijkstra(G, src, dst, "mMTC");
    AStarResult    mmtcA = RunAStar(G, src, dst, "mMTC");

    PrintResult("mMTC", ran.mmtcLatency, ran.mmtcLoss,
                "Dijkstra", mmtcD.path, mmtcD.delay,
                PathLoss(G, mmtcD.path), mmtcD.cost);

    PrintResult("mMTC", ran.mmtcLatency, ran.mmtcLoss,
                "A*", mmtcA.path, mmtcA.delay,
                PathLoss(G, mmtcA.path), mmtcA.cost,
                mmtcA.expandedNodes);

    return 0;
}