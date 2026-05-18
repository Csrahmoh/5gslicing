#pragma once
#include <string>
#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/cc-bwp-helper.h"
#include "ran.h"

struct SliceMetrics {
  double urllcLatency = 0;
  double urllcLoss    = 0;
  double embbLatency  = 0;
  double embbLoss     = 0;
  double mmtcLatency  = 0;
  double mmtcLoss     = 0;
};

SliceMetrics RunRanSimulation();

using namespace ns3;

SliceMetrics RunRanSimulation()
{
    SliceMetrics result;
    double simTime = 10.0;

    NodeContainer gnb; gnb.Create(1);
    NodeContainer urllc; urllc.Create(1);
    NodeContainer embb; embb.Create(1);
    NodeContainer mmtc; mmtc.Create(1);
    NodeContainer server; server.Create(1);

    NodeContainer allUEs;
    allUEs.Add(urllc);
    allUEs.Add(embb);
    allUEs.Add(mmtc);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0, 0, 10));
    pos->Add(Vector(30, 0, 1.5));
    pos->Add(Vector(40, 0, 1.5));
    pos->Add(Vector(50, 0, 1.5));
    pos->Add(Vector(1000, 0, 0));

    mob.SetPositionAllocator(pos);
    mob.Install(gnb);
    mob.Install(urllc);
    mob.Install(embb);
    mob.Install(mmtc);
    mob.Install(server);

    Ptr<NrPointToPointEpcHelper> epc = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nr = CreateObject<NrHelper>();
    nr->SetEpcHelper(epc);

    Ptr<IdealBeamformingHelper> beam = CreateObject<IdealBeamformingHelper>();
    nr->SetBeamformingHelper(beam);

    Ptr<NrChannelHelper> ch = CreateObject<NrChannelHelper>();
    ch->ConfigureFactories("UMi", "Default", "ThreeGpp");

    CcBwpCreator cc;
    CcBwpCreator::SimpleOperationBandConf conf(3.5e9, 20e6, 1);
    OperationBandInfo band = cc.CreateOperationBandContiguousCc(conf);
    ch->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector bwps = CcBwpCreator::GetAllBwps({band});

    // Fixed scheduler: Round Robin
    nr->SetSchedulerTypeId(
        TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR")
    );

    NetDeviceContainer gnbDev = nr->InstallGnbDevice(gnb, bwps);
    NetDeviceContainer ueDev  = nr->InstallUeDevice(allUEs, bwps);

    InternetStackHelper net;
    net.Install(allUEs);
    net.Install(server);

    epc->AssignUeIpv4Address(ueDev);

    Ipv4StaticRoutingHelper routing;
    for (uint32_t i = 0; i < allUEs.GetN(); i++) {
        auto r = routing.GetStaticRouting(allUEs.Get(i)->GetObject<Ipv4>());
        r->SetDefaultRoute(epc->GetUeDefaultGatewayAddress(), 1);
    }

    nr->AttachToClosestGnb(ueDev, gnbDev);

    Ptr<Node> pgw = epc->GetPgwNode();

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("1ms"));

    auto link = p2p.Install(pgw, server.Get(0));

    Ipv4AddressHelper ip;
    ip.SetBase("1.0.0.0", "255.0.0.0");
    auto iface = ip.Assign(link);
    Ipv4Address serverAddr = iface.GetAddress(1);

    nr->ActivateDedicatedEpsBearer(
        ueDev.Get(0),
        NrEpsBearer(NrEpsBearer::GBR_CONV_VOICE),
        NrEpcTft::Default()
    );

    nr->ActivateDedicatedEpsBearer(
        ueDev.Get(1),
        NrEpsBearer(NrEpsBearer::GBR_CONV_VIDEO),
        NrEpcTft::Default()
    );

    nr->ActivateDedicatedEpsBearer(
        ueDev.Get(2),
        NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB),
        NrEpcTft::Default()
    );

    ApplicationContainer apps;

    UdpServerHelper s1(5000);
    apps.Add(s1.Install(server.Get(0)));

    UdpClientHelper c1(serverAddr, 5000);
    c1.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    c1.SetAttribute("PacketSize", UintegerValue(200));
    apps.Add(c1.Install(urllc.Get(0)));

    UdpServerHelper s2(6000);
    apps.Add(s2.Install(server.Get(0)));

    UdpClientHelper c2(serverAddr, 6000);
    c2.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    c2.SetAttribute("PacketSize", UintegerValue(1400));
    apps.Add(c2.Install(embb.Get(0)));

    UdpServerHelper s3(7000);
    apps.Add(s3.Install(server.Get(0)));

    UdpClientHelper c3(serverAddr, 7000);
    c3.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
    c3.SetAttribute("PacketSize", UintegerValue(256));
    apps.Add(c3.Install(mmtc.Get(0)));

    apps.Start(Seconds(1));
    apps.Stop(Seconds(simTime - 1));

    FlowMonitorHelper fm;
    auto monitor = fm.InstallAll();
    auto classifier = DynamicCast<Ipv4FlowClassifier>(fm.GetClassifier());

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();

    for (auto& [id, st] : stats) {
        uint16_t port = classifier->FindFlow(id).destinationPort;

        if (port != 5000 && port != 6000 && port != 7000)
            continue;

        double latency = 0;
        if (st.rxPackets > 0)
            latency = st.delaySum.GetMilliSeconds() / st.rxPackets;

        double loss = 0;
        if (st.txPackets > 0)
            loss = 100.0 * (st.txPackets - st.rxPackets) / st.txPackets;

        if (port == 5000) {
            result.urllcLatency = latency;
            result.urllcLoss = loss;
        }
        else if (port == 6000) {
            result.embbLatency = latency;
            result.embbLoss = loss;
        }
        else if (port == 7000) {
            result.mmtcLatency = latency;
            result.mmtcLoss = loss;
        }
    }

    Simulator::Destroy();
    return result;
}