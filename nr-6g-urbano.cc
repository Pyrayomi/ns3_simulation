// nr-6g-urbano.cc (versão ajustada para evitar colisões e OOM)
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/buildings-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/point-to-point-module.h"

#include "ns3/nr-module.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"

using namespace ns3;

// ---------- grade hexagonal ----------
std::vector<Vector> MakeHexGrid(uint32_t rows, uint32_t cols, double isd, double z = 25.0)
{
    std::vector<Vector> pos;
    const double dx = isd;
    const double dy = isd * std::sqrt(3.0) / 2.0;

    for (uint32_t r = 0; r < rows; r++)
    {
        for (uint32_t c = 0; c < cols; c++)
        {
            double x = c * dx + ((r % 2) ? dx / 2.0 : 0.0);
            double y = r * dy;
            pos.push_back(Vector(x, y, z));
        }
    }
    return pos;
}

// ---------- 3 setores (com offset) ----------
void CreateTriSectorGnbs(Ptr<NrHelper> nr,
                         const BandwidthPartInfoPtrVector &bwps,
                         NodeContainer &sites,
                         NodeContainer &gnbNodes,
                         NetDeviceContainer &gnbDevs)
{
    const double offset = 1.0; // deslocamento maior (m) para evitar posições idênticas

    for (uint32_t i = 0; i < sites.GetN(); i++)
    {
        Ptr<MobilityModel> mm = sites.Get(i)->GetObject<MobilityModel>();
        Vector p = mm->GetPosition();

        for (int s = 0; s < 3; s++)
        {
            double angle = s * 120.0 * M_PI / 180.0; // 0°,120°,240°
            Vector pSector = Vector(
                p.x + offset * std::cos(angle),
                p.y + offset * std::sin(angle),
                p.z
            );

            Ptr<Node> sectorNode = CreateObject<Node>();
            gnbNodes.Add(sectorNode);

            MobilityHelper mh;
            Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
            alloc->Add(pSector);
            mh.SetPositionAllocator(alloc);
            mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            mh.Install(sectorNode);

            NetDeviceContainer d = nr->InstallGnbDevice(sectorNode, bwps);
            gnbDevs.Add(d);
        }
    }
}

int main(int argc, char *argv[])
{
    Time::SetResolution(Time::NS);

    // parâmetros (ajustáveis)
    uint32_t rows = 4, cols = 4;   // 16 sites
    uint32_t ueCount = 1500;       // **reduzido por segurança** — aumente em etapas
    double isd = 600.0;
    double simTime = 10.0;

    // 6G-like
    double centralFreq = 28e9;
    double bandwidth   = 400e6;

    CommandLine cmd;
    cmd.AddValue("ueCount", "Number of UEs", ueCount);
    cmd.Parse(argc, argv);

    // reproducibilidade
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    // Helpers
    Ptr<NrHelper> nr = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epc = CreateObject<NrPointToPointEpcHelper>();
    nr->SetEpcHelper(epc);

    Ptr<IdealBeamformingHelper> bf = CreateObject<IdealBeamformingHelper>();
    nr->SetBeamformingHelper(bf);

    // Core / internet
    Ptr<Node> pgw = epc->GetPgwNode();
    InternetStackHelper internet;
    internet.Install(pgw);

    // BAND / BWP
    CcBwpCreator ccBwp;
    CcBwpCreator::SimpleOperationBandConf bandConf;
    bandConf.m_centralFrequency = centralFreq;
    bandConf.m_channelBandwidth = bandwidth;
    bandConf.m_numCc = 1;
    bandConf.m_numBwp = 1;
    bandConf.m_scenario = BandwidthPartInfo::UMa;

    OperationBandInfo band = ccBwp.CreateOperationBandContiguousCc(bandConf);

    std::vector<std::reference_wrapper<OperationBandInfo>> bands = {std::ref(band)};
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps(bands);

    // Inicializa modelos (propagação / fading / canal)
    nr->InitializeOperationBand(&band);

    // Scheduler + atributos leves
    nr->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));
    nr->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    // SITES
    NodeContainer sites; sites.Create(rows * cols);
    auto centers = MakeHexGrid(rows, cols, isd);
    MobilityHelper mh;
    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    for (auto &v : centers) alloc->Add(v);
    mh.SetPositionAllocator(alloc);
    mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mh.Install(sites);

    // GNBs
    NodeContainer gnbNodes;
    NetDeviceContainer gnbDevs;
    CreateTriSectorGnbs(nr, allBwps, sites, gnbNodes, gnbDevs);

    // UEs
    NodeContainer ueNodes; ueNodes.Create(ueCount);
    internet.Install(ueNodes);

    MobilityHelper ueMob;
    ueMob.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                               "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=3000]"),
                               "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=3000]"));
    ueMob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                           "Bounds", RectangleValue(Rectangle(0, 3000, 0, 3000)),
                           "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                           "Distance", DoubleValue(5.0));
    ueMob.Install(ueNodes);

    // Install devices
    NetDeviceContainer ueDevs = nr->InstallUeDevice(ueNodes, allBwps);

    // UpdateConfig (recomenda doc)
    for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // SANITY CHECK: evita posições exatamente iguais entre UE e gNB
    // se distância < eps (0.1m), aplica nudge pequeno
    const double eps = 0.1;
    for (uint32_t ui = 0; ui < ueNodes.GetN(); ++ui)
    {
        Ptr<MobilityModel> um = ueNodes.Get(ui)->GetObject<MobilityModel>();
        Vector up = um->GetPosition();
        for (uint32_t gi = 0; gi < gnbNodes.GetN(); ++gi)
        {
            Ptr<MobilityModel> gm = gnbNodes.Get(gi)->GetObject<MobilityModel>();
            Vector gp = gm->GetPosition();
            double dx = up.x - gp.x;
            double dy = up.y - gp.y;
            double dz = up.z - gp.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < eps)
            {
                // aplica deslocamento randômico pequeno
                double jitter = 0.5 + (0.5 * (double) (std::rand() % 100) / 100.0);
                um->SetPosition(Vector(up.x + jitter, up.y + jitter, up.z));
            }
        }
    }

    // endereçamento
    Ipv4InterfaceContainer ueIfaces = epc->AssignUeIpv4Address(ueDevs);

    // Attach
    nr->AttachToClosestEnb(ueDevs, gnbDevs);

    // Aplicações: menos estresse por UE
    ApplicationContainer apps;
    uint16_t port = 9000;
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        apps.Add(sink.Install(ueNodes.Get(i)));

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ueIfaces.GetAddress(i), port));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate("1Mb/s"))); // mais leve
        onoff.SetAttribute("PacketSize", UintegerValue(512));
        apps.Add(onoff.Install(pgw));

        port++;
    }
    apps.Start(Seconds(2.0));
    apps.Stop(Seconds(simTime));

    // FlowMonitor
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->SerializeToXmlFile("nr-6g-urbano-metrics.xml", true, true);
    Simulator::Destroy();
    return 0;
}
