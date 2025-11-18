
// 1) Desenho do cenário (urbano denso)
//  Área representativa: simular um “recorte” 3 km × 3 km (9 km²). Em cidade de 10 M hab, é comum heterogeneidade; esse recorte foca na zona densa.
//  Densidade de usuários ativos: para dimensionamento, assuma ~2 000–3 000 UEs/km² ativos em hora carregada (pico). Para simulação, usamos amostragem (ex.: 300–600 UEs no recorte) para manter viável.
//  Topologia de rádio: sites hexagonais com ISD (inter-site distance) de 500–700 m (urbano macro). No exemplo abaixo uso ISD=600 m.
//  Setorização: 3 setores por site (0°, 120°, 240°) com antena direcional (beamwidth ~70°–90°, ganho máx. ~15 dBi).
//  Frequência/banda: LTE 20 MHz (100 RBs) em 1800/2100/2600 MHz (ex.: EARFCN banda 3/7).
//  Potência TX: eNB ~46 dBm (40 W) por portadora; UE 23 dBm.
//  Modelo de propagação: Buildings/Hybrid + Okumura-Hata (Urban Macro) com sombreamento (σ≈7 dB).
//  Agendador: Proportional Fair (PF).
//  Handover: A3-RSRP com hysteresis 3 dB e TTT 160 ms (valores típicos, evitam ping-pong).
//  Tráfego (mix hora-cheia):
//  Web/TCP (BulkSend limitado/HTTP-like): 40% dos UEs, rajadas de 1–4 Mb a cada 3–6 s, RTT realista via EPC.
//  Vídeo/UDP: 40% dos UEs em CBR 1 Mb/s (ou 2–3 Mb/s se quiser estressar).
//  VoIP/UDP: 20% dos UEs 24 kb/s, pacotes a cada 20 ms.
//  KPIs coletadas: atraso, jitter, throughput, taxa de perda, e por-setor (somando flows) para ver saturação/capacidade.
//  Observação: para 10 M hab não simulamos literalmente milhões de UEs; usamos recorte + amostragem e depois escalamos discussões de capacidade (ex.: eficiência espectral média 1–2 b/s/Hz/célula em urbano → 20–40 Mb/s por setor com 20 MHz; 3 setores → 60–120 Mb/s por site, média).
// 2) Parâmetros usados (bons padrões LTE)
//  DL/UL bandwidth: 20 MHz (100 RB)
//  TxPower eNB/UE: 46 dBm / 23 dBm
//  Antena eNB: ParabolicAntennaModel (ganho 15 dBi, beamwidth 70°, orientação 0/120/240)
//  Pathloss: Buildings + OkumuraHata (urban) + shadowing σ≈7 dB
//  Scheduler: ns3::PfFfMacScheduler
//  Handover: ns3::A3RsrpHandoverAlgorithm (Hysteresis=3 dB; TTT=160 ms)
//  ISD: 600 m (ajustável)
//  ISD→altura antena: 25–35 m (macro urbano). No código: 25 m.
//  Mobilidade UEs: RandomWalk 2D lenta (pedestres) ou ConstantPosition para estático (abaixo uso passeio lento).

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/buildings-module.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"



using namespace ns3;

// ---------- cria grade hexagonal ----------
std::vector<Vector> MakeHexGrid(uint32_t rows, uint32_t cols, double isd, double z=25.0)
{
    std::vector<Vector> pos;
    const double dx = isd;
    const double dy = isd * std::sqrt(3.0) / 2.0;

    for(uint32_t r=0; r<rows; r++)
    {
        for(uint32_t c=0; c<cols; c++)
        {
            double x = c*dx + ((r%2)? dx/2.0 : 0.0);
            double y = r*dy;
            pos.push_back(Vector(x,y,z));
        }
    }
    return pos;
}

// ---------- gera 3 setores por site ----------
void CreateTriSectorEnbs( Ptr<LteHelper> lte,
                          NodeContainer &sites,
                          NodeContainer &sectors,
                          NetDeviceContainer &enbDevs )
{
    Config::SetDefault("ns3::ParabolicAntennaModel::Beamwidth", DoubleValue(70.0));
    

    for(uint32_t i=0; i<sites.GetN(); i++)
    {
        Ptr<MobilityModel> mm = sites.Get(i)->GetObject<MobilityModel>();
        Vector p = mm->GetPosition();

        for(int s=0; s<3; s++)
        {
            double bearing = s==0? 0.0 : (s==1?120.0:240.0);

            Ptr<Node> sectorNode = CreateObject<Node>();
            sectors.Add(sectorNode);

            MobilityHelper mv;
            Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
            alloc->Add(p);
            mv.SetPositionAllocator(alloc);
            mv.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            mv.Install(sectorNode);

            lte->SetEnbAntennaModelType("ns3::ParabolicAntennaModel");
            lte->SetEnbAntennaModelAttribute("Orientation", DoubleValue(bearing));

            NetDeviceContainer d = lte->InstallEnbDevice(sectorNode);
            enbDevs.Add(d);
        }
    }
}

int main (int argc, char *argv[])
{
    Time::SetResolution(Time::NS);

    uint32_t rows = 4, cols = 4;
    double isd = 600.0;
    uint32_t ueCount = 6300;
    double simTime = 10.0;

    // ---- Config global LTE PHY (ns-3.40) ----
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(46.0));
    Config::SetDefault("ns3::LteUePhy::TxPower",  DoubleValue(23.0));

    Ptr<LteHelper> lte = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epc = CreateObject<PointToPointEpcHelper>();
    lte->SetEpcHelper(epc);

    // banda 20 MHz
    lte->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100));
    lte->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(100));

    // Pathloss model para ambiente urbano LTE
    lte->SetPathlossModelType( LogDistancePropagationLossModel::GetTypeId() );

    // Exponente urbano (entre 3.5 e 4.0)
    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent",
                    DoubleValue(3.7));

    // ReferenceLoss = FSPL (1m) conforme frequência
    // Para 2600 MHz (LTE Band 7):
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                    DoubleValue(40.7));


    // handover
    lte->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lte->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    lte->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(160)));

    Ptr<Node> pgw = epc->GetPgwNode();
    InternetStackHelper internet;

    // instalar pilha IP no PGW para poder rodar aplicações (sockets) nele
    internet.Install(pgw);
    
    // tornar simulações reprodutíveis (opcional)
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    // ---------- sites ----------
    NodeContainer sites;
    sites.Create(rows*cols);

    auto centers = MakeHexGrid(rows, cols, isd, 25.0);

    {
        MobilityHelper mh;
        Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
        for(auto &v: centers) alloc->Add(v);
        mh.SetPositionAllocator(alloc);
        mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mh.Install(sites);
    }

    // ---------- setores ----------
    NodeContainer enbNodes;
    NetDeviceContainer enbDevs;
    CreateTriSectorEnbs(lte, sites, enbNodes, enbDevs);

    // ---------- UEs ----------
    NodeContainer ueNodes; ueNodes.Create(ueCount);
    internet.Install(ueNodes);

    MobilityHelper ueMob;
    ueMob.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=3000]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=3000]"));
    ueMob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(0,3000,0,3000)),
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1]"),
        "Distance", DoubleValue(5.0));
    ueMob.Install(ueNodes);

    NetDeviceContainer ueDevs = lte->InstallUeDevice(ueNodes);
    Ipv4InterfaceContainer ueIfaces = epc->AssignUeIpv4Address(ueDevs);

    // attach automático
    lte->Attach(ueDevs);

    // ---------- Aplicações ----------
    uint16_t port = 9000;
    ApplicationContainer apps;

    for(uint32_t i=0; i<ueNodes.GetN(); i++)
    {
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        apps.Add(sink.Install(ueNodes.Get(i)));

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ueIfaces.GetAddress(i), port));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate("1Mb/s")));
        onoff.SetAttribute("PacketSize", UintegerValue(600));
        apps.Add(onoff.Install(pgw));

        port++;
    }

    apps.Start(Seconds(2.0));
    apps.Stop(Seconds(simTime));

    // ---------- FlowMonitor ----------
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    // ---------- NetAnim (ns-3.40: NÃO usar Ptr) ----------
    //AnimationInterface anim("lte-urbano.xml");
    //anim.EnablePacketMetadata(true);

    //for(uint32_t i=0;i<enbNodes.GetN();i++){
    //    anim.UpdateNodeDescription(enbNodes.Get(i),"eNB-"+std::to_string(i));
    //    anim.UpdateNodeColor(enbNodes.Get(i),200,0,0);
    //    anim.UpdateNodeSize(enbNodes.Get(i),20,20);
    //}
    //for(uint32_t i=0;i<ueNodes.GetN();i++){
    //    anim.UpdateNodeColor(ueNodes.Get(i),0,110,255);
    //    anim.UpdateNodeSize(ueNodes.Get(i),7,7);
    //}

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->SerializeToXmlFile("lte-urbano-metrics.xml", true, true);

    Simulator::Destroy();
    return 0;
}
