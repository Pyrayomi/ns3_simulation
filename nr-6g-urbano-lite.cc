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

// ---------- utilitário de log colorido ----------
#define GREEN  "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE   "\033[1;34m"
#define RED    "\033[1;31m"
#define RESET  "\033[0m"

void PrintStep(std::string step)
{
    std::cout << GREEN << "[DEBUG]" << RESET
              << " " << step
              << YELLOW << " (" << (clock() / (double)CLOCKS_PER_SEC)
              << "s tempo real)" << RESET << std::endl;
}

// novo: imprime progresso da simulação a cada 1s de tempo simulado
void PrintSimProgress()
{
    std::cout << GREEN << "[SIM]" << RESET
              << " tempo simulado: " << Simulator::Now().GetSeconds() << " s" << std::endl;
    Simulator::Schedule(Seconds(1.0), &PrintSimProgress);
}

// ---------- grade hexagonal ----------
std::vector<Vector> MakeHexGrid(uint32_t rows, uint32_t cols, double isd, double z = 25.0)
{
    std::vector<Vector> pos;
    const double dx = isd;
    const double dy = isd * std::sqrt(3.0) / 2.0;

    for (uint32_t r = 0; r < rows; r++)
        for (uint32_t c = 0; c < cols; c++)
        {
            double x = c * dx + ((r % 2) ? dx / 2.0 : 0.0);
            double y = r * dy;
            pos.push_back(Vector(x, y, z));
        }

    return pos;
}

// ---------- cria 3 setores deslocados por site ----------
void CreateTriSectorGnbs(Ptr<NrHelper> nr,
                         const BandwidthPartInfoPtrVector &bwps,
                         NodeContainer &sites,
                         NodeContainer &gnbNodes,
                         NetDeviceContainer &gnbDevs)
{
    const double offset = 3.0;
    PrintStep("Criando setores por site");

    for (uint32_t i = 0; i < sites.GetN(); i++)
    {
        Ptr<MobilityModel> mm = sites.Get(i)->GetObject<MobilityModel>();
        Vector p = mm->GetPosition();

        for (int s = 0; s < 3; s++)
        {
            double angle = s * 120.0 * M_PI / 180.0;
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

    uint32_t rows = 3, cols = 3;
    uint32_t ueCount = 90;      // reduzido para debug
    double isd = 500.0;
    double simTime = 5.0;
    double centralFreq = 28e9;  // 28 GHz
    double bandwidth = 100e6;   // 100 MHz (6G urbano balanceado)

    CommandLine cmd;
    cmd.AddValue("ueCount", "Número de UEs", ueCount);
    cmd.AddValue("simTime", "Duração da simulação (s)", simTime);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    // ---------- Inicialização ----------
    PrintStep("Inicializando helpers e EPC");
    Ptr<NrHelper> nr = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> epc = CreateObject<NrPointToPointEpcHelper>();
    nr->SetEpcHelper(epc);

    Ptr<IdealBeamformingHelper> bf = CreateObject<IdealBeamformingHelper>();
    nr->SetBeamformingHelper(bf);

    Ptr<Node> pgw = epc->GetPgwNode();
    InternetStackHelper internet;
    internet.Install(pgw);

    // ---------- Banda ----------
    PrintStep("Configurando banda e modelo de propagação");
    CcBwpCreator ccBwp;
    CcBwpCreator::SimpleOperationBandConf bandConf;
    bandConf.m_centralFrequency = centralFreq;
    bandConf.m_channelBandwidth = bandwidth;
    bandConf.m_numCc = 1;
    bandConf.m_numBwp = 1;
    bandConf.m_scenario = BandwidthPartInfo::UMa; // Urbano macro

    OperationBandInfo band = ccBwp.CreateOperationBandContiguousCc(bandConf);

    // ---------- Modelo de perda: LogDistance baseado em FSPL ----------
    // FSPL(1m) = 32.45 + 20*log10(f_MHz)
    double freqMHz = centralFreq / 1e6;
    double refLoss = 32.45 + 20 * std::log10(freqMHz); // FSPL 1m @ 28GHz ≈ 61.4 dB
    PrintStep("Configurando modelo de perda (ThreeGppPropagationLossModel - compatível com ns-3-nr antigo)");
    std::cout << BLUE << "FSPL estimada (1 m @ " << freqMHz << " MHz): " 
            << refLoss << " dB" << RESET << std::endl;

    // Usa o modelo 3GPP padrão (Urban Macro LOS)
    // Durante prototipagem/desenvolvimento desative shadowing para ganhar desempenho
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue(true));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::Frequency", DoubleValue(centralFreq));

    // Não há SmallScaleFading nem Scenario nessa versão

    // ---------- Inicializa banda ----------
    std::vector<std::reference_wrapper<OperationBandInfo>> bands = {std::ref(band)};
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps(bands);
    nr->InitializeOperationBand(&band);
    nr->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));

    // ---------- Sites ----------
    PrintStep("Criando sites e mobilidade");
    NodeContainer sites; sites.Create(rows * cols);
    auto centers = MakeHexGrid(rows, cols, isd);
    MobilityHelper mh;
    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    for (auto &v : centers) alloc->Add(v);
    mh.SetPositionAllocator(alloc);
    mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mh.Install(sites);

    // ---------- gNBs ----------
    NodeContainer gnbNodes;
    NetDeviceContainer gnbDevs;
    CreateTriSectorGnbs(nr, allBwps, sites, gnbNodes, gnbDevs);
    PrintStep("Atualizando configuração dos gNBs");
    for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();

    // ---------- UEs ----------
    PrintStep("Criando UEs");
    NodeContainer ueNodes; ueNodes.Create(ueCount);
    internet.Install(ueNodes);
    MobilityHelper ueMob;
    ueMob.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                               "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=3000]"),
                               "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=3000]"));
    ueMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMob.Install(ueNodes);

    PrintStep("Instalando dispositivos UE");
    NetDeviceContainer ueDevs = nr->InstallUeDevice(ueNodes, allBwps);
    for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // ---------- Endereçamento ----------
    PrintStep("Atribuindo endereços IPv4 para UEs");
    Ipv4InterfaceContainer ueIfaces = epc->AssignUeIpv4Address(ueDevs);

    // ---------- Attach ----------
    PrintStep("Attach dos UEs");
    nr->AttachToClosestEnb(ueDevs, gnbDevs);

    // ---------- Aplicações ----------
    PrintStep("Instalando aplicações");
    uint16_t port = 9000;
    ApplicationContainer apps;

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        apps.Add(sink.Install(ueNodes.Get(i)));

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(ueIfaces.GetAddress(i), port));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate("1Mb/s")));
        onoff.SetAttribute("PacketSize", UintegerValue(512));
        apps.Add(onoff.Install(pgw));

        port++;
    }
    apps.Start(Seconds(0.1));
    apps.Stop(Seconds(simTime));

    // ---------- FlowMonitor ----------
    PrintStep("Iniciando FlowMonitor (somente nós relevantes)");
    FlowMonitorHelper fm;
    // Instala apenas no PGW e nos UEs para reduzir overhead
    NodeContainer monitorNodes;
    monitorNodes.Add(pgw);
    monitorNodes.Add(ueNodes);
    Ptr<FlowMonitor> monitor = fm.Install(monitorNodes);

    // ---------- Execução ----------
    PrintStep("Rodando simulação");
    // agenda logger de progresso para verificar que a simulação está avançando
    Simulator::Schedule(Seconds(0.0), &PrintSimProgress);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    PrintStep("Exportando resultados");
    // Evite escrever per-probe/histogramas pesados durante debug; ative apenas quando precisar
    monitor->SerializeToXmlFile("nr-6g-urbano-lite-debug.flowmon", false, false);
    uint32_t txPackets = 0, rxPackets = 0;
    double delaySum = 0;

    for (auto it = monitor->GetFlowStats().begin(); it != monitor->GetFlowStats().end(); ++it)
    {
        txPackets += it->second.txPackets;
        rxPackets += it->second.rxPackets;
        delaySum += it->second.delaySum.GetSeconds();
    }

    std::cout << GREEN << "\n=== RESUMO ===" << RESET << std::endl;
    std::cout << "Pacotes enviados: " << txPackets << std::endl;
    std::cout << "Pacotes recebidos: " << rxPackets << std::endl;
    if (rxPackets > 0)
        std::cout << "Atraso médio: " << (delaySum / rxPackets) << " s" << std::endl;
    else
        std::cout << RED << "Nenhum pacote recebido (possível limitação de tempo ou acoplamento)" << RESET << std::endl;

    Simulator::Destroy();

    PrintStep("Simulação finalizada com sucesso");
    return 0;
}
