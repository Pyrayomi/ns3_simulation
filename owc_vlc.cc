#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"

#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OWC_VLC_Simulation");

double CalculateVlcLoss(double distance)
{
  double n = 1; // Exponente de perda de propagação para VLC
  double referenceLoss = 1.0; // Perda de propagação na distância de referência para VLC
  // L_VLC(dB) = 10 * n * log10(d) - L_0
  //double loss = referenceLoss * std::pow(distance / referenceDistance, n);
  double loss = 10 * n * std::log10(distance) - referenceLoss;

  return loss;
}

double CalculateOwcLoss(double distance)
{
  double n = 2; // Exponente de perda de propagação para OWC
  double referenceDistance = 1.0; // Distância de referência para OWC
  double referenceLoss = 1.0; // Perda de propagação na distância de referência para OWC
  // L_OWC(dB) = PL(d0) + 10 * alpha * log10(d/d0)
  //double loss = referenceLoss * std::pow(distance / referenceDistance, n);
  double loss = referenceLoss + 10 * n * std::log10(distance / referenceDistance);
  //return 10 * std::log10(loss);
  return loss;
}

std::string AdjustDataRate(const std::string& originalDataRate, double propagationLoss)
{
  double originalDataRateMbps = std::stod(originalDataRate.substr(0, originalDataRate.length() - 4));
  
  // Exemplo de função de ajuste de taxa de dados. Essa função pode ser modificada conforme necessário.
  double adjustedDataRateMbps = originalDataRateMbps * exp(-propagationLoss / 10.0);
  
  return std::to_string(adjustedDataRateMbps) + "Mbps";
}

void RunSimulation(const std::string& dataRate, const std::string& delay, double distance)
{
  NodeContainer nodes;
  nodes.Create(2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute("DataRate", StringValue(dataRate));
  pointToPoint.SetChannelAttribute("Delay", StringValue(delay));
  

  NetDeviceContainer devices;
  devices = pointToPoint.Install(nodes);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(0.0, 0.0, 0.0));
  positionAlloc->Add(Vector(distance, 0.0, 0.0));
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);


  InternetStackHelper stack;
  stack.Install(nodes);

  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer interfaces = address.Assign(devices);

  UdpEchoServerHelper echoServer(9);

  ApplicationContainer serverApps = echoServer.Install(nodes.Get(1));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(10.0));

  UdpEchoClientHelper echoClient(interfaces.GetAddress(1), 9);
  echoClient.SetAttribute("MaxPackets", UintegerValue(1));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
  echoClient.SetAttribute("PacketSize", UintegerValue(1024));

  ApplicationContainer clientApps = echoClient.Install(nodes.Get(0));
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(10.0));

  Simulator::Run();
  Simulator::Destroy();
}

int main(int argc, char* argv[])
{
  Time::SetResolution(Time::NS);
  LogComponentEnable("OWC_VLC_Simulation", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  double distance = 20.0; // Distância entre os nós em metros
  double vlcLoss = CalculateVlcLoss(distance);
  double owcLoss = CalculateOwcLoss(distance);

  // VLC Simulation
  NS_LOG_INFO("VLC Simulation");
  std::string vlcDataRate = AdjustDataRate("1Mbps", vlcLoss);
  NS_LOG_INFO("Distancia: " << distance << " Metros");
  NS_LOG_INFO("Loss VLC: " << vlcLoss << " dB");
  NS_LOG_INFO("Throughput VLC: " << vlcDataRate);
  RunSimulation(vlcDataRate, "2ms", distance);

  NS_LOG_INFO("------------------------------------------------");

  // OWC Simulation
  NS_LOG_INFO("OWC Simulation");
  std::string owcDataRate = AdjustDataRate("1Mbps", owcLoss);
  NS_LOG_INFO("Distancia: " << distance << " Metros");
  NS_LOG_INFO("Loss OWC: " << owcLoss << " dB");
  NS_LOG_INFO("Throughput OWC: " << owcDataRate);
  RunSimulation(owcDataRate, "5ms", distance);

  return 0;
}

