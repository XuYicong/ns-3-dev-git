#include <sstream>
#include <fstream>
#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/config-store-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/internet-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/seq-ts-header.h"
#include "ns3/position-allocator.h"
using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("test");
bool use_ax = true;
bool g_verbose = false;
double total_bytes_received = 0;
double per_sta_bytes_received[200];
double total_latency=0;
int total_packets_received=0;
int n_tf_cycles=-1;
int n_sa=0;
void CountTfCycles (std::string context, Ptr<const Packet> p)
{
  bool isBsrReq=(p->GetSize()== (unsigned)50+10*n_sa || p->GetSize()== (unsigned)40+10*n_sa);
  std::cout<<context<<"\t"<<" Packet length = "<<p->GetSize()<<" Is BSR Ruquest = "<<isBsrReq<<std::endl;
  n_tf_cycles+=isBsrReq;
}
void PrintTrace (std::string context, Ptr<const Packet> p)
{
  std::cout<<context<<"\t"<<" Packet length = "<<p->GetSize()<<" Time = "<<Simulator::Now ().GetMicroSeconds ()<<std::endl;
}

void PrintState (std::string context, const Time start, const Time duration, const WifiPhyState state)
{
  std::cout<<context<<" "<<state<<" Time = "<<Simulator::Now ().GetSeconds ()<<std::endl;
}

void PrintTx (std::string context, Ptr< const Packet > packet, WifiMode mode, WifiPreamble preamble, uint8_t power)
{
  std::cout<<context<<" Time = "<<Simulator::Now ().GetSeconds ()<<std::endl;
}

class wifiNodes
{
public:
  void Initialize (Ptr<MultiModelSpectrumChannel> spectrumChannel, uint32_t bw, uint32_t freq, std::string errorModelType, uint32_t distance, double interval, SpectrumWifiPhyHelper spectrumPhy, uint32_t noNodes, uint32_t tfDuration, uint32_t maxTfSlots, uint32_t tfCw, uint32_t tfCwMin, uint32_t tfCwMax, double alpha, uint32_t nScheduled);
  void SendPacketsUplink (double time, uint32_t seq, uint32_t node_id);
  void SendPacketsDownlink (double time, uint32_t seq, uint32_t node_id);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  void Configure ();
  SpectrumWifiPhyHelper m_spectrumPhy;
  NodeContainer GetStaNodes ();
  NodeContainer GetApNode ();

private:
  uint32_t m_distance;
  uint32_t m_interval;
  Ptr<MultiModelSpectrumChannel> m_spectrumChannel;
  std::string m_errorModelType;
  uint32_t m_bw;
  uint32_t m_freq;
  uint32_t m_noNodes;
  uint32_t m_tfDuration;
  uint32_t m_maxTfSlots;
  uint32_t m_tfCw;
  uint32_t m_tfCwMin;
  uint32_t m_tfCwMax;
  double m_alpha;
  uint32_t m_nScheduled;
  NodeContainer apNode;
  NodeContainer staNodes;
  NetDeviceContainer apDevice;
  NetDeviceContainer staDevices;
  void SendOnePacketDownlink (uint32_t seq, uint32_t node_id);
  void SendOnePacketUplink (uint32_t seq, uint32_t node_id);
};

NodeContainer wifiNodes::GetStaNodes ()
{
  return staNodes;
}

NodeContainer wifiNodes::GetApNode ()
{
  return apNode;
}

void wifiNodes::Initialize (Ptr<MultiModelSpectrumChannel> spectrumChannel, uint32_t bw, uint32_t freq, std::string errorModelType, uint32_t distance, double interval, SpectrumWifiPhyHelper spectrumPhy, uint32_t noNodes, uint32_t tfDuration, uint32_t maxTfSlots, uint32_t tfCw, uint32_t tfCwMin, uint32_t tfCwMax, double alpha, uint32_t nScheduled)
{
  m_interval = interval;
  m_spectrumChannel = spectrumChannel;
  m_bw = bw;
  m_freq = freq;
  m_errorModelType = errorModelType;
  m_distance = distance;
  m_interval = interval;  
  m_spectrumPhy = spectrumPhy;
  m_noNodes = noNodes;
  m_tfDuration = tfDuration;
  m_maxTfSlots = maxTfSlots;
  m_tfCw = tfCw;
  m_tfCwMax = tfCwMax;
  m_tfCwMin = tfCwMin;
  m_alpha = alpha; 
  m_nScheduled = nScheduled;
}

void wifiNodes::Configure ()
{
  staNodes.Create (m_noNodes);
  apNode.Create (1);
  
  //Configure Wi-Fi 
  WifiHelper wifi;
  if (use_ax) 
   {
     wifi.SetStandard (WIFI_STANDARD_80211ax_5GHZ);
   }
  else 
   {
     wifi.SetStandard (WIFI_STANDARD_80211ac);
   }
  WifiMacHelper mac;
  Ssid ssid = Ssid ("wifi");
  StringValue DataRate;
  //DataRate = StringValue ("HeMcs0"); // PhyRate = 58.5 Mbps
  DataRate = StringValue ("OfdmRate6Mbps"); // PhyRate = 58.5 Mbps
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", DataRate,"ControlMode", DataRate);
  
  //Config::SetDefault ("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0));
  m_spectrumPhy.SetChannel (m_spectrumChannel);
  m_spectrumPhy.SetErrorRateModel (m_errorModelType);
  m_spectrumPhy.Set ("Frequency", UintegerValue (m_freq)); // channel 36 at 20 MHz
  //m_spectrumPhy.Set ("ShortGuardEnabled", BooleanValue (false));//Xyct: to compile
  m_spectrumPhy.Set ("ChannelWidth", UintegerValue (m_bw));
	
  mac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid), "TfDuration", UintegerValue (m_tfDuration), "MaxTfSlots", UintegerValue (m_maxTfSlots), "TfCw", UintegerValue (m_tfCw), "TfCwMax", UintegerValue(m_tfCwMax), "TfCwMin", UintegerValue (m_tfCwMin), "Alpha", DoubleValue(m_alpha), "nScheduled", UintegerValue(m_nScheduled));
  apDevice = wifi.Install (m_spectrumPhy, mac, apNode);
  mac.SetType ("ns3::StaWifiMac","Ssid", SsidValue (ssid),"ActiveProbing", BooleanValue (false), "MaxTfSlots", UintegerValue (m_maxTfSlots), "TfCw", UintegerValue (m_tfCw), "TfCwMax", UintegerValue(m_tfCwMax), "TfCwMin", UintegerValue (m_tfCwMin), "Alpha", DoubleValue (m_alpha), "nScheduled", UintegerValue (m_nScheduled));
  staDevices = wifi.Install (m_spectrumPhy, mac, staNodes);
    
  InternetStackHelper internet;
  internet.Install (staNodes);
  internet.Install (apNode);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterfaces = ipv4.Assign (apDevice);
  Ipv4InterfaceContainer staInterfaces = ipv4.Assign (staDevices);

  Ptr<WifiNetDevice> receiver = DynamicCast<WifiNetDevice> (apDevice.Get(0));
  receiver->SetReceiveCallback (MakeCallback (&wifiNodes::Receive, this));
  for (uint32_t i = 0; i < staDevices.GetN (); i++)
    {
      Ptr<WifiNetDevice> receiver = DynamicCast<WifiNetDevice> (staDevices.Get(i));
      receiver->SetReceiveCallback (MakeCallback (&wifiNodes::Receive, this));
    }
}

void wifiNodes::SendOnePacketUplink (uint32_t seq, uint32_t node_id)
{
  Ptr<WifiNetDevice> sender1 = DynamicCast<WifiNetDevice> (staDevices.Get(node_id)); //new
  Ptr<WifiNetDevice> receiver1 = DynamicCast<WifiNetDevice> (apDevice.Get(0));
  Ptr<Packet> p = Create<Packet> (1023);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  Address remoteAddress1 = receiver1->GetAddress ();
  sender1->Send (p, remoteAddress1, 1); // new
}

void wifiNodes::SendOnePacketDownlink (uint32_t seq, uint32_t node_id)
{
  Ptr<WifiNetDevice> receiver1 = DynamicCast<WifiNetDevice> (staDevices.Get(node_id)); //new
  Ptr<WifiNetDevice> sender1 = DynamicCast<WifiNetDevice> (apDevice.Get(0));
  Ptr<Packet> p = Create<Packet> (1023);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  Address remoteAddress1 = receiver1->GetAddress ();
  sender1->Send (p, remoteAddress1, 1); // new
}

void wifiNodes::SendPacketsDownlink (double time, uint32_t seq, uint32_t node_id)
{
  for (uint32_t i = 0; i < m_noNodes; i++)
   {
     Simulator::Schedule (Seconds(time), &wifiNodes::SendOnePacketDownlink, this, seq, i);
   }
}

void wifiNodes::SendPacketsUplink (double time, uint32_t seq, uint32_t node_id)
{
  for (uint32_t i = 0; i < m_noNodes; i++)
   {
     Simulator::Schedule (Seconds(time), &wifiNodes::SendOnePacketUplink, this, seq, i);
   }
}

bool wifiNodes::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  Ptr<Packet> p = pkt->Copy ();
     //NS_LOG_UNCOND("Packet Size = "<<p->GetSize());
  total_packets_received++;
  total_bytes_received += p->GetSize ();
  per_sta_bytes_received [dev->GetNode ()->GetId ()] += p->GetSize ();

  p->RemoveHeader (seqTs);
  double latency=Simulator::Now().GetMicroSeconds();
  latency/=1e6;
  latency-= seqTs.GetTs().GetSeconds();
  total_latency += latency;
  if (true)
   {
     NS_LOG_UNCOND("Packet Size = "<<p->GetSize());
     NS_LOG_UNCOND("####################################################################################");
     NS_LOG_UNCOND("Wifi Sequence = "<<seqTs.GetSeq ()<<"\tSender = "<<sender<<"\tTx Time = "<<seqTs.GetTs ().GetSeconds ()<<"\tRx Time = "<<Simulator::Now ().GetMicroSeconds ()<<" latency = "<<latency);
     NS_LOG_UNCOND("####################################################################################");
   }
  return true;
}

int main (int argc, char *argv[])
{
  Packet::EnablePrinting ();
  Packet::EnableChecking ();
  uint32_t noNodes = 6;
  uint32_t wifiDistance = 1;
  double simulationTime = 10; //seconds
  uint32_t wifiFreq = 5180;
  uint32_t wifiBw = 20;
  std::string errorModelType = "ns3::NistErrorRateModel";
  uint32_t seed = 110;
  uint32_t run = 11;
  double interval = 0.0001;
  uint32_t n_packets = 10000 * simulationTime;
  bool wifiOn = true;
  uint32_t ulMode = 1;
  uint32_t tfDuration = 168;
  uint32_t maxTfSlots = 16;
  uint32_t tfCw = 7;
  uint32_t tfCwMin = 7;
  uint32_t tfCwMax = 63;
  uint32_t nScheduled = 3;
  double alpha = 0;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  for (uint32_t i = 0; i < 200; i++)
   {
     per_sta_bytes_received [i] = 0;
   }

  CommandLine cmd;
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("noNodes", "Number of WiFi nodes", noNodes);
  cmd.AddValue ("wifiDistance", "meters separation between nodes", wifiDistance);
  cmd.AddValue ("errorModelType", "select ns3::NistErrorRateModel or ns3::YansErrorRateModel", errorModelType);
  cmd.AddValue ("verbose","Print Traces",g_verbose);
  cmd.AddValue ("seed","seed",seed);
  cmd.AddValue ("run","run",run);
  cmd.AddValue ("interval", "The inter-packet interval", interval);
  cmd.AddValue ("packets", "Number of packets", n_packets);
  cmd.AddValue ("wifiOn", "", wifiOn);
  cmd.AddValue ("wifiFreq", "WiFi operating frequency in MHz", wifiFreq);
  cmd.AddValue ("use_ax", "AX is used if true, AC used otherwise", use_ax);
  cmd.AddValue ("ulMode", "UL if 1, DL if 0", ulMode);
  cmd.AddValue ("tfDuration", "tf Duration in units of slot_time", tfDuration);
  cmd.AddValue ("maxTfSlots", "maximum time slots for BSR expiry", maxTfSlots);
  cmd.AddValue ("tfCw", "maximum time slots for BSR expiry", tfCw);
  cmd.AddValue ("tfCwMin", "maximum time slots for BSR expiry", tfCwMin);
  cmd.AddValue ("tfCwMax", "maximum time slots for BSR expiry", tfCwMax);
  cmd.AddValue ("nScheduled", "number of scheduled users", nScheduled);
  cmd.AddValue ("alpha", "fraction of DL traffic", alpha);
  cmd.Parse (argc,argv);

  n_packets = 10000 * simulationTime;
  n_sa = nScheduled;
  ConfigStore config;
  config.ConfigureDefaults ();
  
  Ptr<MultiModelSpectrumChannel> spectrumChannel;
  spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel> ();
  spectrumChannel->AddPropagationLossModel (lossModel);
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  spectrumChannel->SetPropagationDelayModel (delayModel);
  
  SpectrumWifiPhyHelper spectrumPhy /*= SpectrumWifiPhyHelper::Default ()*/;
  
  wifiNodes wifi;
  wifi.Initialize (spectrumChannel, wifiBw, wifiFreq, errorModelType, wifiDistance, interval, spectrumPhy, noNodes, tfDuration, maxTfSlots, tfCw, tfCwMin, tfCwMax, alpha, nScheduled);
  wifi.Configure ();
  
  RngSeedManager::SetSeed (seed);  // Changes seed from default of 1 to 3
  RngSeedManager::SetRun (run);   // Changes run number from default of 1 to 7
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0.0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.01));

  if (wifiOn) 
  { 
    if (ulMode == 1)
     {
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsUplink (0.1 + interval * i, i, 0);
     }
    else if (ulMode == 0)
     {
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsDownlink (0.1 + interval * i, i, 0);
     }
    else if (ulMode == 2)
    {
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsUplink (0.1 + interval * i, i, 0);
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsDownlink (0.1 + interval * i, i, 0);
    }
  }

  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (uint32_t i = 0; i < noNodes; i++)
   {
     positionAlloc->Add (Vector (cos(2 * M_PI * i/noNodes), sin(2 * M_PI * i/noNodes), 0.0));
   }
  mobility.Install (wifi.GetApNode());
  mobility.Install (wifi.GetStaNodes());
  
  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",MakeCallback(&CountTfCycles));
  if (g_verbose) {
  	//Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",MakeCallback(&PrintTrace));//Xyct: for compile
  	//Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxBegin",MakeCallback(&PrintTrace));
  	//Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",MakeCallback(&PrintTrace));
  	//Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",MakeCallback(&PrintTrace));
  	Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",MakeCallback(&PrintTrace));
        //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",MakeCallback(&PrintState));
        Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop",MakeCallback(&PrintTrace));
  	Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",MakeCallback(&PrintTrace));
        Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/Tx",MakeCallback(&PrintTx));
  }

  Simulator::Stop (Seconds (simulationTime ));
  config.ConfigureAttributes ();
  
  Simulator::Run ();

  Simulator::Destroy ();

  std::cout<<"Total Packets Received = "<<total_packets_received<<std::endl;
  std::cout<<"Total latency = "<<total_latency<<std::endl;
  std::cout<<"Average latency = "<< total_latency/total_packets_received <<std::endl;
  std::cout<<"Number of TF cycles = "<<n_tf_cycles<<std::endl;
  int n_scheduled_packets = n_tf_cycles * nScheduled;
  std::cout<<"Number of scheduled packets = "<<n_scheduled_packets<<std::endl;
  int n_uora_packets = total_packets_received - n_scheduled_packets;
  std::cout<<"Number of UORA packets = "<<n_uora_packets<<std::endl;
  std::cout<<"Printing Throughputs\n";
  double total_throughput = total_bytes_received * 8/(6000000 * (simulationTime - 1));
  std::cout<<"Aggregate Throughput = "<<total_throughput<< " Mbps\n";
  double beta = (double)n_uora_packets/n_tf_cycles;
  std::cout<<"BSR delivery rate = "<<beta<<std::endl;
  std::cout<<"Efficiency = "<<beta/(9-nScheduled)<<std::endl;

  std::ofstream outfile("../experiments-3.28.txt",std::ios::app);
  outfile<<"cwmax:"<<tfCwMax<<", cwmin:"<<tfCwMin<<", noNodes:"<<noNodes<<", N_ra:"<<(9-nScheduled)<<", BSR_delivery_rate:"<<beta<<", Efficiency = "<<beta/(9-nScheduled)<<", average latency(seconds):"<<total_latency/total_packets_received<<", total_throughput:"<<total_throughput<<", uora_throughput:"<<total_throughput/total_packets_received*n_uora_packets<<std::endl;

  return 0;
}


