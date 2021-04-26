#include <sstream>
#include <fstream>
#include <iomanip>
#include "ns3/gnuplot.h"
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
bool g_verbose = false;
double total_bytes_received = 0;
double per_sta_bytes_received[200];
double total_latency=0, result_normalized_throughput=0, result_colliding_rate_trigger=0, 
       result_colliding_rate_data=0, result_success_rate=0;
char ssidName[200][10];
uint32_t successDuration=0;
int total_packets_received=0, total_packets_sent=0;
int n_tf_cycles=0, n_tf_sent=0;
int n_sta_per_ap=0, time_tf_received;
const uint32_t sifs = 16, slotTime= 9, difs= 34;//in micro seconds
void CountTfCycles (std::string context, uint16_t packetDuration, uint16_t numReceived)
{
    if(numReceived==0){
        n_tf_sent++;//Sent trigger frame
        return;
    }
    uint16_t n_tf_passed = time_tf_received / n_sta_per_ap;
    time_tf_received =0;
    //Reason of using extra variable: there may be collided TFs received
    n_tf_cycles+=n_tf_passed;
  //NS_LOG_UNCOND(context<<" "<<numReceived<<" packets received, "<<"tfCycle No."<<n_tf_cycles);
  total_packets_received += numReceived;
  /* When calculating RU efficiency, should consider no-data-cycle 
   * which means n_tf_cycles plus n_tf_passed
   * When calculating Bianchi efficiency, n_tf_passed are successful periods
   * contributing an RU efficiency of 0
   * so success duration contains n_tf_passed
   */
    successDuration += packetDuration * n_tf_passed;
}
void TriggerReceived(std::string context, bool isDataSent)
{
    time_tf_received++;
    if(isDataSent){
        total_packets_sent++;//Sent data
    }
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
  void Initialize (Ptr<MultiModelSpectrumChannel> spectrumChannel, uint32_t bw, uint32_t freq, std::string errorModelType, SpectrumWifiPhyHelper spectrumPhy,uint32_t noAps, uint32_t noNodes, uint32_t tfDuration, uint32_t maxTfSlots, uint32_t tfCw, uint32_t tfCwMin, uint32_t tfCwMax, double alpha, uint32_t nScheduled, uint32_t noRus, uint32_t packetSize);
  void SendPacketsUplink (double time, uint32_t seq);
  void SendPacketsDownlink (double time, uint32_t seq);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  void Configure ();
  SpectrumWifiPhyHelper m_spectrumPhy;
  NodeContainer GetStaNodes ();
  NodeContainer GetApNodes ();

private:
  Ptr<MultiModelSpectrumChannel> m_spectrumChannel;
  std::string m_errorModelType;
  uint32_t m_bw;
  uint32_t m_freq;
  uint32_t m_noStas;
  uint32_t m_noAps;
  uint32_t m_tfDuration;
  uint32_t m_maxTfSlots;
  uint32_t m_tfCw;
  uint32_t m_tfCwMin;
  uint32_t m_tfCwMax;
  double m_alpha;
  uint32_t m_nScheduled;
  uint32_t m_noRus;
  uint32_t m_packetSize;
  NodeContainer apNodes;
  NodeContainer staNodes;
  NetDeviceContainer apDevices;
  NetDeviceContainer staDevices;
  void SendOnePacketDownlink (uint32_t seq, uint32_t ap_id,uint32_t node_id);
  void SendOnePacketUplink (uint32_t seq, uint32_t ap_id,uint32_t node_id);
};

NodeContainer wifiNodes::GetStaNodes ()
{
  return staNodes;
}

NodeContainer wifiNodes::GetApNodes ()
{
  return apNodes;
}

void wifiNodes::Initialize (Ptr<MultiModelSpectrumChannel> spectrumChannel, uint32_t bw, uint32_t freq, std::string errorModelType, SpectrumWifiPhyHelper spectrumPhy,uint32_t noAps, uint32_t noNodes, uint32_t tfDuration, uint32_t maxTfSlots, uint32_t tfCw, uint32_t tfCwMin, uint32_t tfCwMax, double alpha, uint32_t nScheduled, uint32_t noRus, uint32_t packetSize)
{
  m_spectrumChannel = spectrumChannel;
  m_bw = bw;
  m_freq = freq;
  m_errorModelType = errorModelType;
  m_spectrumPhy = spectrumPhy;
  m_noStas = noNodes;
  m_noAps = noAps;
  m_tfDuration = tfDuration;
  m_maxTfSlots = maxTfSlots;
  m_tfCw = tfCw;
  m_tfCwMax = tfCwMax;
  m_tfCwMin = tfCwMin;
  m_alpha = alpha; 
  m_nScheduled = nScheduled;
  m_noRus = noRus;
  m_packetSize = packetSize;
}

void wifiNodes::Configure ()
{
  staNodes.Create (m_noStas);
  apNodes.Create (m_noAps);
  
  //Configure Wi-Fi 
  WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211ax_5GHZ);
  WifiMacHelper mac;
  StringValue DataRate;
  DataRate = StringValue ("HeMcs2");
  //DataRate = StringValue ("OfdmRate6Mbps"); 
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", DataRate,"ControlMode", DataRate,"NonUnicastMode",DataRate);
  
  //Config::SetDefault ("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0));
  m_spectrumPhy.SetChannel (m_spectrumChannel);
  m_spectrumPhy.SetErrorRateModel (m_errorModelType);
  m_spectrumPhy.Set ("Frequency", UintegerValue (m_freq)); //Note: to change channel width, either frequency or channel number should be changed as well
  //MCS2 + 40MHz + short GI = 45Mbps, + long GI = 40.5Mbps
  //m_spectrumPhy.Set ("ShortGuardEnabled", BooleanValue (false));//Xyct: to compile
  m_spectrumPhy.Set ("ChannelWidth", UintegerValue (m_bw));
  m_spectrumPhy.Set ("TxPowerStart", DoubleValue (26));
  m_spectrumPhy.Set ("TxPowerEnd", DoubleValue (26));
	
  mac.SetType ("ns3::ApWifiMac", "TfDuration", UintegerValue (m_tfDuration), 
    "BeaconGeneration", BooleanValue(false),
    "EnableBeaconJitter",BooleanValue(true),    
    "RuNumber", UintegerValue(m_noRus),
    "MaxTfSlots", UintegerValue (m_maxTfSlots), "TfCw", UintegerValue (m_tfCw), "TfCwMax", UintegerValue(m_tfCwMax), "TfCwMin", UintegerValue (m_tfCwMin), "Alpha", DoubleValue(m_alpha), "nScheduled", UintegerValue(m_nScheduled));
  for (uint32_t i = 0; i < m_noAps; ++i){
    Ssid ssid = Ssid (ssidName[i]);
    mac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid));
    auto tmp = wifi.Install (m_spectrumPhy, mac, apNodes.Get(i));
    wifi.AssignStreams(tmp, i);
    apDevices.Add(tmp );
  }

  mac.SetType ("ns3::StaWifiMac",
    "ActiveProbing", BooleanValue (false),
    "ProbeRequestTimeout", TimeValue(MilliSeconds(m_noStas)),
    "MaxMissedBeacons", UintegerValue (20000),
    "MaxTfSlots", UintegerValue (m_maxTfSlots), "TfCw", UintegerValue (m_tfCw), "TfCwMax", UintegerValue(m_tfCwMax), "TfCwMin", UintegerValue (m_tfCwMin), "Alpha", DoubleValue (m_alpha), "nScheduled", UintegerValue (m_nScheduled));

    uint32_t stasPerAp = m_noStas/m_noAps;
  for (uint32_t i = 0, j=0; i < m_noAps;++i){
    Ssid ssid = Ssid (ssidName[i]);
    mac.SetType ("ns3::StaWifiMac","Ssid", SsidValue (ssid));
    for(uint32_t k=0; k<stasPerAp; ++k,++j){
        auto tmp = wifi.Install (m_spectrumPhy, mac, staNodes.Get(j));
        wifi.AssignStreams(tmp, j);
        staDevices.Add(tmp );
    }
  }
    
  InternetStackHelper internet;
  internet.Install (staNodes);
  internet.Install (apNodes);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterfaces = ipv4.Assign (apDevices);
  Ipv4InterfaceContainer staInterfaces = ipv4.Assign (staDevices);

  for (uint32_t i = 0; i < apDevices.GetN (); i++)
  {
    Ptr<WifiNetDevice> receiver = DynamicCast<WifiNetDevice> (apDevices.Get(i));
    receiver->SetReceiveCallback (MakeCallback (&wifiNodes::Receive, this));
  }
  for (uint32_t i = 0; i < staDevices.GetN (); i++)
    {
      Ptr<WifiNetDevice> receiver = DynamicCast<WifiNetDevice> (staDevices.Get(i));
      receiver->SetReceiveCallback (MakeCallback (&wifiNodes::Receive, this));
    }
}

void wifiNodes::SendOnePacketUplink (uint32_t seq,uint32_t ap_id, uint32_t node_id)
{
  Ptr<WifiNetDevice> sender1 = DynamicCast<WifiNetDevice> (staDevices.Get(node_id)); 
  Ptr<WifiNetDevice> receiver1 = DynamicCast<WifiNetDevice> (apDevices.Get(ap_id));
  Ptr<Packet> p = Create<Packet> (m_packetSize);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  Address remoteAddress1 = receiver1->GetAddress ();
  sender1->Send (p, remoteAddress1, 1); // new
}

void wifiNodes::SendOnePacketDownlink (uint32_t seq,uint32_t ap_id, uint32_t node_id)
{
  Ptr<WifiNetDevice> receiver1 = DynamicCast<WifiNetDevice> (staDevices.Get(node_id)); 
  Ptr<WifiNetDevice> sender1 = DynamicCast<WifiNetDevice> (apDevices.Get(ap_id));
  Ptr<Packet> p = Create<Packet> (m_packetSize);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  Address remoteAddress1 = receiver1->GetAddress ();
  sender1->Send (p, remoteAddress1, 1); // new
}

void wifiNodes::SendPacketsDownlink (double time, uint32_t seq)
{
  for (uint32_t j= 0; j < m_noAps; j++)
  for (uint32_t i = 0; i < m_noStas; i++)
   {
        Simulator::Schedule (Seconds(time), &wifiNodes::SendOnePacketDownlink, this, seq, j, i);
   }
}

void wifiNodes::SendPacketsUplink (double time, uint32_t seq)
{
    uint32_t stasPerAp = m_noStas/m_noAps;
  for (uint32_t j = 0, i = 0; i < m_noStas; i++)
   {
        if((1+j) * stasPerAp<= i) j++;
        Simulator::Schedule (Seconds(time), &wifiNodes::SendOnePacketUplink, this, seq, j, i);
   }
}

bool wifiNodes::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  Ptr<Packet> p = pkt->Copy ();
     //NS_LOG_UNCOND("Packet Size = "<<p->GetSize());
  total_bytes_received += p->GetSize ();
  per_sta_bytes_received [dev->GetNode ()->GetId ()] += p->GetSize ();

  p->RemoveHeader (seqTs);
  double latency=Simulator::Now().GetMicroSeconds();
  latency/=1e6;
  latency-= seqTs.GetTs().GetSeconds();
  total_latency += latency;
  if (false)
   {
     NS_LOG_UNCOND("Packet Size = "<<p->GetSize());
     NS_LOG_UNCOND("####################################################################################");
     NS_LOG_UNCOND("Wifi Sequence = "<<seqTs.GetSeq ()<<"\tSender = "<<sender<<"\tTx Time = "<<seqTs.GetTs ().GetSeconds ()<<"\tRx Time = "<<Simulator::Now ().GetMicroSeconds ()<<" latency = "<<latency);
     NS_LOG_UNCOND("####################################################################################");
   }
  return true;
}
double runOnce(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
void runTrials(Gnuplot2dDataset &throughputDataset, Gnuplot2dDataset &succRateDataset,uint32_t noAps,uint32_t noNodes,uint32_t noRus, uint32_t load, uint32_t packetSize, int trials, uint16_t graph_type){
    double stDev = 0, avgTh=0, throughput[9]={0}, avgRate=0, succRate[9]={0};
    for (int i = 0; i < trials; ++i)
    {
        total_latency = successDuration =time_tf_received
            =total_packets_sent = total_packets_received
            =total_bytes_received = n_tf_cycles = n_tf_sent =0;
        throughput[i] = runOnce(noAps, noNodes, noRus, load, packetSize, i);
        succRate[i] = result_success_rate;
        avgTh += throughput[i];
        avgRate += succRate[i];
    }
    avgTh /= trials;
    avgRate /= trials;
    int x=0;
    switch(graph_type){
	    case 2:x=noNodes;break;
	    case 3:x=packetSize;break;
	    case 4:x=load;break;
	    case 0:x=noAps;break;
	    case 1:x=noRus;break;
	    default: std::cerr<<"Unrecognised graph type when trying to add data point";
    }
    for (int i = 0; i < trials; ++i)
    {
        stDev += pow (throughput[i] - avgTh, 2);
    }
    stDev = sqrt (stDev / (trials - 1));
	throughputDataset.Add(x, avgTh, stDev);
    stDev = 0
    for (int i = 0; i < trials; ++i)
    {
        stDev += pow (succRate[i] - avgRate, 2);
    }
    stDev = sqrt (stDev / (trials - 1));
    succRateDataset.Add(x, avgRate, stDev);
}
int main (int argc, char *argv[])
{
  for (uint32_t i = 0; i < 200; i++)
  {
    sprintf(ssidName[i],"%x",i); 
  }
  //parameters controlling the simulation process
  std::string name[]={"AP","RU","STA","Packet_Size","load"};
  //a row means a graph with the same x-axis. A column means an argument in this graph
    int min[5][5]={
        {1,10,2,128,1000},
        {1,10,1,128,1000},
        {2,2,5,1024,10},
        {1,10,3,128,1000},
        {1,10,1,128,10}};
    int max[5][5]={
        {11, 18, 13, 1024, 1000},
        {3, 18, 11, 1024, 1000},
        {3, 12, 10, 1024, 1000},
        {10, 18, 3, 1024, 1000},
        {3, 18, 3, 1024, 1000}};
    int step[5][5]={
    {2, 2, 8, 8, 10},
    {2, 2, 2, 4, 10},
    {2, 2, 5, 2, 10},
    {3, 2, 2, 128, 10},
    {2, 2, 2, 2, 10}};
    bool isMultiply[5][5]={//is the "step" applied by multiply or addition
        {0,1,0,1,1},
        {0,1,0,1,1},
        {0,0,0,1,1},
        {0,1,0,0,1},
        {0,1,0,1,1}};
    //numbers are enum of argument type
    int order[5][5]={
    {4,3,1,2,0},
    {4,3,1,0,2},
    {4,3,0,2,1},
    {4,1,2,0,3},
    {0,1,2,3,4}};
    int arg[5]={0};
    for(unsigned dia_idx=0; dia_idx<3; dia_idx++)
    {//10 graphs in total
    Gnuplot gnuplot = Gnuplot ("throughput diagram"), succRatePlot= Gnuplot("success rate diagram");
    gnuplot.SetTerminal ("canvas");
    succRatePlot.SetTerminal ("canvas");
  gnuplot.SetLegend (name[order[dia_idx][4]], "throughput");
  succRatePlot.SetLegend (name[order[dia_idx][4]], "MAC layer contention success rate");
  std::stringstream ss;
    ss.str ("");
  int trials = 3;
  ss << "set xrange [" << min[dia_idx][order[dia_idx][4]] << ":" << max[dia_idx][order[dia_idx][4]] << "]\n"
     << "set xtics " << step[dia_idx][order[dia_idx][4]] << "\n"
     << "set grid xtics ytics\n"
     << "set mytics\n"
     << "set style line 1 linewidth 3\n"
     << "set style line 2 linewidth 3\n"
     << "set style line 3 linewidth 3\n"
     << "set style line 4 linewidth 3\n"
     << "set style line 5 linewidth 3\n"
     << "set style line 6 linewidth 3\n"
     << "set style line 7 linewidth 3\n"
     << "set style line 8 linewidth 3\n"
     << "set style increment user";
    gnuplot.SetExtra (ss.str ());
    succRatePlot.SetExtra (ss.str ());
    for(arg[0] = min[dia_idx][order[dia_idx][0]]; arg[0]<=max[dia_idx][order[dia_idx][0]]; (isMultiply[dia_idx][order[dia_idx][0]]?arg[0]*=step[dia_idx][order[dia_idx][0]]:arg[0]+=step[dia_idx][order[dia_idx][0]]))
    for(arg[1] = min[dia_idx][order[dia_idx][1]]; arg[1]<=max[dia_idx][order[dia_idx][1]]; (isMultiply[dia_idx][order[dia_idx][1]]?arg[1]*=step[dia_idx][order[dia_idx][1]]:arg[1]+=step[dia_idx][order[dia_idx][1]]))
    for(arg[2] = min[dia_idx][order[dia_idx][2]]; arg[2]<=max[dia_idx][order[dia_idx][2]]; (isMultiply[dia_idx][order[dia_idx][2]]?arg[2]*=step[dia_idx][order[dia_idx][2]]:arg[2]+=step[dia_idx][order[dia_idx][2]]))
    for(arg[3] = min[dia_idx][order[dia_idx][3]]; arg[3]<=max[dia_idx][order[dia_idx][3]]; (isMultiply[dia_idx][order[dia_idx][3]]?arg[3]*=step[dia_idx][order[dia_idx][3]]:arg[3]+=step[dia_idx][order[dia_idx][3]]))
    {
        Gnuplot2dDataset throughputDataset, succRateDataset;
        throughputDataset.SetErrorBars (Gnuplot2dDataset::Y);
        succRateDataset.SetErrorBars (Gnuplot2dDataset::Y);
        throughputDataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);
        succRateDataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);
        for(arg[4] = min[dia_idx][order[dia_idx][4]]; arg[4]<=max[dia_idx][order[dia_idx][4]]; (isMultiply[dia_idx][order[dia_idx][4]]?arg[4]*=step[dia_idx][order[dia_idx][4]]:arg[4]+=step[dia_idx][order[dia_idx][4]]))
        {
            int nextArg[5];
            //arg[n] is in the order of iteration, while nextArg[n] should be the original order
            for(int tmp=0;tmp<5;tmp++) nextArg[order[dia_idx][tmp]]=arg[tmp];
            // AP, STA, RU, load, packetSize
        for(int tmp=0;tmp<5;tmp++) std::cout<<name[order[dia_idx][tmp]]<<": "<<arg[tmp]<<" ";
        std::cout<<std::endl;
        	runTrials(throughputDataset, succRateDataset, nextArg[0], nextArg[2], nextArg[1], nextArg[4], nextArg[3]-1, trials, order[dia_idx][4]);
        }
        ss.str("");
        for(int tmp=0;tmp<4;tmp++) ss<<name[order[dia_idx][tmp]]<<": "<<arg[tmp]<<" ";
        throughputDataset.SetTitle(ss.str());
        succRateDataset.SetTitle(ss.str());
        gnuplot.AddDataset (throughputDataset);
        succRatePlot.AddDataset (succRateDataset);
    }
    std::ofstream throughputFile(name[order[dia_idx][3]]+"-"+name[order[dia_idx][4]]+"-throughput.plt");
    gnuplot.GenerateOutput (throughputFile);
    throughputFile.close();
    std::ofstream succRateFile(name[order[dia_idx][3]]+"-"+name[order[dia_idx][4]]+"-succRate.plt");
    succRatePlot.GenerateOutput (succRateFile);
    succRateFile.close();
    }
}
//load: maximum is 1000
double runOnce (uint32_t noAps, uint32_t noNodes, uint32_t noRus, uint32_t load, uint32_t packetSize, uint32_t run) 
{
  Packet::EnablePrinting ();
  Packet::EnableChecking ();
  //uint32_t noRus = 9;
  //uint32_t noNodes = 3;
  //uint32_t noAps = 3;
  double simulationTime = 3; //seconds
  uint32_t wifiFreq = 5590;
  uint32_t wifiBw = 40;
  std::string errorModelType = "ns3::TableBasedErrorRateModel";
  uint32_t seed = 110;
  //uint32_t run = 11;
  double interval = 1.0/load;
  uint32_t n_packets = 1000 * simulationTime;
  uint32_t ulMode = 1;//Only UL is supported
  uint32_t tfDuration = 168;
  uint32_t maxTfSlots = 16;
  uint32_t cw = 15;
  uint32_t tfCw = 8;
  uint32_t tfCwMin = 8;
  uint32_t tfCwMax = 64;
  uint32_t nScheduled = 0;
  double alpha = 0;

  for (uint32_t i = 0; i < 200; i++)
   {
     per_sta_bytes_received [i] = 0;
   }

  /*CommandLine cmd;
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("noNodes", "Number of non-AP stations of each AP", noNodes);
  cmd.AddValue ("noAps", "Number of WiFi APs", noAps);
  cmd.AddValue ("errorModelType", "select ns3::NistErrorRateModel or ns3::YansErrorRateModel", errorModelType);
  cmd.AddValue ("verbose","Print Traces",g_verbose);
  cmd.AddValue ("seed","seed",seed);
  cmd.AddValue ("run","run",run);
  cmd.AddValue ("interval", "The inter-packet interval", interval);
  cmd.AddValue ("packets", "Number of packets", n_packets);
  cmd.AddValue ("cw", "Contention window among APs", cw);
  cmd.AddValue ("wifiFreq", "WiFi operating frequency in MHz", wifiFreq);
  cmd.AddValue ("ulMode", "UL if 1, DL if 0", ulMode);
  cmd.AddValue ("tfDuration", "tf Duration in units of slot_time", tfDuration);
  cmd.AddValue ("maxTfSlots", "maximum time slots for BSR expiry", maxTfSlots);
  cmd.AddValue ("tfCw", "maximum time slots for BSR expiry", tfCw);
  cmd.AddValue ("tfCwMin", "maximum time slots for BSR expiry", tfCwMin);
  cmd.AddValue ("tfCwMax", "maximum time slots for BSR expiry", tfCwMax);
  cmd.AddValue ("nScheduled", "number of scheduled users", nScheduled);
  cmd.AddValue ("alpha", "fraction of DL traffic", alpha);
  cmd.Parse (argc,argv);
*/
  Config::SetDefault ("ns3::Txop::MinCw",UintegerValue(cw));
  n_packets = load * simulationTime;
  n_sta_per_ap =noNodes;
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
  wifi.Initialize (spectrumChannel, wifiBw, wifiFreq, errorModelType, spectrumPhy, noAps, noAps*noNodes, tfDuration, maxTfSlots, tfCw, tfCwMin, tfCwMax, alpha, nScheduled, noRus, packetSize);//1023
  wifi.Configure ();
  
  RngSeedManager::SetSeed (seed);  // Changes seed from default of 1 to 3
  RngSeedManager::SetRun (run);   // Changes run number from default of 1 to 7

    if (ulMode == 1)
     {
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsUplink (0.1 + interval * i, i);
     }
    else if (ulMode == 0)
     {
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsDownlink (0.1 + interval * i, i);
     }
    else if (ulMode == 2)
    {
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsUplink (0.1 + interval * i, i);
       for (uint32_t i = 0; i <= n_packets; i++) wifi.SendPacketsDownlink (0.1 + interval * i, i);
    }
    {//Set mobility
  uint32_t width = 2, n = sqrt(noAps-1)+1, delta = width/(n-1?n-1:n), row = 0, col = 0;
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    Ptr<RandomDiscPositionAllocator> disc = CreateObject<RandomDiscPositionAllocator> ();
    Ptr<UniformRandomVariable> radius = CreateObject<UniformRandomVariable> ();
    radius->SetAttribute ("Min", DoubleValue (20));
    radius->SetAttribute ("Max", DoubleValue (25));//This distance, every sta is interferenced
    //radius->SetAttribute ("Max", DoubleValue (30));//This distance, some sta are not interferenced
    disc->SetRho(radius);
    mobility.SetPositionAllocator (disc);
  for(uint32_t j = 0; j < noAps; j++)
  {
     positionAlloc->Add (Vector (row, col, 0));
    disc->SetX(row); disc->SetY(col); disc->SetZ(0);
    std::cout<<row<<" "<<col<<std::endl;
    row += delta;
    if(row>width){
        row=0;
        col+=delta;
    }
    for (uint32_t i = 0; i < noNodes; i++)
    {
        mobility.Install (wifi.GetStaNodes().Get(i+j*noNodes));
    }
  }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (wifi.GetApNodes());
    }
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/tfCycleSuccess",MakeCallback(&CountTfCycles));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/TriggerReceived",MakeCallback(&TriggerReceived));
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
  //std::cout<<"Total latency = "<<total_latency<<std::endl;
  //std::cout<<"Average latency = "<< total_latency/total_packets_received <<std::endl;
  std::cout<<"Number of TF cycles = "<<n_tf_cycles<<std::endl;
  int n_scheduled_packets = n_tf_cycles * nScheduled;
  std::cout<<"Number of scheduled packets = "<<n_scheduled_packets<<std::endl;
  int n_uora_packets = total_packets_received - n_scheduled_packets;
  std::cout<<"Number of UORA packets = "<<n_uora_packets<<std::endl;
  std::cout<<"Printing Throughputs\n";
  double total_throughput = total_bytes_received * 8/(6000000 * (simulationTime - 1));
  std::cout<<"Aggregate Throughput = "<<total_throughput<< " Mbps\n";
  double beta = (double)n_uora_packets/n_tf_cycles;
  std::cout<<"Avg. N Packet per cycle = "<<beta<<std::endl;
  double ofdmaEfficiency = beta/(noRus-nScheduled);
  std::cout<<"RU Efficiency = "<<ofdmaEfficiency<<std::endl;
  double bianchiEfficiency = successDuration/(1e6*(simulationTime-1));
  std::cout<<"Time Efficiency = "<<bianchiEfficiency<<std::endl;
  std::cout<<"Total Efficiency = "<<ofdmaEfficiency * bianchiEfficiency<<std::endl;
  std::cout<<"N data packets sent = "<<total_packets_sent<<std::endl;
  std::cout<<"N trigger sent = "<<n_tf_sent<<std::endl;
  result_normalized_throughput = ofdmaEfficiency * bianchiEfficiency;
  long double data_lost = total_packets_sent - total_packets_received,
           trigger_lost = n_tf_sent - n_tf_cycles;
  result_colliding_rate_data = data_lost / total_packets_sent,
    result_colliding_rate_trigger = trigger_lost / n_tf_sent;
  std::cout<<"data colliding rate= "<<result_colliding_rate_data<<std::endl;
  std::cout<<"trigger colliding rate = "<<result_colliding_rate_trigger<<std::endl;
  result_success_rate = n_uora_packets/(long double)(n_tf_sent*noNodes);
  std::cout<<"success rate = "<<result_success_rate<<std::endl;
  std::ofstream outfile("../experiments-log.txt",std::ios::app);
  outfile<<"load = "<<load<<", packet size = "<<packetSize;
  outfile<<", noRus:"<<noRus<<", noNodes:"<<noNodes<<", noAps:"<<noAps<<", RU Efficiency:"<<ofdmaEfficiency<<", CSMA/CA Efficiency:"<<bianchiEfficiency<<", Total Efficiency:"<<ofdmaEfficiency * bianchiEfficiency<<", total_throughput:"<<total_throughput<<", uora_throughput:"<<total_throughput/total_packets_received*n_uora_packets;
  outfile<<", data colliding rate= "<<result_colliding_rate_data;
  outfile<<", trigger colliding rate = "<<result_colliding_rate_trigger<<std::endl;
    outfile.close();
  return total_throughput*18/noRus;
}


