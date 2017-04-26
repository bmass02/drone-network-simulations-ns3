/*
*
*
*
* NETWORK TOPOLOGY
                MAX_Y             MAX_X
        ___________\____________  /
        |   *                  | /
        |  *        *     *    |/
        |      *         *     |
       /|           *        * |
      / |   *           *      |
     /  |______________________|
 MIN_X              \
                  MIN_Y
*
*
*
*/


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/collision-avoidance-helper.h"
#include "ns3/olsr-repositories.h"
#include "ns3/object.h"
#include "ns3/stats-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DroneAdhoc");

enum PowerManagement { Rx, Tx, TxPower, EdThreshold, None};

std::string averages = "";

class DroneExperiment : public Object
{
  //Constructor and Deconstructor
public:
  static TypeId GetTypeId (void);

  DroneExperiment ();
  ~DroneExperiment ();

  //Runner
public:
  std::vector<Gnuplot2dDataset> Run (PowerManagement method, bool dynamicPowerManagement, bool onStateChange);

  // Configuration Helpers
  void SetDefaults ();
  void Configure ();
  void CreateNodes ();
  void InstallWifi ();
  void InstallMobility ();
  void InstallRouting ();
  void InstallApps ();
  void ConnectCallbacks ();

  // Conversion Helpers
  std::string UniformRange  (uint32_t min, uint32_t max);
  std::string WalkBounds    (uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY);
  double      Percentage    (int amount, int outOf);


  // Callbacks
private:
  void StateTransitionCallback    (std::string path, Time start, Time duration, WifiPhy::State state);
  void NeighborSetChanged         (Ptr<Node> node, ns3::olsr::NeighborSet neighbors);
  void CollisionAvoidanceRxTrace  (const Ptr<Packet> &packet, const Ipv4Address &addr);
  void CollisionAvoidanceTxTrace  (const Ptr<Packet> &packet, const Ipv4Address &addr);

  void ThroughputRxPacketTrace    (Ptr<const Packet> packet);
  void ThroughputTxPacketTrace    (Ptr<const Packet> packet);

  void RecordStats                ();


  // Variables
private:
  // Packet counters
  int m_packetsSent;
  int m_packetsReceived;
  int cp_packetsSent;
  int cp_packetsReceived;

  // Power management shared memory
  std::vector<double>         m_phyStats;
  std::vector<Ptr<WifiPhy>>   m_phyList;

  // Power management statistics
  double m_maxStat;
  double m_minStat;

  // Power management configs
  bool              m_onStateChange;
  bool              m_dynamicPowerManagement;
  PowerManagement   m_method;
  std::string       m_methodString;

  // Drone info
  NodeContainer         m_drones;
  uint32_t              m_numDrones;
  NetDeviceContainer    m_devices;
  ApplicationContainer  m_collisionApps;

  // Drone Environment info (in meters)
  uint32_t m_maxX;
  uint32_t m_minX;
  uint32_t m_maxY;
  uint32_t m_minY;

  // Drone communication info
  uint32_t  m_collPacketSize; // bytes
  double    m_collFrequency; // seconds
  uint32_t  m_imagePacketSize; //
  double    m_imageFrequency; // seconds

  // PHY info
  std::string m_phyMode;

  //Throughput
  Gnuplot2dDataset  m_packetLossRate;
  Gnuplot2dDataset  m_dataThroughput;
  uint32_t          m_dataReceived;
  uint32_t          m_dataSent;
  uint32_t          cp_dataReceived;
  uint32_t          cp_dataSent;
};

TypeId
DroneExperiment::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DroneExperiment")
    .SetParent<Object> ()
    .SetGroupName ("Experiments")
    .AddConstructor<DroneExperiment> ()
    .AddAttribute ("CollisionPacketSize", "The size of the packets used for the CollisionAvoidanceApp.",
                  UintegerValue (1024),
                  MakeUintegerAccessor (&DroneExperiment::m_collPacketSize),
                  MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("CollisionPacketFrequency", "The interval in seconds of how often CollisionAvoidance packets should be sent.",
                  DoubleValue (0.5),
                  MakeDoubleAccessor (&DroneExperiment::m_collFrequency),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("ImagePacketSize", "The size of the packets used for the ImageManagementApp (app not yet created).",
                  UintegerValue (5*1024*1024),
                  MakeUintegerAccessor (&DroneExperiment::m_imagePacketSize),
                  MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("ImagePacketFrequency", "The interval in seconds of how often ImageManagementApp packets should be sent.",
                  DoubleValue (10.0),
                  MakeDoubleAccessor (&DroneExperiment::m_imageFrequency),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("NumDrones", "The number of drones to use in the experiment.",
                  UintegerValue (30),
                  MakeUintegerAccessor (&DroneExperiment::m_numDrones),
                  MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("PHYmode", "The PHY mode to use for the PHY layer.",
                  StringValue ("OfdmRate6Mbps"),
                  MakeStringAccessor (&DroneExperiment::m_phyMode),
                  MakeStringChecker ())
    .AddAttribute ("MaxX", "The right most wall of the drone environment.",
                  UintegerValue (100),
                  MakeUintegerAccessor (&DroneExperiment::m_maxX),
                  MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("MinX", "The left most wall of the drone environment.",
                  UintegerValue (0),
                  MakeUintegerAccessor (&DroneExperiment::m_minX),
                  MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("MaxY", "The upper most wall of the drone environment.",
                  UintegerValue (100),
                  MakeUintegerAccessor (&DroneExperiment::m_maxY),
                  MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("MinY", "The bottom most wall of the drone environment.",
                  UintegerValue (0),
                  MakeUintegerAccessor (&DroneExperiment::m_minY),
                  MakeUintegerChecker<uint32_t> (1))
    ;
  return tid;
}

DroneExperiment::DroneExperiment ()
  : m_numDrones (30),
    m_maxX (100),
    m_minX (0),
    m_maxY (100),
    m_minY (0),
    m_collPacketSize (1024),
    m_collFrequency (0.5),
    m_imagePacketSize (5*1024*1024),
    m_imageFrequency (10.0),
    m_phyMode ("OfdmRate6Mbps")
{
}

DroneExperiment::~DroneExperiment ()
{
}

void
DroneExperiment::SetDefaults ()
{
  //Make sure these are reset just in case the same Experiment is run twice
  m_packetsSent = 0;
  m_packetsReceived = 0;
  m_dataReceived = 0;
  m_dataSent = 0;
  cp_packetsSent = 0;
  cp_packetsReceived = 0;
  cp_dataReceived = 0;
  cp_dataSent = 0;

  m_phyList = {};
  m_phyStats = {};

  // Create enough space for all drones
  m_phyList.resize (0);
  m_phyList.resize (m_numDrones);
  m_phyStats.resize (0);
  m_phyStats.resize (m_numDrones);

  switch (m_method) {
    case Rx:
      m_methodString = "RxGain";
      break;
    case Tx:
      m_methodString = "TxGain";
      break;
    case TxPower:
      m_methodString = "TxPower";
      break;
    case EdThreshold:
      m_methodString = "EnergyDetectionThreshold";
      break;
    case None:
      m_methodString = "None";
      break;
  }

  m_drones = NodeContainer ();
  m_devices = NetDeviceContainer ();

  m_packetLossRate = Gnuplot2dDataset (m_methodString);
  m_packetLossRate.SetStyle (Gnuplot2dDataset::LINES);
  m_dataThroughput = Gnuplot2dDataset (m_methodString);
  m_packetLossRate.SetStyle (Gnuplot2dDataset::LINES);
}

void
DroneExperiment::CreateNodes ()
{
  m_drones.Create (m_numDrones);
}

void
DroneExperiment::InstallWifi ()
{
  // Instantiate and Configure the WifiHelper
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyMode));

  // Instantiate and Configure the PHY layer helper
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();

  // if(!m_dynamicPowerManagement){
  //   double val = 1.0;
  //   switch(m_method){
  //     case Rx:
  //       wifiPhy.Set ("RxGain",DoubleValue (val));
  //       break;
  //     case Tx:
  //       wifiPhy.Set ("TxGain",DoubleValue (val));
  //       break;
  //     case TxPower:
  //       wifiPhy.Set ("TxPowerStart", DoubleValue (val));
  //       wifiPhy.Set ("TxPowerEnd", DoubleValue (val));
  //       break;
  //     case EdThreshold:
  //       wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (val));
  //       break;
  //   }
  // }

  // Instantiate and Configure the WifiChannel helper
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

  // Install the channel in the PHY layer
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;

  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  m_devices = wifi.Install (wifiPhy, wifiMac, m_drones);
}

void
DroneExperiment::InstallMobility ()
{
  MobilityHelper mobility;

  //The PositionAllocator is only used to initialize the MobilityModel; it does not perform the node movement
  mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                 "X", StringValue (UniformRange (m_minX,m_maxX)),
                                 "Y", StringValue (UniformRange (m_minY,m_maxY)));

  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                            "Mode", StringValue ("Time"),
                            "Time", StringValue ("2s"),
                            "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                            "Bounds", StringValue (WalkBounds(m_minX,m_maxX,m_minY,m_maxY)));
  mobility.Install (m_drones);
}

void
DroneExperiment::InstallRouting ()
{
  // Enable OLSR
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  //This setups up OLSR routing for the adhoc network
  //list.Add (RoutingClass, priority)
  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (m_drones);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (m_devices);
}

void
DroneExperiment::InstallApps ()
{
  CollisionAvoidanceHelper collisionHelper;
  collisionHelper.SetAttribute ("PacketFrequency", TimeValue (Seconds (m_collFrequency)));
  collisionHelper.SetAttribute ("PacketSize", UintegerValue (m_collPacketSize));
  m_collisionApps = collisionHelper.Install(m_drones);
}

void
DroneExperiment::ThroughputRxPacketTrace (Ptr<const Packet> packet)
{
  m_packetsReceived += 1;
  m_dataReceived += packet->GetSize ();
}

void
DroneExperiment::ThroughputTxPacketTrace (Ptr<const Packet> packet)
{
  m_packetsSent += 1;
  m_dataSent += packet->GetSize ();
}

void
DroneExperiment::CollisionAvoidanceRxTrace (const Ptr<Packet> &packet, const Ipv4Address &addr)
{
  // NS_LOG_INFO (std::to_string (packet->GetUid()));
  m_packetsReceived += 1;
  m_dataReceived += packet->GetSize ();
  // m_packetLossRate.Add (Simulator::Now(),Percentage (m_dataReceived,m_dataSent));
}

void
DroneExperiment::CollisionAvoidanceTxTrace (const Ptr<Packet> &packet, const Ipv4Address &addr)
{
  // NS_LOG_INFO (std::to_string (packet->GetUid()));
  m_packetsSent += 1;
  m_dataSent += packet->GetSize ();
  // m_throughput.Add (Simulator::Now(),m_dataSent);
}

void
DroneExperiment::NeighborSetChanged (Ptr<Node> node, ns3::olsr::NeighborSet neighbors)
{
  int target = 10;
  int n_index = node->GetId();

  int numNeighbors = neighbors.size ();
  Ptr<WifiNetDevice> wDevice;

  for(uint32_t i = 0; i < node->GetNDevices (); i++)
  {
    Ptr<NetDevice> device = node->GetDevice (i);
    wDevice = DynamicCast<WifiNetDevice> (device);
    if (wDevice != 0)
    {
      break; // found the protocol we are looking for
    }
  }

  if(numNeighbors != target)
  {
    Ptr<WifiPhy> wifiPhy = wDevice->GetPhy ();
    m_phyList[n_index] = wifiPhy;
    double val;
    double offset = (target-numNeighbors)/50.0;

    DoubleValue attrVal;
    if(m_method == TxPower)
    {
      wifiPhy->GetAttribute (m_methodString+"Start",attrVal);
    } else {
      wifiPhy->GetAttribute (m_methodString,attrVal);
    }

    if(m_method == EdThreshold)
    {
      val = attrVal.Get () - offset;
    } else {
      val = attrVal.Get () + offset;
    }

    if(!m_onStateChange)
    {
      if(m_method == TxPower)
      {
        wifiPhy->SetAttribute (m_methodString+"Start", DoubleValue (val));
        wifiPhy->SetAttribute (m_methodString+"End", DoubleValue (val));
      } else {
        wifiPhy->SetAttribute (m_methodString, DoubleValue (val));
      }
    }

    if(val > m_maxStat){m_maxStat = val;}
    if(val < m_minStat){m_minStat = val;}
    m_phyStats[n_index] = val;
  }
}

void
DroneExperiment::StateTransitionCallback (std::string path, Time start, Time duration, WifiPhy::State state)
{
  if(state == YansWifiPhy::IDLE)
  {
    int index = std::stoi (path.substr(10,path.size ()-59));
    Ptr<WifiPhy> wifiPhy = m_phyList[index];
    double val = m_phyStats[index];
    if( wifiPhy != 0)
    {
      if(m_method == TxPower)
      {
        wifiPhy->SetAttribute (m_methodString+"Start", DoubleValue (val));
        wifiPhy->SetAttribute (m_methodString+"End", DoubleValue (val));
      } else {
        wifiPhy->SetAttribute (m_methodString, DoubleValue (val));
      }
    }
  }
}

void
DroneExperiment::ConnectCallbacks ()
{
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::CollisionAvoidanceApp/Rx",MakeCallback(&DroneExperiment::CollisionAvoidanceRxTrace, this));
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::CollisionAvoidanceApp/Tx",MakeCallback(&DroneExperiment::CollisionAvoidanceTxTrace, this));

  if(m_dynamicPowerManagement){
    Config::ConnectWithoutContext("/NodeList/*/$ns3::olsr::RoutingProtocol/NeighborSetChanged",MakeCallback(&DroneExperiment::NeighborSetChanged, this));
    if(m_onStateChange){
      Config::Connect("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/State/State",MakeCallback(&DroneExperiment::StateTransitionCallback, this));
    }
  }
}

std::string
DroneExperiment::UniformRange (uint32_t min, uint32_t max)
{
  return "ns3::UniformRandomVariable[Min=" + std::to_string(min)+ "|Max=" + std::to_string(max) + "]";
}

std::string
DroneExperiment::WalkBounds (uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
{
  return std::to_string(minX) + "|" + std::to_string(maxX) + "|" + std::to_string(minY) + "|" + std::to_string(maxY);
}

double
DroneExperiment::Percentage (int amount, int outOf)
{
  return (amount/((double)outOf))*100.0;
}

void
DroneExperiment::RecordStats ()
{
  if(m_packetsSent != 0){
    int diffSent = m_packetsSent - cp_packetsSent;
    int diffReceived = m_packetsReceived - cp_packetsReceived;
    double packetLossRate = Percentage (diffSent - diffReceived,diffSent);
    m_packetLossRate.Add ((int) Simulator::Now ().GetSeconds (), packetLossRate);
    cp_packetsSent = m_packetsSent;
    cp_packetsReceived = m_packetsReceived;
  }
  if(m_dataReceived != 0){
    m_dataThroughput.Add ((int) Simulator::Now ().GetSeconds (), (m_dataReceived - cp_dataReceived)/2.0);
    cp_dataReceived = m_dataReceived;
  }
  Simulator::Schedule (Seconds (2.0), &DroneExperiment::RecordStats, this);
}

std::vector<Gnuplot2dDataset>
DroneExperiment::Run (PowerManagement method, bool dynamicPowerManagement, bool onStateChange)
{
  m_method = method;
  m_dynamicPowerManagement = dynamicPowerManagement;
  m_onStateChange = onStateChange;

  SetDefaults ();
  CreateNodes ();
  InstallWifi ();
  InstallMobility ();
  InstallRouting ();
  InstallApps ();
  ConnectCallbacks ();

  NS_LOG_INFO ("Performing Simulation for: (" + m_methodString + "," + std::to_string (m_dynamicPowerManagement) + "," + std::to_string (m_onStateChange) + ")");
  m_collisionApps.Start (Seconds (30.0 + 5.0));
  m_collisionApps.Stop (Seconds (30.0 + 120.0));
  Simulator::Schedule (Seconds (36.0), &DroneExperiment::RecordStats, this);
  Simulator::Stop (Seconds (40.0 + 120.0));
  Simulator::Run ();
  Simulator::Destroy ();

  averages += "," + std::to_string (Percentage (m_packetsSent - m_packetsReceived, m_packetsSent));

  NS_LOG_INFO ("Packets Received (via callbacks): " + std::to_string (m_packetsReceived));
  NS_LOG_INFO ("Packet Loss (via callbacks): " + std::to_string (Percentage (m_packetsSent - m_packetsReceived, m_packetsSent)) + "%");
  return {m_packetLossRate, m_dataThroughput};
}

int main (int argc, char *argv[])
{
  LogComponentEnable("DroneAdhoc", LOG_LEVEL_INFO);

  std::string phyMode ("OfdmRate9Mbps");

  uint32_t numDrones = 30;
  uint32_t maxX = 1000; // m
  uint32_t maxY = 1000; // m
  uint32_t collPacketSize = 1; // kilobytes (KB)
  double collFrequency = 0.5; // seconds
  uint32_t imagePacketSize = 5; // megabytes (MB)
  double imageFrequency = 10.0; // seconds
  bool onStateChange = true;

  CommandLine cmd;

  cmd.AddValue ("numDrones", "number of drones in network", numDrones);
  cmd.AddValue ("maxY", "position of wall (top) for drone box (meters)", maxY);
  cmd.AddValue ("maxX", "position of wall (right) for drone box (meters)", maxX);
  cmd.AddValue ("collPacketSize", "collision detection packet size (KB)", collPacketSize);
  cmd.AddValue ("collFrequency", "collision detection packet frequency (seconds)", collFrequency);
  cmd.AddValue ("imagePacketSize", "image packet size (MB)", imagePacketSize);
  cmd.AddValue ("imageFrequency", "image packet frequency (seconds)", imageFrequency);
  cmd.AddValue ("onStateChange", "whether to adjust gains while idle.", onStateChange);

  cmd.Parse (argc, argv);

  std::string dimensions = std::to_string (maxX) + "x" + std::to_string (maxY);
  std::string size = std::to_string (numDrones);
  std::string control = "";
  if(!onStateChange){ control = "not-"; }
  control += "controlled";
  std::string signature = dimensions + "-" + size + "-" + control;
  Gnuplot packetLoss = Gnuplot ("packet-loss-rate-"+signature+".png");
  Gnuplot throughput = Gnuplot ("data-throughput-"+signature+".png");
  std::vector<Gnuplot2dDataset> datasets = {};

  Ptr<DroneExperiment> experiment;

  experiment = CreateObject<DroneExperiment> ();
  experiment->SetAttribute ("NumDrones", UintegerValue (numDrones));
  experiment->SetAttribute ("MaxX", UintegerValue (maxX));
  experiment->SetAttribute ("MaxY", UintegerValue (maxY));
  experiment->SetAttribute ("CollisionPacketSize", UintegerValue (collPacketSize*1024));
  experiment->SetAttribute ("CollisionPacketFrequency", DoubleValue (collFrequency));
  experiment->SetAttribute ("ImagePacketSize", UintegerValue (imagePacketSize*1024*1024));
  experiment->SetAttribute ("ImagePacketFrequency", DoubleValue (imageFrequency));
  experiment->SetAttribute ("PHYmode", StringValue (phyMode));

  averages = std::to_string (numDrones);

  datasets = experiment->Run (Rx, true, onStateChange);
  packetLoss.AddDataset (datasets[0]);
  throughput.AddDataset (datasets[1]);
  datasets = experiment->Run (Tx, true, onStateChange);
  packetLoss.AddDataset (datasets[0]);
  throughput.AddDataset (datasets[1]);
  datasets = experiment->Run (EdThreshold, true, onStateChange);
  packetLoss.AddDataset (datasets[0]);
  throughput.AddDataset (datasets[1]);
  datasets = experiment->Run (None, false, false);
  packetLoss.AddDataset (datasets[0]);
  throughput.AddDataset (datasets[1]);

  std::ofstream runAvgsFile;
  runAvgsFile.open ("drone_avgs.csv", std::ios_base::app);
  runAvgsFile << averages + "\n";
  runAvgsFile.close ();

  packetLoss.GenerateOutput (std::cout);
  throughput.GenerateOutput (std::cout);

  return 0;
}
