
#include "collision-avoidance-app.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/olsr-state.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/wifi-module.h"
#include "ns3/seq-ts-header.h"

namespace ns3 {
  NS_LOG_COMPONENT_DEFINE ("CollisionAvoidanceApp");

  NS_OBJECT_ENSURE_REGISTERED (CollisionAvoidanceApp);

  TypeId
  CollisionAvoidanceApp::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CollisionAvoidanceApp")
      .SetParent<Application> ()
      .SetGroupName ("Applications")
      .AddConstructor<CollisionAvoidanceApp> ()
      .AddAttribute ("PacketSize", "The size of the collision packet sent to neighbors.",
                    UintegerValue (1024),
                    MakeUintegerAccessor (&CollisionAvoidanceApp::m_pktSize),
                    MakeUintegerChecker<uint32_t> (1))
      .AddAttribute ("PacketFrequency", "The time interval of sending a single packet.",
                    TimeValue (Seconds (3)),
                    MakeTimeAccessor (&CollisionAvoidanceApp::m_pktFreq),
                    MakeTimeChecker ())
      .AddAttribute ("Protocol", "The type of protocol to use. (e.g. UdpSocketFactory)",
                    TypeIdValue (UdpSocketFactory::GetTypeId ()),
                    MakeTypeIdAccessor (&CollisionAvoidanceApp::m_tid),
                    MakeTypeIdChecker ())
      .AddAttribute ("JITTER", "The UniformRandomVariable used to create jitter when starting.",
                    StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
                    MakePointerAccessor (&CollisionAvoidanceApp::m_jitter),
                    MakePointerChecker<RandomVariableStream> ())
      .AddTraceSource ("Rx", "Received collision avoidance packet.",
                      MakeTraceSourceAccessor (&CollisionAvoidanceApp::m_rxPacketTrace),
                      "ns3::CollisionAvoidanceApp::CollisionPacketRxTracedCallback")
      .AddTraceSource ("Tx", "Sent collision avoidance packet.",
                      MakeTraceSourceAccessor (&CollisionAvoidanceApp::m_txPacketTrace),
                      "ns3::CollisionAvoidanceApp::CollisionPacketTxTracedCallback")
      ;

    return tid;
  }

  CollisionAvoidanceApp::CollisionAvoidanceApp ()
    : m_state (0),
      m_socket (0),
      m_sent (0),
      m_received (0)
  {
    NS_LOG_FUNCTION (this);
    // m_state = GetOlsrState ();
  }

  CollisionAvoidanceApp::~CollisionAvoidanceApp ()
  {
    NS_LOG_FUNCTION (this);
  }

  void
  CollisionAvoidanceApp::DoDispose (void)
  {
    NS_LOG_FUNCTION (this);

    // Do any cleaning up here.

    Application::DoDispose ();
  }

  void
  CollisionAvoidanceApp::StartApplication ()
  {
    // Specific start up code goes here.
    if(!m_socket)
    {
      m_socket = Socket::CreateSocket (GetNode (), m_tid);
      Ipv4Address ipAddr = GetNode ()->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
      m_socket->Bind (InetSocketAddress (ipAddr, 89));
      // m_socket->ShutdownRecv ();
      m_socket->SetRecvCallback (MakeCallback (&CollisionAvoidanceApp::RecvPacket, this));
    }

    if(m_state == 0)
    {
      m_state = GetOlsrState();
    }

    CancelEvents ();

    m_sendEvent = Simulator::Schedule (m_pktFreq+Seconds (m_jitter->GetValue ()), &CollisionAvoidanceApp::SendPackets, this);
  }

  void
  CollisionAvoidanceApp::StopApplication ()
  {
    // Specific stopping code goes here.
    CancelEvents ();
    if(m_socket != 0)
    {
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_socket->Close ();
    }
    if(m_state != 0)
    {
      m_state = 0;
    }
  }

  void
  CollisionAvoidanceApp::SendPackets ()
  {
    const olsr::NeighborSet neighbors = GetNeighbors ();

    NS_LOG_INFO ("Number of neighbors: " + std::to_string (neighbors.size ()));
    for( auto neighborIter = neighbors.begin (); neighborIter < neighbors.end (); neighborIter++)
    {
      Ipv4Address neighborAddress = neighborIter->neighborMainAddr;

      Ptr<Packet> packet = Create<Packet> (m_pktSize);

      if(m_socket->SendTo (packet, 0, InetSocketAddress (neighborAddress, 89)) >= 0)
      {
        m_sent++;
        m_txPacketTrace (packet, neighborAddress);
      }
    }

    ScheduleNextTx ();
    // Logic for sending packets to neighbors
  }

  void
  CollisionAvoidanceApp::RecvPacket (Ptr<Socket> socket)
  {
    Ptr<Packet> packet;
    Address addr;
    while((packet = socket->RecvFrom (addr)))
    {
      Ipv4Address neighborAddress = InetSocketAddress::ConvertFrom (addr).GetIpv4 ();

      m_received++;
      m_rxPacketTrace (packet, neighborAddress);
    }
  }

  void
  CollisionAvoidanceApp::ScheduleNextTx ()
  {
    // Schedule next SendPackets
    m_sendEvent = Simulator::Schedule (m_pktFreq, &CollisionAvoidanceApp::SendPackets, this);
  }

  void
  CollisionAvoidanceApp::CancelEvents ()
  {
    // Cancel any pending events
    Simulator::Cancel (m_sendEvent);
  }

  olsr::NeighborSet
  CollisionAvoidanceApp::GetNeighbors ()
  {
    return m_state->GetNeighbors ();
  }

  olsr::OlsrState*
  CollisionAvoidanceApp::GetOlsrState ()
  {
    Ptr<Ipv4RoutingProtocol> ipv4Routing = GetNode ()->GetObject<Ipv4> ()->GetRoutingProtocol ();
    Ptr<Ipv4ListRouting> ipv4ListRouting = DynamicCast<Ipv4ListRouting>(ipv4Routing);
    Ptr<olsr::RoutingProtocol> olsrProtocol;
    int16_t priority;
    for(uint32_t i = 0; i < ipv4ListRouting->GetNRoutingProtocols (); i++)
    {
      Ptr<Ipv4RoutingProtocol> proto = ipv4ListRouting->GetRoutingProtocol (i, priority);
      olsrProtocol = DynamicCast<olsr::RoutingProtocol> (proto);
      if (olsrProtocol != 0)
      {
        break; // found the protocol we are looking for
      }
    }
    return &(olsrProtocol->m_state);
  }
}
