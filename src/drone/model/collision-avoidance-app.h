

#ifndef COLLISION_AVOIDANCE_APP_H
#define COLLISION_AVOIDANCE_APP_H

#include "ns3/application.h"
#include "ns3/olsr-state.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/event-id.h"
#include "ns3/packet-loss-counter.h"

#include <unordered_map>

namespace ns3 {
  class Socket;

  class CollisionAvoidanceApp : public Application
  {
  public:
    static TypeId GetTypeId (void);

    CollisionAvoidanceApp ();

    virtual ~CollisionAvoidanceApp ();

    typedef void (* CollisionPacketTxTracedCallback)(const Ptr<Packet> & packet, const Ipv4Address & dest);
    typedef void (* CollisionPacketRxTracedCallback)(const Ptr<Packet> & packet, const Ipv4Address & dest);
  protected:
    virtual void DoDispose (void);
  private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void SendPackets (void);
    void ScheduleNextTx (void);
    void CancelEvents (void);
    void RecvPacket (Ptr<Socket> socket);

    olsr::NeighborSet GetNeighbors (void);
    olsr::OlsrState* GetOlsrState (void);

    uint32_t          m_pktSize;
    Time              m_pktFreq;
    olsr::OlsrState*  m_state;
    EventId           m_sendEvent;
    TypeId            m_tid;
    Ptr<Socket>       m_socket;
    Ptr<RandomVariableStream>             m_jitter;
    uint32_t  m_sent;
    uint32_t  m_received;

    TracedCallback <const Ptr<Packet> &, const Ipv4Address &> m_rxPacketTrace;
    TracedCallback <const Ptr<Packet> &, const Ipv4Address &> m_txPacketTrace;
  };
}

#endif /* COLLISION_AVOIDANCE_APP_H */
