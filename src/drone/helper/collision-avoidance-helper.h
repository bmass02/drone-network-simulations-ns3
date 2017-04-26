
#ifndef COLLISION_AVOIDANCE_HELPER_H
#define COLLISION_AVOIDANCE_HELPER_H

#include "ns3/object-factory.h"
#include "ns3/collision-avoidance-app.h"
#include "ns3/application-container.h"
#include "ns3/attribute.h"
#include "ns3/node-container.h"

namespace ns3 {

  class CollisionAvoidanceHelper
  {
  public:
    CollisionAvoidanceHelper ();

    void SetAttribute (std::string name, const AttributeValue &value);

    ApplicationContainer Install (NodeContainer c) const;

    ApplicationContainer Install (Ptr<Node> node) const;

    ApplicationContainer Install (std::string nodeName) const;

  private:
    Ptr<Application> InstallPriv (Ptr<Node> node) const;

    ObjectFactory m_factory;
  };
}

#endif /* COLLISION_AVOIDANCE_HELPER_H */
