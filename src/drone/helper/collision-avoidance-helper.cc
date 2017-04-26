
#include "collision-avoidance-helper.h"
#include "ns3/node.h"
#include "ns3/names.h"
#include "ns3/log.h"

namespace ns3 {
  NS_LOG_COMPONENT_DEFINE ("CollisionAvoidanceHelper");

  CollisionAvoidanceHelper::CollisionAvoidanceHelper ()
  {
    m_factory.SetTypeId ("ns3::CollisionAvoidanceApp");
  }

  void
  CollisionAvoidanceHelper::SetAttribute (std::string name, const AttributeValue &value)
  {
    m_factory.Set (name, value);
  }

  ApplicationContainer
  CollisionAvoidanceHelper::Install (NodeContainer c) const
  {
    ApplicationContainer apps;
    for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
      {
        apps.Add (InstallPriv (*i));
      }

    return apps;
  }

  ApplicationContainer
  CollisionAvoidanceHelper::Install (Ptr<Node> node) const
  {
    return ApplicationContainer (InstallPriv (node));
  }

  ApplicationContainer
  CollisionAvoidanceHelper::Install (std::string nodeName) const
  {
    Ptr<Node> node = Names::Find<Node> (nodeName);
    return ApplicationContainer (InstallPriv (node));
  }

  Ptr<Application>
  CollisionAvoidanceHelper::InstallPriv (Ptr<Node> node) const
  {
    NS_LOG_INFO ("installing node");
    Ptr<Application> app = m_factory.Create<Application> ();
    node->AddApplication (app);

    return app;
  }
}
