/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "ns3/periodic-sender-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/simulator.h"
#include "ns3/log.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("PeriodicSenderHelper");

PeriodicSenderHelper::PeriodicSenderHelper ()
{
  m_factory.SetTypeId ("ns3::PeriodicSender");

  // m_factory.Set ("PacketSizeRandomVariable", StringValue
  //                  ("ns3::ParetoRandomVariable[Bound=10|Shape=2.5]"));

  m_initialDelay = CreateObject<UniformRandomVariable> ();
  m_initialDelay->SetAttribute ("Min", DoubleValue (0));

  m_intervalProb = CreateObject<UniformRandomVariable> ();
  m_intervalProb->SetAttribute ("Min", DoubleValue (0));
  m_intervalProb->SetAttribute ("Max", DoubleValue (1));

  m_pktSize = 10;
  m_pktSizeRV = 0;

  m_nodeID = 0;
}

PeriodicSenderHelper::~PeriodicSenderHelper ()
{
}

void
PeriodicSenderHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
PeriodicSenderHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
PeriodicSenderHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
PeriodicSenderHelper::InstallPriv (Ptr<Node> node) const
{
  NS_LOG_FUNCTION (this << node);

  Ptr<PeriodicSender> app = m_factory.Create<PeriodicSender> ();

  Time interval;
  if (m_period == Seconds (0))
    {
      double intervalProb = m_intervalProb->GetValue ();
      NS_LOG_DEBUG ("IntervalProb = " << intervalProb);

      // Based on TR 45.820
      if (intervalProb < 0.4)
        {
          interval = Days (1);
        }
      else if (0.4 <= intervalProb  && intervalProb < 0.8)
        {
          interval = Hours (2);
        }
      else if (0.8 <= intervalProb  && intervalProb < 0.95)
        {
          interval = Hours (1);
        }
      else
        {
          interval = Minutes (30);
        }
    }
  else
    {
      interval = m_period;
    }

  app->SetInterval (interval);
  NS_LOG_DEBUG ("Created an application with interval = " <<
                interval.GetHours () << " hours");

  app->SetInitialDelay (Seconds (m_initialDelay->GetValue (0, interval.GetSeconds ())));
  app->SetPacketSize (m_pktSize);
  if (m_pktSizeRV)
    {
      app->SetPacketSizeRandomVariable (m_pktSizeRV);
    }

  app->SetNode (node);

  app->SetNodeID(m_nodeID);   //added by juan
  app->SetApp(m_app);   //added by juan
  app->SetBoolTraces(m_boolTraces);   //added by juan

  app->SetSeed(m_seed);   //added by juan
  app->SetRun(m_run);   //added by juan

  app->SetPacketFrag(m_packetFrag);   //added by juan

  app->SetDataset(m_dataset);   //added by juan

  app->SetTraffics(m_traffics);   //added by juan

  app->SetLoad(m_load);   //added by juan

  app->SetNetworkSize(m_networkSize);   //added by juan
  
  app->SetPosX(m_posX);   //added by juan
  
  app->SetPosY(m_posY);   //added by juan

  app->SetPacketIntervalDistribution(m_packetIntervalDistribution);   //added by juan

  node->AddApplication (app);





  return app;
}

void
PeriodicSenderHelper::SetPeriod (Time period)
{
  m_period = period;
}

void
PeriodicSenderHelper::SetPacketSizeRandomVariable (Ptr <RandomVariableStream> rv)
{
  m_pktSizeRV = rv;
}

void
PeriodicSenderHelper::SetPacketSize (uint8_t size)
{
  m_pktSize = size;
}

//added by juan

void
PeriodicSenderHelper::SetNodeID(uint32_t nodeID)
{
  m_nodeID = nodeID;
}
void
PeriodicSenderHelper::SetApp(uint32_t app)
{
  m_app = app;
}
void
PeriodicSenderHelper::SetBoolTraces(uint32_t boolTraces)
{
  m_boolTraces = boolTraces;
}

void
PeriodicSenderHelper::SetSeed(uint32_t seed)
{
  m_seed = seed;
}
void
PeriodicSenderHelper::SetRun(uint32_t run)
{
  m_run = run;
}

void
PeriodicSenderHelper::SetNetworkSize(uint32_t networkSize)
{
  m_networkSize = networkSize;
}

void
PeriodicSenderHelper::SetPosX(double posX)
{
  m_posX = posX;
}

void
PeriodicSenderHelper::SetPosY(double posY)
{
  m_posY = posY;
}

void
PeriodicSenderHelper::SetLoad(std::string load)
{
  m_load = load;
}
void
PeriodicSenderHelper::SetTraffics(uint32_t traffics)
{
  m_traffics = traffics;
}



void
PeriodicSenderHelper::SetDataset(std::string dataset)
{
  m_dataset = dataset;
}
void
PeriodicSenderHelper::SetPacketFrag(uint32_t packetFrag)
{
  m_packetFrag = packetFrag;
}

void
PeriodicSenderHelper::SetPacketIntervalDistribution (std::string cadena)
{
   NS_LOG_FUNCTION (this << cadena);
   m_packetIntervalDistribution = cadena;
}

//added by juan


}
} // namespace ns3
