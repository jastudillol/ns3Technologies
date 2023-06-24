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

#ifndef PERIODIC_SENDER_HELPER_H
#define PERIODIC_SENDER_HELPER_H

#include "ns3/object-factory.h"
#include "ns3/address.h"
#include "ns3/attribute.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/application-container.h"
#include "ns3/periodic-sender.h"
#include <stdint.h>
#include <string>
#include "ns3/string.h"


namespace ns3 {
namespace lorawan {

/**
 * This class can be used to install PeriodicSender applications on a wide
 * range of nodes.
 */
class PeriodicSenderHelper
{
public:
  PeriodicSenderHelper ();

  ~PeriodicSenderHelper ();

  void SetAttribute (std::string name, const AttributeValue &value);

  ApplicationContainer Install (NodeContainer c) const;

  ApplicationContainer Install (Ptr<Node> node) const;

  /**
   * Set the period to be used by the applications created by this helper.
   *
   * A value of Seconds (0) results in randomly generated periods according to
   * the model contained in the TR 45.820 document.
   *
   * \param period The period to set
   */
  void SetPeriod (Time period);

  void SetPacketSizeRandomVariable (Ptr <RandomVariableStream> rv);

  void SetPacketSize (uint8_t size);

  //begin added by juan
  void SetNodeID (uint32_t nodeID);
  void SetApp (uint32_t app);
  void SetBoolTraces (uint32_t boolTraces);

  void SetSeed (uint32_t seed);
  void SetRun (uint32_t run);
  void SetDataset (std::string dataset);

  void SetLoad (std::string load);
  void SetTraffics (uint32_t traffics);

  void SetPacketFrag (uint32_t packetFrag);
  void SetPacketIntervalDistribution (std::string cadena);

  void SetNetworkSize (uint32_t networkSize);
  
  void SetPosX (double posX);
  
  void SetPosY (double posY);

  //end added by juan

private:
  Ptr<Application> InstallPriv (Ptr<Node> node) const;

  ObjectFactory m_factory;

  Ptr<UniformRandomVariable> m_initialDelay;

  Ptr<UniformRandomVariable> m_intervalProb;

  Time m_period; //!< The period with which the application will be set to send
                 // messages

  Ptr<RandomVariableStream> m_pktSizeRV; // whether or not a random component is added to the packet size

  uint8_t m_pktSize; // the packet size.

  //Added by juan
  uint32_t m_nodeID; //
  uint32_t m_app; //
  uint32_t m_boolTraces; // para habilitar si se van habilitar la impresion de las trazas d
  uint32_t m_seed;
  uint32_t m_run;

  std::string m_load;
  uint32_t m_networkSize;
  uint32_t m_traffics;
  
  double m_posX;
  
  double m_posY;

  uint32_t m_packetFrag;

  std::string m_dataset;

  std::string m_packetIntervalDistribution;

  //Added by juan

};

} // namespace ns3

}
#endif /* PERIODIC_SENDER_HELPER_H */
