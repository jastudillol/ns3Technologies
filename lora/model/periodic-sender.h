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

#ifndef PERIODIC_SENDER_H
#define PERIODIC_SENDER_H

#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/lorawan-mac.h"
#include "ns3/attribute.h"

//ADDED BY JUAN BEGIN
#include "ns3/traced-callback.h"
#include "ns3/boolean.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/packet.h"
//ADDED BY JUAN END


namespace ns3 {
namespace lorawan {

class PeriodicSender : public Application
{
public:
  PeriodicSender ();
  ~PeriodicSender ();

  static TypeId GetTypeId (void);

  /**
   * Set the sending interval
   * \param interval the interval between two packet sendings
   */
  void SetInterval (Time interval);

  /**
   * Get the sending inteval
   * \returns the interval between two packet sends
   */
  Time GetInterval (void) const;

  /**
   * Set the initial delay of this application
   */
  void SetInitialDelay (Time delay);

  /**
   * Set packet size
   */
  void SetPacketSize (uint8_t size);

  /**
   * Set if using randomness in the packet size
   */
  void SetPacketSizeRandomVariable (Ptr <RandomVariableStream> rv);

  /**
   * Send a packet using the LoraNetDevice's Send method
   */
  void SendPacket (void);

  /**
   * Start the application by scheduling the first SendPacket event
   */
  void StartApplication (void);

  /**
   * Stop the application
   */
  void StopApplication (void);


  // BEGIN ADDED JUAN

  //void SetPacketIntervalDistribution (std::string cadena);
  //std::string GetPacketIntervalDistribution () const;
  // END ADDED BY JUAN

  //Begin added by juan

  void SetNodeID (uint32_t nodeID);
  uint32_t GetNodeID (void) const;

  void SetApp (uint32_t nodeID);
  uint32_t GetApp (void) const;

  void SetBoolTraces (uint32_t boolTraces);
  uint32_t GetBoolTraces (void) const;

  void SetSeed (uint32_t seed);
  uint32_t GetSeed (void) const;

  void SetRun (uint32_t run);
  uint32_t GetRun (void) const;

  void SetDataset (std::string dataset);
  std::string GetDataset (void) const;

  void SetLoad(std::string load);
  std::string GetLoad (void) const;

  void SetTraffics (uint32_t traffics);
  uint32_t GetTraffics (void) const;

  void SetPacketFrag (uint32_t packetFrag);
  uint32_t GetPacketFrag (void) const;

  void SetNetworkSize (uint32_t networkSize);
  uint32_t GetNetworkSize (void) const;
  
  void SetPosX (double posX);
  double GetPosX (void) const;

  void SetPosY (double posY);
  double GetPosY (void) const;


  void SetPacketIntervalDistribution (std::string cadena);
  std::string GetPacketIntervalDistribution () const;


  //End added by juan



private:
  /**
   * The interval between to consecutive send events
   */
  Time m_interval;

  /**
   * The initial delay of this application
   */
  Time m_initialDelay;

  /**
   * The sending event scheduled as next
   */
  EventId m_sendEvent;

  /**
   * The MAC layer of this node
   */
  Ptr<LorawanMac> m_mac;

  /**
   * The packet size.
   */
  uint8_t m_basePktSize;


  /**
   * The random variable that adds bytes to the packet size
   */
  //Ptr<RandomVariableStream> m_pktSizeRV;
  Ptr<RandomVariableStream> m_pktSizeRV;




  //ADDED BY LUIS BEGIN

  uint32_t m_nodeID; //Para imprimir los nodos ID
  uint32_t m_app; // Para imprimir la app
  uint32_t m_boolTraces; // para habilitar si se van habilitar la impresion de las trazas d
  uint32_t m_seed;
  uint32_t m_run;
  std::string m_dataset;
  std::string m_load;
  uint32_t m_networkSize;
  uint32_t m_traffics;
  
  double m_posX;
  double m_posY;

  uint32_t m_packetFrag; //packet Fragment
  std::string m_packetIntervalDistribution;

  //ADDED BY LUIS END


};

} //namespace ns3

}
#endif /* SENDER_APPLICATION */
