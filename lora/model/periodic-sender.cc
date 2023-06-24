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

#include "ns3/periodic-sender.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/string.h"
#include "ns3/lora-net-device.h"
#include <fstream>

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("PeriodicSender");

NS_OBJECT_ENSURE_REGISTERED (PeriodicSender);

TypeId
PeriodicSender::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PeriodicSender")
    .SetParent<Application> ()
    .AddConstructor<PeriodicSender> ()
    .SetGroupName ("lorawan")
    .AddAttribute ("Interval", "The interval between packet sends of this app",
                   TimeValue (Seconds (0)),
                   MakeTimeAccessor (&PeriodicSender::GetInterval,
                                     &PeriodicSender::SetInterval),
                   MakeTimeChecker ())

   .AddAttribute ("nodeID", "nodeID",
				   UintegerValue  (0),
				   MakeUintegerAccessor (&PeriodicSender::SetNodeID,
										&PeriodicSender::GetNodeID),
				MakeUintegerChecker<uint32_t> ())

   .AddAttribute ("app", "app",
				   UintegerValue  (0),
				   MakeUintegerAccessor (&PeriodicSender::SetApp,
										&PeriodicSender::GetApp),
				MakeUintegerChecker<uint32_t> ())

   .AddAttribute ("boolTraces", "boolTraces",
				   UintegerValue  (0),
				   MakeUintegerAccessor (&PeriodicSender::SetBoolTraces,
										&PeriodicSender::GetBoolTraces),
				MakeUintegerChecker<uint32_t> ())

   .AddAttribute ("seed", "seed",
				   UintegerValue  (1),
				   MakeUintegerAccessor (&PeriodicSender::SetSeed,
										&PeriodicSender::GetSeed),
				MakeUintegerChecker<uint32_t> ())

	.AddAttribute ("packetFrag", "packetFrag",
					   UintegerValue  (1),
					   MakeUintegerAccessor (&PeriodicSender::SetPacketFrag,
											&PeriodicSender::GetPacketFrag),
					MakeUintegerChecker<uint32_t> ())

   .AddAttribute ("run", "run",
				   UintegerValue  (1),
				   MakeUintegerAccessor (&PeriodicSender::SetRun,
										&PeriodicSender::GetRun),
				MakeUintegerChecker<uint32_t> ())

   .AddAttribute ("dataset", "dataset",
				   StringValue  ("dataset"),
				   MakeStringAccessor (&PeriodicSender::SetDataset,
										&PeriodicSender::GetDataset),
					MakeStringChecker ())

   .AddAttribute ("load", "load",
				   StringValue  ("load"),
				   MakeStringAccessor (&PeriodicSender::SetLoad,
										&PeriodicSender::GetLoad),
					MakeStringChecker ())

  .AddAttribute ("networkSize", "networkSize",
				   UintegerValue  (1),
				   MakeUintegerAccessor (&PeriodicSender::SetNetworkSize,
										&PeriodicSender::GetNetworkSize),
				MakeUintegerChecker<uint32_t> ())

  .AddAttribute ("traffics", "traffics",
				   UintegerValue  (1),
				   MakeUintegerAccessor (&PeriodicSender::SetTraffics,
										&PeriodicSender::GetTraffics),
				MakeUintegerChecker<uint32_t> ())
        
    .AddAttribute ("posX", "posX",
				   DoubleValue  (1),
				   MakeDoubleAccessor (&PeriodicSender::SetPosX,
										&PeriodicSender::GetPosX),
				MakeDoubleChecker<double> ())
        
    .AddAttribute ("posY", "posY",
				   DoubleValue  (1),
				   MakeDoubleAccessor (&PeriodicSender::SetPosY,
										&PeriodicSender::GetPosY),
				MakeDoubleChecker<double> ())
        
    

	.AddAttribute ("PacketIntervalDistribution",
				   "The distribution of the packet interval Generated.",
				   StringValue ("Fixed"),
				   MakeStringAccessor (&PeriodicSender::SetPacketIntervalDistribution,
										&PeriodicSender::GetPacketIntervalDistribution),
				   MakeStringChecker ())

				   ;



  return tid;
}

PeriodicSender::PeriodicSender ()
  : m_interval (Seconds (10)),
  m_initialDelay (Seconds (1)),
	//m_initialDelay (Seconds (100)),

  m_basePktSize (10),
  m_pktSizeRV (0)

{
  NS_LOG_FUNCTION_NOARGS ();
}

PeriodicSender::~PeriodicSender ()
{
  NS_LOG_FUNCTION_NOARGS ();
}


void
PeriodicSender::SetInterval (Time interval)
{
  NS_LOG_FUNCTION (this << interval);
  m_interval = interval;
}

Time
PeriodicSender::GetInterval (void) const
{
  NS_LOG_FUNCTION (this);
  return m_interval;
}


//=====BEGIN ADDED BY JUAN
void
PeriodicSender::SetNodeID (uint32_t nodeID)
{
	m_nodeID = nodeID;
}
uint32_t
PeriodicSender::GetNodeID (void) const
{
  return m_nodeID;
}
void
PeriodicSender::SetApp (uint32_t app)
{
	m_app = app;
}
uint32_t
PeriodicSender::GetApp (void) const
{
  return m_app;
}
void
PeriodicSender::SetBoolTraces (uint32_t boolTraces)
{
	m_boolTraces = boolTraces;
}
uint32_t
PeriodicSender::GetBoolTraces (void) const
{
  return m_boolTraces;
}
void
PeriodicSender::SetSeed(uint32_t seed)
{
  m_seed = seed;
}
uint32_t
PeriodicSender::GetSeed (void) const
{
  return m_seed;
}
void
PeriodicSender::SetRun(uint32_t run)
{
  m_run = run;
}
uint32_t
PeriodicSender::GetRun (void) const
{
  return m_run;
}
void
PeriodicSender::SetDataset(std::string dataset)
{
  m_dataset = dataset;
}
std::string
PeriodicSender::GetDataset (void) const
{
  return m_dataset;
}

void
PeriodicSender::SetLoad(std::string load)
{
  m_load = load;
}
std::string
PeriodicSender::GetLoad (void) const
{
  return m_load;
}

void
PeriodicSender::SetTraffics (uint32_t traffics)
{
	m_traffics = traffics;
}
uint32_t
PeriodicSender::GetTraffics (void) const
{
  return m_traffics;
}

void
PeriodicSender::SetPacketFrag (uint32_t packetFrag)
{
	m_packetFrag = packetFrag;
}
uint32_t
PeriodicSender::GetPacketFrag (void) const
{
  return m_packetFrag;
}

void
PeriodicSender::SetNetworkSize (uint32_t networkSize)
{
	m_networkSize = networkSize;
}
uint32_t
PeriodicSender::GetNetworkSize (void) const
{
  return m_networkSize;
}

void
PeriodicSender::SetPosX (double posX)
{
	m_posX = posX;
}
double
PeriodicSender::GetPosX (void) const
{
  return m_posX;
}

void
PeriodicSender::SetPosY (double posY)
{
	m_posY = posY;
}
double
PeriodicSender::GetPosY (void) const
{
  return m_posY;
}

void
PeriodicSender::SetPacketIntervalDistribution (std::string cadena)
{
   NS_LOG_FUNCTION (this << cadena);
   m_packetIntervalDistribution = cadena;
}

std::string
PeriodicSender::GetPacketIntervalDistribution  () const
{
   return m_packetIntervalDistribution;
}


//=====END ADDED BY JUAN


void
PeriodicSender::SetInitialDelay (Time delay)
{
  NS_LOG_FUNCTION (this << delay);
  m_initialDelay = delay;
}


void
PeriodicSender::SetPacketSizeRandomVariable (Ptr <RandomVariableStream> rv)
{
  m_pktSizeRV = rv;
}


void
PeriodicSender::SetPacketSize (uint8_t size)
{
  m_basePktSize = size;
}

/*
void
PeriodicSender::SendPacket (void)
{
  NS_LOG_FUNCTION (this);

  // Create and send a new packet
  Ptr<Packet> packet;
  if (m_pktSizeRV)
    {
      int randomsize = m_pktSizeRV->GetInteger ();
      packet = Create<Packet> (m_basePktSize + randomsize);
    }
  else
    {
      packet = Create<Packet> (m_basePktSize);
    }
  m_mac->Send (packet);

  // Schedule the next SendPacket event
  m_sendEvent = Simulator::Schedule (m_interval, &PeriodicSender::SendPacket,
                                     this);

  NS_LOG_DEBUG ("Sent a packet of size " << packet->GetSize ());
}
*/

void
PeriodicSender::SendPacket (void)
{
  NS_LOG_FUNCTION (this);

  //===BEGIN ADDED BY JUAN

  double new_interval;
  new_interval=m_interval.GetSeconds();  // Comentar para hacer el trafico VBR

  // Create and send a new packet
  Ptr<Packet> packet;
  packet = Create<Packet> (m_basePktSize);

/*
	// WE create a message fill with 0ss
	std::string relleno1 = "";
	for (uint32_t iteratorRell = 0; iteratorRell < 2; ++iteratorRell)
	{
		relleno1 = relleno1 +  "0";
	}

	//Auxiliar packet to generate the common id identifier for different applications
	std::ostringstream msg;
	msg << 	"1," << relleno1 << '\0';

	//paquete auxiliar, is not sent

	Ptr<Packet> p = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length());
	std::ostringstream msg1;
	msg1 	<< 	p->GetUid() <<"," //=====> 	COMMON ID IDENTIFIER FOR DIFFERENT APPS
		    << relleno1
		    << '\0';

	//Paquete q se va enviar

	Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg1.str().c_str(), msg1.str().length());

*/
  m_mac->Send (packet);

  // Schedule the next SendPacket event

	if (GetPacketIntervalDistribution()=="Exponential"){
		Ptr<ExponentialRandomVariable> _interval = CreateObject<ExponentialRandomVariable> ();
		new_interval = _interval->GetValue(m_interval.GetSeconds(),0);
	}


  m_sendEvent = Simulator::Schedule (Seconds(new_interval), &PeriodicSender::SendPacket,
                                     this);
  //m_sentNewPacketApp(packet);

  if (GetBoolTraces()==1){

	  std::ofstream myfile;

		std::ostringstream auxFile; auxFile.str("");
		//auxFile << "/home/juan/Dropbox/UNB/LORA/ns-3/traces/"
		auxFile << "./tracesCOMNET/"
				<< GetDataset()
				//<<"_pktSize_" << uint32_t (pktSizeApp1)
				<<"_App_numSeeds_"<<GetSeed()
				<<"_numRun_"<<GetRun()
				<<"_networkSize_"<<GetNetworkSize()
				<<".txt";

	  myfile.open(auxFile.str().c_str(), std::ios::app);


	  //std::cout << packet->GetSize() << "\n";

	  myfile   << Simulator::Now ().GetNanoSeconds() << ","
    	  	  	  << GetNodeID()	<< ","
				  << "packetCreated,"
				  << packet->GetUid() << ","
				  << packet->GetSize() << ","
				  << GetSeed () << ","
				  << GetRun () << ","
				  << GetApp() << ","
				  << GetNetworkSize() << ","
				  << "LORA" << ","
				  << GetTraffics() << ","
				  << GetLoad() << ","

          << GetPosX() << ","
          << GetPosY()
				  << "\n";
  }

  NS_LOG_DEBUG ("Sent a packet of size " << packet->GetSize ());

}

void
PeriodicSender::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  // Make sure we have a MAC layer
  if (m_mac == 0)
    {
      // Assumes there's only one device
      Ptr<LoraNetDevice> loraNetDevice = m_node->GetDevice (0)->GetObject<LoraNetDevice> ();

      m_mac = loraNetDevice->GetMac ();
      NS_ASSERT (m_mac != 0);
    }

  // Schedule the next SendPacket event
  Simulator::Cancel (m_sendEvent);
  NS_LOG_DEBUG ("Starting up application with a first event with a " <<
                m_initialDelay.GetSeconds () << " seconds delay");
  m_sendEvent = Simulator::Schedule (m_initialDelay,
                                     &PeriodicSender::SendPacket, this);
  NS_LOG_DEBUG ("Event Id: " << m_sendEvent.GetUid ());
}

void
PeriodicSender::StopApplication (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Simulator::Cancel (m_sendEvent);
}

}
}
