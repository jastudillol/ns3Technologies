/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 MIRKO BANCHI
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
 * Authors: Mirko Banchi <mk.banchi@gmail.com>
 *          Sebastien Deronne <sebastien.deronne@gmail.com>
 *
 *          Topologia de 4 nodos, para ver dos caminos y toda la tabla con todos los caminos desde el nodo origen al destino
 *
 *                0
 *         O			O
 *         		  O
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mesh-helper.h"
#include <math.h>

#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
//#include "ns3/qos-tag.h"
//#include "ns3/packet.h"
//#include <qos-tag.h>


//#include "qos-tag.h"
//#include "ns3/tag.h"


using namespace ns3;

//============GLOBAL VARIABLES

int numSeeds=3;
int numRun=71;
int numTest=2196;
double txRate = 6;

uint32_t networkSize = 0;

//=====Simulation PARAMETERS
double iniConvergeTime = 1; //seconds
double midConvergeTime = 50; //seconds
double stopConvergeTime = 100; //seconds

int numberTraffics = 5 ;
std::string load = "L1";

//=====Parameters of the simulation time

double iniSimulationTime  = 200; //Seconds
double simulationTime = 250; //seconds
uint32_t numNodos = 1164; // numero de nodos en la simulacion
double m_txpower = 42;
int distance=650; // distance between nodes
uint32_t numberOfInterfaces = 1; //number of of interfaces  DONT TOUCH
double routingTableExpired = 20;
std::string m_root = "00:00:00:00:0a:c5";

//+++++Physical parameters
double RxSensitivity = -101.0;
double CcaEdThreshold = -62.0;
double TxGain = 0;
double RxGain = 0;

std::string phyLayer = "802.11a";

//========Trasmission and reception power
double m_txpowerMETERS;
double m_txpowerROUTERS;
double m_txpowerCOLLECTORS;

//======Mesh Wifi Interface MAC parameters
double beaconInterval = 1000;
double RandomStart = 0.5;
bool BeaconGeneration = true;

//======Peer link parameters
uint32_t MaxBeaconLoss = 100;
uint32_t MaxRetries = 4;
uint32_t MaxPacketFailure = 5;
double RetryTimeout = 40;  //40
double ConfirmTimeout = 40;
double HoldingTimeout = 40;

//======PMP parameters
uint32_t MaxNumberOfPeerLinks = 20;
uint32_t MaxBeaconShiftValue = 15 ;
bool EnableBeaconCollisionAvoidance = false;

//======HWMP parameters
double Dot11MeshHWMPactivePathTimeout = 18000;
double Dot11MeshHWMPactiveRootTimeout = 18000;
double Dot11MeshHWMPpathToRootInterval	= 18000;
double Dot11MeshHWMPperrMinInterval	= 400;
uint32_t MaxTtl	= 40;
uint32_t Dot11MeshHWMPmaxPREQretries =	3;
uint32_t UnicastPreqThreshold = 1;
uint32_t UnicastPerrThreshold = 32;
uint32_t UnicastDataThreshold =	1;
bool DoFlag	= false;
bool RfFlag	= true;

//===QoS supported
bool m_QoS = false;


//====Patch Traffic parameters
bool m_enablePatch = true;
double m_packetIntervalMeanPatch=500; //0.1;
uint32_t m_packetSizeMeanPatch=50;


//====Electric Vehicle Traffic parameters
bool m_enableEV = false;
double m_packetIntervalMeanEV=0.1; //0.1;
uint32_t m_packetSizeMeanEV=40;
double m_percentageMetersEV = 10; // {0,100}
std::string m_edcaEV = "AC_BE";

//====Meter Reading Traffic parameters
bool m_enableMR = false;
double m_packetIntervalMeanMR=15;
uint32_t m_packetSizeMeanMR=480; //480
double m_percentageMetersMR = 100; // {0,100}
std::string m_edcaMR = "AC_BE";

//====Power Quality Traffic parameters
bool m_enablePQ = false;
double m_packetIntervalMeanPQ=0.5; //0.5
uint32_t m_packetSizeMeanPQ=512;
double m_percentageMetersPQ = 20; // {0,100}
std::string m_edcaPQ = "AC_BE";

//====Demand Response Traffic parameters
bool m_enableDR = false;
double m_packetIntervalMeanDR = 1; //SE ENVIA UNO EN EL TIEMPO DE SIMULACION
double m_percentageMetersDR = 20; // {0,100}
uint32_t m_packetSizeMeanDR=60;
std::string m_edcaDR = "AC_BE";

//====Billing Data Traffic parameters
bool m_enableBD = false;
double m_packetIntervalMeanBD = 3600; //0.1;
uint32_t m_packetSizeMeanBD = 40;
double m_percentageMetersBD = 100; // {0,100}
std::string m_edcaBD = "AC_BE";

//====Alarm Data Traffic parameters
bool m_enableALARM = false;
double m_packetIntervalMeanALARM = 3600; //SE ENVIA UNO EN EL TIEMPO DE SIMULACION
double m_percentageMetersALARM = 20; // {0,100}
uint32_t m_packetSizeMeanALARM=40;
std::string m_edcaALARM = "AC_BE";

std::string deviceINFORMATION_file = "./meterPositions/POSITIONS2.csv";

std::map<uint32_t,double>  m_NodeIDtoX;//
std::map<uint32_t,double>  m_NodeIDtoY;//



CommandLine cmd;

NodeContainer nodes;
NetDeviceContainer meshDevices;
Ipv4InterfaceContainer interfaces;

std::string dataset = "MEETING_July5";

bool DEBUG = false;

NS_LOG_COMPONENT_DEFINE ("mesh-Smart-Grid-UNB");
std::ofstream mcsResultsFile; // Para guardar los resultados de throughput
std::ofstream cbrFile; // Para guardar los resultados de CBR
std::ofstream debugFile; // Para guardar los resultados de throughput
FlowMonitorHelper flowmon;
Ptr<FlowMonitor> monitor;

std::map<uint32_t,double>  m_rho;// Almacenar el valor de rho por interfaz
double m_weight = 0.005;
double measureMean=0.001;
EventId sendEvent;

void installFlowMonitor(){
	//FLow monitor
	NS_LOG_INFO ("Create Flow Monitor");
	// Install FlowMonitor on all nodes
	monitor= flowmon.InstallAll();
}

void CBRSample ( uint32_t node,
	double _simulationTime)
{

	// access i-th mesh point, production code should add NS_ASSERT (mp != 0);
	Ptr<MeshPointDevice> m_mp1 = DynamicCast<MeshPointDevice> (meshDevices.Get (node));
	std::vector<Ptr<NetDevice> > interfaces1 = m_mp1->GetInterfaces();

	for (std::vector<Ptr<NetDevice> >::const_iterator i1 = interfaces1.begin();
	i1 != interfaces1.end(); i1++)
	{
		// Checking for compatible net device
		Ptr<WifiNetDevice> wifiNetDev1 = (*i1)->GetObject<WifiNetDevice>();

		int m_currentRho;
		if (wifiNetDev1->GetMac()->GetWifiPhy()->IsStateIdle()) {
			m_currentRho = 0;
		} else {
			m_currentRho = 1;
		}

		m_rho[node] = m_weight * m_currentRho + (1 - m_weight) * m_rho[node];


	}

	//std::cout << node << "\t" << m_rho[node] <<"\n";

	Ptr<ExponentialRandomVariable> _interval = CreateObject<ExponentialRandomVariable> ();
	double newMeasure = _interval->GetValue(measureMean,0); // The new measure has to be done without memory in order to avoid syncronization

	if (Simulator::Now ().GetSeconds()<_simulationTime){ // Metodo recursivo
		sendEvent=Simulator::Schedule(Seconds(newMeasure), &CBRSample , node, _simulationTime) ;
	}
	else{
		Simulator::Cancel (sendEvent);
	}



}

void printCBRSample ( uint32_t node,
	double _simulationTime)
{


	cbrFile << Simulator::Now ().GetSeconds()
			//<< "\t" << context
			<< "\t" << node
			<<"\t" << m_rho[node]
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << networkSize

			//<< "\t" << oss.str()
			<< "\n";


	double newMeasure = 1;
	if (Simulator::Now ().GetSeconds()<_simulationTime){ // Metodo recursivo
		sendEvent=Simulator::Schedule(Seconds(newMeasure), &printCBRSample , node, _simulationTime) ;
	}
	else{
		Simulator::Cancel (sendEvent);
	}



}



void getMetricsFlowMonitor()
{
	//Flow monitor
	// Define variables to calculate the metrics
	int k=0;
	int totaltxPackets = 0;
	int totalrxPackets = 0;
	double totaltxbytes = 0;
	double totalrxbytes = 0;
	double totaldelay = 0;
	double totalHopCount = 0;
	double totalrxbitrate = 0;
	double totaltxbitrate = 0;
	double difftx, diffrx;
	double pdf_value, rxbitrate_value, txbitrate_value, delay_value, hc_value;
	double pdf_total, rxbitrate_total, txbitrate_total, delay_total, hc_total;

	//Print per flow statistics

	monitor->CheckForLostPackets ();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier ());
	std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

	int numFlows = 0 ;

	std::ofstream myResults;

	myResults.open("./results/"+dataset+"results"+"_"+std::to_string(numSeeds)+"_"+std::to_string(numRun)+".csv");

	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin ();
		  i != stats.end (); ++i)
	{

	  //std::cout << i->second.timeFirstTxPacket.GetSeconds() << "\n";

		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
		if (t.destinationPort<6999) //Para no tener en cuenta los paquetes de patch traffic
		{
			difftx = i->second.timeLastTxPacket.GetSeconds() -i->second.timeFirstTxPacket.GetSeconds();
			diffrx = i->second.timeLastRxPacket.GetSeconds() -i->second.timeFirstRxPacket.GetSeconds();
			pdf_value = (double) i->second.rxPackets / (double) i->second.txPackets * 100;
			txbitrate_value = (double) i->second.txBytes * 8 / 1024 / difftx;
			rxbitrate_value = (double) i->second.rxBytes * 8 / 1024 / diffrx;
			if (i->second.rxPackets != 0){
			//rxbitrate_value = (double)i->second.rxPackets * m_packetSizeMeanMR * 8 /1024 / diffrx;
				rxbitrate_value = (double)i->second.rxBytes * 8 /1024 / diffrx;
				txbitrate_value = (double)i->second.txBytes * 8 /1024 / difftx;
				delay_value = (double) i->second.delaySum.GetSeconds() /(double) i->second.rxPackets;
				hc_value = (double) i->second.timesForwarded /(double) i->second.rxPackets;
				hc_value = hc_value+1;
			}
			else{
				rxbitrate_value = 0;
				txbitrate_value = 0;
				delay_value = 0;
				hc_value = -1000;
			}

			// We are only interested in the metrics of the data flows
			if ((!t.destinationAddress.IsSubnetDirectedBroadcast("255.255.255.0")))
			{
				k++;// Plot the statistics for each data flow
				std::cout << "\nFlow " << k << " (" << t.sourceAddress << " -> "<< t.destinationAddress << ")\n";
				std::cout << "Application port: " << (uint16_t) t.destinationPort << "\n";
				std::cout << "Tx Packets: " << i->second.txPackets << "\n";
				std::cout << "Rx Packets: " << i->second.rxPackets << "\n";
				std::cout << "Tx Bytes: " << i->second.txBytes << "\n";
				std::cout << "Rx Bytes: " << i->second.rxBytes << "\n";
				std::cout << "Lost Packets: " << i->second.lostPackets << "\n";
				std::cout << "Dropped Packets: " << i->second.packetsDropped.size() << "\n";
				std::cout << "PDR:  " << pdf_value << " %\n";
				std::cout << "Average delay: " << delay_value << "s\n";
				std::cout << "Hop count: " << hc_value << "\n";
				std::cout << "Tx bitrate: " << txbitrate_value << " kbps\n";
				std::cout << "Rx bitrate: " <<  rxbitrate_value << " kbps\n\n";


				myResults << k << ","
						<< t.sourceAddress << ","
						<< t.destinationAddress << ","
						<< (uint16_t) t.destinationPort << ","
						<< i->second.txPackets << ","
						<< i->second.rxPackets << ","
						<< i->second.lostPackets << ","
						<< i->second.packetsDropped.size() << ","
						<< pdf_value << ","
						<< delay_value << ","
						<< hc_value << ","
						<< txbitrate_value << ","
						<< rxbitrate_value << ","
						<< numSeeds << ","
						<< numRun << ","
						<< txRate

						<<"\n";


				// Acumulate for average statistics
				totaltxPackets += i->second.txPackets;
				totalrxPackets += i->second.rxPackets;
				totaldelay += i->second.delaySum.GetSeconds();
				totalHopCount += hc_value;
				totalrxbitrate += rxbitrate_value;
				totalrxbytes += i->second.rxBytes;
				totaltxbitrate += txbitrate_value;
				totaltxbytes += i->second.txBytes;
			}
			numFlows = numFlows +1;
		}
	}

	// Average all nodes statistics
	if (totaltxPackets != 0){
		pdf_total = (double) totalrxPackets / (double) totaltxPackets * 100;
	}
	else{
		pdf_total = 0;
	}

	if (totalrxPackets != 0){
		rxbitrate_total = totalrxbitrate;
		delay_total = (double) totaldelay / (double) totalrxPackets;
		hc_total = (double) totalHopCount / (double) numFlows;
	}
	else{
		rxbitrate_total = 0;
		delay_total = 0;
		hc_total = -1000;
	}

	//print all nodes statistics
	std::cout << "\nTotal Statics: "<< "\n";
	std::cout << "Total PDR: " << pdf_total << " \n";

	std::cout << "Total Delay: " << delay_total << " s\n";
	std::cout << "Total Hop count: " << hc_total << " \n";
	std::cout << "Total Tx Packets: " << totaltxPackets << " \n";
	std::cout << "Total Rx Packets: " << totalrxPackets << " \n";
	std::cout << "Total Tx bitrate: " << txbitrate_total << " kbps\n";
	std::cout << "Total Rx bitrate: " << rxbitrate_total << " kbps\n";
	std::cout << "Total Tx Bytes: " << totaltxbytes << " \n";
	std::cout << "Total Rx Bytes: " << totalrxbytes << " \n\n";

}

void CMDparameters (){



	cmd.AddValue ("iniSimulationTime", "iniSimulationTime", iniSimulationTime);
	cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
	cmd.AddValue ("networkSize", "networkSize", networkSize);

	cmd.AddValue ("numberTraffics", "numberTraffics", numberTraffics);
	cmd.AddValue ("load", "load", load);

	cmd.AddValue ("iniConvergeTime", "iniConvergeTime", iniConvergeTime);
	cmd.AddValue ("midConvergeTime", "midConvergeTime", midConvergeTime);
	cmd.AddValue ("stopConvergeTime", "stopConvergeTime", stopConvergeTime);

	cmd.AddValue ("numNodos", "Number of nodes", numNodos);
	cmd.AddValue ("numSeeds", "numSeeds", numSeeds);
	cmd.AddValue ("numRun", "numRun", numRun);
	cmd.AddValue ("numTest", "numTest", numTest);
	cmd.AddValue ("distance", "distance between nodes", distance); // solo para dos nodos
	cmd.AddValue ("m_txpower", "transmision and reception power", m_txpower); // solo para dos nodos
	cmd.AddValue ("dataset", "dataset", dataset);
	cmd.AddValue ("deviceINFORMATION_file", "deviceINFORMATION_file", deviceINFORMATION_file);

	//==Physical parameters
	cmd.AddValue ("RxSensitivity", "RxSensitivity", RxSensitivity);
	cmd.AddValue ("CcaEdThreshold", "CcaEdThreshold", CcaEdThreshold);
	cmd.AddValue ("TxGain", "TxGain", TxGain);
	cmd.AddValue ("RxGain", "RxGain", RxGain);

	cmd.AddValue ("txRate", "txRate", txRate);
	cmd.AddValue ("phyLayer", "phyLayer", phyLayer);

	cmd.AddValue ("m_txpowerMETERS", "m_txpowerMETERS", m_txpowerMETERS);
	cmd.AddValue ("m_txpowerROUTERS", "m_txpowerROUTERS", m_txpowerROUTERS);
	cmd.AddValue ("m_txpowerCOLLECTORS", "m_txpowerCOLLECTORS", m_txpowerCOLLECTORS);

	//===Mesh Wifi Interface MAC
	cmd.AddValue ("beaconInterval", "beaconInterval", beaconInterval);
	cmd.AddValue ("RandomStart", "RandomStart", RandomStart);
	cmd.AddValue ("BeaconGeneration", "BeaconGeneration", BeaconGeneration);

	//====Peer link parameters
	cmd.AddValue ("MaxBeaconLoss", "MaxBeaconLoss", MaxBeaconLoss);
	cmd.AddValue ("MaxRetries", "MaxRetries", MaxRetries);
	cmd.AddValue ("MaxPacketFailure", "MaxPacketFailure", MaxPacketFailure);
	cmd.AddValue ("RetryTimeout", "RetryTimeout", RetryTimeout);
	cmd.AddValue ("ConfirmTimeout", "ConfirmTimeout", ConfirmTimeout);
	cmd.AddValue ("HoldingTimeout", "HoldingTimeout", HoldingTimeout);

	//====PMP parameters
	cmd.AddValue ("MaxNumberOfPeerLinks", "MaxNumberOfPeerLinks", MaxNumberOfPeerLinks);
	cmd.AddValue ("MaxBeaconShiftValue", "MaxBeaconShiftValue", MaxBeaconShiftValue);
	cmd.AddValue ("EnableBeaconCollisionAvoidance", "EnableBeaconCollisionAvoidance", EnableBeaconCollisionAvoidance);

	//====HWMP parameters
	cmd.AddValue ("Dot11MeshHWMPactivePathTimeout", "Dot11MeshHWMPactivePathTimeout", Dot11MeshHWMPactivePathTimeout);
	cmd.AddValue ("Dot11MeshHWMPactiveRootTimeout", "Dot11MeshHWMPactiveRootTimeout", Dot11MeshHWMPactiveRootTimeout);
	cmd.AddValue ("Dot11MeshHWMPpathToRootInterval", "Dot11MeshHWMPpathToRootInterval", Dot11MeshHWMPpathToRootInterval);
	cmd.AddValue ("Dot11MeshHWMPperrMinInterval", "Dot11MeshHWMPperrMinInterval", Dot11MeshHWMPperrMinInterval);
	cmd.AddValue ("MaxTtl", "MaxTtl", MaxTtl);
	cmd.AddValue ("Dot11MeshHWMPmaxPREQretries", "Dot11MeshHWMPmaxPREQretries", Dot11MeshHWMPmaxPREQretries);
	cmd.AddValue ("UnicastPreqThreshold", "UnicastPreqThreshold", UnicastPreqThreshold);
	cmd.AddValue ("UnicastPerrThreshold", "UnicastPerrThreshold", UnicastPerrThreshold);
	cmd.AddValue ("UnicastDataThreshold", "UnicastDataThreshold", UnicastDataThreshold);
	cmd.AddValue ("DoFlag", "DoFlag", DoFlag	);
	cmd.AddValue ("RfFlag", "RfFlag", RfFlag);

	cmd.AddValue ("m_QoS", "m_QoS", m_QoS);

	cmd.AddValue ("routingTableExpired", "routingTableExpired", routingTableExpired);
	cmd.AddValue ("m_root", "m_root", m_root);

	//== Patch traffic // for the converge time
	cmd.AddValue ("m_enablePatch", "m_enablePatch", m_enablePatch);
	cmd.AddValue ("m_packetIntervalMeanPatch", "m_packetIntervalMeanPatch", m_packetIntervalMeanPatch);
	cmd.AddValue ("m_packetSizeMeanPatch", "m_packetSizeMeanPatch", m_packetSizeMeanPatch);

	//== Electic Vehicle traffic
	cmd.AddValue ("m_enableEV", "m_enableEV", m_enableEV);
	cmd.AddValue ("m_packetIntervalMeanEV", "m_packetIntervalMeanEV", m_packetIntervalMeanEV);
	cmd.AddValue ("m_packetSizeMeanEV", "m_packetSizeMeanEV", m_packetSizeMeanEV);
	cmd.AddValue ("m_percentageMetersEV", "m_percentageMetersEV", m_percentageMetersEV);

	//== Meter Reading traffic
	cmd.AddValue ("m_enableMR", "m_enableMR", m_enableMR);
	cmd.AddValue ("m_packetIntervalMeanMR", "m_packetIntervalMeanMR", m_packetIntervalMeanMR);
	cmd.AddValue ("m_packetSizeMeanMR", "m_packetSizeMeanMR", m_packetSizeMeanMR);
	cmd.AddValue ("m_percentageMetersMR", "m_percentageMetersMR", m_percentageMetersMR);

	//== Power Quality traffic
	cmd.AddValue ("m_enablePQ", "m_enablePQ", m_enablePQ);
	cmd.AddValue ("m_packetIntervalMeanPQ", "m_packetIntervalMeanPQ", m_packetIntervalMeanPQ);
	cmd.AddValue ("m_packetSizeMeanPQ", "m_packetSizeMeanPQ", m_packetSizeMeanPQ);
	cmd.AddValue ("m_percentageMetersPQ", "m_percentageMetersPQ", m_percentageMetersPQ);

	//== Billing Data traffic
	cmd.AddValue ("m_enableBD", "m_enableBD", m_enableBD);
	cmd.AddValue ("m_packetIntervalMeanBD", "m_packetIntervalMeanBD", m_packetIntervalMeanBD);
	cmd.AddValue ("m_packetSizeMeanBD", "m_packetSizeMeanBD", m_packetSizeMeanBD);
	cmd.AddValue ("m_percentageMetersBD", "m_percentageMetersBD", m_percentageMetersBD);


	//== Alarm Data traffic
	cmd.AddValue ("m_enableALARM", "m_enableALARM", m_enableALARM);
	cmd.AddValue ("m_packetIntervalMeanALARM", "m_packetIntervalMeanALARM", m_packetIntervalMeanALARM);
	cmd.AddValue ("m_percentageMetersALARM", "m_percentageMetersALARM", m_percentageMetersALARM);
	cmd.AddValue ("m_packetSizeMeanALARM", "m_packetSizeMeanALARM", m_packetSizeMeanALARM);

	//== Demand response traffic
	cmd.AddValue ("m_enableDR", "m_enableDR", m_enableDR);
	cmd.AddValue ("m_packetIntervalMeanDR", "m_packetIntervalMeanDR", m_packetIntervalMeanDR);
	cmd.AddValue ("m_percentageMetersDR", "m_percentageMetersDR", m_percentageMetersDR);
	cmd.AddValue ("m_packetSizeMeanDR", "m_packetSizeMeanDR", m_packetSizeMeanDR);

	//===============QUEUE PARAMETERS
	cmd.AddValue ("m_edcaEV", "m_edcaEV", m_edcaEV);
	cmd.AddValue ("m_edcaDR", "m_edcaDR", m_edcaDR);
	cmd.AddValue ("m_edcaMR", "m_edcaMR", m_edcaMR);
	cmd.AddValue ("m_edcaPQ", "m_edcaPQ", m_edcaPQ);
	cmd.AddValue ("m_edcaBD", "m_edcaBD", m_edcaBD);
	cmd.AddValue ("m_edcaALARM", "m_edcaALARM", m_edcaALARM);



}


void configureParametersInitial (){
	//=============Mesh Wifi Interface MAC
		Config::SetDefault ("ns3::MeshWifiInterfaceMac::BeaconInterval", TimeValue (Seconds (beaconInterval)));
		Config::SetDefault ("ns3::MeshWifiInterfaceMac::RandomStart", TimeValue (Seconds (RandomStart)));
		Config::SetDefault ("ns3::MeshWifiInterfaceMac::BeaconGeneration", BooleanValue (BeaconGeneration));

	//====Configure the parameters of the Peer Link
		Config::SetDefault ("ns3::dot11s::PeerLink::MaxBeaconLoss", UintegerValue (MaxBeaconLoss));
		Config::SetDefault ("ns3::dot11s::PeerLink::MaxRetries", UintegerValue (MaxRetries));
		Config::SetDefault ("ns3::dot11s::PeerLink::MaxPacketFailure", UintegerValue (MaxPacketFailure));
		Config::SetDefault ("ns3::dot11s::PeerLink::RetryTimeout", TimeValue (Seconds (RetryTimeout)));
		Config::SetDefault ("ns3::dot11s::PeerLink::ConfirmTimeout", TimeValue (Seconds (ConfirmTimeout)));
		Config::SetDefault ("ns3::dot11s::PeerLink::HoldingTimeout", TimeValue (Seconds (HoldingTimeout)));


	//====Configure Peer  Management   Protocol (PMP) parameterss
		Config::SetDefault ("ns3::dot11s::PeerManagementProtocol::EnableBeaconCollisionAvoidance",BooleanValue (EnableBeaconCollisionAvoidance));
		Config::SetDefault("ns3::dot11s::PeerManagementProtocol::MaxNumberOfPeerLinks",UintegerValue  (MaxNumberOfPeerLinks));
		Config::SetDefault("ns3::dot11s::PeerManagementProtocol::MaxBeaconShiftValue",UintegerValue  (MaxBeaconShiftValue));

	// Configure the parameters of the HWMP
		//Lifetime of reactive routing information
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactivePathTimeout",TimeValue (Seconds (Dot11MeshHWMPactivePathTimeout)));
		//Lifetime of proactive routing information
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactiveRootTimeout",TimeValue (Seconds (Dot11MeshHWMPactiveRootTimeout)));
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPrannInterval",TimeValue (Seconds (routingTableExpired*1024)));
		//Interval between two successive proactive PREQs
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPpathToRootInterval",TimeValue (Seconds (Dot11MeshHWMPpathToRootInterval)));
		//Interval between two successive PERRs
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPperrMinInterval",TimeValue (Seconds (Dot11MeshHWMPperrMinInterval)));
		//"Initial value of Time To Live field",
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::MaxTtl",UintegerValue  (MaxTtl));
		//Maximum number of retries before we suppose the destination to be unreachable
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPmaxPREQretries", 	UintegerValue (Dot11MeshHWMPmaxPREQretries));
		//Maximum number of PREQ receivers, when we send a PREQ as a chain of unicasts
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastPreqThreshold",	UintegerValue (UnicastPreqThreshold));
		//Maximum number of PERR receivers, when we send a PERR as a chain of unicastss
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastPerrThreshold",	UintegerValue (UnicastPerrThreshold));
		//Maximum number of broadcast receivers, when we send a broadcast as a chain of unicasts
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastDataThreshold",	UintegerValue (UnicastDataThreshold));
		//Destination only HWMP flag
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::DoFlag", BooleanValue (DoFlag));
		// Reply and forward flag
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::RfFlag", BooleanValue (RfFlag));
}

void configureParametersRun (){

	beaconInterval = simulationTime;
	RetryTimeout = 40;
	ConfirmTimeout = 40;
	HoldingTimeout = 40;

	Dot11MeshHWMPpathToRootInterval = simulationTime;

	//=============Mesh Wifi Interface MAC
		Config::SetDefault ("ns3::MeshWifiInterfaceMac::BeaconInterval", TimeValue (Seconds (beaconInterval)));
		//Config::SetDefault ("ns3::MeshWifiInterfaceMac::RandomStart", TimeValue (Seconds (RandomStart)));
		//Config::SetDefault ("ns3::MeshWifiInterfaceMac::BeaconGeneration", BooleanValue (BeaconGeneration));

	//====Configure the parameters of the Peer Link
		//Config::SetDefault ("ns3::dot11s::PeerLink::MaxBeaconLoss", UintegerValue (MaxBeaconLoss));
		//Config::SetDefault ("ns3::dot11s::PeerLink::MaxRetries", UintegerValue (MaxRetries));
		//Config::SetDefault ("ns3::dot11s::PeerLink::MaxPacketFailure", UintegerValue (MaxPacketFailure));
		Config::SetDefault ("ns3::dot11s::PeerLink::RetryTimeout", TimeValue (Seconds (RetryTimeout)));
		Config::SetDefault ("ns3::dot11s::PeerLink::ConfirmTimeout", TimeValue (Seconds (ConfirmTimeout)));
		Config::SetDefault ("ns3::dot11s::PeerLink::HoldingTimeout", TimeValue (Seconds (HoldingTimeout)));


	//====Configure Peer  Management   Protocol (PMP) parameterss
		//Config::SetDefault ("ns3::dot11s::PeerManagementProtocol::EnableBeaconCollisionAvoidance",BooleanValue (EnableBeaconCollisionAvoidance));
		//Config::SetDefault("ns3::dot11s::PeerManagementProtocol::MaxNumberOfPeerLinks",UintegerValue  (MaxNumberOfPeerLinks));
		//Config::SetDefault("ns3::dot11s::PeerManagementProtocol::MaxBeaconShiftValue",UintegerValue  (MaxBeaconShiftValue));

	// Configure the parameters of the HWMP
		//Lifetime of reactive routing information
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactivePathTimeout",TimeValue (Seconds (Dot11MeshHWMPactivePathTimeout)));
		//Lifetime of proactive routing information
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactiveRootTimeout",TimeValue (Seconds (Dot11MeshHWMPactiveRootTimeout)));
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPrannInterval",TimeValue (Seconds (routingTableExpired*1024)));
		//Interval between two successive proactive PREQs
		Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPpathToRootInterval",TimeValue (Seconds (Dot11MeshHWMPpathToRootInterval)));
		//Interval between two successive PERRs
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPperrMinInterval",TimeValue (Seconds (Dot11MeshHWMPperrMinInterval)));
		//"Initial value of Time To Live field",
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::MaxTtl",UintegerValue  (MaxTtl));
		//Maximum number of retries before we suppose the destination to be unreachable
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPmaxPREQretries", 	UintegerValue (Dot11MeshHWMPmaxPREQretries));
		//Maximum number of PREQ receivers, when we send a PREQ as a chain of unicasts
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastPreqThreshold",	UintegerValue (UnicastPreqThreshold));
		//Maximum number of PERR receivers, when we send a PERR as a chain of unicastss
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastPerrThreshold",	UintegerValue (UnicastPerrThreshold));
		//Maximum number of broadcast receivers, when we send a broadcast as a chain of unicasts
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastDataThreshold",	UintegerValue (UnicastDataThreshold));
		//Destination only HWMP flag
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::DoFlag", BooleanValue (DoFlag));
		// Reply and forward flag
		//Config::SetDefault ("ns3::dot11s::HwmpProtocol::RfFlag", BooleanValue (RfFlag));
}

void printSettings (){

	std::cout << "**********************" << dataset <<"**********************" << "\n";
	std::cout << "QoS: " << m_QoS <<"\n";
	std::cout << "Seed: " << numSeeds << " Run: " << numRun <<"\n";
	std::cout << "ConvergeTime: " << iniConvergeTime << ","  << midConvergeTime << ","  << stopConvergeTime << " Simulation time: "  << simulationTime <<"\n";

	std::cout << "\nAPPLICATIONS TRANSMITTED" <<"\n\n";

	if (m_enablePatch) std::cout << "Patch traffic :: pktSize :: " << m_packetSizeMeanPatch << " :: pktInt :: " << m_packetIntervalMeanPatch <<"\n";
	if (m_enableEV) std::cout << "EV traffic :: pktSize :: " << m_packetSizeMeanEV << " :: pktInt :: " << m_packetIntervalMeanEV << " :: queue :: " << m_edcaEV << "\n";
	if (m_enableMR) std::cout << "MR traffic :: pktSize :: " << m_packetSizeMeanMR << " :: pktInt :: " << m_packetIntervalMeanMR << " :: queue :: " << m_edcaMR << "\n";
	if (m_enablePQ) std::cout << "PQ traffic :: pktSize :: " << m_packetSizeMeanPQ << " :: pktInt :: " << m_packetIntervalMeanPQ << " :: queue :: " << m_edcaPQ << "\n";
	if (m_enableDR) std::cout << "DR traffic :: pktSize :: " << m_packetSizeMeanDR << " :: pktInt :: " << m_packetIntervalMeanDR << " :: queue :: " << m_edcaDR << "\n";
	//if (m_enableMR) std::cout << "MR traffic :: pktSize :: " << m_packetSizeMeanMR << " :: pktInt :: " << m_packetIntervalMeanMR << " :: queue :: " << m_edcaMR << "\n";
	if (m_enableALARM) std::cout << "ALARM traffic :: pktSize :: " << m_packetSizeMeanALARM << " :: pktInt :: " << m_packetIntervalMeanALARM << " :: queue :: " << m_edcaALARM << "\n";


}

std::vector<std::string>
splitString (std::string _cadena, char _delim){

	std::string str =_cadena;
	char delim = _delim; //

	std::stringstream ss(str);
	std::string token;
	std::vector<std::string> vectorMensaje;

	while (std::getline(ss, token, delim)) {
		vectorMensaje.push_back(token);
	}

	return vectorMensaje;

}

uint32_t getNumberOfNodes(std::string deviceINFORMATION_file){
	std::ifstream in(deviceINFORMATION_file); // Open for reading
	std::string s;
	uint32_t numberOfNodes = 0;
	while(getline(in, s))
	{ // Discards newline char
		numberOfNodes = numberOfNodes+1;
	}
	return numberOfNodes;
}

///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
////////////////////TRACES SOURCES AND SINK
struct Dato
{
  std::string type;             ///< type of change
  Mac48Address destination;     ///< route destination
  Mac48Address retransmitter;   ///< route source
  uint32_t interface;           ///< interface index
  uint32_t metric;              ///< metric of route
  Time lifetime;                ///< lifetime of route
  uint32_t seqnum;              ///< sequence number of route
};
//==========Class WifiMac
void
WifiMac_MacTxCallback (std::string context, Ptr<const Packet> packet)
{
	std::ostringstream oss;
	packet->Print(oss);

	//Split the context
	std::vector<std::string> contextVector=splitString(context, '/'); // /NodeList/0/$ns3::Ipv4L3Protocol/Rx
	uint32_t node = atoi(contextVector[2].c_str()); //node where the trace was triggered,
	std::string trace =  contextVector[7]; //trace type

	//Split the packet content
	std::vector<std::string> packetVector=splitString(oss.str(), ' '); // ns3::Ipv4Header (tos 0x0 DSCP Default ECN Not-ECT ttl 1 id 0 protocol 17 offset (bytes) 0 flags [none] length: 48 10.1.1.3 > 10.1.1.255) ns3::UdpHeader (length: 28 698 > 698) ns3::olsr::PacketHeader () ns3::olsr::MessageHeader ()

	std::string source = packetVector[23];

	std::string destination =  packetVector[25]; //10.2.2.22), now we get rid of the ')'
	std::vector<std::string> _destination=splitString(destination, ')');
	Ipv4Address _destinationIpAddress = _destination[0].c_str(); //Convertir a variable IPv4Address

	std::string port =  packetVector[31];
	std::vector<std::string> _port=splitString(port, ')');
	uint32_t _Port = atoi(_port[0].c_str());

	Ptr<Ipv4> _ipv4 = nodes.Get(node)->GetObject<Ipv4>(); //To obtain the Ip adddress
	Ipv4InterfaceAddress _iaddr = _ipv4->GetAddress (1,0);
	Ipv4Address _addri = _iaddr.GetLocal ();

	/*
	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
      << "\t" << m_NodeIDtoX[node]
			<< "\t" << m_NodeIDtoY[node]

			//<< "\t" << oss.str()
			<< "\n";
			*/

	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
			<< "\t" << "WIFI"
			<< "\t" << numberTraffics
			<< "\t" << load

			//<< "\t" << oss.str()
			<< "\n";

	//========Para asignar el QoS segun Tesis de Ruben Rumipamba



}
void
WifiMac_MacTxDropCallback (std::string context, Ptr<const Packet> packet)
{
	std::ostringstream oss;
	packet->Print(oss);
	debugFile << Simulator::Now ().GetNanoSeconds()
			<< "\t" << context
			<< "\t" << oss.str()
			<< "\n";
}
void
WifiMac_MacRxCallback (std::string context, Ptr<const Packet> packet)
{
	std::ostringstream oss;
	packet->Print(oss);

	//Split the context
	std::vector<std::string> contextVector=splitString(context, '/'); // /NodeList/0/$ns3::Ipv4L3Protocol/Rx
	uint32_t node = atoi(contextVector[2].c_str()); //node where the trace was triggered,
	std::string trace =  contextVector[7]; //trace type

	//Split the packet content
	std::vector<std::string> packetVector=splitString(oss.str(), ' '); // ns3::Ipv4Header (tos 0x0 DSCP Default ECN Not-ECT ttl 1 id 0 protocol 17 offset (bytes) 0 flags [none] length: 48 10.1.1.3 > 10.1.1.255) ns3::UdpHeader (length: 28 698 > 698) ns3::olsr::PacketHeader () ns3::olsr::MessageHeader ()

	std::string source = packetVector[23];

	std::string destination =  packetVector[25]; //10.2.2.22), now we get rid of the ')'
	std::vector<std::string> _destination=splitString(destination, ')');
	Ipv4Address _destinationIpAddress = _destination[0].c_str(); //Convertir a variable IPv4Address

	std::string port =  packetVector[31];
	std::vector<std::string> _port=splitString(port, ')');
	uint32_t _Port = atoi(_port[0].c_str());

	Ptr<Ipv4> _ipv4 = nodes.Get(node)->GetObject<Ipv4>(); //To obtain the Ip adddress
	Ipv4InterfaceAddress _iaddr = _ipv4->GetAddress (1,0);
	Ipv4Address _addri = _iaddr.GetLocal ();

	/*
	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
      << "\t" << m_NodeIDtoX[node]
			<< "\t" << m_NodeIDtoY[node]


			//<< "\t" << oss.str()
			<< "\n";

			*/


	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
			<< "\t" << "WIFI"
			<< "\t" << numberTraffics
			<< "\t" << load

			//<< "\t" << oss.str()
			<< "\n";
}
void
WifiMac_MacRxDropCallback (std::string context, Ptr<const Packet> packet)
{
	std::ostringstream oss;
	packet->Print(oss);

	debugFile << Simulator::Now ().GetNanoSeconds()
				<< "\t" << context
			<< "\t" << oss.str()
			<< "\n";
}
//=========Class Ipv4L3Protocol Class
void
Ipv4L3Protocol_TxCallback (std::string context, Ptr< const Packet > packet, Ptr< Ipv4 > ipv4, uint32_t interface)
{
	std::ostringstream oss;
	packet->Print(oss);

	//Split the context
	std::vector<std::string> contextVector=splitString(context, '/'); // /NodeList/0/$ns3::Ipv4L3Protocol/Rx
	uint32_t node = atoi(contextVector[2].c_str()); //node where the trace was triggered,
	std::string trace =  contextVector[4]; //trace type


	//Split the packet content
	std::vector<std::string> packetVector=splitString(oss.str(), ' '); // ns3::Ipv4Header (tos 0x0 DSCP Default ECN Not-ECT ttl 1 id 0 protocol 17 offset (bytes) 0 flags [none] length: 48 10.1.1.3 > 10.1.1.255) ns3::UdpHeader (length: 28 698 > 698) ns3::olsr::PacketHeader () ns3::olsr::MessageHeader ()

	std::string source = packetVector[20];

	std::string destination =  packetVector[22]; //10.2.2.22), now we get rid of the ')'
	std::vector<std::string> _destination=splitString(destination, ')');
	Ipv4Address _destinationIpAddress = _destination[0].c_str(); //Convertir a variable IPv4Address

	std::string port =  packetVector[28];
	std::vector<std::string> _port=splitString(port, ')');
	uint32_t _Port = atoi(_port[0].c_str());

	Ptr<Ipv4> _ipv4 = nodes.Get(node)->GetObject<Ipv4>(); //To obtain the Ip adddress
	Ipv4InterfaceAddress _iaddr = _ipv4->GetAddress (1,0);
	Ipv4Address _addri = _iaddr.GetLocal ();

	/*
	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
      << "\t" << m_NodeIDtoX[node]
			<< "\t" << m_NodeIDtoY[node]

			//<< "\t" << oss.str()
			<< "\n";
			*/


	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
			<< "\t" << "WIFI"
			<< "\t" << numberTraffics
			<< "\t" << load

			//<< "\t" << oss.str()
			<< "\n";
}
void
Ipv4L3Protocol_RxCallback (std::string context, Ptr< const Packet > packet, Ptr< Ipv4 > ipv4, uint32_t interface)
{
	std::ostringstream oss;
	packet->Print(oss);

	//Split the context
	std::vector<std::string> contextVector=splitString(context, '/'); // /NodeList/0/$ns3::Ipv4L3Protocol/Rx
	uint32_t node = atoi(contextVector[2].c_str()); //node where the trace was triggered,
	std::string trace =  contextVector[4]; //trace type


	//Split the packet content
	std::vector<std::string> packetVector=splitString(oss.str(), ' '); // ns3::Ipv4Header (tos 0x0 DSCP Default ECN Not-ECT ttl 1 id 0 protocol 17 offset (bytes) 0 flags [none] length: 48 10.1.1.3 > 10.1.1.255) ns3::UdpHeader (length: 28 698 > 698) ns3::olsr::PacketHeader () ns3::olsr::MessageHeader ()

	std::string source = packetVector[20];

	std::string destination =  packetVector[22]; //10.2.2.22), now we get rid of the ')'
	std::vector<std::string> _destination=splitString(destination, ')');
	Ipv4Address _destinationIpAddress = _destination[0].c_str(); //Convertir a variable IPv4Address

	std::string port =  packetVector[28];
	std::vector<std::string> _port=splitString(port, ')');
	uint32_t _Port = atoi(_port[0].c_str());

	Ptr<Ipv4> _ipv4 = nodes.Get(node)->GetObject<Ipv4>(); //To obtain the Ip adddress
	Ipv4InterfaceAddress _iaddr = _ipv4->GetAddress (1,0);
	Ipv4Address _addri = _iaddr.GetLocal ();

	/*
	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
      << "\t" << m_NodeIDtoX[node]
			<< "\t" << m_NodeIDtoY[node]

			//<< "\t" << oss.str()
			<< "\n";

			*/


	mcsResultsFile << Simulator::Now ().GetNanoSeconds()
			//<< "\t" << context
			<< "\t" << node
			<< "\t" << trace
			<< "\t" << packet->GetUid()
			<< "\t" << packet->GetSize()
			<< "\t" << _addri
			<< "\t" << source
			<< "\t" << _destinationIpAddress
			<< "\t" << _Port
			<< "\t" << numSeeds
			<< "\t" << numRun
			<< "\t" << txRate
			<< "\t" << m_QoS
			<< "\t" << networkSize
			<< "\t" << "WIFI"
			<< "\t" << numberTraffics
			<< "\t" << load

			//<< "\t" << oss.str()
			<< "\n";
}
void
HwmpProtocol_RouteChangeCallback (std::string context, ns3::dot11s::RouteChange d)
{

	debugFile << Simulator::Now ().GetNanoSeconds()
				<< "\t" << context
				<< "\t" << d.type
			<< "\t" << d.destination
			<< "\n";
}

void traceSources(){
	std::ostringstream trazaText;

	trazaText.str(""); //MacTx:  A packet has been received from higher layers and is being processed in preparation for queueing for transmission.
	trazaText << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx";
	Config::Connect (trazaText.str(), MakeCallback(&WifiMac_MacTxCallback));

	if (DEBUG){
		trazaText.str(""); //MacTxDrop:  A packet has been received from higher layers and is being processed in preparation for queueing for transmission.
		trazaText << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop";
		Config::Connect (trazaText.str(), MakeCallback(&WifiMac_MacTxDropCallback));
	}

	trazaText.str(""); //MacRx:  A packet has been received from higher layers and is being processed in preparation for queueing for transmission.
	trazaText << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx";
	Config::Connect (trazaText.str(), MakeCallback(&WifiMac_MacRxCallback));

	if (DEBUG){
		trazaText.str(""); //MacRx:  A packet has been received from higher layers and is being processed in preparation for queueing for transmission.
		trazaText << "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop";
		Config::Connect (trazaText.str(), MakeCallback(&WifiMac_MacRxDropCallback));
	}

	trazaText.str(""); //Tx: Send ipv4 packet to outgoing interface.
	trazaText << "/NodeList/*/$ns3::Ipv4L3Protocol/Tx";
	Config::Connect (trazaText.str(), MakeCallback(&Ipv4L3Protocol_TxCallback));

	trazaText.str(""); //Tx: Send ipv4 packet to outgoing interface.
	trazaText << "/NodeList/*/$ns3::Ipv4L3Protocol/Rx";
	Config::Connect (trazaText.str(), MakeCallback(&Ipv4L3Protocol_TxCallback));

	if (DEBUG){
		//HWMP trazas
		trazaText.str("");
		trazaText << "/NodeList/*/DeviceList/*/$ns3::MeshPointDevice/RoutingProtocol/$ns3::dot11s::HwmpProtocol/RouteChange" ;
		Config::Connect (trazaText.str(), MakeCallback(&HwmpProtocol_RouteChangeCallback ));
	}

}

void populateArpCache(){
	Ptr<Packet> dummy = Create<Packet> ();
	Ptr<ArpCache> arp = CreateObject<ArpCache> ();
	arp->SetAliveTimeout (Seconds (3600 * 24 * 365));
	for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End ();	++i)
    {
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);
		for (ObjectVectorValue::Iterator j = interfaces.Begin (); j !=interfaces.End (); j++)
        {
			Ptr<Ipv4Interface> ipIface =	j->second->GetObject<Ipv4Interface> ();
			NS_ASSERT (ipIface != 0);
			Ptr<NetDevice> device = ipIface->GetDevice ();
			NS_ASSERT (device != 0);
			Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress ());
			for (uint32_t k = 0; k < ipIface->GetNAddresses (); k ++)
			{
				Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal ();
				if (ipAddr == Ipv4Address::GetLoopback ())
                {
					continue;
                }
				Ipv4Header ipHeader;
				ArpCache::Entry *entry = arp->Add (ipAddr);
				entry->MarkWaitReply (ns3::ArpCache::Ipv4PayloadHeaderPair(dummy,ipHeader));
				entry->MarkAlive (addr);
				entry->ClearPendingPacket();
				entry->MarkPermanent ();
            }
        }
    }

	for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End ();	++i)
    {
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);
		for (ObjectVectorValue::Iterator j = interfaces.Begin (); j !=	interfaces.End (); j ++)
        {
			Ptr<Ipv4Interface> ipIface =j->second->GetObject<Ipv4Interface> ();
			ipIface->SetAttribute ("ArpCache", PointerValue (arp));
        }
    }
}

Ptr<ListPositionAllocator> SetPositions_TxPOWER (std::string device_file, double TxGain, double RxGain, uint32_t numberOfInterfaces,
											double m_txpowerMETERS, double m_txpowerROUTERS, double m_txpowerCOLLECTORS
											)
{

	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	std::ifstream in(device_file); // Open for reading
	std::string s;
	while(getline(in, s))
	{ // Discards newline char
		std::string str =s;
		char delim = ',' ;
		std::stringstream ss(str);
		std::string token; // Elemento obtenido
		std::vector<std::string> cadenaNodoApp;
		while (std::getline(ss, token, delim)) {
			cadenaNodoApp.push_back(token);
		}
		uint32_t nodeID = atoi(cadenaNodoApp[0].c_str());
		double posX = atof(cadenaNodoApp[2].c_str()) ;
		double posY = atof(cadenaNodoApp[1].c_str()) ;
		std::string DEVICE = cadenaNodoApp[3].c_str();

  	    positionAlloc->Add (Vector (posX, posY, 0.0));
		// access i-th mesh point, production code should add NS_ASSERT (mp != 0);
		Ptr<MeshPointDevice> mp = DynamicCast<MeshPointDevice> (meshDevices.Get (nodeID));
		// get list of MP interfaces, production code should add NS_ASSERT (ifaces.size() == 2)
		std::vector< Ptr<NetDevice> > ifaces = mp->GetInterfaces ();
		for (uint32_t j = 0; j < numberOfInterfaces; ++j)
		{
		  // access j-th interface, it's worth to add assertion again
		  Ptr<WifiNetDevice> iface = DynamicCast<WifiNetDevice> (ifaces [j]);
		  // finally set physical parameters
		  if (DEVICE == "METER"){
				iface->GetPhy()->SetTxPowerStart(m_txpowerMETERS);
				iface->GetPhy()->SetTxPowerEnd(m_txpowerMETERS);
		  }
		  if (DEVICE == "ROUTER"){
				iface->GetPhy()->SetTxPowerStart(m_txpowerROUTERS);
				iface->GetPhy()->SetTxPowerEnd(m_txpowerROUTERS);
		  }
		  if (DEVICE == "COLLECTOR"){
				iface->GetPhy()->SetTxPowerStart(m_txpowerCOLLECTORS);
				iface->GetPhy()->SetTxPowerEnd(m_txpowerCOLLECTORS);
		  }

		  iface->GetPhy()->SetTxGain(TxGain);
		  iface->GetPhy()->SetRxGain(RxGain);

		}
	}
	return positionAlloc;

}

Ptr<ListPositionAllocator> SetPositions_TxPOWER_TWONODES (double distance, double TxGain, double RxGain, uint32_t numberOfInterfaces,
											double m_txpower
											)
{
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
	positionAlloc->Add (Vector (0, 0, 0.0));
	positionAlloc->Add (Vector (distance, 0, 0.0));


	for (uint32_t nodPos = 0; nodPos < 2; nodPos++)
	{
		Ptr<MeshPointDevice> mp = DynamicCast<MeshPointDevice> (meshDevices.Get (nodPos));
		// get list of MP interfaces, production code should add NS_ASSERT (ifaces.size() == 2)
		std::vector< Ptr<NetDevice> > ifaces = mp->GetInterfaces ();
		for (uint32_t j = 0; j < numberOfInterfaces; ++j)
		{
		  // access j-th interface, it's worth to add assertion again
		  Ptr<WifiNetDevice> iface = DynamicCast<WifiNetDevice> (ifaces [j]);
		  // finally set physical parameters
		  iface->GetPhy()->SetTxPowerStart(m_txpower);
		  iface->GetPhy()->SetTxPowerEnd(m_txpower);
		  //iface->GetPhy()->SetNTxPower(2);
		  iface->GetPhy()->SetTxGain(TxGain);
		  iface->GetPhy()->SetRxGain(RxGain);
		  }
	}

	return positionAlloc;
}

void checkNodePositions (uint32_t numNodos, uint32_t numberOfInterfaces, std::string phyLayer)
{
	//In order to see the position and the mac address
	for (uint32_t nodPos = 0; nodPos < numNodos; nodPos++)
	{
 

 
	  std::cout << "Nodo "<<nodPos+1<<": \t" <<nodes.Get(nodPos)->GetDevice(0)->GetAddress() << "\t";
	  Ptr<MobilityModel> mob = nodes.Get(nodPos)->GetObject<MobilityModel> ();
	  if (! mob) continue; // Strange -- node has no mobility modelinstalled. Skip.
			  Vector pos = mob->GetPosition ();
			  std::cout << "Position "  << " is at (" << pos.x << ", " <<
	  pos.y << ", " << pos.z << "), TRANSMISSION POWER (dbm):  ";
     
    m_NodeIDtoX[(nodPos)] = pos.x;
		m_NodeIDtoY[(nodPos)] = pos.y;

		Ptr<MeshPointDevice> mp = DynamicCast<MeshPointDevice> (meshDevices.Get (nodPos));
		// get list of MP interfaces, production code should add NS_ASSERT (ifaces.size() == 2)
		std::vector< Ptr<NetDevice> > ifaces = mp->GetInterfaces ();
		for (uint32_t j = 0; j < numberOfInterfaces; ++j)
		{
		  // access j-th interface, it's worth to add assertion again
		  Ptr<WifiNetDevice> iface = DynamicCast<WifiNetDevice> (ifaces [j]);
		  // finally set physical parameters
		  std::cout << iface->GetPhy()->GetTxPowerStart()
								<< ", " << iface->GetPhy()->GetTxPowerEnd()
								<< " Gain (db), " << iface->GetPhy()->GetTxGain()
								<< ", " << iface->GetPhy()->GetRxGain()
								<< " phyLayer: " << phyLayer

								  <<"\n" ;
		  }
	}
}


void createServerPATCH(std::string deviceINFORMATION_file, double stopTime, uint32_t patchPort1,uint32_t patchPort2 , uint32_t nodoDestino)
	{
	UdpServerHelper Server6999 (patchPort1);
	ApplicationContainer ServerApps6999=Server6999.Install(nodes.Get(nodoDestino));
	ServerApps6999.Start (Seconds (1.0));
	ServerApps6999.Stop (Seconds (stopTime + 1));

	//Para crear el patch trafico desde los colectores hacia los meters
	std::ifstream in(deviceINFORMATION_file); // Open for reading
	std::string s;
	while(getline(in, s))
	{ // Discards newline char
		std::string str =s;
		char delim = ',' ;
		std::stringstream ss(str);
		std::string token; // Elemento obtenido
		std::vector<std::string> cadenaNodoApp;
		while (std::getline(ss, token, delim)) {
			cadenaNodoApp.push_back(token);
		}
		uint32_t m_source = atoi(cadenaNodoApp[0].c_str());
		std::string DEVICE = cadenaNodoApp[3].c_str();
		if  ((DEVICE == "METER")|| (DEVICE == "ROUTER")){
			UdpServerHelper Server7000 (patchPort2);
			ApplicationContainer ServerApps7000=Server7000.Install(nodes.Get(m_source));
			ServerApps7000.Start (Seconds (1.0));
			ServerApps7000.Stop (Seconds (stopTime + 1));
		}
	}
}

void createClientPATCH(std::string deviceINFORMATION_file, uint32_t nodoDestino, uint32_t port1, uint32_t port2,
		double iniTime, double midTime, double stopTime, uint32_t pktSize, double pktInt)
{
	std::ifstream in(deviceINFORMATION_file); // Open for reading
		std::string s;
		while(getline(in, s))
		{ // Discards newline char
			std::string str =s;
			char delim = ',' ;
			std::stringstream ss(str);
			std::string token; // Elemento obtenido
			std::vector<std::string> cadenaNodoApp;
			while (std::getline(ss, token, delim)) {
				cadenaNodoApp.push_back(token);
			}
			uint32_t m_source = atoi(cadenaNodoApp[0].c_str());
			std::string DEVICE = cadenaNodoApp[3].c_str();

			if ( (DEVICE == "METER") || (DEVICE == "ROUTER") ){

					///////////////////////////////////////////////////////
					////////////////Para el downlink traffic

					InetSocketAddress mdestTosPatch (interfaces.GetAddress (nodoDestino), port1);
					//Para calcular cuando se va enviar el primer paquetepaquete
					double minIniTime = iniTime;	double maxIniTime = midTime *0.05;
					Ptr<UniformRandomVariable> randomVarTimePatch = CreateObject<UniformRandomVariable> ();
					randomVarTimePatch->SetAttribute ("Min", DoubleValue (minIniTime));
					randomVarTimePatch->SetAttribute ("Max", DoubleValue (maxIniTime));

					uint32_t iniTimePatch = randomVarTimePatch->GetInteger();
					uint32_t stopTimePatch= midTime;

					UdpClientHelper udpClient_Patch (mdestTosPatch); // Direccion del nodo destino y el numero de puerto
					udpClient_Patch.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)((stopTimePatch-iniTimePatch)*(1/pktInt))));
					udpClient_Patch.SetAttribute ("Interval", TimeValue(Seconds(pktInt))); //packets/s
					udpClient_Patch.SetAttribute ("PacketSize", UintegerValue (pktSize));
					udpClient_Patch.SetAttribute ("PacketIntervalDistribution", StringValue ("Exponential"));

					// Client scheduling
					ApplicationContainer udpClientApps_Patch =udpClient_Patch.Install (nodes.Get (m_source));
					udpClientApps_Patch.Start (Seconds (iniTimePatch+m_source*0.015));
					//udpClientApps_Patch.Start (Seconds (iniTimePatch+m_source+m_source*0.015));
					udpClientApps_Patch.Stop (Seconds(stopTimePatch));

					///////////////////////////////////////////////////////
					////////////////Para el downlink traffic

					InetSocketAddress mdestTosPatchR (interfaces.GetAddress (m_source), port2);
					//Para calcular cuando se va enviar el primer paquetepaquete
					minIniTime = midTime;	maxIniTime = stopTime *0.05;
					Ptr<UniformRandomVariable> randomVarTimePatchR = CreateObject<UniformRandomVariable> ();
					randomVarTimePatchR->SetAttribute ("Min", DoubleValue (minIniTime));
					randomVarTimePatchR->SetAttribute ("Max", DoubleValue (maxIniTime));

					uint32_t iniTimePatchR = randomVarTimePatchR->GetInteger();
					uint32_t stopTimePatchR = stopTime;

					UdpClientHelper udpClient_PatchR (mdestTosPatchR); // Direccion del nodo destino y el numero de puerto
					udpClient_PatchR.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)((stopTimePatchR - iniTimePatchR)*(1/pktInt))));
					udpClient_PatchR.SetAttribute ("Interval", TimeValue(Seconds(pktInt))); //packets/s
					udpClient_PatchR.SetAttribute ("PacketSize", UintegerValue (pktSize));
					udpClient_PatchR.SetAttribute ("PacketIntervalDistribution", StringValue ("Exponential"));

					// Client scheduling
					ApplicationContainer udpClientApps_PatchR =udpClient_PatchR.Install (nodes.Get (nodoDestino));
					udpClientApps_PatchR.Start (Seconds (iniTimePatchR+m_source*0.015));
					//udpClientApps_PatchR.Start (Seconds (iniTimePatchR+m_source + m_source*0.015));
					udpClientApps_PatchR.Stop (Seconds(stopTimePatchR));



			}

		}

}


void createServerUL (uint32_t port, uint32_t nodoDestino, double iniTime, double stopTime)
{
	UdpServerHelper Server (port);
	ApplicationContainer ServerApps=Server.Install(nodes.Get(nodoDestino));
	ServerApps.Start (Seconds (iniTime));
	ServerApps.Stop (Seconds (stopTime + 1));
}

void createServerDL (std::string deviceINFORMATION_file, uint32_t port, double iniTime, double stopTime)
{
	std::ifstream in(deviceINFORMATION_file); // Open for reading
	std::string s;
	while(getline(in, s))
	{ // Discards newline char
		std::string str =s;
		char delim = ',' ;
		std::stringstream ss(str);
		std::string token; // Elemento obtenido
		std::vector<std::string> cadenaNodoApp;
		while (std::getline(ss, token, delim)) {
			cadenaNodoApp.push_back(token);
		}
		uint32_t m_source = atoi(cadenaNodoApp[0].c_str());
		std::string DEVICE = cadenaNodoApp[3].c_str();

		if  (DEVICE == "METER"){

			// SERVER APP7 (DEMAND RESPONSE DATA)
			UdpServerHelper Server7 (port);
			ApplicationContainer ServerApps7=Server7.Install(nodes.Get(m_source));
			ServerApps7.Start (Seconds (iniTime));
			ServerApps7.Stop (Seconds (stopTime + 1));

		}

	}
}

void createAppUL (std::string deviceINFORMATION_file, uint32_t nodoDestino, double iniTime, double stopTime, double percentageMeters,
					double pktInt, uint32_t pktSize, uint32_t port, std::string edcaQueue, std::string distribution)
{

	std::ifstream in(deviceINFORMATION_file); // Open for reading
	std::string s;
	while(getline(in, s))
	{ // Discards newline char
		std::string str =s;
		char delim = ',' ;
		std::stringstream ss(str);
		std::string token; // Elemento obtenido
		std::vector<std::string> cadenaNodoApp;
		while (std::getline(ss, token, delim)) {
			cadenaNodoApp.push_back(token);
		}
		uint32_t m_source = atoi(cadenaNodoApp[0].c_str());
		std::string DEVICE = cadenaNodoApp[3].c_str();

		if  (DEVICE == "METER")
		{

			double minPerc = 0.0;	double maxPerc = 100.0;
			Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable> ();
			randomVar->SetAttribute ("Min", DoubleValue (minPerc));
			randomVar->SetAttribute ("Max", DoubleValue (maxPerc));
			uint32_t randomValue = randomVar->GetInteger();

			if (randomValue <=percentageMeters)
			{ //Solo un porcentaje de meters

				InetSocketAddress mdestTos (interfaces.GetAddress (nodoDestino), port);

				if (m_QoS){
					if (edcaQueue=="AC_BE") mdestTos.SetTos (0x70); //{0x70, 0x28, 0xb8, 0xc0}; //AC_BE, AC_BK, AC_VI, AC_VO
					if (edcaQueue=="AC_BK") mdestTos.SetTos (0x28); //{0x70, 0x28, 0xb8, 0xc0}; //AC_BE, AC_BK, AC_VI, AC_VO
					if (edcaQueue=="AC_VI") mdestTos.SetTos (0xb8); //{0x70, 0x28, 0xb8, 0xc0}; //AC_BE, AC_BK, AC_VI, AC_VO
					if (edcaQueue=="AC_VO") mdestTos.SetTos (0xc0); //{0x70, 0x28, 0xb8, 0xc0}; //AC_BE, AC_BK, AC_VI, AC_VO
				}

				//Para calcular cuando se va enviar el primer paquete
				double minIniTime = iniTime;	double maxIniTime = stopTime;
				//double minIniTime = iniTime;	double maxIniTime = iniTime+2;

				Ptr<UniformRandomVariable> randomVarTime = CreateObject<UniformRandomVariable> ();
				randomVarTime->SetAttribute ("Min", DoubleValue (minIniTime));
				randomVarTime->SetAttribute ("Max", DoubleValue (maxIniTime));
				uint32_t iniTimeAPP = randomVarTime->GetInteger();

				//Para calcular cuando se va enviar el ultimo paquete
				uint32_t stopTimeAPP = stopTime; // si es periodico, seria el tiempo del ultimo paquete
				if (distribution=="Exponential") //si es un evento se calculo el tiempo cuando finaliza el evento
				{
					double minStopTime = iniTimeAPP ;	double maxStopTime = stopTime;
					Ptr<UniformRandomVariable> randomVarTimeStop = CreateObject<UniformRandomVariable> ();
					randomVarTimeStop->SetAttribute ("Min", DoubleValue (minStopTime));
					randomVarTimeStop->SetAttribute ("Max", DoubleValue (maxStopTime));
					stopTimeAPP = randomVarTimeStop->GetInteger();
				}

				UdpClientHelper udpClient (mdestTos); // Direccion del nodo destino y el numero de puerto
				udpClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)((stopTimeAPP-iniTimeAPP)*(1/pktInt))));
				udpClient.SetAttribute ("Interval", TimeValue(Seconds(pktInt))); //packets/s
				udpClient.SetAttribute ("PacketSize", UintegerValue (pktSize));
				udpClient.SetAttribute ("PacketIntervalDistribution", StringValue (distribution));

				// Client scheduling
				ApplicationContainer udpClientApps = udpClient.Install (nodes.Get (m_source));
				udpClientApps.Start (Seconds (iniTimeAPP+m_source*0.01));
				udpClientApps.Stop (Seconds(stopTimeAPP));
			}


		}

	}


}


void createAppDL (std::string deviceINFORMATION_file, uint32_t nodoDestino, double iniTime, double stopTime, double percentageMeters,
					double pktInt, uint32_t pktSize, uint32_t port, std::string distribution)
{

	std::ifstream in(deviceINFORMATION_file); // Open for reading
	std::string s;
	while(getline(in, s))
	{ // Discards newline char
		std::string str =s;
		char delim = ',' ;
		std::stringstream ss(str);
		std::string token; // Elemento obtenido
		std::vector<std::string> cadenaNodoApp;
		while (std::getline(ss, token, delim)) {
			cadenaNodoApp.push_back(token);
		}
		uint32_t m_source = atoi(cadenaNodoApp[0].c_str());
		std::string DEVICE = cadenaNodoApp[3].c_str();

		if (DEVICE == "METER")
		{
			double minPerc = 0.0;	double maxPerc = 100.0;
			Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable> ();
			randomVar->SetAttribute ("Min", DoubleValue (minPerc));
			randomVar->SetAttribute ("Max", DoubleValue (maxPerc));
			uint32_t randomValue = randomVar->GetInteger();

			if (randomValue <=percentageMeters){ //Solo un porcentaje de meters

				InetSocketAddress mdestTos (interfaces.GetAddress (m_source), port); //el nodo destino es el meters


				//Para calcular cuando se va enviar el paquete
				double minIniTime = iniTime;	double maxIniTime = stopTime;
				Ptr<UniformRandomVariable> randomVarTime = CreateObject<UniformRandomVariable> ();
				randomVarTime->SetAttribute ("Min", DoubleValue (minIniTime));
				randomVarTime->SetAttribute ("Max", DoubleValue (maxIniTime));

				uint32_t iniTimeAPP = randomVarTime->GetInteger();
				uint32_t stopTimeAPP = stopTime;

				if (distribution=="Exponential") //si es un evento, se calculo el tiempo cuando finaliza el evento
				{
					double minStopTime = iniTimeAPP ;	double maxStopTime = stopTime;
					Ptr<UniformRandomVariable> randomVarTimeStop = CreateObject<UniformRandomVariable> ();
					randomVarTimeStop->SetAttribute ("Min", DoubleValue (minStopTime));
					randomVarTimeStop->SetAttribute ("Max", DoubleValue (maxStopTime));
					stopTimeAPP = randomVarTimeStop->GetInteger();
				}

				UdpClientHelper udpClient (mdestTos); // Direccion del nodo destino y el numero de puerto
				udpClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)((stopTimeAPP-iniTimeAPP)*(1/pktInt))));
				udpClient.SetAttribute ("Interval", TimeValue(Seconds(pktInt))); //packets/s
				udpClient.SetAttribute ("PacketSize", UintegerValue (pktSize));
				udpClient.SetAttribute ("PacketIntervalDistribution", StringValue (distribution));

				// Client scheduling
				ApplicationContainer udpClientApps = udpClient.Install (nodes.Get (nodoDestino));
				udpClientApps.Start (Seconds (iniTimeAPP+m_source*0.01));
				udpClientApps.Stop (Seconds(stopTimeAPP));
			}
		}

	}
}

int main (int argc, char *argv[])
{
	Packet::EnableChecking();
	Packet::EnablePrinting();

	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	///////COMMAND LINE PARAMETERS
	CMDparameters ();
	cmd.Parse (argc,argv);
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	//==========================SEED MANAGER
	RngSeedManager::SetSeed (numSeeds);  // Changes seed from default of 1 to 3
	RngSeedManager::SetRun (numRun);   // Changes run number from default of 1 to 7
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////

	numNodos = getNumberOfNodes(deviceINFORMATION_file);
	nodes.Create (numNodos);

	std::ostringstream auxFile;

	auxFile.str("");
	auxFile << "./tracesCOMNET/" << dataset << "_QoS_" << m_QoS <<"_numSeeds_"<<numSeeds<<"_numRun_"<<numRun<<"_networkSize_"<<networkSize<<".txt";
	mcsResultsFile.open (auxFile.str().c_str());

	auxFile.str("");
	auxFile << "./CBRFiles/" << dataset << "_QoS_" << m_QoS <<"_numSeeds_"<<numSeeds<<"_numRun_"<<numRun<<"_networkSize_"<<networkSize<<".csv";
	cbrFile.open (auxFile.str().c_str());


	if (DEBUG){
		std::ostringstream auxFile; auxFile.str("");
		auxFile << "./debugTraces/"<< dataset 	<<"_numSeeds_"<<numSeeds <<"_numRun_"<<numRun<<".txt";
		debugFile.open (auxFile.str().c_str());
	}


//====Configure YansWifi (Caracteristicas del canal y el medio)
	YansWifiPhyHelper wifiPhy;

	wifiPhy.Set ("RxSensitivity", DoubleValue (RxSensitivity) );
	wifiPhy.Set ("CcaEdThreshold", DoubleValue (CcaEdThreshold) );
	wifiPhy.Set ("TxGain", DoubleValue (TxGain) );
	wifiPhy.Set ("RxGain", DoubleValue (RxGain) );
	wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");

	YansWifiChannelHelper wifiChannel; // = YansWifiChannelHelper::Default ();
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",StringValue ("2.7"));




	wifiPhy.SetChannel (wifiChannel.Create ());

	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	//////////////////// CONFIGURE PARAMETERS
	configureParametersInitial ();
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////


//==== Create mesh helper and set stack installer to it
	// Stack installer creates all needed protocols and install them to device
	MeshHelper mesh;
	mesh = MeshHelper::Default ();

	//modo reactivo
	//mesh.SetStackInstaller ("ns3::Dot11sStack");

	mesh.SetStackInstaller ("ns3::Dot11sStack", "Root", Mac48AddressValue(Mac48Address (m_root.c_str ())));


	mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
	mesh.SetMacType ("RandomStart", TimeValue (Seconds (0.1)));
	mesh.SetNumberOfInterfaces (numberOfInterfaces);

	//mesh.SetAckPolicySelectorForAc()


	if (phyLayer=="802.11a") mesh.SetStandard (WIFI_STANDARD_80211a);
	if (phyLayer=="802.11b") mesh.SetStandard (WIFI_STANDARD_80211b);

	if ((txRate==6) && (phyLayer == "802.11a"))
	{
		TxGain = 1;	RxGain = 1;
		//m_txpowerMETERS = 42; m_txpowerROUTERS = 42;m_txpowerCOLLECTORS = 42;
		mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",
				StringValue ("OfdmRate6Mbps"),"ControlMode",
				StringValue ("OfdmRate6Mbps"));
	}

	if (phyLayer =="802.11b")
	{

		TxGain = 0; RxGain = 0;
		//m_txpowerMETERS = 33.0; //300 m
		//m_txpowerMETERS = 43.0; //750 m
		//m_txpowerROUTERS = 58.5;
		//m_txpowerCOLLECTORS = 58.5;

		/////////////
		/*
		TxGain = 10; RxGain = 10;
		//m_txpowerMETERS = 22.5; //750 m
		m_txpowerMETERS = 12; //300 m
		m_txpowerROUTERS = 38;	m_txpowerCOLLECTORS = 38;
		*/
		//////////////////////////

		mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",
				StringValue ("DsssRate1Mbps"),"ControlMode",
			StringValue ("DsssRate1Mbps"));
	}



	if (m_QoS) mesh.SetMacType("QosSupported",BooleanValue(true));


	//if (m_QoS)	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::MeshWifiInterfaceMac/QosSupported", BooleanValue (true));

	if (m_QoS)	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::MeshWifiInterfaceMac/QosSupported", BooleanValue (true));




	meshDevices = mesh.Install (wifiPhy, nodes);

/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/////////////////Load the positions of METERS, ROUTERS AND COLLECTORS,
	/////and set the TX POWER DEPENDING THE DEVICE TYPES

	MobilityHelper mobility;
	if (numNodos==2)
		mobility.SetPositionAllocator (SetPositions_TxPOWER_TWONODES (distance, TxGain, RxGain, numberOfInterfaces,m_txpower));

	if (numNodos != 2)
		mobility.SetPositionAllocator (SetPositions_TxPOWER (deviceINFORMATION_file, TxGain, RxGain, numberOfInterfaces,
										m_txpowerMETERS, m_txpowerROUTERS, m_txpowerCOLLECTORS	));

	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (nodes);


/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/////////////==== Configure Internet stack
	InternetStackHelper internetStack;
	internetStack.Install (nodes);
	Ipv4AddressHelper address;
	address.SetBase ("10.0.0.0", "255.0.0.0");

	interfaces = address.Assign (meshDevices);
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///////////////////////////NODES POSITION
	checkNodePositions (numNodos, numberOfInterfaces, phyLayer);
/////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/////////////////////// Populate Arp Cache
	populateArpCache();
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

	int nodoDestino=numNodos-1; //

/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//////////////////////////CREATE SERVER APPS

	/* SERVER PATCH */if (m_enablePatch) createServerPATCH(deviceINFORMATION_file, stopConvergeTime, 6999, 7000, nodoDestino);
	/* SERVER ELECTRIC VEHICLE TRAFFIC */ if (m_enableEV) createServerUL (1, nodoDestino, stopConvergeTime, simulationTime);
	/* SERVER METER READING TRAFFIC */ if (m_enableMR) createServerUL (3, nodoDestino, stopConvergeTime, simulationTime);
	/* SERVER POWER QUALITY TRAFFIC */ if (m_enablePQ) createServerUL (4, nodoDestino, stopConvergeTime, simulationTime);
	/* SERVER BILLING DATA TRAFFIC */ if (m_enableBD) createServerUL (5, nodoDestino, stopConvergeTime, simulationTime);
	/* SERVER ALARM DATA TRAFFIC */ if (m_enableALARM) createServerUL (6, nodoDestino, stopConvergeTime, simulationTime);
	/* SERVER DEMAND RESPONSE DATA TRAFFIC */if (m_enableDR) createServerDL (deviceINFORMATION_file, 7, stopConvergeTime, simulationTime);

/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//////////////////////////CREATE CLIENT APPS




	/* CLIENT PATCH */ if (m_enablePatch) createClientPATCH(deviceINFORMATION_file, nodoDestino, 6999, 7000,
															iniConvergeTime, midConvergeTime, stopConvergeTime,
															m_packetSizeMeanPatch, m_packetIntervalMeanPatch);

	/* CLIENT EV DATA TRAFFIC*/ if (m_enableEV) createAppUL (deviceINFORMATION_file, nodoDestino, iniSimulationTime, simulationTime,
															m_percentageMetersEV,m_packetIntervalMeanEV, m_packetSizeMeanEV, 1, m_edcaEV, "Exponential");

	/* CLIENT METER READING DATA TRAFFIC*/ if (m_enableMR) createAppUL (deviceINFORMATION_file, nodoDestino, iniSimulationTime, simulationTime,
																m_percentageMetersMR,m_packetIntervalMeanMR, m_packetSizeMeanMR, 3, m_edcaMR, "Fixed");

	/* CLIENT POWER QUALITY DATA TRAFFIC*/ if (m_enablePQ) createAppUL (deviceINFORMATION_file, nodoDestino, iniSimulationTime, simulationTime,
																	m_percentageMetersPQ,m_packetIntervalMeanPQ, m_packetSizeMeanPQ, 4, m_edcaPQ, "Fixed");

	/* CLIENT DEMAND RESPONSE DATA TRAFFIC*/ if (m_enableDR) createAppDL (deviceINFORMATION_file, nodoDestino, iniSimulationTime, simulationTime, m_percentageMetersDR,
																				m_packetIntervalMeanDR, m_packetSizeMeanDR, 7, "Exponential");


	/* CLIENT ALARM DATA TRAFFIC*/ if (m_enableALARM) createAppUL (deviceINFORMATION_file, nodoDestino, iniSimulationTime, simulationTime, m_percentageMetersALARM,
																	m_packetIntervalMeanALARM, m_packetSizeMeanALARM, 6, m_edcaALARM, "Fixed");

	/* CLIENT BILLING DATA TRAFFIC*/ if (m_enableBD) createAppUL (deviceINFORMATION_file, nodoDestino, iniSimulationTime, simulationTime, m_percentageMetersBD,
																	m_packetIntervalMeanBD, m_packetSizeMeanBD, 5, m_edcaBD, "Fixed");


	////////////////////////////////////
	///////////////////////////////////
	///////////cambiar la configuracion despues del tiempo de convergencia

	//Simulator::Schedule(Seconds(stopConvergeTime), &configureParametersRun) ;
		//////////////////////////////////
	///////////////////////////////


	printSettings ();

//====Configure tracing

	traceSources();

	//===to trigger the channel utilization measurement
	for (uint32_t idNodo = 0; idNodo < numNodos; ++idNodo)
	{
		sendEvent=Simulator::Schedule(Seconds(1.01+idNodo*0.01), &CBRSample ,
				idNodo,  simulationTime) ;
	}

	//===print the samples
	for (uint32_t idNodo = 0; idNodo < numNodos; ++idNodo)
	{
		sendEvent=Simulator::Schedule(Seconds(1.01+idNodo*0.01), &printCBRSample ,
				idNodo,  simulationTime) ;
	}

	installFlowMonitor();

	Simulator::Stop (Seconds (simulationTime + 40));

	Simulator::Run ();

	getMetricsFlowMonitor();

	Simulator::Destroy ();


	mcsResultsFile.close();
	cbrFile.close();
	debugFile.close();

   return 0;
}
