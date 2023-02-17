/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Junling Bu <linlinjavaer@gmail.com>
 */
#include "ns3/command-line.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"


using namespace ns3;
/**
 * This simulation is to show the routing service of WaveNetDevice described in IEEE 09.4.
 *
 * note: although txPowerLevel is supported now, the "TxPowerLevels"
 * attribute of YansWifiPhy is 1 which means phy devices only support 1
 * levels. Thus, if users want to control txPowerLevel, they should set
 * these attributes of YansWifiPhy by themselves..
 */
class WaveNetDeviceExample
{
public:
  /// Send WSMP example function
  void SendWsmpExample (void);


private:

  /**
   * Send one WSMP packet function
   * \param channel the channel to use
   * \param seq the sequence
   */
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq);



  /**
   * Receive function
   * \param dev the device
   * \param pkt the packet
   * \param mode the mode
   * \param sender the sender address
   * \returns true if successful
   */
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);


  /// Create WAVE nodes function
  void CreateWaveNodes (void);


  NodeContainer nodes; ///< the nodes
  NetDeviceContainer devices; ///< the devices
};


void
WaveNetDeviceExample::CreateWaveNodes (void)
{

  //creating sender and receiver 
  nodes = NodeContainer ();
  nodes.Create (2);

  //installing mobility on the nodes since wifi nodes needs mobility to be installed
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);


  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (waveChannel.Create ());
  wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11); //Set the data link type of PCAP traces to be used. This function has to be called before EnablePcap(), so that the header of the pcap file can be written correctly. Here we set it to IEEE 802.11 Wireless LAN headers 
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  devices = waveHelper.Install (wavePhy, waveMac, nodes);

  for (uint32_t i = 0; i != devices.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i)); //we use DynamicCast to use a subclass of a base class
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));  //Set the callback to be used to notify higher layers when a packet has been received.

    }

  // Tracing
  wavePhy.EnablePcap ("wave-simple-device", devices);
}

bool
WaveNetDeviceExample::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  std::cout << "receive a packet: " << std::endl
            << "  reciever = " << dev->GetAddress() << "," << std::endl
            << "  sender = " << sender << "," << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().As (Time::S) << "," << std::endl
            << "  recvTime = " << Now ().As (Time::S) << "," << std::endl
            << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;
  return true;
}

void
WaveNetDeviceExample::SendOneWsmpPacket  (uint32_t channel, uint32_t seq)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;   //WSM packets received from the lower layers with an Ethernet Type of 0x88DC are delivered to the WSM protocol. 
  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();

  const TxInfo txInfo = TxInfo (channel);
  Ptr<Packet> p  = Create<Packet> (100);   //Create a packet with a zero-filled payload of size 100.
  SeqTsHeader seqTs;       //SeqTsHeader: Packet header to carry sequence number and timestamp
  seqTs.SetSeq (seq);      //Set the sequence number in the header
  p->AddHeader (seqTs);    //Add header to a packet
  sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
}

void
WaveNetDeviceExample::SendWsmpExample ()
{
  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);   //alternating access to CCH and SCH
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,sender,schInfo);
  // An important point is that the receiver should also be assigned channel
  // access for the same channel to receive packets.
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);

  // send WSMP packets
  // the first packet will be queued currently and be transmitted in next SCH interval
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH1, 1);
  // the second packet will be queued currently and then be transmitted , because of in the CCH interval.
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, 2);
  // the third packet will be dropped because of no channel access for SCH2.
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH2, 3);

  // release SCH access
  Simulator::Schedule (Seconds (2.0), &WaveNetDevice::StopSch, sender, SCH1);
  Simulator::Schedule (Seconds (2.0), &WaveNetDevice::StopSch, receiver, SCH1);
  // the fourth packet will be queued and be transmitted because of default CCH access assigned automatically.
  Simulator::Schedule (Seconds (3.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, 4);
  // the fifth packet will be dropped because of no SCH1 access assigned
  Simulator::Schedule (Seconds (3.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH1, 5);

  Simulator::Stop (Seconds (5.0));
  
  Simulator::Run ();
  Simulator::Destroy ();
}


int
main (int argc, char *argv[])
{
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);
  
  WaveNetDeviceExample example;
  std::cout << "run WAVE WSMP routing service case:" << std::endl;
  example.SendWsmpExample ();   //WAVE systems use a highly efficient messaging protocol known as WAVE Short Message Protocol (WSMP)


  return 0;
}