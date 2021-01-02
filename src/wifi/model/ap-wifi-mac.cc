/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006, 2009 INRIA
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
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Mirko Banchi <mk.banchi@gmail.com>
 */

#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "ns3/random-variable-stream.h"
#include "ap-wifi-mac.h"
#include "mac-low.h"
#include "mac-tx-middle.h"
#include "mac-rx-middle.h"
#include "mgt-headers.h"
#include "msdu-aggregator.h"
#include "amsdu-subframe-header.h"
#include "wifi-phy.h"
#include "wifi-net-device.h"
#include "ht-configuration.h"
#include "he-configuration.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ApWifiMac");

NS_OBJECT_ENSURE_REGISTERED (ApWifiMac);

TypeId
ApWifiMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ApWifiMac")
    .SetParent<InfrastructureWifiMac> ()
    .SetGroupName ("Wifi")
    .AddConstructor<ApWifiMac> ()
    .AddAttribute ("BeaconInterval",
                   "Delay between two beacons",
                   TimeValue (MicroSeconds (102400)),
                   MakeTimeAccessor (&ApWifiMac::GetBeaconInterval,
                                     &ApWifiMac::SetBeaconInterval),
                   MakeTimeChecker ())
    .AddAttribute ("CfpMaxDuration", "The maximum size of the CFP (used when AP supports PCF)",
                   TimeValue (MicroSeconds (51200)),
                   MakeTimeAccessor (&ApWifiMac::GetCfpMaxDuration,
                                     &ApWifiMac::SetCfpMaxDuration),
                   MakeTimeChecker ())
    .AddAttribute ("BeaconJitter",
                   "A uniform random variable to cause the initial beacon starting time (after simulation time 0) "
                   "to be distributed between 0 and the BeaconInterval.",
                   StringValue ("ns3::UniformRandomVariable"),
                   MakePointerAccessor (&ApWifiMac::m_beaconJitter),
                   MakePointerChecker<UniformRandomVariable> ())
    .AddAttribute ("EnableBeaconJitter",
                   "If beacons are enabled, whether to jitter the initial send event.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&ApWifiMac::m_enableBeaconJitter),
                   MakeBooleanChecker ())
    .AddAttribute ("BeaconGeneration",
                   "Whether or not beacons are generated.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ApWifiMac::SetBeaconGeneration),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableNonErpProtection", "Whether or not protection mechanism should be used when non-ERP STAs are present within the BSS."
                   "This parameter is only used when ERP is supported by the AP.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&ApWifiMac::m_enableNonErpProtection),
                   MakeBooleanChecker ())
  ;
  return tid;
}

ApWifiMac::ApWifiMac ()
  : m_enableBeaconGeneration (false)
{
  NS_LOG_FUNCTION (this);
  m_beaconTxop = CreateObject<Txop> ();
  m_beaconTxop->SetAifsn (1);
  m_beaconTxop->SetMinCw (0);
  m_beaconTxop->SetMaxCw (0);
  m_beaconTxop->SetMacLow (m_low);
  m_beaconTxop->SetChannelAccessManager (m_channelAccessManager);
  m_beaconTxop->SetTxMiddle (m_txMiddle);
  m_beaconTxop->SetTxOkCallback (MakeCallback (&ApWifiMac::TxOk, this));
  m_rxMiddle->SetPcfCallback (MakeCallback (&ApWifiMac::SendNextCfFrame, this));

  //Let the lower layers know that we are acting as an AP.
  SetTypeOfStation (AP);

  m_tfPacketDuration = 12932;
  m_muModeToStart = false;
  for (int ru = 0; ru < 9; ru++)
   {
     m_lowMu[ru]->SetKillTriggerFrameBeaconRetransmissionCallback (MakeCallback (&ApWifiMac::KillTriggerFrameBeaconRetransmission, this));
   }
  m_beaconTxop->SetTfAccessGrantCallback (MakeCallback (&ApWifiMac::TriggerFrameExpire, this));
  m_itCfPollingList = m_cfPollingList.begin ();
}

ApWifiMac::~ApWifiMac ()
{
  NS_LOG_FUNCTION (this);
  m_staList.clear ();
  m_nonErpStations.clear ();
  m_nonHtStations.clear ();
  m_cfPollingList.clear ();
}

void
ApWifiMac::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_beaconTxop->Dispose ();
  m_beaconTxop = 0;
  m_enableBeaconGeneration = false;
  m_beaconEvent.Cancel ();
  m_cfpEvent.Cancel ();
  RegularWifiMac::DoDispose ();
}

void
ApWifiMac::SetAddress (Mac48Address address)
{
  NS_LOG_FUNCTION (this << address);
  //As an AP, our MAC address is also the BSSID. Hence we are
  //overriding this function and setting both in our parent class.
  RegularWifiMac::SetAddress (address);
  RegularWifiMac::SetBssid (address);
}

void
ApWifiMac::SetBeaconGeneration (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  if (!enable)
    {
      m_beaconEvent.Cancel ();
    }
  else if (enable && !m_enableBeaconGeneration)
    {
      m_beaconEvent = Simulator::ScheduleNow (&ApWifiMac::SendOneBeacon, this);
    }
  m_enableBeaconGeneration = enable;
}

Time
ApWifiMac::GetBeaconInterval (void) const
{
  NS_LOG_FUNCTION (this);
  return m_low->GetBeaconInterval ();
}

Time
ApWifiMac::GetCfpMaxDuration (void) const
{
  NS_LOG_FUNCTION (this);
  return m_low->GetCfpMaxDuration ();
}

void
ApWifiMac::SetWifiRemoteStationManager (const Ptr<WifiRemoteStationManager> stationManager)
{
  NS_LOG_FUNCTION (this << stationManager);
  m_beaconTxop->SetWifiRemoteStationManager (stationManager);
  RegularWifiMac::SetWifiRemoteStationManager (stationManager);
  m_stationManager->SetPcfSupported (GetPcfSupported ());
}

void
ApWifiMac::SetLinkUpCallback (Callback<void> linkUp)
{
  NS_LOG_FUNCTION (this << &linkUp);
  RegularWifiMac::SetLinkUpCallback (linkUp);

  //The approach taken here is that, from the point of view of an AP,
  //the link is always up, so we immediately invoke the callback if
  //one is set
  linkUp ();
}

void
ApWifiMac::SetBeaconInterval (Time interval)
{
  NS_LOG_FUNCTION (this << interval);
  if ((interval.GetMicroSeconds () % 1024) != 0)
    {
      NS_FATAL_ERROR ("beacon interval should be multiple of 1024us (802.11 time unit), see IEEE Std. 802.11-2012");
    }
  if (interval.GetMicroSeconds () > (1024 * 65535))
    {
      NS_FATAL_ERROR ("beacon interval should be smaller then or equal to 65535 * 1024us (802.11 time unit)");
    }
  m_low->SetBeaconInterval (interval);
}

void
ApWifiMac::SetCfpMaxDuration (Time duration)
{
  NS_LOG_FUNCTION (this << duration);
  if ((duration.GetMicroSeconds () % 1024) != 0)
    {
      NS_LOG_WARN ("CFP max duration should be multiple of 1024us (802.11 time unit)");
    }
  m_low->SetCfpMaxDuration (duration);
}

int64_t
ApWifiMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_beaconJitter->SetStream (stream);
  return 1;
}

bool
ApWifiMac::GetShortSlotTimeEnabled (void) const
{
  if (m_nonErpStations.size () != 0)
    {
      return false;
    }
  if (GetErpSupported () && GetShortSlotTimeSupported ())
    {
      for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
        {
          if (!m_stationManager->GetShortSlotTimeSupported (i->second))
            {
              return false;
            }
        }
      return true;
    }
  return false;
}

bool
ApWifiMac::GetShortPreambleEnabled (void) const
{
  if (GetErpSupported () && m_phy->GetShortPhyPreambleSupported ())
    {
      for (std::list<Mac48Address>::const_iterator i = m_nonErpStations.begin (); i != m_nonErpStations.end (); i++)
        {
          if (!m_stationManager->GetShortPreambleSupported (*i))
            {
              return false;
            }
        }
      return true;
    }
  return false;
}

bool
ApWifiMac::IsNonGfHtStasPresent (void) const
{
  bool isNonGfHtStasPresent = false;
  for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
    {
      if (!m_stationManager->GetGreenfieldSupported (i->second))
        {
          isNonGfHtStasPresent = true;
          break;
        }
    }
  m_stationManager->SetUseGreenfieldProtection (isNonGfHtStasPresent);
  return isNonGfHtStasPresent;
}

uint16_t
ApWifiMac::GetVhtOperationalChannelWidth (void) const
{
  uint16_t channelWidth = m_phy->GetChannelWidth ();
  for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
    {
      if (m_stationManager->GetVhtSupported (i->second))
        {
          if (m_stationManager->GetChannelWidthSupported (i->second) < channelWidth)
            {
              channelWidth = m_stationManager->GetChannelWidthSupported (i->second);
            }
        }
    }
  return channelWidth;
}

void
ApWifiMac::ForwardDown (Ptr<Packet> packet, Mac48Address from,
                        Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << from << to);
  //If we are not a QoS AP then we definitely want to use AC_BE to
  //transmit the packet. A TID of zero will map to AC_BE (through \c
  //QosUtilsMapTidToAc()), so we use that as our default here.
  uint8_t tid = 0;

  //If we are a QoS AP then we attempt to get a TID for this packet
  if (GetQosSupported ())
    {
      tid = QosUtilsGetTidForPacket (packet);
      //Any value greater than 7 is invalid and likely indicates that
      //the packet had no QoS tag, so we revert to zero, which'll
      //mean that AC_BE is used.
      if (tid > 7)
        {
          tid = 0;
        }
    }

  ForwardDown (packet, from, to, tid);
}

void
ApWifiMac::ForwardDown (Ptr<Packet> packet, Mac48Address from,
                        Mac48Address to, uint8_t tid)
{
  NS_LOG_FUNCTION (this << packet << from << to << +tid);
  WifiMacHeader hdr;

  //For now, an AP that supports QoS does not support non-QoS
  //associations, and vice versa. In future the AP model should
  //support simultaneously associated QoS and non-QoS STAs, at which
  //point there will need to be per-association QoS state maintained
  //by the association state machine, and consulted here.
  if (GetQosSupported ())
    {
      hdr.SetType (WIFI_MAC_QOSDATA);
      hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
      hdr.SetQosNoEosp ();
      hdr.SetQosNoAmsdu ();
      //Transmission of multiple frames in the same Polled TXOP is not supported for now
      hdr.SetQosTxopLimit (0);
      //Fill in the QoS control field in the MAC header
      hdr.SetQosTid (tid);
    }
  else
    {
      hdr.SetType (WIFI_MAC_DATA);
    }

  if (GetQosSupported ())
    {
      hdr.SetNoOrder (); // explicitly set to 0 for the time being since HT control field is not yet implemented (set it to 1 when implemented)
    }
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (from);
  hdr.SetDsFrom ();
  hdr.SetDsNotTo ();

  if (GetQosSupported ())
    {
      //Sanity check that the TID is valid
      NS_ASSERT (tid < 8);
      if (m_muModeToStart)
       {
         RegularWifiMac::RUAllocations::const_iterator it;
         it = m_tfAlloc.find (to);
         if (it == m_tfAlloc.end ())
          {
            m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
          }
         else
          {
            m_edcaMu[it->second][QosUtilsMapTidToAc (tid)]->QueueButDontSend (packet, hdr);
          }
       }
      else if (!GetMuMode ())
       {
         m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
       }
      else 
       {
         RegularWifiMac::RUAllocations::const_iterator it;
         it = m_tfAlloc.find (to);
         if (it == m_tfAlloc.end ())
          {
            m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
          }
         else
          {
            m_channelAccessManagerMu[it->second]->UpdateBusyDuration (); // Removes BUSY state from dcfManagerMu
            m_edcaMu[it->second][QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
          }
       }
    }
  else
    {
      if (m_muModeToStart)
       {
         RegularWifiMac::RUAllocations::const_iterator it;
         it = m_tfAlloc.find (to);
         if (it == m_tfAlloc.end ())
          {
            m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
          }
         else
          {
            m_edcaMu[it->second][QosUtilsMapTidToAc (tid)]->QueueButDontSend (packet, hdr);
          }
       }
      else if (!GetMuMode ())
       {
         m_txop->Queue (packet, hdr);
       }
      else 
       {
         RegularWifiMac::RUAllocations::const_iterator it;
         it = m_tfAlloc.find (to);
         if (it == m_tfAlloc.end ())
          {
            m_txop->Queue (packet, hdr);
          }
         else
          {
            m_channelAccessManagerMu[it->second]->UpdateBusyDuration ();
            m_txopMu[it->second]->Queue (packet, hdr);
          }
       }
    }
}

void
ApWifiMac::Enqueue (Ptr<Packet> packet, Mac48Address to, Mac48Address from)
{
  NS_LOG_FUNCTION (this << packet << to << from);
  if (to.IsGroup () || m_stationManager->IsAssociated (to))
    {
      ForwardDown (packet, from, to);
    }
  else
    {
      NotifyTxDrop (packet);
    }
}

void
ApWifiMac::Enqueue (Ptr<Packet> packet, Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << to);
  //We're sending this packet with a from address that is our own. We
  //get that address from the lower MAC and make use of the
  //from-spoofing Enqueue() method to avoid duplicated code.
  Enqueue (packet, to, m_low->GetAddress ());
}

bool
ApWifiMac::SupportsSendFrom (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

SupportedRates
ApWifiMac::GetSupportedRates (void) const
{
  NS_LOG_FUNCTION (this);
  SupportedRates rates;
  //Send the set of supported rates and make sure that we indicate
  //the Basic Rate set in this set of supported rates.
  for (uint8_t i = 0; i < m_phy->GetNModes (); i++)
    {
      WifiMode mode = m_phy->GetMode (i);
      uint64_t modeDataRate = mode.GetDataRate (m_phy->GetChannelWidth ());
      NS_LOG_DEBUG ("Adding supported rate of " << modeDataRate);
      rates.AddSupportedRate (modeDataRate);
      //Add rates that are part of the BSSBasicRateSet (manufacturer dependent!)
      //here we choose to add the mandatory rates to the BSSBasicRateSet,
      //except for 802.11b where we assume that only the non HR-DSSS rates are part of the BSSBasicRateSet
      if (mode.IsMandatory () && (mode.GetModulationClass () != WIFI_MOD_CLASS_HR_DSSS))
        {
          NS_LOG_DEBUG ("Adding basic mode " << mode.GetUniqueName ());
          m_stationManager->AddBasicMode (mode);
        }
    }
  //set the basic rates
  for (uint8_t j = 0; j < m_stationManager->GetNBasicModes (); j++)
    {
      WifiMode mode = m_stationManager->GetBasicMode (j);
      uint64_t modeDataRate = mode.GetDataRate (m_phy->GetChannelWidth ());
      NS_LOG_DEBUG ("Setting basic rate " << mode.GetUniqueName ());
      rates.SetBasicRate (modeDataRate);
    }
  //If it is a HT AP, then add the BSSMembershipSelectorSet
  //The standard says that the BSSMembershipSelectorSet
  //must have its MSB set to 1 (must be treated as a Basic Rate)
  //Also the standard mentioned that at least 1 element should be included in the SupportedRates the rest can be in the ExtendedSupportedRates
  if (GetHtSupported ())
    {
      for (uint8_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
        {
          rates.AddBssMembershipSelectorRate (m_phy->GetBssMembershipSelector (i));
        }
    }
  return rates;
}

DsssParameterSet
ApWifiMac::GetDsssParameterSet (void) const
{
  NS_LOG_FUNCTION (this);
  DsssParameterSet dsssParameters;
  if (GetDsssSupported ())
    {
      dsssParameters.SetDsssSupported (1);
      dsssParameters.SetCurrentChannel (m_phy->GetChannelNumber ());
    }
  return dsssParameters;
}

CapabilityInformation
ApWifiMac::GetCapabilities (void) const
{
  NS_LOG_FUNCTION (this);
  CapabilityInformation capabilities;
  capabilities.SetShortPreamble (GetShortPreambleEnabled ());
  capabilities.SetShortSlotTime (GetShortSlotTimeEnabled ());
  capabilities.SetEss ();
  if (GetPcfSupported ())
    {
      capabilities.SetCfPollable ();
    }
  return capabilities;
}

ErpInformation
ApWifiMac::GetErpInformation (void) const
{
  NS_LOG_FUNCTION (this);
  ErpInformation information;
  information.SetErpSupported (1);
  if (GetErpSupported ())
    {
      information.SetNonErpPresent (!m_nonErpStations.empty ());
      information.SetUseProtection (GetUseNonErpProtection ());
      if (GetShortPreambleEnabled ())
        {
          information.SetBarkerPreambleMode (0);
        }
      else
        {
          information.SetBarkerPreambleMode (1);
        }
    }
  return information;
}

EdcaParameterSet
ApWifiMac::GetEdcaParameterSet (void) const
{
  NS_LOG_FUNCTION (this);
  EdcaParameterSet edcaParameters;
  if (GetQosSupported ())
    {
      edcaParameters.SetQosSupported (1);
      Ptr<QosTxop> edca;
      Time txopLimit;

      edca = m_edca.find (AC_BE)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetBeAci (0);
      edcaParameters.SetBeCWmin (edca->GetMinCw ());
      edcaParameters.SetBeCWmax (edca->GetMaxCw ());
      edcaParameters.SetBeAifsn (edca->GetAifsn ());
      edcaParameters.SetBeTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edca = m_edca.find (AC_BK)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetBkAci (1);
      edcaParameters.SetBkCWmin (edca->GetMinCw ());
      edcaParameters.SetBkCWmax (edca->GetMaxCw ());
      edcaParameters.SetBkAifsn (edca->GetAifsn ());
      edcaParameters.SetBkTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edca = m_edca.find (AC_VI)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetViAci (2);
      edcaParameters.SetViCWmin (edca->GetMinCw ());
      edcaParameters.SetViCWmax (edca->GetMaxCw ());
      edcaParameters.SetViAifsn (edca->GetAifsn ());
      edcaParameters.SetViTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edca = m_edca.find (AC_VO)->second;
      txopLimit = edca->GetTxopLimit ();
      edcaParameters.SetVoAci (3);
      edcaParameters.SetVoCWmin (edca->GetMinCw ());
      edcaParameters.SetVoCWmax (edca->GetMaxCw ());
      edcaParameters.SetVoAifsn (edca->GetAifsn ());
      edcaParameters.SetVoTxopLimit (static_cast<uint16_t> (txopLimit.GetMicroSeconds () / 32));

      edcaParameters.SetQosInfo (0);
    }
  return edcaParameters;
}

CfParameterSet
ApWifiMac::GetCfParameterSet (void) const
{
  CfParameterSet cfParameterSet;
  if (GetPcfSupported () && !m_cfPollingList.empty ())
    {
      cfParameterSet.SetPcfSupported (1);
      cfParameterSet.SetCFPCount (0);
      cfParameterSet.SetCFPPeriod (1);
      cfParameterSet.SetCFPMaxDurationUs (GetCfpMaxDuration ().GetMicroSeconds ());
      cfParameterSet.SetCFPDurRemainingUs (GetCfpMaxDuration ().GetMicroSeconds ());
    }
  return cfParameterSet;
}

HtOperation
ApWifiMac::GetHtOperation (void) const
{
  NS_LOG_FUNCTION (this);
  HtOperation operation;
  if (GetHtSupported ())
    {
      operation.SetHtSupported (1);
      operation.SetPrimaryChannel (m_phy->GetChannelNumber ());
      operation.SetRifsMode (false);
      operation.SetNonGfHtStasPresent (IsNonGfHtStasPresent ());
      if (m_phy->GetChannelWidth () > 20)
        {
          operation.SetSecondaryChannelOffset (1);
          operation.SetStaChannelWidth (1);
        }
      if (m_nonHtStations.empty ())
        {
          operation.SetHtProtection (NO_PROTECTION);
        }
      else
        {
          operation.SetHtProtection (MIXED_MODE_PROTECTION);
        }
      uint64_t maxSupportedRate = 0; //in bit/s
      for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
        {
          WifiMode mcs = m_phy->GetMcs (i);
          if (mcs.GetModulationClass () != WIFI_MOD_CLASS_HT)
            {
              continue;
            }
          uint8_t nss = (mcs.GetMcsValue () / 8) + 1;
          NS_ASSERT (nss > 0 && nss < 5);
          uint64_t dataRate = mcs.GetDataRate (m_phy->GetChannelWidth (), GetHtConfiguration ()->GetShortGuardIntervalSupported () ? 400 : 800, nss);
          if (dataRate > maxSupportedRate)
            {
              maxSupportedRate = dataRate;
              NS_LOG_DEBUG ("Updating maxSupportedRate to " << maxSupportedRate);
            }
        }
      uint8_t maxSpatialStream = m_phy->GetMaxSupportedTxSpatialStreams ();
      uint8_t nMcs = m_phy->GetNMcs ();
      for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
        {
          if (m_stationManager->GetHtSupported (i->second))
            {
              uint64_t maxSupportedRateByHtSta = 0; //in bit/s
              for (uint8_t j = 0; j < (std::min (nMcs, m_stationManager->GetNMcsSupported (i->second))); j++)
                {
                  WifiMode mcs = m_phy->GetMcs (j);
                  if (mcs.GetModulationClass () != WIFI_MOD_CLASS_HT)
                    {
                      continue;
                    }
                  uint8_t nss = (mcs.GetMcsValue () / 8) + 1;
                  NS_ASSERT (nss > 0 && nss < 5);
                  uint64_t dataRate = mcs.GetDataRate (m_stationManager->GetChannelWidthSupported (i->second), m_stationManager->GetShortGuardIntervalSupported (i->second) ? 400 : 800, nss);
                  if (dataRate > maxSupportedRateByHtSta)
                    {
                      maxSupportedRateByHtSta = dataRate;
                    }
                }
              if (maxSupportedRateByHtSta < maxSupportedRate)
                {
                  maxSupportedRate = maxSupportedRateByHtSta;
                }
              if (m_stationManager->GetNMcsSupported (i->second) < nMcs)
                {
                  nMcs = m_stationManager->GetNMcsSupported (i->second);
                }
              if (m_stationManager->GetNumberOfSupportedStreams (i->second) < maxSpatialStream)
                {
                  maxSpatialStream = m_stationManager->GetNumberOfSupportedStreams (i->second);
                }
            }
        }
      operation.SetRxHighestSupportedDataRate (static_cast<uint16_t> (maxSupportedRate / 1e6)); //in Mbit/s
      operation.SetTxMcsSetDefined (nMcs > 0);
      operation.SetTxMaxNSpatialStreams (maxSpatialStream);
      //To be filled in once supported
      operation.SetObssNonHtStasPresent (0);
      operation.SetDualBeacon (0);
      operation.SetDualCtsProtection (0);
      operation.SetStbcBeacon (0);
      operation.SetLSigTxopProtectionFullSupport (0);
      operation.SetPcoActive (0);
      operation.SetPhase (0);
      operation.SetRxMcsBitmask (0);
      operation.SetTxRxMcsSetUnequal (0);
      operation.SetTxUnequalModulation (0);
    }
  return operation;
}

VhtOperation
ApWifiMac::GetVhtOperation (void) const
{
  NS_LOG_FUNCTION (this);
  VhtOperation operation;
  if (GetVhtSupported ())
    {
      operation.SetVhtSupported (1);
      uint16_t channelWidth = GetVhtOperationalChannelWidth ();
      if (channelWidth == 160)
        {
          operation.SetChannelWidth (2);
        }
      else if (channelWidth == 80)
        {
          operation.SetChannelWidth (1);
        }
      else
        {
          operation.SetChannelWidth (0);
        }
      uint8_t maxSpatialStream = m_phy->GetMaxSupportedRxSpatialStreams ();
      for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
        {
          if (m_stationManager->GetVhtSupported (i->second))
            {
              if (m_stationManager->GetNumberOfSupportedStreams (i->second) < maxSpatialStream)
                {
                  maxSpatialStream = m_stationManager->GetNumberOfSupportedStreams (i->second);
                }
            }
        }
      for (uint8_t nss = 1; nss <= maxSpatialStream; nss++)
        {
          uint8_t maxMcs = 9; //TBD: hardcode to 9 for now since we assume all MCS values are supported
          operation.SetMaxVhtMcsPerNss (nss, maxMcs);
        }
    }
  return operation;
}

HeOperation
ApWifiMac::GetHeOperation (void) const
{
  NS_LOG_FUNCTION (this);
  HeOperation operation;
  if (GetHeSupported ())
    {
      operation.SetHeSupported (1);
      uint8_t maxSpatialStream = m_phy->GetMaxSupportedRxSpatialStreams ();
      for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
        {
          if (m_stationManager->GetHeSupported (i->second))
            {
              if (m_stationManager->GetNumberOfSupportedStreams (i->second) < maxSpatialStream)
                {
                  maxSpatialStream = m_stationManager->GetNumberOfSupportedStreams (i->second);
                }
            }
        }
      for (uint8_t nss = 1; nss <= maxSpatialStream; nss++)
        {
          operation.SetMaxHeMcsPerNss (nss, 11); //TBD: hardcode to 11 for now since we assume all MCS values are supported
        }
      UintegerValue bssColor;
      GetHeConfiguration ()->GetAttribute ("BssColor", bssColor);
      operation.SetBssColor (bssColor.Get ());
    }
  return operation;
}

void
ApWifiMac::SendProbeResp (Mac48Address to)
{
  NS_LOG_FUNCTION (this << to);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_PROBE_RESPONSE);
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtProbeResponseHeader probe;
  probe.SetSsid (GetSsid ());
  probe.SetSupportedRates (GetSupportedRates ());
  probe.SetBeaconIntervalUs (GetBeaconInterval ().GetMicroSeconds ());
  probe.SetCapabilities (GetCapabilities ());
  m_stationManager->SetShortPreambleEnabled (GetShortPreambleEnabled ());
  m_stationManager->SetShortSlotTimeEnabled (GetShortSlotTimeEnabled ());
  if (GetDsssSupported ())
    {
      probe.SetDsssParameterSet (GetDsssParameterSet ());
    }
  if (GetErpSupported ())
    {
      probe.SetErpInformation (GetErpInformation ());
    }
  if (GetQosSupported ())
    {
      probe.SetEdcaParameterSet (GetEdcaParameterSet ());
    }
  if (GetHtSupported ())
    {
      probe.SetExtendedCapabilities (GetExtendedCapabilities ());
      probe.SetHtCapabilities (GetHtCapabilities ());
      probe.SetHtOperation (GetHtOperation ());
      hdr.SetNoOrder ();
    }
  if (GetVhtSupported ())
    {
      probe.SetVhtCapabilities (GetVhtCapabilities ());
      probe.SetVhtOperation (GetVhtOperation ());
    }
  if (GetHeSupported ())
    {
      probe.SetHeCapabilities (GetHeCapabilities ());
      probe.SetHeOperation (GetHeOperation ());
    }
  packet->AddHeader (probe);

  //The standard is not clear on the correct queue for management
  //frames if we are a QoS AP. The approach taken here is to always
  //use the DCF for these regardless of whether we have a QoS
  //association or not.
  m_txop->Queue (packet, hdr);
}

void
ApWifiMac::SendAssocResp (Mac48Address to, bool success, bool isReassoc)
{
  NS_LOG_FUNCTION (this << to << success << isReassoc);
  WifiMacHeader hdr;
  hdr.SetType (isReassoc ? WIFI_MAC_MGT_REASSOCIATION_RESPONSE : WIFI_MAC_MGT_ASSOCIATION_RESPONSE);
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtAssocResponseHeader assoc;
  StatusCode code;
  if (success)
    {
      code.SetSuccess ();
      uint16_t aid = 0;
      bool found = false;
      if (isReassoc)
        {
          for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); ++i)
            {
              if (i->second == to)
                {
                  aid = i->first;
                  found = true;
                  break;
                }
            }
        }
      if (!found)
        {
          aid = GetNextAssociationId ();
          m_staList.insert (std::make_pair (aid, to));
        }
      assoc.SetAssociationId (aid);
    }
  else
    {
      code.SetFailure ();
    }
  assoc.SetSupportedRates (GetSupportedRates ());
  assoc.SetStatusCode (code);
  assoc.SetCapabilities (GetCapabilities ());
  if (GetErpSupported ())
    {
      assoc.SetErpInformation (GetErpInformation ());
    }
  if (GetQosSupported ())
    {
      assoc.SetEdcaParameterSet (GetEdcaParameterSet ());
    }
  if (GetHtSupported ())
    {
      assoc.SetExtendedCapabilities (GetExtendedCapabilities ());
      assoc.SetHtCapabilities (GetHtCapabilities ());
      assoc.SetHtOperation (GetHtOperation ());
      hdr.SetNoOrder ();
    }
  if (GetVhtSupported ())
    {
      assoc.SetVhtCapabilities (GetVhtCapabilities ());
      assoc.SetVhtOperation (GetVhtOperation ());
    }
  if (GetHeSupported ())
    {
      assoc.SetHeCapabilities (GetHeCapabilities ());
      assoc.SetHeOperation (GetHeOperation ());
    }
  packet->AddHeader (assoc);

  //The standard is not clear on the correct queue for management
  //frames if we are a QoS AP. The approach taken here is to always
  //use the DCF for these regardless of whether we have a QoS
  //association or not.
  m_txop->Queue (packet, hdr);
}

void
ApWifiMac::SendOneBeacon (void)
{
  NS_LOG_FUNCTION (this);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_BEACON);
  hdr.SetAddr1 (Mac48Address::GetBroadcast ());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
  MgtBeaconHeader beacon;
  beacon.SetSsid (GetSsid ());
  beacon.SetSupportedRates (GetSupportedRates ());
  beacon.SetBeaconIntervalUs (GetBeaconInterval ().GetMicroSeconds ());
  beacon.SetCapabilities (GetCapabilities ());
  m_stationManager->SetShortPreambleEnabled (GetShortPreambleEnabled ());
  m_stationManager->SetShortSlotTimeEnabled (GetShortSlotTimeEnabled ());
  if (GetPcfSupported ())
    {
      beacon.SetCfParameterSet (GetCfParameterSet ());
    }
  if (GetDsssSupported ())
    {
      beacon.SetDsssParameterSet (GetDsssParameterSet ());
    }
  if (GetErpSupported ())
    {
      beacon.SetErpInformation (GetErpInformation ());
    }
  if (GetQosSupported ())
    {
      beacon.SetEdcaParameterSet (GetEdcaParameterSet ());
    }
  if (GetHtSupported ())
    {
      beacon.SetExtendedCapabilities (GetExtendedCapabilities ());
      beacon.SetHtCapabilities (GetHtCapabilities ());
      beacon.SetHtOperation (GetHtOperation ());
      hdr.SetNoOrder ();
    }
  if (GetVhtSupported ())
    {
      beacon.SetVhtCapabilities (GetVhtCapabilities ());
      beacon.SetVhtOperation (GetVhtOperation ());
    }
  if (GetHeSupported ())
    {
      beacon.SetHeCapabilities (GetHeCapabilities ());
      beacon.SetHeOperation (GetHeOperation ());
    }
  packet->AddHeader (beacon);

  //The beacon has it's own special queue, so we load it in there
  m_beaconTxop->Queue (packet, hdr);
  m_beaconEvent = Simulator::Schedule (GetBeaconInterval (), &ApWifiMac::SendOneBeacon, this);

  //If a STA that does not support Short Slot Time associates,
  //the AP shall use long slot time beginning at the first Beacon
  //subsequent to the association of the long slot time STA.
  if (GetErpSupported ())
    {
      if (GetShortSlotTimeEnabled () == true)
        {
          //Enable short slot time
          m_phy->SetSlot (MicroSeconds (9));
        }
      else
        {
          //Disable short slot time
          m_phy->SetSlot (MicroSeconds (20));
        }
    }
}

void
ApWifiMac::KillTriggerFrameBeaconRetransmission (void)
{
  /*
   * This function will be called by the PHY layer 
   * as soon as the AP starts receiving a TF Response
   * In this case, a Trigger Frame needs to be sent out
   * with allocations corresponding to STAs who were
   * successful in sending out their BSRs.
   */
  if (!m_tfBeaconExpire.IsExpired ())
   {
     std::cout<<"Canceling TF retransmission because at least one STA transmitted, time = "<<Simulator::Now ().GetMicroSeconds () << std::endl;	//Xyct: The two below is NOT commented out by me
     //m_tfBeaconExpire.Cancel ();
     //m_cancelEvent.Cancel ();
     Time timeToExpire = MicroSeconds (440) + m_low->GetSifs ();
     std::cout<<"Collision canceling scheduled at "<<(Now () + timeToExpire).GetMicroSeconds () << std::endl;
   }
}

void
ApWifiMac::TriggerFrameExpire ()
{
   m_muModeToStart = false;
   /*
    * At this point TF has gained access to the medium, now declare the 20 MHz channelAccessManager busy until MU mode ends
    */
   Time timeToExpire = CalculateCurrentTfDuration () + m_low->GetSifs () + m_low->GetSlotTime (); // Difs
   //m_tfBeaconExpire = Simulator::Schedule (timeToExpire, &ApWifiMac::StartMuModeUplink, this);
   std::cout<<"DIFS canceling scheduled at "<<(Now () + timeToExpire).GetMicroSeconds () << std::endl;
   m_channelAccessManager->NotifyMaybeCcaBusyStartNow (GetTfDuration () * m_low->GetSlotTime ()); // Tell the 20 MHz channelAccessManager that you are busy until MU mode ends  
   m_muModeExpireEvent = Simulator::Schedule (GetTfDuration () * m_low->GetSlotTime (), &ApWifiMac::StopMuMode, this);
   if (!m_muUlFlag) // Don't start MU Mode at AP if this is a UL TF
    {
      Simulator::Schedule (MicroSeconds (200), &ApWifiMac::StartMuModeDelayed, this); // Hack: The TF takes 200 microseconds to reach the RX, so start MU mode only after TF reaches STAs
    }
}

void
ApWifiMac::SendNextCfFrame (void)
{
  if (!GetPcfSupported ())
    {
      return;
    }
  if (m_txop->CanStartNextPolling ())
    {
      SendCfPoll ();
    }
  else if (m_low->IsCfPeriod ())
    {
      SendCfEnd ();
    }
}

void
ApWifiMac::StartMuModeDelayed (void)
{
  SetMuMode (1);
  for (RegularWifiMac::RUAllocations::iterator it = m_tfAlloc.begin (); it!=m_tfAlloc.end (); ++it)
   {
     m_channelAccessManagerMu[it->second]->UpdateBusyDuration ();
     for (uint32_t ac = 0; ac < 8; ac++)
      {
        m_edcaMu[it->second][QosUtilsMapTidToAc(ac)]->StartAccessIfNeeded ();
      }
     m_channelAccessManagerMu[it->second]->DoRestartAccessTimeoutIfNeeded ();
   }
}

void 
ApWifiMac::SendBsrAck (Mac48Address to, uint32_t ru)
{
  NS_LOG_FUNCTION (this);

  WifiMacHeader hdr;
  hdr.SetBsrAck ();
  hdr.SetAddr1 (to);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();

  MgtBsrAckHeader bsrAck;
  bsrAck.SetRu (ru);
  packet->AddHeader (bsrAck);

  std::cout<<"Queueing BSR ACK, time = "<<Simulator::Now ().GetMicroSeconds () << std::endl;  
  m_edcaMu[ru][AC_BE]->SetAifsn (0);
  m_edcaMu[ru][AC_BE]->SetMinCw (0);
  m_edcaMu[ru][AC_BE]->SetMaxCw (0);
  m_edcaMu[ru][AC_BE]->Queue (packet, hdr);
  m_channelAccessManagerMu[ru]->UpdateBusyDuration ();
  m_edcaMu[ru][AC_BE]->StartAccessIfNeeded ();
  m_channelAccessManagerMu[ru]->DoRestartAccessTimeoutIfNeeded ();
}

void
ApWifiMac::SendTriggerFrame(bool flag)
{
  NS_LOG_FUNCTION (this);
  /*
   * The beaconDca is used to send TF beacon and TF, so the below 
   * parameters do not apply for TF and TF beacon; the Tx will start 
   * 25 microseconds after scheduling because of Sifs and 1 Slot. 
   * The below parameters are applicable for DL data packets
   */
  if (m_tfPacketDuration == 0)
   {
     if (!flag)
      {
        std::cout << "No data to send in DL, not sending TF\n";
        Simulator::ScheduleNow (&ApWifiMac::StopMuMode, this);
        return; 
      }
   }
  if (!flag)
   {
    for (uint32_t ru = 0; ru < 9; ru++)
     {
       m_txopMu[ru]->SetAifsn (0);
       m_txopMu[ru]->SetMinCw (0);
       m_txopMu[ru]->SetMaxCw (0);
       for (uint32_t ac = 0; ac < 8; ac++)
        {
	  m_edcaMu[ru][QosUtilsMapTidToAc(ac)]->SetAifsn (0);
	  m_edcaMu[ru][QosUtilsMapTidToAc(ac)]->SetMinCw (0);
	  m_edcaMu[ru][QosUtilsMapTidToAc(ac)]->SetMaxCw (0);
        }
     }
   }
  m_muModeToStart = true;
  WifiMacHeader hdr;
  hdr.SetTriggerFrame ();
  hdr.SetAddr1 (Mac48Address::GetBroadcast());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Ptr<Packet> packet = Create<Packet> ();
 
  CalculateTfDuration ();
  MgtTFHeader tf;
  tf.SetNoSta (m_tfAlloc.size ());
  tf.SetRUAllocations (m_tfAlloc);
  tf.SetUplinkFlag (flag);
  tf.SetTfDuration (GetTfDuration ());
  packet->AddHeader (tf);
  //packet->Print(std::cout);
  std::cout<<"Sending TF, time = "<< Simulator::Now ().GetMicroSeconds ()<<std::endl;
  std::cout<<"Number of STAs allocated = " << m_tfAlloc.size () <<std::endl;
  std::cout<<"Number of STAs associated = " << m_staList.size () <<std::endl;
  for (RegularWifiMac::RUAllocations::iterator it = m_tfAlloc.begin (); it!=m_tfAlloc.end (); ++it)
   {
	std::cout <<"STA = "<<it->first <<"\t RU = "<<it->second<<std::endl;
   }
  m_beaconTxop->Queue (packet, hdr);
  m_channelAccessManager->UpdateBusyDuration ();
  //m_beaconTxop->StartAccessIfNeeded ();//Xyct: because queue calls this
  m_channelAccessManager->DoRestartAccessTimeoutIfNeeded ();
}


void
ApWifiMac::StartMuModeUplink (void) //starts the OFDMA based MU Mode
{
  NS_ASSERT (m_triggerFrameUplinkEvent.IsExpired ());
  m_muUlFlag = 1;
  RegularWifiMac::RUAllocations alloc;
  uint32_t j = 0;
  uint32_t n_sc = GetNScheduled ();
  for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
   {
     if (j >= n_sc)
      {
	alloc.insert (std::pair<Mac48Address, uint32_t>(Mac48Address::GetBroadcast(), 9-n_sc));        
      }
     else 
      {
        alloc.insert (std::pair<Mac48Address, uint32_t>(i->second, j%9)); 
      }
     j++;
     if (j == 9)
      {
        break;
      }
   }
  m_tfAlloc = alloc;
  m_triggerFrameUplinkEvent = Simulator::ScheduleNow (&ApWifiMac::SendTriggerFrame, this, m_muUlFlag);
}


void
ApWifiMac::StartMuModeDownlink (void) //starts OFDMA mode in DL
{
  NS_ASSERT (m_triggerFrameDownlinkEvent.IsExpired ());
  m_muUlFlag = 0;
  RegularWifiMac::RUAllocations alloc;
  uint32_t j = 0;
    for (std::map<uint16_t, Mac48Address>::const_iterator i = m_staList.begin (); i != m_staList.end (); i++)
      {
	alloc.insert (std::pair<Mac48Address, uint32_t>(i->second, j%9)); // Hack: It does not matter which 9 users are scheduled, so always schedule first 9 
        j++;
        if (j == 9)
         {
           break;
         }
      }
  m_tfAlloc = alloc;
  uint32_t temp[9];
  uint32_t max_data = 0;
  for (uint32_t i = 0; i < 9; i++)
    {
      m_phyMu[i]->SetMuMode (1); //Just making sure that all MU PHYs are in the MU Mode
      temp[i] = m_low->CalculateApPayloadDuration (); // The data is buffered initially into regular queues, so using m_low instead of m_lowMu. Must fix after the same queue is used
    }
  for (uint32_t i = 0; i < 9; i++)
    {
      if (temp[i] > max_data)
       {
         max_data = temp[i];
       }
    }
  m_tfPacketDuration = max_data;
  CalculateTfDuration ();
  m_triggerFrameDownlinkEvent = Simulator::ScheduleNow (&ApWifiMac::SendTriggerFrame, this, m_muUlFlag); //Downlink TF
}

void
ApWifiMac::SendCfPoll (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (GetPcfSupported ());
  m_txop->SendCfFrame (WIFI_MAC_DATA_NULL_CFPOLL, *m_itCfPollingList);
}

void
ApWifiMac::SendCfEnd (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (GetPcfSupported ());
  m_txop->SendCfFrame (WIFI_MAC_CTL_END, Mac48Address::GetBroadcast ());
}

void
ApWifiMac::TxOk (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this);
  RegularWifiMac::TxOk (hdr);
  if ((hdr.IsAssocResp () || hdr.IsReassocResp ())
      && m_stationManager->IsWaitAssocTxOk (hdr.GetAddr1 ()))
    {
      NS_LOG_DEBUG ("associated with sta=" << hdr.GetAddr1 ());
      m_stationManager->RecordGotAssocTxOk (hdr.GetAddr1 ());
    }
  else if (hdr.IsBeacon () && GetPcfSupported ())
    {
      if (!m_cfPollingList.empty ())
        {
          SendCfPoll ();
        }
      else
        {
          SendCfEnd ();
        }
    }
  else if (hdr.IsCfPoll ())
    {
      IncrementPollingListIterator ();
    }
}

void
ApWifiMac::TxFailed (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this);
  RegularWifiMac::TxFailed (hdr);

  if ((hdr.IsAssocResp () || hdr.IsReassocResp ())
      && m_stationManager->IsWaitAssocTxOk (hdr.GetAddr1 ()))
    {
      NS_LOG_DEBUG ("association failed with sta=" << hdr.GetAddr1 ());
      m_stationManager->RecordGotAssocTxFailed (hdr.GetAddr1 ());
    }
  else if (hdr.IsCfPoll ())
    {
      IncrementPollingListIterator ();
      SendNextCfFrame ();
    }
}

Time 
ApWifiMac::CalculateCurrentTfDuration (void)
{
  Ptr<Packet> trigger_frame = Create<Packet> ();
  MgtTFHeader tf;
  tf.SetNoSta (m_tfAlloc.size ());
  tf.SetRUAllocations (m_tfAlloc);
  tf.SetUplinkFlag (m_muUlFlag);
  tf.SetTfDuration (GetTfDuration ());
  trigger_frame->AddHeader (tf); 
  WifiMacHeader hdr;
  hdr.SetTriggerFrame ();
  hdr.SetAddr1 (Mac48Address::GetBroadcast());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  return m_low->CalculateTfBeaconDuration (trigger_frame, hdr);
}

void
ApWifiMac::CalculateTfDuration (void)
{
  Ptr<Packet> trigger_frame = Create<Packet> ();
  MgtTFHeader tf;
  tf.SetNoSta (m_tfAlloc.size ());
  tf.SetRUAllocations (m_tfAlloc);
  tf.SetUplinkFlag (m_muUlFlag);
  tf.SetTfDuration (GetTfDuration ());
  trigger_frame->AddHeader (tf); 
  WifiMacHeader hdr;
  hdr.SetTriggerFrame ();
  hdr.SetAddr1 (Mac48Address::GetBroadcast());
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  Time t1 = m_low->CalculateTfBeaconDuration (trigger_frame, hdr);
  Time t2 = MicroSeconds (m_tfPacketDuration);
  Time total;
  if (m_tfPacketDuration > 0)
   {
     if (GetNScheduled () < 9)
      {
     	total = t1 + m_low->GetSifs () + MicroSeconds (440) + m_low->GetSifs () + MicroSeconds (416) + m_low->GetSifs () + t2 + m_low->GetSifs () + MicroSeconds (224); //164: time taken for ACK
      }
     else if (GetNScheduled () == 9)
      {
	total = t1 + m_low->GetSifs () + t2 + m_low->GetSifs () + MicroSeconds (224);
      }
     // AP ----TF----> STA ----TFResp----> AP ----BSRACK----> STA ----Payload----> AP ----ACK----> STA
   }
  else
   { //Xyct: GetSifs() compiles error 
     total = t1 + m_low->GetSifs ();
   }
  std::cout<<"t1 = "<<t1.GetMicroSeconds () << std::endl;
  std::cout<<"t2 = "<<t2.GetMicroSeconds () << std::endl;
  std::cout<<"total = "<<total.GetMicroSeconds () << std::endl;
  uint32_t tfDuration = total.GetMicroSeconds () / m_low->GetSlotTime ().GetMicroSeconds ();
  std::cout<<"tfDuration = "<<tfDuration + 1 << std::endl;
  SetTfDuration (tfDuration + 1);
}

void
ApWifiMac::Receive (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  const WifiMacHeader* hdr = &mpdu->GetHeader ();
  Ptr<const Packet> packet = mpdu->GetPacket ();
  Mac48Address from = hdr->GetAddr2 ();
  if (hdr->IsData ())
    {
      Mac48Address bssid = hdr->GetAddr1 ();
      if (!hdr->IsFromDs ()
          && hdr->IsToDs ()
          && bssid == GetAddress ()
          && m_stationManager->IsAssociated (from))
        {
          Mac48Address to = hdr->GetAddr3 ();
          if (to == GetAddress ())
            {
              NS_LOG_DEBUG ("frame for me from=" << from);
              if (hdr->IsQosData ())
                {
                  if (hdr->IsQosAmsdu ())
                    {
                      NS_LOG_DEBUG ("Received A-MSDU from=" << from << ", size=" << packet->GetSize ());
                      DeaggregateAmsduAndForward (mpdu);
                      packet = 0;
                    }
                  else
                    {
                      ForwardUp (packet, from, bssid);
                    }
                }
              else if (hdr->HasData ())
                {
                  ForwardUp (packet, from, bssid);
                }
            }
          else if (to.IsGroup ()
                   || m_stationManager->IsAssociated (to))
            {
              NS_LOG_DEBUG ("forwarding frame from=" << from << ", to=" << to);
              Ptr<Packet> copy = packet->Copy ();

              //If the frame we are forwarding is of type QoS Data,
              //then we need to preserve the UP in the QoS control
              //header...
              if (hdr->IsQosData ())
                {
                  ForwardDown (copy, from, to, hdr->GetQosTid ());
                }
              else
                {
                  ForwardDown (copy, from, to);
                }
              ForwardUp (packet, from, to);
            }
          else
            {
              ForwardUp (packet, from, to);
            }
        }
      else if (hdr->IsFromDs ()
               && hdr->IsToDs ())
        {
          //this is an AP-to-AP frame
          //we ignore for now.
          NotifyRxDrop (packet);
        }
      else
        {
          //we can ignore these frames since
          //they are not targeted at the AP
          NotifyRxDrop (packet);
        }
      return;
    }
  else if (hdr->IsMgt ())
    {
      if (hdr->IsTFResponse ())
        {
          MgtTFRespHeader resp;
      Ptr<Packet> copy = packet->Copy ();
          copy->RemoveHeader (resp);
          Simulator::ScheduleNow (&ApWifiMac::SendBsrAck, this, from, resp.GetRu ());
        }
      else if (hdr->IsProbeReq ())
        {
          NS_ASSERT (hdr->GetAddr1 ().IsBroadcast ());
          MgtProbeRequestHeader probeRequestHeader;
          packet->PeekHeader (probeRequestHeader);
          Ssid ssid = probeRequestHeader.GetSsid ();
          if (ssid == GetSsid () || ssid.IsBroadcast ())
            {
              NS_LOG_DEBUG ("Probe request received from " << from << ": send probe response");
              SendProbeResp (from);
            }
          return;
        }
      else if (hdr->GetAddr1 () == GetAddress ())
        {
          if (hdr->IsAssocReq ())
            {
              NS_LOG_DEBUG ("Association request received from " << from);
              //first, verify that the the station's supported
              //rate set is compatible with our Basic Rate set
              MgtAssocRequestHeader assocReq;
              packet->PeekHeader (assocReq);
              CapabilityInformation capabilities = assocReq.GetCapabilities ();
              m_stationManager->AddSupportedPhyPreamble (from, capabilities.IsShortPreamble ());
              SupportedRates rates = assocReq.GetSupportedRates ();
              bool problem = false;
              bool isHtStation = false;
              bool isOfdmStation = false;
              bool isErpStation = false;
              bool isDsssStation = false;
              for (uint8_t i = 0; i < m_stationManager->GetNBasicModes (); i++)
                {
                  WifiMode mode = m_stationManager->GetBasicMode (i);
                  if (!rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
                    {
                      if ((mode.GetModulationClass () == WIFI_MOD_CLASS_DSSS) || (mode.GetModulationClass () == WIFI_MOD_CLASS_HR_DSSS))
                        {
                          isDsssStation = false;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_ERP_OFDM)
                        {
                          isErpStation = false;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_OFDM)
                        {
                          isOfdmStation = false;
                        }
                      if (isDsssStation == false && isErpStation == false && isOfdmStation == false)
                        {
                          problem = true;
                          break;
                        }
                    }
                  else
                    {
                      if ((mode.GetModulationClass () == WIFI_MOD_CLASS_DSSS) || (mode.GetModulationClass () == WIFI_MOD_CLASS_HR_DSSS))
                        {
                          isDsssStation = true;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_ERP_OFDM)
                        {
                          isErpStation = true;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_OFDM)
                        {
                          isOfdmStation = true;
                        }
                    }
                }
              m_stationManager->AddSupportedErpSlotTime (from, capabilities.IsShortSlotTime () && isErpStation);
              if (GetHtSupported ())
                {
                  //check whether the HT STA supports all MCSs in Basic MCS Set
                  HtCapabilities htcapabilities = assocReq.GetHtCapabilities ();
                  if (htcapabilities.IsSupportedMcs (0))
                    {
                      isHtStation = true;
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!htcapabilities.IsSupportedMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetVhtSupported ())
                {
                  //check whether the VHT STA supports all MCSs in Basic MCS Set
                  VhtCapabilities vhtcapabilities = assocReq.GetVhtCapabilities ();
                  if (vhtcapabilities.GetVhtCapabilitiesInfo () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!vhtcapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetHeSupported ())
                {
                  //check whether the HE STA supports all MCSs in Basic MCS Set
                  HeCapabilities hecapabilities = assocReq.GetHeCapabilities ();
                  if (hecapabilities.GetSupportedMcsAndNss () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!hecapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (problem)
                {
                  NS_LOG_DEBUG ("One of the Basic Rate set mode is not supported by the station: send association response with an error status");
                  SendAssocResp (hdr->GetAddr2 (), false, false);
                }
              else
                {
                  NS_LOG_DEBUG ("The Basic Rate set modes are supported by the station");
                  //record all its supported modes in its associated WifiRemoteStation
                  for (uint8_t j = 0; j < m_phy->GetNModes (); j++)
                    {
                      WifiMode mode = m_phy->GetMode (j);
                      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
                        {
                          m_stationManager->AddSupportedMode (from, mode);
                        }
                    }
                  if (GetPcfSupported () && capabilities.IsCfPollable ())
                    {
                      m_cfPollingList.push_back (from);
                      if (m_itCfPollingList == m_cfPollingList.end ())
                        {
                          IncrementPollingListIterator ();
                        }
                    }
                  if (GetHtSupported ())
                    {
                      HtCapabilities htCapabilities = assocReq.GetHtCapabilities ();
                      if (htCapabilities.IsSupportedMcs (0))
                        {
                          m_stationManager->AddStationHtCapabilities (from, htCapabilities);
                        }
                    }
                  if (GetVhtSupported ())
                    {
                      VhtCapabilities vhtCapabilities = assocReq.GetVhtCapabilities ();
                      //we will always fill in RxHighestSupportedLgiDataRate field at TX, so this can be used to check whether it supports VHT
                      if (vhtCapabilities.GetRxHighestSupportedLgiDataRate () > 0)
                        {
                          m_stationManager->AddStationVhtCapabilities (from, vhtCapabilities);
                          for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
                            {
                              WifiMode mcs = m_phy->GetMcs (i);
                              if (mcs.GetModulationClass () == WIFI_MOD_CLASS_VHT && vhtCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  if (GetHtSupported ())
                    {
                      ExtendedCapabilities extendedCapabilities = assocReq.GetExtendedCapabilities ();
                      //TODO: to be completed
                    }
                  if (GetHeSupported ())
                    {
                      HeCapabilities heCapabilities = assocReq.GetHeCapabilities ();
                      if (heCapabilities.GetSupportedMcsAndNss () != 0)
                        {
                          m_stationManager->AddStationHeCapabilities (from, heCapabilities);
                          for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
                            {
                              WifiMode mcs = m_phy->GetMcs (i);
                              if (mcs.GetModulationClass () == WIFI_MOD_CLASS_HE && heCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  m_stationManager->RecordWaitAssocTxOk (from);
                  if (!isHtStation)
                    {
                      m_nonHtStations.push_back (hdr->GetAddr2 ());
                      m_nonHtStations.unique ();
                    }
                  if (!isErpStation && isDsssStation)
                    {
                      m_nonErpStations.push_back (hdr->GetAddr2 ());
                      m_nonErpStations.unique ();
                    }
                  NS_LOG_DEBUG ("Send association response with success status");
                  SendAssocResp (hdr->GetAddr2 (), true, false);
                }
              return;
            }
          else if (hdr->IsReassocReq ())
            {
              NS_LOG_DEBUG ("Reassociation request received from " << from);
              //first, verify that the the station's supported
              //rate set is compatible with our Basic Rate set
              MgtReassocRequestHeader reassocReq;
              packet->PeekHeader (reassocReq);
              CapabilityInformation capabilities = reassocReq.GetCapabilities ();
              m_stationManager->AddSupportedPhyPreamble (from, capabilities.IsShortPreamble ());
              SupportedRates rates = reassocReq.GetSupportedRates ();
              bool problem = false;
              bool isHtStation = false;
              bool isOfdmStation = false;
              bool isErpStation = false;
              bool isDsssStation = false;
              for (uint8_t i = 0; i < m_stationManager->GetNBasicModes (); i++)
                {
                  WifiMode mode = m_stationManager->GetBasicMode (i);
                  if (!rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
                    {
                      if ((mode.GetModulationClass () == WIFI_MOD_CLASS_DSSS) || (mode.GetModulationClass () == WIFI_MOD_CLASS_HR_DSSS))
                        {
                          isDsssStation = false;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_ERP_OFDM)
                        {
                          isErpStation = false;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_OFDM)
                        {
                          isOfdmStation = false;
                        }
                      if (isDsssStation == false && isErpStation == false && isOfdmStation == false)
                        {
                          problem = true;
                          break;
                        }
                    }
                  else
                    {
                      if ((mode.GetModulationClass () == WIFI_MOD_CLASS_DSSS) || (mode.GetModulationClass () == WIFI_MOD_CLASS_HR_DSSS))
                        {
                          isDsssStation = true;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_ERP_OFDM)
                        {
                          isErpStation = true;
                        }
                      else if (mode.GetModulationClass () == WIFI_MOD_CLASS_OFDM)
                        {
                          isOfdmStation = true;
                        }
                    }
                }
              m_stationManager->AddSupportedErpSlotTime (from, capabilities.IsShortSlotTime () && isErpStation);
              if (GetHtSupported ())
                {
                  //check whether the HT STA supports all MCSs in Basic MCS Set
                  HtCapabilities htcapabilities = reassocReq.GetHtCapabilities ();
                  if (htcapabilities.IsSupportedMcs (0))
                    {
                      isHtStation = true;
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!htcapabilities.IsSupportedMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetVhtSupported ())
                {
                  //check whether the VHT STA supports all MCSs in Basic MCS Set
                  VhtCapabilities vhtcapabilities = reassocReq.GetVhtCapabilities ();
                  if (vhtcapabilities.GetVhtCapabilitiesInfo () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!vhtcapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (GetHeSupported ())
                {
                  //check whether the HE STA supports all MCSs in Basic MCS Set
                  HeCapabilities hecapabilities = reassocReq.GetHeCapabilities ();
                  if (hecapabilities.GetSupportedMcsAndNss () != 0)
                    {
                      for (uint8_t i = 0; i < m_stationManager->GetNBasicMcs (); i++)
                        {
                          WifiMode mcs = m_stationManager->GetBasicMcs (i);
                          if (!hecapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                            {
                              problem = true;
                              break;
                            }
                        }
                    }
                }
              if (problem)
                {
                  NS_LOG_DEBUG ("One of the Basic Rate set mode is not supported by the station: send reassociation response with an error status");
                  SendAssocResp (hdr->GetAddr2 (), false, true);
                }
              else
                {
                  NS_LOG_DEBUG ("The Basic Rate set modes are supported by the station");
                  //update all its supported modes in its associated WifiRemoteStation
                  for (uint8_t j = 0; j < m_phy->GetNModes (); j++)
                    {
                      WifiMode mode = m_phy->GetMode (j);
                      if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth ())))
                        {
                          m_stationManager->AddSupportedMode (from, mode);
                        }
                    }
                  if (GetHtSupported ())
                    {
                      HtCapabilities htCapabilities = reassocReq.GetHtCapabilities ();
                      if (htCapabilities.IsSupportedMcs (0))
                        {
                          m_stationManager->AddStationHtCapabilities (from, htCapabilities);
                        }
                    }
                  if (GetVhtSupported ())
                    {
                      VhtCapabilities vhtCapabilities = reassocReq.GetVhtCapabilities ();
                      //we will always fill in RxHighestSupportedLgiDataRate field at TX, so this can be used to check whether it supports VHT
                      if (vhtCapabilities.GetRxHighestSupportedLgiDataRate () > 0)
                        {
                          m_stationManager->AddStationVhtCapabilities (from, vhtCapabilities);
                          for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
                            {
                              WifiMode mcs = m_phy->GetMcs (i);
                              if (mcs.GetModulationClass () == WIFI_MOD_CLASS_VHT && vhtCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  if (GetHtSupported ())
                    {
                      ExtendedCapabilities extendedCapabilities = reassocReq.GetExtendedCapabilities ();
                      //TODO: to be completed
                    }
                  if (GetHeSupported ())
                    {
                      HeCapabilities heCapabilities = reassocReq.GetHeCapabilities ();
                      if (heCapabilities.GetSupportedMcsAndNss () != 0)
                        {
                          m_stationManager->AddStationHeCapabilities (from, heCapabilities);
                          for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
                            {
                              WifiMode mcs = m_phy->GetMcs (i);
                              if (mcs.GetModulationClass () == WIFI_MOD_CLASS_HE && heCapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                                {
                                  m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                                  //here should add a control to add basic MCS when it is implemented
                                }
                            }
                        }
                    }
                  m_stationManager->RecordWaitAssocTxOk (from);
                  if (!isHtStation)
                    {
                      m_nonHtStations.push_back (hdr->GetAddr2 ());
                      m_nonHtStations.unique ();
                    }
                  if (!isErpStation && isDsssStation)
                    {
                      m_nonErpStations.push_back (hdr->GetAddr2 ());
                      m_nonErpStations.unique ();
                    }
                  NS_LOG_DEBUG ("Send reassociation response with success status");
                  SendAssocResp (hdr->GetAddr2 (), true, true);
                }
              return;
            }
          else if (hdr->IsDisassociation ())
            {
              NS_LOG_DEBUG ("Disassociation received from " << from);
              m_stationManager->RecordDisassociated (from);
              for (std::map<uint16_t, Mac48Address>::const_iterator j = m_staList.begin (); j != m_staList.end (); j++)
                {
                  if (j->second == from)
                    {
                      m_staList.erase (j);
                      break;
                    }
                }
              for (std::list<Mac48Address>::const_iterator j = m_nonErpStations.begin (); j != m_nonErpStations.end (); j++)
                {
                  if ((*j) == from)
                    {
                      m_nonErpStations.erase (j);
                      break;
                    }
                }
              for (std::list<Mac48Address>::const_iterator j = m_nonHtStations.begin (); j != m_nonHtStations.end (); j++)
                {
                  if ((*j) == from)
                    {
                      m_nonHtStations.erase (j);
                      break;
                    }
                }
              for (std::list<Mac48Address>::const_iterator j = m_cfPollingList.begin (); j != m_cfPollingList.end (); ++j)
                {
                  if ((*j) == from)
                    {
                      m_cfPollingList.erase (j);
                      break;
                    }
                }
              return;
            }
        }
    }

  //Invoke the receive handler of our parent class to deal with any
  //other frames. Specifically, this will handle Block Ack-related
  //Management Action frames.
  RegularWifiMac::Receive (Create<WifiMacQueueItem> (packet, *hdr));
}

void
ApWifiMac::DeaggregateAmsduAndForward (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  for (auto& i : *PeekPointer (mpdu))
    {
      if (i.second.GetDestinationAddr () == GetAddress ())
        {
          ForwardUp (i.first, i.second.GetSourceAddr (),
                     i.second.GetDestinationAddr ());
        }
      else
        {
          Mac48Address from = i.second.GetSourceAddr ();
          Mac48Address to = i.second.GetDestinationAddr ();
          NS_LOG_DEBUG ("forwarding QoS frame from=" << from << ", to=" << to);
          ForwardDown (i.first->Copy (), from, to, mpdu->GetHeader ().GetQosTid ());
        }
    }
}

void
ApWifiMac::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  m_beaconTxop->Initialize ();
  m_beaconEvent.Cancel ();
  if (m_enableBeaconGeneration)
    {
      if (m_enableBeaconJitter)
        {
          Time jitter = MicroSeconds (static_cast<int64_t> (m_beaconJitter->GetValue (0, 1) * (GetBeaconInterval ().GetMicroSeconds ())));
          NS_LOG_DEBUG ("Scheduling initial beacon for access point " << GetAddress () << " at time " << jitter);
          m_beaconEvent = Simulator::Schedule (jitter, &ApWifiMac::SendOneBeacon, this);
        }
      else
        {
          NS_LOG_DEBUG ("Scheduling initial beacon for access point " << GetAddress () << " at time 0");
          m_beaconEvent = Simulator::ScheduleNow (&ApWifiMac::SendOneBeacon, this);
          NS_LOG_DEBUG ("Scheduling initial Trigger Frame beacon for access point " << GetAddress () << " at time 0");
          Simulator::Schedule (MicroSeconds(999920), &ApWifiMac::StartMuModeUplink, this);
        }
    }
  RegularWifiMac::DoInitialize ();
  
}

bool
ApWifiMac::GetUseNonErpProtection (void) const
{
  bool useProtection = !m_nonErpStations.empty () && m_enableNonErpProtection;
  m_stationManager->SetUseNonErpProtection (useProtection);
  return useProtection;
}

void
ApWifiMac::StopMuMode (void)
{
  NS_LOG_UNCOND ("Inside ApWifiMac:Stopping MuMode, time = "<<Simulator::Now ().GetMicroSeconds ());
  SetMuMode (0);
  SetRuBits (0);
  m_tfAlloc.clear ();
  m_txop->CancelNextPacket ();

  for (uint32_t ru = 0; ru < 9; ru++)
   {
     m_txopMu[ru]->StopMuMode ();
     //m_channelAccessManagerMu[ru]->NotifyMaybeCcaBusyStartNow (Seconds (1));//Xyct: RE: dereference null pointer
     for (uint32_t ac = 0; ac < 8; ac++)
      {
	m_edcaMu[ru][QosUtilsMapTidToAc(ac)]->StopMuMode ();
      }
   }
  Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable> ();
  rv->SetAttribute ("Min", DoubleValue (0.0));
  rv->SetAttribute ("Max", DoubleValue (1.0));
  
  if (rv->GetValue () < m_alpha)
   {
     m_tfPacketDuration = 0;
     Simulator::ScheduleNow (&ApWifiMac::StartMuModeDownlink, this);
   }
  else
   {
     m_tfPacketDuration = 12932;
     Simulator::ScheduleNow (&ApWifiMac::StartMuModeUplink, this);
   }
}

uint16_t
ApWifiMac::GetNextAssociationId (void)
{
  //Return the first free AID value between 1 and 2007
  for (uint16_t nextAid = 1; nextAid <= 2007; nextAid++)
    {
      if (m_staList.find (nextAid) == m_staList.end ())
        {
          return nextAid;
        }
    }
  NS_FATAL_ERROR ("No free association ID available!");
  return 0;
}

void
ApWifiMac::IncrementPollingListIterator (void)
{
  NS_LOG_FUNCTION (this);
  m_itCfPollingList++;
  if (m_itCfPollingList == m_cfPollingList.end ())
    {
      m_itCfPollingList = m_cfPollingList.begin ();
    }
}

} //namespace ns3
