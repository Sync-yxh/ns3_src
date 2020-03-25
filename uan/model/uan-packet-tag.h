#ifndef UAN_PACKET_TAG_H
#define UAN_PACKET_TAG_H

#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/uan-module.h"

using namespace ns3;

class UanPacketTag : public Tag
{
public:
    UanPacketTag ();

    enum PacketType
    {
        DATA_PACKET = 1,
        STATE_PACKET = 2,
        INIT_PACKET = 3,
    };

    void SetPacketType (PacketType t);

    PacketType GetPacketType (void) const;

    void SetDestAddress(Address a);

    Mac8Address GetDestAddress (void) const;

    void SetSourceAddress(Address a);

    Mac8Address GetSourceAddress (void) const;

    void SetRelayAddress(Address a);

    Mac8Address GetRelayAddress (void) const;

    void SetPacketIndex(uint32_t i);

    uint32_t GetPacketIndex(void) const;

    void SetEnergy(uint8_t e);

    uint8_t GetEnergy(void) const;

    void SetDepth(uint32_t d);

    uint32_t GetDepth(void) const;

    void SetBandwidth(uint32_t b);

    uint32_t GetBandwidth(void) const;

    void SetConnectNum(uint8_t c);

    uint8_t GetConnectNum(void) const;

    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;
    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual void Print (std::ostream &os) const;

    PacketType m_packetType; // Packet type
    Mac8Address m_destAddr; //  Destination address
    Mac8Address m_srcAddr; // Source address
    Mac8Address m_relayAddr;
    
    uint32_t m_index; // Packet index
    uint8_t m_energy;
    uint32_t m_depth;
    uint32_t m_bandwidth;
    uint8_t m_connect;
};

#endif