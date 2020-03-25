#include "uan-packet-tag.h"

using namespace ns3;

UanPacketTag::UanPacketTag():
    m_index(0)
{}

void UanPacketTag::SetPacketType(UanPacketTag::PacketType t)
{
    m_packetType = t;
}

UanPacketTag::PacketType UanPacketTag::GetPacketType() const
{
    return m_packetType;
}

void UanPacketTag::SetDestAddress(Address a)
{
    m_destAddr = Mac8Address::ConvertFrom(a);
}

Mac8Address UanPacketTag::GetDestAddress (void) const
{
    return m_destAddr;
}

void UanPacketTag::SetSourceAddress(Address a)
{
    m_srcAddr = Mac8Address::ConvertFrom(a);
}

Mac8Address UanPacketTag::GetSourceAddress (void) const
{
    return m_srcAddr;
}

void UanPacketTag::SetRelayAddress(Address a)
{
    m_relayAddr = Mac8Address::ConvertFrom(a);
}

Mac8Address UanPacketTag::GetRelayAddress (void) const
{
    return m_relayAddr;
}

void UanPacketTag::SetPacketIndex(uint32_t i)
{
    m_index = i;
}

uint32_t UanPacketTag::GetPacketIndex(void) const
{
    return m_index;
}

void UanPacketTag::SetEnergy(uint8_t e)
{
    m_energy = e;
}

uint8_t UanPacketTag::GetEnergy(void) const
{
    return m_energy;
}

void UanPacketTag::SetDepth(uint32_t d)
{
    m_depth = d;
}

uint32_t UanPacketTag::GetDepth(void) const
{
    return m_depth;
}

void UanPacketTag::SetBandwidth(uint32_t b)
{
    m_bandwidth = b;
}

uint32_t UanPacketTag::GetBandwidth(void) const
{
    return m_bandwidth;
}

void UanPacketTag::SetConnectNum(uint8_t c)
{
    m_connect = c;
}

uint8_t UanPacketTag::GetConnectNum(void) const
{
    return m_connect;
}


NS_OBJECT_ENSURE_REGISTERED (UanPacketTag);

TypeId UanPacketTag::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::UanPacketTag")
            .SetParent<Tag> ()
            .SetGroupName("Network")
            .AddConstructor<UanPacketTag> ();
    return tid;
}
TypeId UanPacketTag::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

uint32_t UanPacketTag::GetSerializedSize (void) const
{
    return  1 + 1 + 1 + 1 + 4 + 1 + 4 + 4 + 1 ; // type + dest + src + relay + index + energy + depth + bandwidth + connect(uchar)
}
void UanPacketTag::Serialize (TagBuffer i) const
{
    i.WriteU8 (m_packetType);
    uint8_t dst;
    uint8_t src;
    uint8_t relay;
    m_destAddr.CopyTo(&dst);
    m_srcAddr.CopyTo(&src);
    m_relayAddr.CopyTo(&relay);
    i.WriteU8(dst);
    i.WriteU8(src);
    i.WriteU8(relay);
    i.WriteU32(m_index);
    i.WriteU8(m_energy);
    i.WriteU32(m_depth);
    i.WriteU32(m_bandwidth);
    i.WriteU8(m_connect);
}
void UanPacketTag::Deserialize (TagBuffer i)
{
    m_packetType = (PacketType) i.ReadU8 ();
    m_destAddr = Mac8Address(i.ReadU8());
    m_srcAddr = Mac8Address(i.ReadU8());
    m_relayAddr = Mac8Address(i.ReadU8());
    m_index = uint32_t(i.ReadU32());
    m_energy = uint8_t(i.ReadU8());
    m_depth = uint32_t(i.ReadU32());
    m_bandwidth = uint32_t(i.ReadU32());
    m_connect = uint8_t(i.ReadU8());
}
void UanPacketTag::Print (std::ostream &os) const
{
    os << "packetType=" << m_packetType;
}

