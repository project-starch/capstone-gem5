#ifndef MY_CACHE_H
#define MY_CACHE_H

#include "mem/port.hh"
#include "params/SimpleCache.hh"
#include "sim/sim_object.hh"
#include "sim/clocked_object.hh"

namespace gem5 {

class SimpleCache: public ClockedObject {
    private:
        class CPUSidePort: public SlavePort  {
            private:
                SimpleCache* owner;
                PortID idx;
                PacketPtr retry_send_pkt;
                bool pending_send_retry;
            public:
                CPUSidePort(const std::string& name, PortID idx, SimpleCache* owner) :
                    SlavePort(name, owner),
                    owner(owner),
                    idx(idx),
                    retry_send_pkt(nullptr),
                    pending_send_retry(false)
                { }
                void sendPacket(PacketPtr pkt);
                void trySendRetry();
                AddrRangeList getAddrRanges() const override;

            protected:
                void recvFunctional(PacketPtr pkt) override;
                bool recvTimingReq(PacketPtr pkt) override;
                Tick recvAtomic(PacketPtr pkt) override {
                    panic("atomic requests unsupported.");
                }
                void recvRespRetry() override;
        };

        class MemSidePort: public MasterPort {
            private:
                SimpleCache* owner;
                PacketPtr retry_send_pkt;
            public:
                MemSidePort(const std::string& name, SimpleCache* owner) :
                    MasterPort(name, owner),
                    owner(owner),
                    retry_send_pkt(nullptr)
                { }

                void sendPacket(PacketPtr pkt);

            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;
                void recvRangeChange() override;
        };


        std::vector<CPUSidePort> cpu_side_ports;
        MemSidePort mem_side_port;
        bool blocked;
        PortID blocked_port; // blocked due to request from this port
    public:
        SimpleCache(const SimpleCacheParams& params);
        Port& getPort(const std::string& if_name, PortID idx) override;
        void handleFunctional(PacketPtr pkt);
        bool handleTimingReq(PacketPtr pkt, PortID from_port);
        bool handleTimingResp(PacketPtr pkt);
        AddrRangeList getAddrRanges() const;
        void handleRangeChange();
};

}

#endif
