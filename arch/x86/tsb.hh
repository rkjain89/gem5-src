#ifndef __ARCH_X86_TSB_HH__
#define __ARCH_X86_TSB_HH__

#include <list>
#include <string>
#include <vector>

#include "arch/x86/regs/segment.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/tlb.hh"
#include "arch/x86/vtophys.hh"
#include "base/types.hh"
#include "base/trie.hh"
#include "cpu/thread_context.hh"
#include "mem/request.hh"
#include "mem/mem_object.hh"
#include "mem/packet_access.hh"
#include "params/X86TLB.hh"
#include "sim/sim_object.hh"
#include "sim/tlb.hh"
#include "sim/system.hh"

class ThreadContext;
class Packet;

namespace X86ISA
{


class TSB : public MemObject
{
protected:

    // Port for accessing memory
    class TSBPort : public MasterPort {
    public:
        TSBPort(const std::string &_name, TSB* _tsb) :
            MasterPort(_name, _tsb), tsb(_tsb)
            {}
            
    protected:
        TSB *tsb;
        
        bool recvTimingResp(PacketPtr pkt);
        
        /**
         * Snooping a coherence request, do nothing.
         */
        void recvTimingSnoopReq(PacketPtr pkt) { }
        Tick recvAtomicSnoop(PacketPtr pkt) { return 0; }
        void recvFunctionalSnoop(PacketPtr pkt) { }
        void recvRetry();
        bool isSnooping() const { return true; }
    };

    friend class TSBPort;


    System*   sys;
    TlbEntry* tsbEntries;
    MasterID  masterId;
    TLB*      tlb;
    TSBPort   port;
    uint32_t  size;

    class TSBState
    {
    private:
        enum State {
            Ready,
            Waiting,
        };

    protected:
        TSB*                    tsb;
        ThreadContext          *tc;
        RequestPtr              req;
        RequestPtr              tlbReq;
        State                   state;
        State                   nextState;
        int                     dataSize;
        bool                    enableNX;
        unsigned                inflight;
        TlbEntry*               retEntry;
        TlbEntry                writeEntry;
        PacketPtr               read;
        std::vector<PacketPtr>  writes;

        BaseTLB::Mode           mode;
        bool                    functional;
        bool                    timing;
        bool                    retrying;
        bool                    started;
        Addr                    virtAddr;
        bool                    refresh_lru;
        PacketPtr               writePtr;
        TLB::Translation*       translation;
        Walker*                 walker;

    public:
        
        TSBState(TSB* _tsb, bool _isFunctional = false);

        bool recvPacket(PacketPtr);

        bool isRetrying() {
            return retrying;
        }

        void           retry();
        void           sendPackets();
        void           initState(Addr, bool, RequestPtr, PacketPtr, bool, 
                                 ThreadContext*, BaseTLB::Translation*,
                                 Walker*, BaseTLB::Mode);

        TlbEntry*      startTranslation();
        void           setRequestPtr(RequestPtr);

        bool           isTiming() {
            return timing;
        }

        ThreadContext* getThreadContext() {
            return tc;
        }

    }; // class TSBState

    struct TSBSenderState : public Packet::SenderState {
        TSBState* senderTSB;
        TSBSenderState(TSBState* _senderTSB) :
            senderTSB(_senderTSB) {}
    };

    friend class TSBState;

    std::list<TlbEntry*> freeList;

    TlbEntryTrie trie;
        
    uint64_t lruSeq;

    // State for timing and atomic accesses (need multiple per walker in
    // the case of multiple outstanding requests in timing mode)
    std::list<TSBState*> currStates;
    TSBState funcState;

    TlbEntry writeEntry;

public:

    typedef X86TSBParams Params;
    TSB(const Params *params) :
        MemObject(params), sys(params->system), masterId(sys->getMasterId(name())),
        port(name() + ".port", this), size(params->size), funcState(this)
        {
            assert(size != 0);
            
            tsbEntries = new TlbEntry[size];
            std::memset(tsbEntries, 0, sizeof(TlbEntry) * size);
    
            for (int x = 0; x < size; x++) {
                tsbEntries[x].trieHandle = NULL;
                freeList.push_back(&tsbEntries[x]);
            }
        }
        
    void flushAll();

    BaseMasterPort &getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID);
    
    MasterID getMasterId() {
        return masterId;
    }
        
    void flushNonGlobal();

    void setTLB(TLB* _tlb) {
        tlb = _tlb;
    }
    
    TLB* getTLB() {
        return tlb;
    }

    bool recvTimingResp(PacketPtr);

    void evictLRU();
    // Addr getAddr();
    void recvRetry();
    bool sendTiming(TSBState*, PacketPtr);
    

    TlbEntry* readTSB(Addr, bool, PacketPtr, ThreadContext*, 
                      TLB::Translation*, RequestPtr, BaseTLB::Mode);

    TlbEntry* lookup(Addr, bool, RequestPtr, PacketPtr&, ThreadContext*, 
                     TLB::Translation*, Walker*, BaseTLB::Mode);
    TlbEntry* insert(Addr, BaseTLB::Mode);

    uint64_t nextSeq() {
        return ++lruSeq;
    }

    void setWriteEntry(TlbEntry entryToBeWritten) {
        writeEntry = entryToBeWritten;
    }
};

} // namespace X86ISA

#endif // __ARCH_X86_TSB_HH__
