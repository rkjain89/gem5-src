#include "arch/x86/tsb.hh"
#include "arch/x86/tlb.hh"
#include "arch/x86/pagetable_walker.hh"

namespace X86ISA {

// TSB::TSB(uint32_t _size) : size(_size)
// {
// }

bool
TSB::TSBPort::recvTimingResp(PacketPtr pkt)
{
    return tsb->recvTimingResp(pkt);
}

void
TSB::TSBPort::recvRetry()
{
    tsb->recvRetry();
}

void
TSB::recvRetry()
{
    std::list<TSBState*>::iterator iter;
    for (iter = currStates.begin(); iter != currStates.end(); iter++) {
        TSBState* tsbState = *(iter);
        if (tsbState->isRetrying()) {
            tsbState->retry();
        }
    }
}

TSB::TSBState::TSBState(TSB* _tsb, bool _isFunctional) :
    tsb(_tsb), state(Ready), nextState(Ready), inflight(0), 
    functional(_isFunctional), timing(false), retrying(false), started(false)
{
}

void
TSB::TSBState::retry()
{
    retrying = false;
    sendPackets();
}

bool
TSB::TSBState::recvPacket(PacketPtr pkt) 
{
    assert(pkt->isResponse());
    assert(inflight);
    assert(state == Waiting);
    assert(!read);
    inflight--;
    
    if (pkt->isRead()) {
        // @todo someone should pay for this
        pkt->busFirstWordDelay = pkt->busLastWordDelay = 0;

        state           = nextState;
        nextState       = Ready;
        PacketPtr write = NULL;
        read            = pkt;
        Addr      vaddr = pkt->req->getVaddr();
        retEntry        = tsb->readTSB(vaddr, true, write, tc, translation, tlbReq, mode);
        state           = Waiting;
        read            = NULL;
        if (write) {
            writes.push_back(write);
        }
        // sendPackets();
    } else {
        Addr      vaddr = pkt->req->getVaddr();
        retEntry        = tsb->readTSB(vaddr, true, pkt, tc, translation, tlbReq, mode);
        read            = NULL;
    }

    if (inflight == 0 && read == NULL && writes.size() == 0) {
        state     = Ready;
        nextState = Waiting;
        
        if (retEntry) {
            bool delayedResponse;
            Fault fault = tsb->getTLB()->translate(tlbReq, tc, NULL, mode,
                                                   delayedResponse, true);
            assert(!delayedResponse);
            // Let the CPU continue.
            translation->finish(fault, tlbReq, tc, mode);
            return true;
        }
        return false;
    }

    return false;
}


void
TSB::TSBState::sendPackets()
{
    //If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    // doneTranslation = false;

    //Reads always have priority
    if (read) {
        PacketPtr pkt = read;
        read = NULL;
        inflight++;
        if (!tsb->sendTiming(this, pkt)) {
            read = pkt;
            inflight--;
            return;
        }
    }

    //Send off as many of the writes as we can.
    while (writes.size()) {
        PacketPtr write = writes.back();
        writes.pop_back();
        inflight++;
        if (!tsb->sendTiming(this, write)) {
            retrying = true;
            writes.push_back(write);
            inflight--;
            return;
        }
    }
}

void
TSB::TSBState::setRequestPtr(RequestPtr _req) {
    req = _req;
}

bool 
TSB::sendTiming(TSBState* sendingState, PacketPtr pkt)
{
    pkt->pushSenderState(new TSBSenderState(sendingState));
    return port.sendTimingReq(pkt);
}

bool
TSB::recvTimingResp(PacketPtr pkt)
{
    TSBSenderState* senderStateLocalVar =
        dynamic_cast<TSBSenderState*>(pkt->popSenderState());
    TSBState* senderTSB = senderStateLocalVar->senderTSB;
    senderTSB->setRequestPtr(pkt->req);

    bool flag = senderTSB->recvPacket(pkt);

    std::list<TSBState*>::iterator iter;
    for (iter = currStates.begin(); iter != currStates.end(); iter++) {
        TSBState* state = *(iter);
        if (state == senderTSB) {
            iter = currStates.erase(iter);
            break;
        }
    }

    delete senderTSB;
    
    return flag;
}

TlbEntry*
TSB::insert(Addr vpn)
{
    TlbEntry *newEntry = trie.lookup(vpn);
    if (newEntry) {
        assert(newEntry->vaddr == vpn);
        return newEntry;
    }

    if (freeList.empty())
        evictLRU();

    newEntry = freeList.front();
    freeList.pop_front();

    assert(writeEntry.vaddr != UINTMAX_MAX);
    *newEntry        = writeEntry;
    newEntry->lruSeq = nextSeq();
    newEntry->vaddr  = vpn;
    writeEntry.updateVaddr(UINTMAX_MAX);
    // writeEntry       = NULL;

    // Do the actual insert here! 
    newEntry->trieHandle =
    trie.insert(vpn, TlbEntryTrie::MaxBits - writeEntry.logBytes, newEntry);
    return newEntry;
}

TlbEntry*
TSB::readTSB(Addr                 va, 
             bool                 update_lru,
             PacketPtr&           write,
             ThreadContext*       tc,
             TLB::Translation*    translation,
             RequestPtr           req,
             BaseTLB::Mode        mode)
{
    TlbEntry* entry = NULL;
    if (write) {
        entry = insert(va);
        // How do I model the actual write??? Or is it done???
        return entry;
    }

    entry = trie.lookup(va);
    if (entry && update_lru)
        entry->lruSeq = nextSeq();

    if (entry) {
        tlb->insert(entry->vaddr, *entry, tc);
    } else {
        (tlb->getWalker())->start(tc, translation, req, mode);
        // if (sys->isTimingMode()) {
        //     // This gets ignored in atomic mode.
        //     delayedResponse = true;
        // }
        return NULL;
    }
    
    return entry;
}

TlbEntry*
TSB::lookup(Addr                  va, 
            bool                  update_lru,
            RequestPtr            req, 
            PacketPtr&            write,
            ThreadContext*        tc,
            TLB::Translation*     translation,
            Walker*               walker,
            BaseTLB::Mode         mode)
{
    TSBState* newState = new TSBState(this);
    newState->initState(va, update_lru, req, write, 
                        sys->isTimingMode(), tc, 
                        translation, walker, mode);

    int dataSize = 8;
    
    Request::Flags flags = req->getFlags();
    flags.set(Request::PHYSICAL, true);
    flags.set(Request::UNCACHEABLE, true);
    Addr       tsbAddr = tc->readMiscRegNoEffect(MISCREG_CR5);
    RequestPtr request = new Request(tsbAddr, dataSize, 
                                     flags, masterId);
    request->setVaddr(va);
    newState->setRequestPtr(request);


    if (!currStates.empty()) {
        assert(newState->isTiming());
        currStates.push_back(newState);
        return NULL;
    } else {
        if (mode == BaseTLB::Execute) 
            currStates.push_back(newState);
        TlbEntry* entry = newState->startTranslation();
        if (!newState->isTiming()) {
            currStates.pop_front();
            delete newState;
        }
        if (mode != BaseTLB::Execute) 
            delete newState;
        return entry;
    }
}

void
TSB::TSBState::initState(Addr _va, bool _update_lru, 
                         RequestPtr _req, PacketPtr& _write, 
                         bool _timing, ThreadContext* _tc,
                         TLB::Translation* _translation,
                         Walker* _walker, BaseTLB::Mode _mode)
{
    virtAddr    = _va;
    refresh_lru = _update_lru;
    writePtr    = _write;
    timing      = _timing;
    tlbReq      = _req;
    tc          = _tc;
    translation = _translation;
    walker      = _walker;
    mode        = _mode;
}

TlbEntry*
TSB::TSBState::startTranslation()
{
    TlbEntry* entry = NULL;
    started         = true;
    Addr     addr   = virtAddr;
    if (!writePtr) {
        // tsb->setWriteEntry(evictedEntry);
        // RequestPtr req = new Request(tsb->getAddr(),
        read = new Packet(req, MemCmd::ReadReq);
        read->allocate();
    }

    if (timing) {
        if (mode == BaseTLB::Execute) {
            nextState = state;
            state = Waiting;
            sendPackets();
            return NULL;
        } else {
            walker->start(tc, translation, tlbReq, mode);
        }
    } else {
        do {
            tsb->port.sendAtomic(read);
            PacketPtr write = NULL;
            entry = tsb->readTSB(addr, true, write, tc, translation, tlbReq, mode);
            state = nextState;
            nextState = Ready;
            if (write)
                tsb->port.sendAtomic(write);
        } while(read);
        state = Ready;
        nextState = Waiting;
    }
    
    return retEntry;
}

void
TSB::evictLRU() {
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    unsigned lru = 0;
    for (unsigned i = 1; i < size; i++) {
        if (tsbEntries[i].lruSeq < tsbEntries[lru].lruSeq)
            lru = i;
    }

    assert(tsbEntries[lru].trieHandle);
    trie.remove(tsbEntries[lru].trieHandle);
    tsbEntries[lru].trieHandle = NULL;
    freeList.push_back(&tsbEntries[lru]);
}

void
TSB::flushAll()
{
    // DPRINTF(TSB, "Invalidating all entries.\n");
    if (!tsbEntries)
        return;

    for (unsigned i = 0; i < size; i++) {
        if (tsbEntries[i].trieHandle) {
            trie.remove(tsbEntries[i].trieHandle);
            tsbEntries[i].trieHandle = NULL;
            freeList.push_back(&tsbEntries[i]);
        }
    }
}

BaseMasterPort &
TSB::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return MemObject::getMasterPort(if_name, idx);
}

} // namespace X86ISA

X86ISA::TSB*
X86TSBParams::create()
{
    return new X86ISA::TSB(this);
}
