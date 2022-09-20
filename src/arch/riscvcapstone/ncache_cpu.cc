/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010-2013,2015,2017-2018, 2020-2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/riscvcapstone/ncache_cpu.hh"
#include "arch/riscvcapstone/faults.hh"
#include "arch/riscvcapstone/regs/int.hh"
#include "arch/generic/decoder.hh"
#include "base/compiler.hh"
#include "config/the_isa.hh"
#include "cpu/exetrace.hh"
#include "debug/Config.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/HtmCpu.hh"
#include "debug/Mwait.hh"
#include "debug/SimpleCPU.hh"
#include "debug/CapstoneNCache.hh"
#include "debug/CapstoneNodeOps.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "node_controller.hh"
#include "params/BaseTimingSimpleNCacheCPU.hh"
#include "base/trace.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

namespace gem5::RiscvcapstoneISA
{

void
TimingSimpleNCacheCPU::init()
{
    BaseSimpleCPU::init();
}

void
TimingSimpleNCacheCPU::TimingCPUPort::TickEvent::schedule(PacketPtr _pkt, Tick t)
{
    pkt = _pkt;
    cpu->schedule(this, t);
}

TimingSimpleNCacheCPU::TimingSimpleNCacheCPU(const BaseTimingSimpleNCacheCPUParams &p)
    : BaseSimpleCPU(p), node_controller(p.node_controller),
      ncache_status(NCACHE_INSTR_EXECUTION),
      fetchTranslation(this), icachePort(this),
      dcachePort(this), ncache_port(this), 
      ifetch_pkt(NULL), dcache_pkt(NULL), ncache_pkt(NULL), 
      previousCycle(0),
      fetchEvent([this]{ fetch(); }, name()),
      instPendingMem(NULL)
{
    _status = Idle;
}



TimingSimpleNCacheCPU::~TimingSimpleNCacheCPU()
{
}

DrainState
TimingSimpleNCacheCPU::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();

    if (switchedOut())
        return DrainState::Drained;

    if (_status == Idle ||
        (_status == BaseSimpleCPU::Running && isCpuDrained())) {
        DPRINTF(Drain, "No need to drain.\n");
        activeThreads.clear();
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "Requesting drain.\n");

        // The fetch event can become descheduled if a drain didn't
        // succeed on the first attempt. We need to reschedule it if
        // the CPU is waiting for a microcode routine to complete.
        if (_status == BaseSimpleCPU::Running && !fetchEvent.scheduled())
            schedule(fetchEvent, clockEdge());

        return DrainState::Draining;
    }
}

void
TimingSimpleNCacheCPU::drainResume()
{
    assert(!fetchEvent.scheduled());
    if (switchedOut())
        return;

    DPRINTF(SimpleCPU, "Resume\n");
    verifyMemoryMode();

    assert(!threadContexts.empty());

    _status = BaseSimpleCPU::Idle;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (threadInfo[tid]->thread->status() == ThreadContext::Active) {
            threadInfo[tid]->execContextStats.notIdleFraction = 1;

            activeThreads.push_back(tid);

            _status = BaseSimpleCPU::Running;

            // Fetch if any threads active
            if (!fetchEvent.scheduled()) {
                schedule(fetchEvent, nextCycle());
            }
        } else {
            threadInfo[tid]->execContextStats.notIdleFraction = 0;
        }
    }

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}

bool
TimingSimpleNCacheCPU::tryCompleteDrain()
{
    if (drainState() != DrainState::Draining)
        return false;

    DPRINTF(Drain, "tryCompleteDrain.\n");
    if (!isCpuDrained())
        return false;

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    signalDrainDone();

    return true;
}

void
TimingSimpleNCacheCPU::switchOut()
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    [[maybe_unused]] SimpleThread* thread = t_info.thread;

    // hardware transactional memory
    // Cannot switch out the CPU in the middle of a transaction
    assert(!t_info.inHtmTransactionalState());

    BaseSimpleCPU::switchOut();

    assert(!fetchEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(!t_info.stayAtPC);
    assert(thread->pcState().microPC() == 0);

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
}


void
TimingSimpleNCacheCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    previousCycle = curCycle();
}

void
TimingSimpleNCacheCPU::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The timing CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}

void
TimingSimpleNCacheCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "ActivateContext %d\n", thread_num);

    assert(thread_num < numThreads);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 1;
    if (_status == BaseSimpleCPU::Idle)
        _status = BaseSimpleCPU::Running;

    // kick things off by initiating the fetch of the next instruction
    if (!fetchEvent.scheduled())
        schedule(fetchEvent, clockEdge(Cycles(0)));

    if (std::find(activeThreads.begin(), activeThreads.end(), thread_num)
         == activeThreads.end()) {
        activeThreads.push_back(thread_num);
    }

    BaseCPU::activateContext(thread_num);
}


void
TimingSimpleNCacheCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num < numThreads);
    activeThreads.remove(thread_num);

    // hardware transactional memory
    // Cannot suspend context in the middle of a transaction.
    assert(!threadInfo[curThread]->inHtmTransactionalState());

    if (_status == Idle)
        return;

    assert(_status == BaseSimpleCPU::Running);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 0;

    if (activeThreads.empty()) {
        _status = Idle;

        if (fetchEvent.scheduled()) {
            deschedule(fetchEvent);
        }
    }

    BaseCPU::suspendContext(thread_num);
}

bool
TimingSimpleNCacheCPU::handleReadPacket(PacketPtr pkt)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const RequestPtr &req = pkt->req;

    // hardware transactional memory
    // sanity check
    if (req->isHTMCmd()) {
        assert(!req->isLocalAccess());
    }

    // We're about the issues a locked load, so tell the monitor
    // to start caring about this address
    if (pkt->isRead() && pkt->req->isLLSC()) {
        thread->getIsaPtr()->handleLockedRead(pkt->req);
    }
    if (req->isLocalAccess()) {
        Cycles delay = req->localAccessor(thread->getTC(), pkt);
        new IprEvent(pkt, this, clockEdge(delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else{
        assert(_status != DcacheRetry);
        if (!dcachePort.sendTimingReq(pkt)) {
            _status = DcacheRetry;
            dcache_pkt = pkt;
        } else {
            _status = DcacheWaitResponse;
            // memory system takes ownership of packet
            dcache_pkt = NULL;
        }

    }
    return dcache_pkt == NULL;
}

void
TimingSimpleNCacheCPU::sendData(const RequestPtr &req, uint8_t *data, uint64_t *res,
                          bool read)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    PacketPtr pkt = buildPacket(req, read); // simply createRead or createWrite
    pkt->dataDynamic<uint8_t>(data);

    // hardware transactional memory
    // If the core is in transactional mode or if the request is HtmCMD
    // to abort a transaction, the packet should reflect that it is
    // transactional and also contain a HtmUid for debugging.
    const bool is_htm_speculative = t_info.inHtmTransactionalState();
    if (is_htm_speculative || req->isHTMAbort()) {
        pkt->setHtmTransactional(t_info.getHtmTransactionUid());
    }
    if (req->isHTMAbort())
        DPRINTF(HtmCpu, "htmabort htmUid=%u\n", t_info.getHtmTransactionUid());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt->makeResponse();
        handleDCacheResp(pkt);
    } else if (read) {
        DPRINTF(CapstoneNCache, "Senddata read %llx\n", req->getVaddr());
        handleReadPacket(pkt);

        //sendNCacheReq(pkt->getAddr());
    } else {
        bool do_access = true;  // flag to suppress cache access


        if (req->isLLSC()) {
            do_access = thread->getIsaPtr()->handleLockedWrite(
                    req, dcachePort.cacheBlockMask);
        } else if (req->isCondSwap()) {
            assert(res);
            req->setExtraData(*res);
        }

        if (do_access) {
            dcache_pkt = pkt;
            handleWritePacket();
            threadSnoop(pkt, curThread);
        } else {
            _status = DcacheWaitResponse;
            handleDCacheResp(pkt);
        }
        //sendNCacheReq(pkt->getAddr());
    }
}

void
TimingSimpleNCacheCPU::sendNCacheCommand(NodeControllerCommand* cmd) {
    RequestPtr ncache_req = std::make_shared<Request>();
    // pass the address
    //ncache_req->setPaddr(addr);
    //assert(ncache_req->hasPaddr() && !ncache_req->hasSize());
    PacketPtr ncache_pkt = Packet::createRead(ncache_req);
    ncache_pkt->dataDynamic<NodeControllerCommand>(cmd);

    if(ncache_port.sendTimingReq(ncache_pkt)) {
        DPRINTF(CapstoneNCache, "NCache packet sent\n");
        this->ncache_pkt = NULL;
    } else {
        DPRINTF(CapstoneNCache, "NCache packet to retry\n");
        this->ncache_pkt = ncache_pkt;
    }
}

void
TimingSimpleNCacheCPU::sendNCacheReq(Addr addr) {
    std::optional<NodeID> node_id;
    // TODO: here we can't rely on the address alone any more
    if(!node_id) {
        nodeResps.push(NULL);
        return;
    }

    NodeControllerQuery* cmd = new NodeControllerQuery();
    cmd->nodeId = node_id.value();

    sendNCacheCommand(cmd);
}

void
TimingSimpleNCacheCPU::sendSplitData(const RequestPtr &req1, const RequestPtr &req2,
                               const RequestPtr &req, uint8_t *data, bool read)
{
    DPRINTF(CapstoneNCache, "NCache split data invoked\n");
    SimpleExecContext &t_info = *threadInfo[curThread];
    PacketPtr pkt1, pkt2;
    buildSplitPacket(pkt1, pkt2, req1, req2, req, data, read);

    // hardware transactional memory
    // HTM commands should never use SplitData
    assert(!req1->isHTMCmd() && !req2->isHTMCmd());

    // If the thread is executing transactionally,
    // reflect this in the packets.
    if (t_info.inHtmTransactionalState()) {
        pkt1->setHtmTransactional(t_info.getHtmTransactionUid());
        pkt2->setHtmTransactional(t_info.getHtmTransactionUid());
    }

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt1->makeResponse();
        handleDCacheResp(pkt1);
    } else if (read) {
        DPRINTF(CapstoneNCache, "SendSplitData read\n");
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleReadPacket(pkt1)) {
            assert(dcache_pkt == NULL);
            send_state->clearFromParent();
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleReadPacket(pkt2)) {
                send_state->clearFromParent();
            }
        }

        //sendNCacheReq(req->getVaddr());
    } else {
        dcache_pkt = pkt1;
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleWritePacket()) { // assert
            send_state->clearFromParent();
            dcache_pkt = pkt2;
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleWritePacket()) {
                send_state->clearFromParent();
            }
        }

        //sendNCacheReq(req->getVaddr());
    }
}

void
TimingSimpleNCacheCPU::translationFault(const Fault &fault)
{
    // fault may be NoFault in cases where a fault is suppressed,
    // for instance prefetches.
    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if ((fault != NoFault) && traceData) {
        traceFault();
    }

    if(!issueNCacheCommands()){
        postExecute();

        advanceInst(fault);
    }
}

PacketPtr
TimingSimpleNCacheCPU::buildPacket(const RequestPtr &req, bool read)
{
    return read ? Packet::createRead(req) : Packet::createWrite(req);
}

void
TimingSimpleNCacheCPU::buildSplitPacket(PacketPtr &pkt1, PacketPtr &pkt2,
        const RequestPtr &req1, const RequestPtr &req2, const RequestPtr &req,
        uint8_t *data, bool read)
{
    pkt1 = pkt2 = NULL;

    assert(!req1->isLocalAccess() && !req2->isLocalAccess());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        pkt1 = buildPacket(req, read);
        return;
    }

    pkt1 = buildPacket(req1, read);
    pkt2 = buildPacket(req2, read);

    PacketPtr pkt = new Packet(req, pkt1->cmd.responseCommand());

    pkt->dataDynamic<uint8_t>(data);
    pkt1->dataStatic<uint8_t>(data);
    pkt2->dataStatic<uint8_t>(data + req1->getSize());

    SplitMainSenderState * main_send_state = new SplitMainSenderState;
    pkt->senderState = main_send_state;
    main_send_state->fragments[0] = pkt1;
    main_send_state->fragments[1] = pkt2;
    main_send_state->outstanding = 2;
    pkt1->senderState = new SplitFragmentSenderState(pkt, 0);
    pkt2->senderState = new SplitFragmentSenderState(pkt, 1);
}

Fault
TimingSimpleNCacheCPU::initiateMemRead(Addr addr, unsigned size,
                                 Request::Flags flags,
                                 const std::vector<bool>& byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Fault fault;
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Read;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId(), pc, thread->contextId());
    req->setByteEnable(byte_enable);

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;
    if (split_addr > addr) {
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, new uint8_t[size],
                                      NULL, mode);
        DataTranslation<TimingSimpleNCacheCPU *> *trans1 =
            new DataTranslation<TimingSimpleNCacheCPU *>(this, state, 0);
        DataTranslation<TimingSimpleNCacheCPU *> *trans2 =
            new DataTranslation<TimingSimpleNCacheCPU *>(this, state, 1);

        thread->mmu->translateTiming(req1, thread->getTC(), trans1, mode);
        thread->mmu->translateTiming(req2, thread->getTC(), trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, new uint8_t[size], NULL, mode);
        DataTranslation<TimingSimpleNCacheCPU *> *translation
            = new DataTranslation<TimingSimpleNCacheCPU *>(this, state);
        thread->mmu->translateTiming(req, thread->getTC(), translation, mode);
    }

    return NoFault;
}

bool
TimingSimpleNCacheCPU::handleWritePacket()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const RequestPtr &req = dcache_pkt->req;
    if (req->isLocalAccess()) {
        Cycles delay = req->localAccessor(thread->getTC(), dcache_pkt);
        new IprEvent(dcache_pkt, this, clockEdge(delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else {
        assert(_status != DcacheRetry);
        if (!dcachePort.sendTimingReq(dcache_pkt)) {
            _status = DcacheRetry;
        } else {
            _status = DcacheWaitResponse;
            // memory system takes ownership of packet
            dcache_pkt = NULL;
        }
    }
    return dcache_pkt == NULL;
}

Fault
TimingSimpleNCacheCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, Request::Flags flags, uint64_t *res,
                          const std::vector<bool>& byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    uint8_t *newData = new uint8_t[size];
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Write;

    if (data == NULL) {
        assert(flags & Request::STORE_NO_DATA);
        // This must be a cache block cleaning request
        memset(newData, 0, size);
    } else {
        memcpy(newData, data, size);
    }

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId(), pc, thread->contextId());
    req->setByteEnable(byte_enable);

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;

    // TODO: TimingSimpleNCacheCPU doesn't support arbitrarily long multi-line mem.
    // accesses yet

    if (split_addr > addr) {
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, newData, res, mode);
        DataTranslation<TimingSimpleNCacheCPU *> *trans1 =
            new DataTranslation<TimingSimpleNCacheCPU *>(this, state, 0);
        DataTranslation<TimingSimpleNCacheCPU *> *trans2 =
            new DataTranslation<TimingSimpleNCacheCPU *>(this, state, 1);

        thread->mmu->translateTiming(req1, thread->getTC(), trans1, mode);
        thread->mmu->translateTiming(req2, thread->getTC(), trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, newData, res, mode);
        DataTranslation<TimingSimpleNCacheCPU *> *translation =
            new DataTranslation<TimingSimpleNCacheCPU *>(this, state);
        thread->mmu->translateTiming(req, thread->getTC(), translation, mode);
    }

    // Translation faults will be returned via finishTranslation()
    return NoFault;
}

Fault
TimingSimpleNCacheCPU::initiateMemAMO(Addr addr, unsigned size,
                                Request::Flags flags,
                                AtomicOpFunctorPtr amo_op)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Fault fault;
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Write;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(addr, size, flags,
                            dataRequestorId(), pc, thread->contextId(),
                            std::move(amo_op));

    assert(req->hasAtomicOpFunctor());

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);

    // AMO requests that access across a cache line boundary are not
    // allowed since the cache does not guarantee AMO ops to be executed
    // atomically in two cache lines
    // For ISAs such as x86 that requires AMO operations to work on
    // accesses that cross cache-line boundaries, the cache needs to be
    // modified to support locking both cache lines to guarantee the
    // atomicity.
    if (split_addr > addr) {
        panic("AMO requests should not access across a cache line boundary\n");
    }

    _status = DTBWaitResponse;

    WholeTranslationState *state =
        new WholeTranslationState(req, new uint8_t[size], NULL, mode);
    DataTranslation<TimingSimpleNCacheCPU *> *translation
        = new DataTranslation<TimingSimpleNCacheCPU *>(this, state);
    thread->mmu->translateTiming(req, thread->getTC(), translation, mode);

    return NoFault;
}

void
TimingSimpleNCacheCPU::threadSnoop(PacketPtr pkt, ThreadID sender)
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (tid != sender) {
            if (getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                wakeup(tid);
            }
            threadInfo[tid]->thread->getIsaPtr()->handleLockedSnoop(pkt,
                    dcachePort.cacheBlockMask);
        }
    }
}

void
TimingSimpleNCacheCPU::finishTranslation(WholeTranslationState *state)
{
    _status = BaseSimpleCPU::Running;

    if (state->getFault() != NoFault) {
        if (state->isPrefetch()) {
            state->setNoFault();
        }
        delete [] state->data;
        state->deleteReqs();
        translationFault(state->getFault());
    } else {
        if (!state->isSplit) {
            sendData(state->mainReq, state->data, state->res,
                     state->mode == BaseMMU::Read);
        } else {
            sendSplitData(state->sreqLow, state->sreqHigh, state->mainReq,
                          state->data, state->mode == BaseMMU::Read);
        }
    }

    delete state;
}


void
TimingSimpleNCacheCPU::fetch()
{
    // Change thread if multi-threaded
    swapActiveThread();

    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    DPRINTF(SimpleCPU, "Fetch\n");

    if (!curStaticInst || !curStaticInst->isDelayedCommit()) {
        checkForInterrupts();
        checkPcEventQueue();
    }

    // We must have just got suspended by a PC event
    if (_status == Idle)
        return;

    MicroPC upc = thread->pcState().microPC();
    bool needToFetch = !isRomMicroPC(upc) && !curMacroStaticInst;

    if (needToFetch) {
        _status = BaseSimpleCPU::Running;
        RequestPtr ifetch_req = std::make_shared<Request>();
        ifetch_req->taskId(taskId());
        ifetch_req->setContext(thread->contextId());
        setupFetchRequest(ifetch_req);
        DPRINTF(SimpleCPU, "Translating address %#x\n", ifetch_req->getVaddr());
        thread->mmu->translateTiming(ifetch_req, thread->getTC(),
                &fetchTranslation, BaseMMU::Execute);
    } else {
        _status = IcacheWaitResponse;
        completeIfetch(NULL);

        updateCycleCounts();
        updateCycleCounters(BaseCPU::CPU_STATE_ON);
    }
}


void
TimingSimpleNCacheCPU::sendFetch(const Fault &fault, const RequestPtr &req,
                           ThreadContext *tc)
{
    auto &decoder = threadInfo[curThread]->thread->decoder;

    if (fault == NoFault) {
        DPRINTF(SimpleCPU, "Sending fetch for addr %#x(pa: %#x)\n",
                req->getVaddr(), req->getPaddr());
        ifetch_pkt = new Packet(req, MemCmd::ReadReq);
        ifetch_pkt->dataStatic(decoder->moreBytesPtr());
        DPRINTF(SimpleCPU, " -- pkt addr: %#x\n", ifetch_pkt->getAddr());

        if (!icachePort.sendTimingReq(ifetch_pkt)) {
            // Need to wait for retry
            _status = IcacheRetry;
        } else {
            // Need to wait for cache to respond
            _status = IcacheWaitResponse;
            // ownership of packet transferred to memory system
            ifetch_pkt = NULL;
        }
    } else {
        DPRINTF(SimpleCPU, "Translation of addr %#x faulted\n", req->getVaddr());
        // fetch fault: advance directly to next instruction (fault handler)
        _status = BaseSimpleCPU::Running;
        advanceInst(fault);
    }

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
}


void
TimingSimpleNCacheCPU::advanceInst(const Fault &fault)
{
    SimpleExecContext &t_info = *threadInfo[curThread];

    if (_status == Faulting)
        return;

    if (fault != NoFault) {
        // hardware transactional memory
        // If a fault occurred within a transaction
        // ensure that the transaction aborts
        if (t_info.inHtmTransactionalState() &&
            !std::dynamic_pointer_cast<GenericHtmFailureFault>(fault)) {
            DPRINTF(HtmCpu, "fault (%s) occurred - "
                "replacing with HTM abort fault htmUid=%u\n",
                fault->name(), t_info.getHtmTransactionUid());

            Fault tmfault = std::make_shared<GenericHtmFailureFault>(
                t_info.getHtmTransactionUid(),
                HtmFailureFaultCause::EXCEPTION);

            advancePC(tmfault);
            reschedule(fetchEvent, clockEdge(), true);
            _status = Faulting;
            return;
        }

        DPRINTF(SimpleCPU, "Fault occured. Handling the fault\n");

        advancePC(fault);

        // A syscall fault could suspend this CPU (e.g., futex_wait)
        // If the _status is not Idle, schedule an event to fetch the next
        // instruction after 'stall' ticks.
        // If the cpu has been suspended (i.e., _status == Idle), another
        // cpu will wake this cpu up later.
        if (_status != Idle) {
            DPRINTF(SimpleCPU, "Scheduling fetch event after the Fault\n");

            Tick stall = std::dynamic_pointer_cast<SyscallRetryFault>(fault) ?
                         clockEdge(syscallRetryLatency) : clockEdge();
            reschedule(fetchEvent, stall, true);
            _status = Faulting;
        }

        return;
    }

    if (!t_info.stayAtPC)
        advancePC(fault);

    if (tryCompleteDrain())
        return;

    serviceInstCountEvents();

    if (_status == BaseSimpleCPU::Running) {
        // kick off fetch of next instruction... callback from icache
        // response will cause that instruction to be executed,
        // keeping the CPU running.
        fetch();
    }
}


void
TimingSimpleNCacheCPU::completeIfetch(PacketPtr pkt)
{
    SimpleExecContext& t_info = *threadInfo[curThread];

    DPRINTF(SimpleCPU, "Complete ICache Fetch for addr %#x\n", pkt ?
            pkt->getAddr() : 0);

    // received a response from the icache: execute the received
    // instruction
    panic_if(pkt && pkt->isError(), "Instruction fetch (%s) failed: %s",
            pkt->getAddrRange().to_string(), pkt->print());
    assert(_status == IcacheWaitResponse);

    _status = BaseSimpleCPU::Running;

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if (pkt)
        pkt->req->setAccessLatency();


    preExecute();

    // hardware transactional memory
    if (curStaticInst && curStaticInst->isHtmStart()) {
        // if this HtmStart is not within a transaction,
        // then assign it a new htmTransactionUid
        if (!t_info.inHtmTransactionalState())
            t_info.newHtmTransactionUid();
        SimpleThread* thread = t_info.thread;
        thread->htmTransactionStarts++;
        DPRINTF(HtmCpu, "htmTransactionStarts++=%u\n",
            thread->htmTransactionStarts);
    }

    if (curStaticInst && curStaticInst->isMemRef()) {
        // issue commands for checking capabilities
        panic_if(curStaticInst->isLoad() && curStaticInst->isStore(), "an instruction cannot be both store and load");
        RiscvStaticInst* rv_inst = dynamic_cast<RiscvStaticInst*>(curStaticInst.get());
        assert(rv_inst != NULL);
        issueCapChecks(t_info, curStaticInst.get(), rv_inst->getAddr(&t_info, traceData));

        // load or store: just send to dcache
        Fault fault = curStaticInst->initiateAcc(&t_info, traceData);


        // load
        if(curStaticInst->isLoad()) { // probably better remove
            preOverwriteDest(t_info, rv_inst);

            // check if the value to be loaded in would be a capability
            DPRINTF(CapstoneNodeOps, "load from %llx\n", rv_inst->getAddr(&t_info, traceData));
            CapLoc mem_loc = CapLoc::makeMem(rv_inst->getAddr(&t_info, traceData));
            NodeID node_id = node_controller->queryCapTrack(mem_loc);
            if(node_id != NODE_ID_INVALID) {
                // if yes, record the reg as a capability
                // TODO: strictly this should be done during wb
                panic_if(rv_inst->numDestRegs() != 1, "load instruction should have exactly 1 destination register");
                node_controller->addCapTrack(
                        CapLoc::makeReg(t_info.thread->threadId(),
                            rv_inst->destRegIdx(0).index()),
                        node_id);
                ncToIssue.push(new NodeControllerRcUpdate(node_id, 1));
            }
        } else if(curStaticInst->isStore()) {
            // check for overwriting in-memory capability
            DPRINTF(CapstoneNodeOps, "store to %llx\n", rv_inst->getAddr(&t_info, traceData));
            CapLoc mem_loc = CapLoc::makeMem(rv_inst->getAddr(&t_info, traceData));
            NodeID mem_node = node_controller->queryCapTrack(mem_loc);
            if(mem_node != NODE_ID_INVALID) {
                // the capability at the memory location will be overwritten
                node_controller->removeCapTrack(mem_loc);
                ncToIssue.push(new NodeControllerRcUpdate(mem_node, -1));
            }

            // check for writing capability to memory
            panic_if(rv_inst->numSrcRegs() != 2, "store instructions should have exactly 2 source registers"); 
            RegId reg_id = rv_inst->srcRegIdx(1);
            if(reg_id.classValue() == RegClassType::IntRegClass){
                RegIndex reg_idx = reg_id.index();
                CapLoc reg_loc = CapLoc::makeReg(t_info.thread->threadId(), reg_idx);
                NodeID reg_node = node_controller->queryCapTrack(reg_loc);
                if(reg_node != NODE_ID_INVALID) {
                    node_controller->addCapTrack(mem_loc, reg_node);
                    ncToIssue.push(new NodeControllerRcUpdate(reg_node, 1));
                }
            }
        }

        // If we're not running now the instruction will complete in a dcache
        // response callback or the instruction faulted and has started an
        // ifetch
        if (_status == BaseSimpleCPU::Running) {
            if (fault != NoFault && traceData) {
                traceFault();
            }

            if(!issueNCacheCommands()){
                postExecute();
                // @todo remove me after debugging with legion done
                if (curStaticInst && (!curStaticInst->isMicroop() ||
                            curStaticInst->isFirstMicroop()))
                    instCnt++;
                advanceInst(fault);
            }
        }
    } else if (curStaticInst) {
        // non-memory instruction: execute completely now

        // track the capabilities
        int num_src = curStaticInst->numSrcRegs();
        int num_dest = curStaticInst->numDestRegs();
        if(curStaticInst->isSyscall()){
            overwriteIntReg(t_info.tcBase(), RiscvcapstoneISA::ReturnValueReg);
        } else {
            preOverwriteDest(t_info, curStaticInst.get());
        }

        Fault fault = curStaticInst->execute(&t_info, traceData);

        // after execution
        // check which destinations contain new capabilities
        if(!curStaticInst->isSyscall()){
            for(int j = 0; j < num_dest; j ++){
                const RegId& dest_id = curStaticInst->destRegIdx(j);
                if(dest_id.classValue() != RegClassType::IntRegClass)
                    continue;
                RegIndex dest_idx = dest_id.index();
                RegVal dest_val = t_info.tcBase()->readIntReg(dest_idx);
                //RegVal dest_val = t_info.getRegOperand(curStaticInst.get(), j);
                std::optional<int> dest_obj = node_controller->lookupAddr((Addr)dest_val);
                if(!dest_obj)
                    continue;
                DPRINTF(CapstoneNodeOps, "Consider dest reg %d (%d)\n", j, dest_idx);
                for(int i = 0; i < num_src; i ++){
                    const RegId& src_id = curStaticInst->srcRegIdx(i);
                    if(src_id.classValue() != RegClassType::IntRegClass)
                        continue;
                    RegIndex src_idx = src_id.index();
                    // check whether it is a cap
                    CapLoc src_loc = CapLoc::makeReg(t_info.thread->threadId(), src_idx);
                    NodeID src_node = node_controller->queryCapTrack(src_loc);
                    if(src_node == NODE_ID_INVALID)
                        continue;
                    RegVal src_val = t_info.getRegOperand(curStaticInst.get(), i);
                    std::optional<int> src_obj = node_controller->lookupAddr((Addr)src_val);
                    panic_if(!src_obj, "capabilities should always be associated with objects,"
                            " value = %llx, index = %u",
                            src_val, src_idx);
                    if(dest_obj.value() != src_obj.value())
                        continue;
                    DPRINTF(CapstoneNodeOps, "Consider src reg %d (%d)\n", i, src_idx);
                    // src and dest are in the same region and the source is a capability
                    CapLoc dest_loc = CapLoc::makeReg(t_info.thread->threadId(), dest_idx);
                    NodeID dest_node = node_controller->queryCapTrack(dest_loc);
                    panic_if(dest_node != NODE_ID_INVALID,
                            "dest reg %u already associated with a node (syscall=%d) %llu, src-node = %llu", dest_idx,
                            curStaticInst->isSyscall(), dest_node, src_node);
                    // TODO: decide between two options:
                    // 1. allocate a new linear capability
                    // 2. treat this as a non-linear capability
                    // doing 2 for now
                    DPRINTF(CapstoneNodeOps, "add cap track to (%u, %u)\n",
                            t_info.thread->threadId(),
                            dest_idx);
                    node_controller->addCapTrack(dest_loc, src_node);
                    ncToIssue.push(new NodeControllerRcUpdate(src_node, 1));
                    break;
                }
            }
        }

        // non-memory instructions might involve touching the node cache
        RiscvStaticInst* rv_inst = dynamic_cast<RiscvStaticInst*>(curStaticInst.get());
        panic_if(rv_inst == NULL, "non-RISC-V instructions unsupported!");
        InstStateMachinePtr sm = rv_inst->getStateMachine(&t_info);
        sm->setup(&t_info);
        if(rv_inst->pendingMem(sm, &t_info)){
            instPendingMem = rv_inst;
            statePendingMem = sm;
            faultPendingMem = fault;
        } else{
            completeInstExec(fault);
        }
    } else {
        advanceInst(NoFault);
    }

    if (pkt) {
        delete pkt;
    }
}

void
TimingSimpleNCacheCPU::completeInstExec(Fault fault) {
    DPRINTF(CapstoneNodeOps, "complete inst exec\n");
    // keep an instruction count
    if (fault == NoFault)
        countInst();
    else{
        if (traceData) {
            traceFault();
        }
    }

    if(!issueNCacheCommands()){
        postExecute();
        // @todo remove me after debugging with legion done
        if (curStaticInst && (!curStaticInst->isMicroop() ||
                    curStaticInst->isFirstMicroop()))
            instCnt++;
        advanceInst(fault);
    }
}

void
TimingSimpleNCacheCPU::IcachePort::ITickEvent::process()
{
    cpu->completeIfetch(pkt);
}

bool
TimingSimpleNCacheCPU::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "Received fetch response %#x\n", pkt->getAddr());

    // hardware transactional memory
    // Currently, there is no support for tracking instruction fetches
    // in an transaction's read set.
    if (pkt->htmTransactionFailedInCache()) {
        panic("HTM transactional support for"
              " instruction stream not yet supported\n");
    }

    // we should only ever see one response per cycle since we only
    // issue a new request once this response is sunk
    assert(!tickEvent.scheduled());
    // delay processing of returned data until next CPU clock edge
    tickEvent.schedule(pkt, cpu->clockEdge());

    return true;
}

void
TimingSimpleNCacheCPU::IcachePort::recvReqRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->ifetch_pkt != NULL);
    assert(cpu->_status == IcacheRetry);
    PacketPtr tmp = cpu->ifetch_pkt;
    if (sendTimingReq(tmp)) {
        cpu->_status = IcacheWaitResponse;
        cpu->ifetch_pkt = NULL;
    }
}

void
TimingSimpleNCacheCPU::handleDCacheResp(PacketPtr pkt)
{
    // hardware transactional memory

    SimpleExecContext *t_info = threadInfo[curThread];
    [[maybe_unused]] const bool is_htm_speculative =
        t_info->inHtmTransactionalState();

    // received a response from the dcache: complete the load or store
    // instruction
    panic_if(pkt->isError(), "Data access (%s) failed: %s",
            pkt->getAddrRange().to_string(), pkt->print());
    assert(_status == DcacheWaitResponse || _status == DTBWaitResponse ||
           pkt->req->getFlags().isSet(Request::NO_ACCESS));

    pkt->req->setAccessLatency();

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if (pkt->senderState) {
        // hardware transactional memory
        // There shouldn't be HtmCmds occurring in multipacket requests
        if (pkt->req->isHTMCmd()) {
            panic("unexpected HTM case");
        }

        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;
        delete send_state;

        if (pkt->isHtmTransactional()) {
            assert(is_htm_speculative);

            big_pkt->setHtmTransactional(
                pkt->getHtmTransactionUid()
            );
        }

        if (pkt->htmTransactionFailedInCache()) {
            assert(is_htm_speculative);
            big_pkt->setHtmTransactionFailedInCache(
                pkt->getHtmTransactionFailedInCacheRC()
            );
        }

        delete pkt;

        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);
        // Record the fact that this packet is no longer outstanding.
        assert(main_send_state->outstanding != 0);
        main_send_state->outstanding--;

        if (main_send_state->outstanding) {
            return; // if there is still split req to send, do nothing for now
        } else {
            delete main_send_state;
            big_pkt->senderState = NULL;
            pkt = big_pkt;
        }
    }

    _status = BaseSimpleCPU::Running;

    Fault fault;

    // hardware transactional memory
    // sanity checks
    // ensure htmTransactionUids are equivalent
    if (pkt->isHtmTransactional())
        assert (pkt->getHtmTransactionUid() ==
                t_info->getHtmTransactionUid());

    // can't have a packet that fails a transaction while not in a transaction
    if (pkt->htmTransactionFailedInCache())
        assert(is_htm_speculative);

    // shouldn't fail through stores because this would be inconsistent w/ O3
    // which cannot fault after the store has been sent to memory
    if (pkt->htmTransactionFailedInCache() && !pkt->isWrite()) {
        const HtmCacheFailure htm_rc =
            pkt->getHtmTransactionFailedInCacheRC();
        DPRINTF(HtmCpu, "HTM abortion in cache (rc=%s) detected htmUid=%u\n",
            htmFailureToStr(htm_rc), pkt->getHtmTransactionUid());

        // Currently there are only two reasons why a transaction would
        // fail in the memory subsystem--
        // (1) A transactional line was evicted from the cache for
        //     space (or replacement policy) reasons.
        // (2) Another core/device requested a cache line that is in this
        //     transaction's read/write set that is incompatible with the
        //     HTM's semantics, e.g. another core requesting exclusive access
        //     of a line in this core's read set.
        if (htm_rc == HtmCacheFailure::FAIL_SELF) {
            fault = std::make_shared<GenericHtmFailureFault>(
                t_info->getHtmTransactionUid(),
                HtmFailureFaultCause::SIZE);
        } else if (htm_rc == HtmCacheFailure::FAIL_REMOTE) {
            fault = std::make_shared<GenericHtmFailureFault>(
                t_info->getHtmTransactionUid(),
                HtmFailureFaultCause::MEMORY);
        } else {
            panic("HTM - unhandled rc %s", htmFailureToStr(htm_rc));
        }
        endHandlingDCacheResp(pkt, fault);
    } else {
        completeDCacheLoad(pkt);
    }
}

void
TimingSimpleNCacheCPU::endHandlingDCacheResp(PacketPtr pkt,
        Fault fault) {
    SimpleExecContext* t_info = threadInfo[curThread];
    // hardware transactional memory
    // Track HtmStop instructions,
    // e.g. instructions which commit a transaction.
    if (curStaticInst && curStaticInst->isHtmStop()) {
        t_info->thread->htmTransactionStops++;
        DPRINTF(HtmCpu, "htmTransactionStops++=%u\n",
            t_info->thread->htmTransactionStops);
    }

    // keep an instruction count
    if (fault == NoFault)
        countInst();
    else if (traceData) {
        traceFault();
    }

    delete pkt;

    if(!issueNCacheCommands()){
        postExecute();
        advanceInst(fault);
    }
}

void
TimingSimpleNCacheCPU::updateCycleCounts()
{
    const Cycles delta(curCycle() - previousCycle);

    baseStats.numCycles += delta;

    previousCycle = curCycle();
}

void
TimingSimpleNCacheCPU::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // Making it uniform across all CPUs:
    // The CPUs need to be woken up only on an invalidation packet
    // (when using caches) or on an incoming write packet (when not
    // using caches) It is not necessary to wake up the processor on
    // all incoming packets
    if (pkt->isInvalidate() || pkt->isWrite()) {
        for (auto &t_info : cpu->threadInfo) {
            t_info->thread->getIsaPtr()->handleLockedSnoop(pkt,
                    cacheBlockMask);
        }
    } else if (pkt->req && pkt->req->isTlbiExtSync()) {
        // We received a TLBI_EXT_SYNC request.
        // In a detailed sim we would wait for memory ops to complete,
        // but in our simple case we just respond immediately
        auto reply_req = Request::createMemManagement(
            Request::TLBI_EXT_SYNC_COMP,
            cpu->dataRequestorId());

        // Extra Data = the transaction ID of the Sync we're completing
        reply_req->setExtraData(pkt->req->getExtraData());
        PacketPtr reply_pkt = Packet::createRead(reply_req);

        // TODO - reserve some credit for these responses?
        if (!sendTimingReq(reply_pkt)) {
            panic("Couldn't send TLBI_EXT_SYNC_COMP message");
        }
    }
}

void
TimingSimpleNCacheCPU::DcachePort::recvFunctionalSnoop(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }
}

bool
TimingSimpleNCacheCPU::DcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "Received load/store response %#x\n", pkt->getAddr());

    // The timing CPU is not really ticked, instead it relies on the
    // memory system (fetch and load/store) to set the pace.
    if (!tickEvent.scheduled()) {
        // Delay processing of returned data until next CPU clock edge
        tickEvent.schedule(pkt, cpu->clockEdge());
        return true;
    } else {
        // In the case of a split transaction and a cache that is
        // faster than a CPU we could get two responses in the
        // same tick, delay the second one
        if (!retryRespEvent.scheduled())
            cpu->schedule(retryRespEvent, cpu->clockEdge(Cycles(1)));
        return false;
    }
}

void
TimingSimpleNCacheCPU::DcachePort::DTickEvent::process()
{
    cpu->handleDCacheResp(pkt);
}

void
TimingSimpleNCacheCPU::DcachePort::recvReqRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->dcache_pkt != NULL);
    assert(cpu->_status == DcacheRetry);
    PacketPtr tmp = cpu->dcache_pkt;
    if (tmp->senderState) {
        // This is a packet from a split access.
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(tmp->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;

        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);

        if (sendTimingReq(tmp)) {
            // If we were able to send without retrying, record that fact
            // and try sending the other fragment.
            send_state->clearFromParent();
            int other_index = main_send_state->getPendingFragment();
            if (other_index > 0) {
                tmp = main_send_state->fragments[other_index];
                cpu->dcache_pkt = tmp;
                if ((big_pkt->isRead() && cpu->handleReadPacket(tmp)) ||
                        (big_pkt->isWrite() && cpu->handleWritePacket())) {
                    main_send_state->fragments[other_index] = NULL;
                }
            } else {
                cpu->_status = DcacheWaitResponse;
                // memory system takes ownership of packet
                cpu->dcache_pkt = NULL;
            }
        }
    } else if (sendTimingReq(tmp)) {
        cpu->_status = DcacheWaitResponse;
        // memory system takes ownership of packet
        cpu->dcache_pkt = NULL;
    }
}

TimingSimpleNCacheCPU::IprEvent::IprEvent(Packet *_pkt, TimingSimpleNCacheCPU *_cpu,
    Tick t)
    : pkt(_pkt), cpu(_cpu)
{
    cpu->schedule(this, t);
}

void
TimingSimpleNCacheCPU::IprEvent::process()
{
    cpu->handleDCacheResp(pkt);
}

const char *
TimingSimpleNCacheCPU::IprEvent::description() const
{
    return "Timing Simple CPU Delay IPR event";
}


void
TimingSimpleNCacheCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

Fault
TimingSimpleNCacheCPU::initiateMemMgmtCmd(Request::Flags flags)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const Addr addr = 0x0ul;
    const Addr pc = thread->pcState().instAddr();
    const int size = 8;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId());

    req->setPC(pc);
    req->setContext(thread->contextId());
    req->taskId(taskId());
    req->setInstCount(t_info.numInst);

    assert(req->isHTMCmd() || req->isTlbiCmd());

    // Use the payload as a sanity check,
    // the memory subsystem will clear allocated data
    uint8_t *data = new uint8_t[size];
    assert(data);
    uint64_t rc = 0xdeadbeeflu;
    memcpy (data, &rc, size);

    // debugging output
    if (req->isHTMCmd()) {
        if (req->isHTMStart())
            DPRINTF(HtmCpu, "HTMstart htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else if (req->isHTMCommit())
            DPRINTF(HtmCpu, "HTMcommit htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else if (req->isHTMCancel())
            DPRINTF(HtmCpu, "HTMcancel htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else
            panic("initiateMemMgmtCmd: unknown HTM CMD");
    }

    sendData(req, data, nullptr, true);

    return NoFault;
}

void
TimingSimpleNCacheCPU::htmSendAbortSignal(ThreadID tid, uint64_t htm_uid,
                                    HtmFailureFaultCause cause)
{
    SimpleExecContext& t_info = *threadInfo[tid];
    SimpleThread* thread = t_info.thread;

    const Addr addr = 0x0ul;
    const Addr pc = thread->pcState().instAddr();
    const int size = 8;
    const Request::Flags flags =
        Request::PHYSICAL|Request::STRICT_ORDER|Request::HTM_ABORT;

    if (traceData)
        traceData->setMem(addr, size, flags);

    // notify l1 d-cache (ruby) that core has aborted transaction

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId());

    req->setPC(pc);
    req->setContext(thread->contextId());
    req->taskId(taskId());
    req->setInstCount(t_info.numInst);
    req->setHtmAbortCause(cause);

    assert(req->isHTMAbort());

    uint8_t *data = new uint8_t[size];
    assert(data);
    uint64_t rc = 0lu;
    memcpy (data, &rc, size);

    sendData(req, data, nullptr, true);
}


/**
 * Added by jason
 *
 * */

Port& TimingSimpleNCacheCPU::getPort(const std::string& name, PortID idx) {
    if(name == "ncache_port")
        return ncache_port;
    return BaseSimpleCPU::getPort(name, idx);
}

void TimingSimpleNCacheCPU::NCachePort::NCacheRespTickEvent::schedule(
        PacketPtr pkt,
        Cycles cycles) {
    DPRINTF(CapstoneNCache, "NCache response tick event scheduled\n");
    this->pkt = pkt;
    cpu->schedule(this, cpu->clockEdge(cycles));
}

void
TimingSimpleNCacheCPU::handleNCacheResp(PacketPtr pkt) {
    switch(ncache_status){
        case NCACHE_INSTR_EXECUTION:
            if(instPendingMem == NULL) {
                completeNCacheLoad(pkt);
            } else {
                SimpleExecContext* xc = threadInfo[curThread];
                Fault fault = instPendingMem->handleMemResp(statePendingMem, xc, pkt);
                //Fault fault = instPendingMem->handleMemResp(statePendingMem, xc, pkt);
                delete pkt;
                if(!instPendingMem->pendingMem(statePendingMem, xc)){
                    instPendingMem = NULL;
                    statePendingMem = nullptr;
                    completeInstExec(faultPendingMem);
                    faultPendingMem = NoFault;
                }
            }
            break;
        case NCACHE_ISSUE_COMMANDS:
            handleIssueNCacheCommandsResp(pkt);
            break;
        default:
            panic("invalid ncache state");
    }
    
    //switch(ncache_status) {
        //case NCACHE_INSTR_EXECUTION:
            //panic("should never receive ncache resp at INSTR_EXECUTION");
            //break;
        //case NCACHE_ISSUE_COMMANDS:
            //handleIssueNCacheCommandsResp(pkt);
            //break;
        //default:
            //panic("invalid ncache state");
    //}
}

void TimingSimpleNCacheCPU::NCachePort::NCacheRespTickEvent::process() {
    cpu->handleNCacheResp(pkt);
}

bool TimingSimpleNCacheCPU::NCachePort::recvTimingResp(PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "NCache response received\n");
    if(tickEvent.scheduled()){
        DPRINTF(CapstoneNCache, "NCache resp tick event already scheduled\n");
        if(!retryEvent.scheduled())
            cpu->schedule(&retryEvent, cpu->clockEdge(Cycles(1)));
        return false;
    }
    tickEvent.schedule(pkt); // handle the data at the next CPU cycle
    return true;
}

void TimingSimpleNCacheCPU::completeNCacheLoad(PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "NCache load complete\n");

    //assert(nodeResps.empty() || dataResps.empty());
    if(!dataResps.empty()){
        PacketPtr data_pkt = dataResps.front();
        dataResps.pop();
        completeDataAccess(data_pkt, pkt);
    }
    //} else{
        //nodeResps.push(pkt);
    //}
}


void TimingSimpleNCacheCPU::completeDCacheLoad(PacketPtr pkt) {
    DPRINTF(CapstoneNCache, "NCache completeDCacheLoad read %llx\n", pkt->getAddr());
    //if(ncache_status == NCACHE_INSTR_EXECUTION) {
        // now we can finish
    completeDataAccess(pkt, NULL); // TODO: skipping node check for now
    //} else{
        //dataResps.push(pkt); // TODO: query is unncessary
    //}
}

void TimingSimpleNCacheCPU::completeDataAccess(
        PacketPtr data_pkt,
        PacketPtr node_pkt) {
    DPRINTF(CapstoneNCache, "NCache completeDataAccess\n");

    // perform checks on the revocation node
    if(node_pkt == NULL) {
        DPRINTF(CapstoneNCache, "NCache node query skipped\n");
    } else{
        char node_mdata = *node_pkt->getPtr<char>();
        delete node_pkt;

        DPRINTF(CapstoneNCache, "NCache node value: %u\n", node_mdata);
    }

    SimpleExecContext* t_info = threadInfo[curThread];
    //if(node_mdata) {
        Fault fault = curStaticInst->completeAcc(data_pkt, t_info,
                traceData);
        endHandlingDCacheResp(data_pkt, fault);
    //} else{
        //endHandlingDCacheResp(data_pkt, 
                //std::make_shared<AddressFault>(data_pkt->getAddr(),
                    //data_pkt->isRead() ?
                        //ExceptionCode::LOAD_ACCESS :
                        //ExceptionCode::STORE_ACCESS));
    //}
}

void TimingSimpleNCacheCPU::NCachePort::recvReqRetry() {
    assert(cpu->ncache_pkt != NULL);
    
    if(sendTimingReq(cpu->ncache_pkt)) {
        cpu->ncache_pkt = NULL;
    }
}

// Since the syscall emulation is not timing, we delay the actual node allocation
// We simply mark the return register as "to-allocate"
// Then before the CPU next does anything, wait until
// the node is allocated
void
TimingSimpleNCacheCPU::allocObject(ThreadContext* tc, AddrRange obj) {
    node_controller->allocObject(obj);
}

void
TimingSimpleNCacheCPU::freeObject(ThreadContext* tc, Addr base_addr) {
    // FIXME: decide what to do here
    // might need to keep track of the ranges in each capability rather than centrally
    // node_controller->freeObject(base_addr);
}

void
TimingSimpleNCacheCPU::preOverwriteDest(SimpleExecContext& t_info, StaticInst* inst) {
    int num_dest = curStaticInst->numDestRegs();
    // before execution
    // check which destinations will be overwritten
    for(int i = 0; i < num_dest; i ++){
        const RegId& dest_id = curStaticInst->destRegIdx(i);
        if(dest_id.classValue() != RegClassType::IntRegClass)
            continue;
        RegIndex dest_idx = dest_id.index();
        CapLoc loc = CapLoc::makeReg(t_info.thread->threadId(), dest_idx);
        NodeID node_id = node_controller->queryCapTrack(loc);
        if(node_id == NODE_ID_INVALID)
            continue;
        node_controller->removeCapTrack(loc);
        ncToIssue.push(new NodeControllerRcUpdate(node_id, -1));
        node_id = node_controller->queryCapTrack(loc);
        panic_if(node_id != NODE_ID_INVALID, "erase failed");
    }
}

bool
TimingSimpleNCacheCPU::issueNCacheCommands() {
    if(!ncToIssue.empty()){
        DPRINTF(CapstoneNodeOps, "issued command\n");
        NodeControllerCommandPtr cmd = ncToIssue.front();
        ncToIssue.pop();

        sendNCacheCommand(cmd);
        ncache_status = NCACHE_ISSUE_COMMANDS;
        return true;
    }
    return false;
}

void
TimingSimpleNCacheCPU::handleIssueNCacheCommandsResp(PacketPtr pkt) {
    DPRINTF(CapstoneNodeOps, "received resp for issued command\n");
    //assert(ncache_status == NCACHE_ISSUE_COMMANDS);
    delete pkt;
    if(ncToIssue.empty()) {
        ncache_status = NCACHE_INSTR_EXECUTION;
        //if(curStaticInst->isMemRef() && !dataResps.empty()){
            //PacketPtr data_pkt = dataResps.front();
            //dataResps.pop();
            //completeDataAccess(pkt, NULL);
        //} else{
        // TODO: for now the ncache commands are issued after
        // dcache responses are ready
        postExecute();
        advanceInst(NoFault);
        //}
    } else{
        issueNCacheCommands();
    }
}

// issue node controller commands that check the validity of the involved capability
void
TimingSimpleNCacheCPU::issueCapChecks(SimpleExecContext& t_info, 
        StaticInst* inst, Addr addr) {
    DPRINTF(CapstoneNodeOps, "To issue cap check 0x%llx\n", addr);
    std::optional<int> target_obj_idx = node_controller->lookupAddr(addr);
    if(!target_obj_idx)
        return;
    int num_src = inst->numSrcRegs();
    for(int i = 0; i < num_src; i ++){
        const RegId& src_id = inst->srcRegIdx(i);
        if(src_id.classValue() != RegClassType::IntRegClass)
            continue;
        RegIndex src_idx = src_id.index();
        RegVal src_val = t_info.getRegOperand(inst, i);
        std::optional<int> obj_idx = node_controller->lookupAddr((Addr)src_val);
        if(!obj_idx || obj_idx.value() != target_obj_idx.value())
            continue;
        NodeID node_id = node_controller->queryCapTrack(
            CapLoc::makeReg(t_info.tcBase()->threadId(), src_idx));
        if(node_id == NODE_ID_INVALID)
            continue;
        DPRINTF(CapstoneNodeOps, "Issued cap check %u\n", node_id);
        ncToIssue.push(new NodeControllerQuery(node_id));
        break;
    }
}

void
TimingSimpleNCacheCPU::overwriteIntReg(ThreadContext* tc, int reg_idx) {
    CapLoc loc = CapLoc::makeReg(tc->threadId(), reg_idx);
    NodeID node_id = node_controller->queryCapTrack(loc);
    if(node_id != NODE_ID_INVALID) {
        node_controller->removeCapTrack(loc);
        ncToIssue.push(new NodeControllerRcUpdate(node_id, -1));
    }
}

} // namespace gem5


