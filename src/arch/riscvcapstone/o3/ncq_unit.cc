#include "cpu/thread_context.hh"
#include "arch/riscvcapstone/o3/ncq_unit.hh"
#include "arch/riscvcapstone/o3/dyn_inst.hh"
#include "arch/riscvcapstone/o3/ncq.hh"
#include "arch/riscvcapstone/o3/lsq.hh"
#include "arch/riscvcapstone/o3/iew.hh"
#include "debug/NCQ.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {


NCQUnit::NCQUnit(ThreadID thread_id, int queue_size,
        NCQ* ncq, IEW* iew) :
    threadId(thread_id),
    ncQueue(queue_size),
    queueSize(queue_size),
    ncq(ncq), iew(iew)
{
}

void
NCQUnit::insertInstruction(const DynInstPtr& inst) {
    assert(!ncQueue.full());
    ncQueue.advance_tail();
    ncQueue.back() = NCQEntry(inst);

    inst->ncqIdx = ncQueue.tail();
    inst->ncqIt = ncQueue.end() - 1;

    assert(!ncQueue.empty());
    assert(ncQueue.size() != 0);
    DPRINTF(NCQ, "Pushed instruction %u to %u of NCQ thread %u\n",
            inst->seqNum, inst->ncqIdx, threadId);
}

void
NCQUnit::tick() {
}

Fault
NCQUnit::pushCommand(const DynInstPtr& inst, NodeCommandPtr cmd) {
    assert(inst->ncqIdx != -1); // inst has been inserted to this queue
    NCQEntry& ncq_entry = *(inst->ncqIt);
    assert(ncq_entry.inst->seqNum == inst->seqNum); // indeed the same inst in the entry

    ncq_entry.commands.push_back(cmd);

    return NoFault;
}

bool
NCQUnit::isFull() {
    return ncQueue.full();
}

void
NCQUnit::commitBefore(InstSeqNum seq_num) {
    DPRINTF(NCQ, "Committing instructions before %u in thread %u NCQ" 
            " (containing %u instructions)\n",
            seq_num, threadId, ncQueue.size());
    for(NCQIterator it = ncQueue.begin(); 
            it != ncQueue.end() && it->inst->seqNum <= seq_num;
            ++ it) {
        DPRINTF(NCQ, "Marking commands as canWB\n");
        it->canWB = true;
    }
}

void
NCQUnit::cleanupCommands(){
    while(!ncQueue.empty()) {
        auto& front = ncQueue.front();
        if(front.canWB && front.completed()) {
            ncQueue.pop_front();
        } else{
            break;
        }
    }
}

void
NCQUnit::writebackCommands(){
    // not doing lots of reordering right now
    for(NCQIterator it = ncQueue.begin();
            it != ncQueue.end() && ncq->canSend(); ++ it) {
        if(!it->inst->isNodeInitiated() || it->completed())
            // not doing anything for instructions not yet executed
            continue;
        std::vector<NodeCommandPtr>& commands = it->commands;
        DPRINTF(NCQ, "Instruction %u with %u commands (completed = %u)\n", it->inst->seqNum, commands.size(), 
                it->completedCommands);
        for(NodeCommandIterator nc_it = commands.begin();
                nc_it != commands.end() && ncq->canSend();
                ++ nc_it) {
            NodeCommandPtr nc_ptr = *nc_it;
            assert(nc_ptr);
            DPRINTF(NCQ, "Command status = %u, before commit = %u\n",
                    static_cast<unsigned int>(nc_ptr->status),
                    nc_ptr->beforeCommit());
            if(nc_ptr->status == NodeCommand::COMPLETED || 
                nc_ptr->status == NodeCommand::AWAIT_CACHE)
                //(!nc_ptr->beforeCommit() && !it->canWB))
                continue;

            if(nc_ptr->status == NodeCommand::NOT_STARTED) {
                auto& cond_ptr = nc_ptr->condition;
                auto saved_req = it->inst->savedRequest;
                if(cond_ptr && saved_req &&
                        (!saved_req->isComplete() || !cond_ptr->satisfied(saved_req))){
                    DPRINTF(NCQ, "Command bypassed as condition is not satisfied\n");
                    // cannot process if the request has not completed or 
                    // the condition is not satisfied
                    continue;
                }
            }

            DPRINTF(NCQ, "Checking command dependencies\n");

            // check for dependencies
            // the naive way. Bruteforce
            bool dep_ready = true;
            for(NCQIterator it_o = ncQueue.begin();
                    dep_ready && it_o != ncQueue.end();
                    ++ it_o) {
                for(NodeCommandIterator nc_it_o = it_o->commands.begin(); 
                        nc_it_o != it_o->commands.end() && (it_o != it ||
                            nc_it_o != nc_it); 
                        ++ nc_it_o) {
                    NodeCommandPtr nc_ptr_o = *nc_it_o;
                    assert(nc_ptr_o);
                    if(nc_ptr_o->status != NodeCommand::COMPLETED && 
                            !ncOrder.reorderAllowed(nc_ptr_o, nc_ptr)){
                        dep_ready = false;
                        break;
                    }
                }
                if(it_o == it)
                    break;
            }
            
            if(!dep_ready)
                continue;

            DPRINTF(NCQ, "Command ready to execute (instruction %u)\n",
                    it->inst->seqNum);

            // the command can be executed
            // one state transition in the state machine
            PacketPtr pkt = nc_ptr->transition();
            if(pkt) {
                ncq->trySendPacket(pkt, threadId);
                DPRINTF(NCQ, "Packet sent for command\n");
                // record which command the packet originates from
                // to deliver the packet back once the response if received
                assert(packetIssuers.find(pkt->id) == packetIssuers.end());
                packetIssuers[pkt->id] = nc_ptr;
            } else if(nc_ptr->status == NodeCommand::COMPLETED) {
                completeCommand(nc_ptr);
            }
        }

    }
}


void
NCQUnit::completeCommand(NodeCommandPtr node_command){
    DynInstPtr& inst = node_command->inst;
    DPRINTF(NCQ, "Command for instruction %u completed\n", inst->seqNum);
    inst->completeNodeAcc(node_command);
    ++ inst->ncqIt->completedCommands;
    if(inst->ncqIt->completed() &&
            inst->hasNodeWB()) {
        DPRINTF(NCQ, "Instruction %u can now be committed\n",
                inst->seqNum);
        inst->setExecuted();
        iew->instToCommit(inst);
    }
}

bool
NCQUnit::handleCacheResp(PacketPtr pkt) {
    auto it = packetIssuers.find(pkt->id);
    assert(it != packetIssuers.end());
    NodeCommandPtr node_cmd = it->second;
    assert(node_cmd);
    packetIssuers.erase(it);
    DPRINTF(NCQ, "Node cache response received for instruction %u, cmd beforeCommit = %u\n",
            it->second->inst->seqNum, it->second->beforeCommit());
    node_cmd->handleResp(pkt);
    DPRINTF(NCQ, "Command handler new status = %u\n", static_cast<unsigned int>(node_cmd->status));
    if(node_cmd->status == NodeCommand::COMPLETED) {
        completeCommand(node_cmd);
    }

    return true;
}

bool
NCQUnit::passedQuery(const DynInstPtr& inst) const {
    assert(inst->threadNumber == threadId);
    if(!inst->isNodeOp()) // no associated command
        return true;
    assert(inst->ncqIdx != -1);
    auto& commands = inst->ncqIt->commands;
    for(auto it = commands.begin();
            it != commands.end();
            ++ it) {
        NodeCommandPtr& node_command = *it;
        if(node_command->beforeCommit() &&
                (node_command->status != NodeCommand::COMPLETED ||
                 node_command->error())) {
            return false;
        }
    }
    return true;
}


}
}
}


