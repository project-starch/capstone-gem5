#ifndef __CAPSTONE_CAP_H_
#define __CAPSTONE_CAP_H_

#include<utility>
#include<cstdio>
#include<cstddef>
#include<cstdint>
#include<cstring>
#include "arch/riscvcapstone/base_types.hh"
#include "arch/riscvcapstone/types.hh"

namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

const size_t CAPSTONE_COMPRESSED_CAP_SIZE = 128; // in bits
const size_t CAPSTONE_UNCOMPRESSED_CAP_SIZE = 256; // in bits


struct CompressedCapBound {
    uint8_t bE;
    uint16_t b;
    uint8_t tE;
    uint16_t t;
    uint8_t iE;

    CompressedCapBound() {}

    CompressedCapBound(uint32_t raw_bound);
    CompressedCapBound(uint64_t base, uint64_t top, uint64_t addr);
    
    std::pair<uint64_t,uint64_t> decode(uint64_t addr) const;
    uint64_t start(uint64_t addr) const;
    uint64_t end(uint64_t addr) const;
    uint32_t toRaw() const;
};

enum class CapPerm {
    NA = 0, // no access
    XO = 1,
    WO = 2,
    WX = 3,
    RO = 4,
    RX = 5,
    RW = 6,
    RWX = 7
};

enum class CapType {
    LIN = 0,
    NONLIN = 1,
    REV = 2,
    UNINIT = 3,
    SEALED = 4,
    SEALEDRET = 5,
    EXIT = 6
};

// TODO: refactoring needed

/**
 * Representation of a Capstone capability
 * TODO: can consider defining a separate more ergonomic representation and only
 * translate into this when storing it in memory
 * */
class CompressedCap {
    private:
        // bool valid: 1;
        uint64_t _cursor;
        //CompressedCapBound compressedBound;
        uint32_t _bound: 27;
        uint8_t _perm: 3;
        uint8_t _type: 3;
        NodeID _node_id: 31;
        // uint8_t async: 2;
        // uint8_t reg: 5;

    //CompressedCap& setBound(const AddrRange& addr_range) {
        //this->setBound(addr_range.start(), addr_range.end());
        //return *this;
    //}

    public:
        CompressedCap() = default;

        CompressedCap(uint128_t v) {
            static_assert(sizeof(*this) == sizeof(v));
            memcpy(this, &v, sizeof(v));
        }

        CapPerm perm() const {
            return static_cast<CapPerm>(_perm);
        }

        CapType type() const {
            return static_cast<CapType>(_type);
        }

        uint64_t cursor() const {
            return _cursor;
        }

        uint64_t start() const;
        uint64_t end() const;

        NodeID nodeId() const {
            return _node_id;
        }

        CompressedCap& setBound(uint64_t start, uint64_t end) {
            return this->setAddresses(start, end, _cursor);
        }

        CompressedCap& setCursor(uint64_t cursor) {
            return this->setAddresses(this->start(), this->end(), cursor);
        }

        CompressedCap& setAddresses(uint64_t start, uint64_t end, uint64_t cursor);

        CompressedCap& setPerm(CapPerm perm) {
            _perm = static_cast<uint8_t>(perm);
            return *this;
        }

        CompressedCap& setType(CapType type) {
            _type = static_cast<uint8_t>(type);
            return *this;
        }

        CompressedCap& setNodeId(NodeID node_id) {
            _node_id = node_id;
            return *this;
        }

        bool hasReadPerm() {
            return (_perm >> 2) & 1;
        }

        bool hasWritePerm() {
            return (_perm >> 1) & 1;
        }

        bool hasExecPerm() {
            return _perm & 1;
        }

        operator uint128_t() const {
            uint128_t v;
            memcpy(&v, this, sizeof(v));
            return v;
        }
        
        void reset() {
            setAddresses(0, 0, 0);
        }
} __attribute__((packed));


static_assert(sizeof(CompressedCap) == (CAPSTONE_COMPRESSED_CAP_SIZE >> 3));

class UncompressedCap {
    private:
        uint64_t _cursor;
        uint64_t _start;
        uint64_t _end;
        uint8_t _perm: 3;
        uint8_t _type: 3;
        NodeID _node_id: 31;
        uint32_t _unused: 27;

    public:
        UncompressedCap() = default;

        UncompressedCap(uint256_t v) {
            static_assert(sizeof(*this) == sizeof(v));
            memcpy(this, &v, sizeof(v));
        }

        CapPerm perm() const {
            return static_cast<CapPerm>(_perm);
        }

        CapType type() const {
            return static_cast<CapType>(_type);
        }

        uint64_t cursor() const {
            return _cursor;
        }

        uint64_t start() const {
            return _start;
        }

        uint64_t end() const {
            return _end;
        }

        UncompressedCap& setBound(uint64_t start, uint64_t end) {
            _start = start;
            _end = end;
            return *this;
        }

        UncompressedCap& setCursor(uint64_t cursor) {
            _cursor = cursor;
            return *this;
        }

        UncompressedCap& setAddresses(uint64_t start, uint64_t end, uint64_t cursor) {
            return setBound(start, end).setCursor(cursor);
        }

        UncompressedCap& setPerm(CapPerm perm) {
            _perm = static_cast<uint8_t>(perm);
            return *this;
        }

        UncompressedCap& setType(CapType type) {
            _type = static_cast<uint8_t>(type);
            return *this;
        }

        UncompressedCap& setNodeId(NodeID node_id) {
            _node_id = node_id;
            return *this;
        }

        NodeID nodeId() const {
            return _node_id;
        }

        bool hasReadPerm() {
            return (_perm >> 2) & 1;
        }

        bool hasWritePerm() {
            return (_perm >> 1) & 1;
        }

        bool hasExecPerm() {
            return _perm & 1;
        }

        operator uint256_t() const {
            uint256_t v;
            memcpy(&v, this, sizeof(v));
            return v;
        }
        
        void reset() {
            setAddresses(0, 0, 0);
        }
} __attribute__((packed));

static_assert(sizeof(UncompressedCap) == (CAPSTONE_UNCOMPRESSED_CAP_SIZE >> 3));
static_assert(sizeof(CompressedCap) == (CAPSTONE_COMPRESSED_CAP_SIZE >> 3));

#ifdef CAPSTONE_USE_UNCOMPRESSED
using Cap = UncompressedCap;
const size_t CAPSTONE_CAP_SIZE = CAPSTONE_UNCOMPRESSED_CAP_SIZE;
#else
using Cap = CompressedCap;
const size_t CAPSTONE_CAP_SIZE = CAPSTONE_COMPRESSED_CAP_SIZE;
#endif

inline bool
capInBound(const Cap& cap) {
    uint64_t cursor = cap.cursor();
    return cursor >= cap.start() && cursor < cap.end();
}

inline bool
capInBound(const Cap& cap, uint64_t addr) {
    return addr >= cap.start() && addr < cap.end();
}

inline bool
capInBoundForMemAcc(const Cap& cap, uint64_t addr, uint64_t size) {
    switch(cap.type()) {
        case CapType::LIN:
        case CapType::NONLIN:
        case CapType::UNINIT:
                if(addr < cap.start() || addr > cap.end() - size)
                    return false;
                return true;
        case CapType::SEALEDRET:
        case CapType::EXIT:
                if(addr < cap.start() + 3*sizeof(Cap) || addr > cap.start() + 33*sizeof(Cap) - size)
                    return false;
                return true;
        default: return false;
    }
}

}
}
}

#endif

