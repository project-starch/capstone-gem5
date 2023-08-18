#include<utility>
#include<cassert>
#include "base/trace.hh"
#include "debug/Cap.hh"
#include "arch/riscvcapstone/o3/cap.hh"
#include "arch/riscvcapstone/o3/bitops.hh"


namespace gem5 {
namespace RiscvcapstoneISA {
namespace o3 {

CompressedCapBound::CompressedCapBound(uint32_t raw_bound) :
    bE(CAPSTONE_GETBITS(raw_bound, 0, 2)),
    b(CAPSTONE_GETBITS(raw_bound, 3, 13)),
    tE(CAPSTONE_GETBITS(raw_bound, 14, 16)),
    t(CAPSTONE_GETBITS(raw_bound, 17, 25)),
    iE(CAPSTONE_GETBITS(raw_bound, 26, 26)) {
}

std::pair<uint64_t, uint64_t>
CompressedCapBound::decode(uint64_t addr) const {
    uint8_t E;
    uint16_t T = t << 3;
    uint16_t B = b << 3;
    uint8_t Lcarry;
    uint8_t Lmsb;
    
    DPRINTX(Cap, "Addr %llx, (%llx, %llx, %llx, %llx, %llx)",
        addr, bE, b, tE, t, iE);

    if(iE == 0) {
        E = 0;
        T |= tE;
        B |= bE;
        Lmsb = 0;
    } else {
        E = (tE << 3) | bE;
        Lmsb = 1;
    }
    
    if(T < (B & ((1 << 12) - 1))) {
        Lcarry = 1;
    } else{
        Lcarry = 0;
    }

    DPRINTX(Cap, " E = %llx, Lmsb = %u, Lcarry = %u, T = %llx, B = %llx", E, 
        Lmsb, Lcarry,
        T, B);


    T |= ((B >> 12) + Lcarry + Lmsb) << 12;
    DPRINTX(Cap, " T' = %llx", T);

    assert(E < 64);
    uint64_t top = (addr & ((((uint64_t)-1) >> (E + 14)) << (E + 14))) |
        (T << E);
    uint64_t base = (addr & ((((uint64_t)-1) >> (E + 14)) << (E + 14))) |
        (B << E);

    // correction
    uint64_t R = (B >> 11) - 1;
    uint64_t A = (addr >> (E + 11)) & 7;
    uint64_t T3 = (T >> 11) & 7;
    uint64_t B3 = (B >> 11) & 7;

    bool condAR = A < R;
    bool condTR = T3 < R;
    bool condBR = B3 < R;
    
    if(!condAR && condTR) {
        top += (uint64_t)1 << (E + 14);
    } else if(condAR && !condTR) {
        top -= (uint64_t)1 << (E + 14);
    }

    if(!condAR && condBR) {
        base += (uint64_t)1 << (E + 14);
    } else if(condAR && !condBR) {
        base -= (uint64_t)1 << (E + 14);
    }
    
    DPRINTX(Cap, " -> (%llx, %llx) correction = (%u, %u, %u, %u)\n",
        base, top,
        !condAR && condTR,
        condAR && !condTR,
        !condAR && condBR,
        condAR && !condBR);

    return std::make_pair(base, top);
}

uint64_t
CompressedCapBound::start(uint64_t addr) const {
    return decode(addr).first;
}

uint64_t
CompressedCapBound::end(uint64_t addr) const {
    return decode(addr).second;
}


// constructor that encode the given bound
CompressedCapBound::
CompressedCapBound(uint64_t base, uint64_t top, uint64_t addr) {
    assert(top >= base);
    DPRINTX(Cap, "(%llx, %llx, %llx)",
        base, top, addr);
    
    uint64_t len = top - base;
    int E = 51;
    uint64_t T, B;
    for(int i = 63; i >= 13 && ((len >> i) & 1) == 0; -- i, -- E);
    if(E == 0 && ((len >> 12) & 1) == 0) {
        iE = 0;
        B = base & ((1 << 14) - 1);
        T = top & ((1 << 12) - 1);
        bE = B & 7;
        b = B >> 3;
        tE = T & 7;
        t = T >> 3;
    } else {
        iE = 1;
        B = (base >> (E + 3)) & ((1 << 11) - 1);
        T = (top >> (E + 3)) & ((1 << 9) - 1);
        DPRINTX(Cap, " B = %llx, T = %llx, top = %llx, E = %llx", B, T, top, E);
        if(top > ((top >> (E + 3)) << (E + 3))) {
            ++ T;
            DPRINTX(Cap, " top rounded up");
            for(int j = 1; j <= 9; j ++) {
                if(T == (1 << j)) {
                    // msb shifts
                    DPRINTX(Cap, " (msb shifted)");
                    T = 0;
                    B >>= 1;
                    ++ E;
                    break;
                }
            }
        }
        bE = E & 7;
        tE = E >> 3;
        b = B;
        t = T;
    }
    
    DPRINTX(Cap, " -> (%llx, %llx, %llx, %llx, %llx)\n",
        bE, b, tE, t, iE);
}

uint32_t
CompressedCapBound::toRaw() const {
#define _SHIFT(v,n) ((uint32_t)(v) << (n))
    return _SHIFT(bE, 0) |
        _SHIFT(b, 3) |
        _SHIFT(tE, 14) |
        _SHIFT(t, 17) |
        _SHIFT(iE, 26);
#undef _SHIFT
}

uint64_t
CompressedCap::start() const {
    CompressedCapBound bound(this->_bound);
    return bound.start(_cursor);
}

uint64_t
CompressedCap::end() const {
    CompressedCapBound bound(this->_bound);
    return bound.end(_cursor);
}

CompressedCap&
CompressedCap::setAddresses(uint64_t start, uint64_t end, uint64_t cursor) {
    CompressedCapBound bound(start, end, cursor);
    _bound = bound.toRaw();
    _cursor = cursor;
    return *this;
}

NewCap::NewCap(uint128_t c) {
    __uint128_t v;
    memcpy(&v, &c, sizeof(c));
    _type = (v >> 94) & ((1 << 3) - 1);
    _node_id = (v >> 97) & ((1 << 31) - 1);

    CapType type = static_cast<CapType>(_type);

    if(type == CapType::LIN || type == CapType::NONLIN || type == CapType::REV || type == CapType::UNINIT) {
        _cursor = v & ((1 << 64) - 1);
        uint32_t bound = (v >> 64) & ((1 << 27) - 1);
        DPRINTX(Cap, "Decode: bound = %x\n", bound);
        CompressedCapBound cb(bound);
        _start = cb.start(_cursor);
        _end = cb.end(_cursor);
        DPRINTX(Cap, "Decode: start = %x, end = %x\n", _start, _end);
        DPRINTX(Cap, "Decode: cursor = %x\n", _cursor);
        _perm = (v >> 91) & ((1 << 3) - 1);
    } else {
        _start = v & ((1 << 64) - 1);

        if(type == CapType::SEALED) {
            _async = (v >> 92) & ((1 << 2) - 1);
        } else {
            uint64_t cursor_offset = (v >> 64) & ((1 << 23) - 1);
            _cursor = _start + cursor_offset;

            if(type == CapType::SEALEDRET) {
                _reg = (v >> 87) & ((1 << 5) - 1);
                _async = (v >> 92) & ((1 << 2) - 1);
            }
        }
    }
}

NewCap::operator uint128_t() const {
    __uint128_t v = 0;

    CapType type = static_cast<CapType>(_type);

    if(type == CapType::LIN || type == CapType::NONLIN || type == CapType::REV || type == CapType::UNINIT) {
        uint32_t bound = CompressedCapBound(_start, _end, _cursor).toRaw();
        v = __uint128_t(_cursor) | (__uint128_t(bound) << 64) | (__uint128_t(_perm) << 91) | (__uint128_t(_type) << 94) | (__uint128_t(_node_id) << 97);
        DPRINTX(Cap, "Encode: bound = %x\n", bound);
    } else {
        if(type == CapType::SEALED) {
            v = __uint128_t(_start) | (__uint128_t(_async) << 92) | (__uint128_t(_type) << 94) | (__uint128_t(_node_id) << 97);
        } else {
            uint64_t cursor_offset = (_cursor - _start) & ((1 << 23) - 1);
            uint8_t reg_5bits = _reg & ((1 << 5) - 1);

            if (type == CapType::SEALEDRET) {
                v = __uint128_t(_start) | (__uint128_t(cursor_offset) << 64) | (__uint128_t(reg_5bits) << 87) | (__uint128_t(_async) << 92) | (__uint128_t(_type) << 94) | (__uint128_t(_node_id) << 97);
            } else { //CapType::EXIT
                v = __uint128_t(_start) | (__uint128_t(cursor_offset) << 64) | (__uint128_t(reg_5bits) << 94) | (__uint128_t(_node_id) << 97);
            }
        }
    }

    uint128_t* c = new uint128_t;
    memcpy(c, &v, sizeof(v));
    return *c;
}
}
}
}
