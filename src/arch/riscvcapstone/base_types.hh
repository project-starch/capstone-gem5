#ifndef __CAPSTONE_BASE_TYPES_H__
#define __CAPSTONE_BASE_TYPES_H__

#include<cstring>
#include<type_traits>

struct uint128_t {
    uint64_t lo, hi;

    uint128_t() = default;
};

static_assert(sizeof(uint128_t) == 16);
static_assert(std::is_aggregate_v<uint128_t>);

struct uint256_t {
    uint128_t lo, hi;

    uint256_t() = default;
};

static_assert(sizeof(uint256_t) == 32);
static_assert(std::is_aggregate_v<uint256_t>);


#endif

