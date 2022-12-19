#ifndef __CAPSTONE_BITOPS_H__
#define __CAPSTONE_BITOPS_H__

#define CAPSTONE_GETBITS(v,low,high) (((v) >> (low)) & ((1 << ((high) - (low) + 1)) - 1))

#endif
