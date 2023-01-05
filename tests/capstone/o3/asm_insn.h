#define NODE_ID_INVALID ((-1) & ((1 << 31) - 1))

// Capability-related constants
#define CAP_PERM_NA 0
#define CAP_PERM_RO 1
#define CAP_PERM_RW 2
#define CAP_PERM_RWX 3

#define CAP_TYPE_LIN 0
#define CAP_TYPE_NONLIN 1
#define CAP_TYPE_REV 2
#define CAP_TYPE_UNINIT 3
#define CAP_TYPE_SEALED 4
#define CAP_TYPE_SEALEDRET 5


