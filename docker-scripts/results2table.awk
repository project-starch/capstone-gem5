# Copyright (c) 2023 National University of Singapore
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/====/ {
    workload = $2
    sub(/_/, "\\_", workload)
}

/capstone\/riscv.nommu/ {
    gsub(/\//, " ")
    res_riscv = $5
    res_capstone = $4
}

/Allocate/ {
    allocate_count = $2
}

/RcUpdate/ {
    rc_update_count = $2
}

/Revoke/ {
    revoke_count = $2
}

/Query/ {
    query_count = $2
}

/Misses\/Hits/ {
    gsub(/\//, " ")
    misses = $3
    hits = $4
    if(res_riscv == 0)
        ratio = 0
    else
        ratio = res_capstone/res_riscv
    if(ratio < 1)
        ratio = 1
    
    tot_req = misses + hits
    if(tot_req > 0)
        miss_rate = misses / tot_req * 100
    else
        miss_rate = "N/A"

    printf "%s & %.3f & %.3f & %.3f & %d & %d & %.3f & %d & %d & %d & %d \\\\\n",
        workload, res_capstone, res_riscv,
            (ratio - 1) * 100, misses, hits, miss_rate,
            allocate_count, query_count, rc_update_count, revoke_count
}

ENDFILE {
    print "\\hline"
}
