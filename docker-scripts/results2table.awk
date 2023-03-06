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
