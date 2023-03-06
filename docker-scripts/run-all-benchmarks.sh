function run_benchmark() {
    benchmark="$1"
    benchmark_id="$2"
    echo "Started running benchmark $benchmark"
    cd "$BENCHMARK_DIR"/$benchmark
    if [ -z "$benchmark_id" ]; then
        for i in 0 1 2; do
            if [ -f "$BENCHMARK_DIR"/$benchmark/$RUN_SCRIPT_NAME$i.sh ]; then
                GEM5_OUT="$OUTPUT_DIR"/$benchmark/$i
                mkdir -p "$GEM5_OUT"
                source "$BENCHMARK_DIR"/$benchmark/$RUN_SCRIPT_NAME$i.sh \
                    | tee "$GEM5_OUT"/stdout
                
                # prepare the input file for the second workload of 625.x264_s
                if [ "$i" = "0" -a "$benchmark" = "625.x264_s"]; then
                    cp "$BENCHMARK_DIR"/$benchmark/x264_stats.log.temp "$BENCHMARK_DIR"/$benchmark/x264_stats.log
                    cp "$BENCHMARK_DIR"/$benchmark/x264_stats.log.mbtree.temp "$BENCHMARK_DIR"/$benchmark/x264_stats.log.mbtree
                fi
            fi
        done
    else
        GEM5_OUT="$OUTPUT_DIR"/$benchmark/$benchmark_id
        mkdir -p "$GEM5_OUT"
        source "$BENCHMARK_DIR"/$benchmark/$RUN_SCRIPT_NAME$benchmark_id.sh \
            | tee "$GEM5_OUT"/stdout
    fi
}

function run_all_benchmarks() {
    while read benchmark; do
        if [ "$BENCHMARK_MULTIPROC" = "Y" ]; then
            run_benchmark "$benchmark" > /dev/null 2>&1 &
        else
            run_benchmark "$benchmark"
        fi
    done << EOF
600.perlbench_s
602.gcc_s
605.mcf_s
620.omnetpp_s
623.xalancbmk_s
625.x264_s
631.deepsjeng_s
641.leela_s
648.exchange2_s
657.xz_s
EOF

    if [ "$BENCHMARK_MULTIPROC" = "Y" ]; then
        wait
    fi
}
