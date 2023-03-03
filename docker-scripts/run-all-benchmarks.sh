function run_benchmark() {
    benchmark="$1"
    echo "Started running benchmark $benchmark"
    cd "$BENCHMARK_DIR"/$benchmark
    for i in 0 1 2; do
        if [ -f "$BENCHMARK_DIR"/$benchmark/$RUN_SCRIPT_NAME$i.sh ]; then
            GEM5_OUT="$OUTPUT_DIR"/$benchmark/$i
            mkdir -p "$GEM5_OUT"
            source "$BENCHMARK_DIR"/$benchmark/$RUN_SCRIPT_NAME$i.sh \
                | tee "$GEM5_OUT"/stdout
        fi
    done
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
