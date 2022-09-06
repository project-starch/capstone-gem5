cd coreutils
mkdir -p install
./configure --prefix=$(pwd)/install --host=riscv64-unknown-linux-gnu LDFLAGS="-static -Wl,--wrap=malloc -Wl,--wrap=free -L$(pwd)/.." LIBS="-lc -lpthread -linterp"
make -j$(nproc)
make install
