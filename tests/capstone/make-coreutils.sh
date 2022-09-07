make

if [ ! -d ./coreutils ]; then 
    wget -q -O /dev/stdout https://ftp.gnu.org/gnu/coreutils/coreutils-9.1.tar.gz | tar xzf /dev/stdin
    mv coreutils-9.1 coreutils
fi

cd coreutils
mkdir -p install
./configure --prefix=$(pwd)/install --host=riscv64-unknown-linux-gnu LDFLAGS="-static -Wl,-z,muldefs -Wl,--wrap=malloc -Wl,--wrap=free -L$(pwd)/.." LIBS="-lc -lpthread -linterp"
make -j$(nproc)
make install
