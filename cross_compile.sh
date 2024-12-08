if [ -z "$CROSS_COMPILE_PREFIX" ]; then
    echo "No CROSS_COMPILE_PREFIX set, using native tools."
    export CC=gcc
    export CXX=g++
    export AR=ar
    export LD=ld
    export OBJCOPY=objcopy
    export OBJDUMP=objdump
    export SIZE=size
    export NM=nm
else
    export CC=$CROSS_COMPILE_PREFIX-gcc
    export CXX=$CROSS_COMPILE_PREFIX-g++
    export AR=$CROSS_COMPILE_PREFIX-ar
    export LD=$CROSS_COMPILE_PREFIX-ld
    export OBJCOPY=$CROSS_COMPILE_PREFIX-objcopy
    export OBJDUMP=$CROSS_COMPILE_PREFIX-objdump
    export SIZE=$CROSS_COMPILE_PREFIX-size
    export NM=$CROSS_COMPILE_PREFIX-nm
fi

echo "CC: $CC"
echo "CXX: $CXX"
echo "AR: $AR"

make "$@"
