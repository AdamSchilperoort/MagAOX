#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/../_common.sh
set -euo pipefail

# Check if OpenBLAS is already installed
if [[ -f "/opt/MagAOX/vendor/lib/libopenblas.a" ]] || [[ -f "/opt/MagAOX/vendor/lib/libopenblas.so" ]]; then
    log_info "OpenBLAS is already installed in /opt/MagAOX/vendor/lib, skipping build"
    exit 0
fi

# For explicit TARGET OpenBLAS make option, detect CPU and set their corresponding variable 
detect_cpu_target() {
    local arch=$(uname -m)
    
    if [[ "$arch" == "aarch64" ]] || [[ "$arch" == "arm64" ]]; then
        # ARM64 architectures
        local cpu_part=$(grep "^CPU part" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "0")
        local cpu_revision=$(grep "^CPU revision" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "0")
        
        # ARM Cortex-A series detection
        case "$cpu_part" in
            0xd03) # Cortex-A53
                echo "CORTEXA53"
                return 0
                ;;
            0xd07) # Cortex-A57
                echo "CORTEXA57"
                return 0
                ;;
            0xd08) # Cortex-A72
                echo "CORTEXA72"
                return 0
                ;;
            0xd0b) # Cortex-A76
                echo "CORTEXA76"
                return 0
                ;;
            0xd0c) # Cortex-A55
                echo "CORTEXA55"
                return 0
                ;;
            0xd0d) # Cortex-A77
                echo "CORTEXA77"
                return 0
                ;;
            0xd0e) # Cortex-A78
                echo "CORTEXA78"
                return 0
                ;;
            0xd41) # Cortex-A710
                echo "CORTEXA710"
                return 0
                ;;
            0xd44) # Cortex-A510
                echo "CORTEXA510"
                return 0
                ;;
            0xd49) # Cortex-X1
                echo "CORTEXX1"
                return 0
                ;;
            0xd4a) # Cortex-X2
                echo "CORTEXX2"
                return 0
                ;;
            0xd4b) # Cortex-X3
                echo "CORTEXX3"
                return 0
                ;;
            *)
                echo "ARMV8"
                return 0
                ;;
        esac
        
    elif [[ "$arch" == "armv7l" ]] || [[ "$arch" == "arm" ]]; then
        # ARM 32-bit architectures
        local hardware=$(grep "^Hardware" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "Unknown")
        
        case "$hardware" in
            *"BCM2708"*|*"BCM2835"*) # Raspberry Pi 1
                echo "ARMV6"
                return 0
                ;;
            *"BCM2709"*|*"BCM2836"*) # Raspberry Pi 2
                echo "ARMV7"
                return 0
                ;;
            *"BCM2710"*|*"BCM2837"*) # Raspberry Pi 3
                echo "ARMV8"
                return 0
                ;;
            *"BCM2711"*|*"BCM2838"*) # Raspberry Pi 4
                echo "ARMV8"
                return 0
                ;;
            *)
                echo "ARMV7"
                return 0
                ;;
        esac
        
    elif [[ "$arch" == "ppc64le" ]] || [[ "$arch" == "ppc64" ]]; then
        # PowerPC architectures
        echo "POWER8"
        return 0
        
    elif [[ "$arch" == "x86_64" ]]; then
        # x86_64 architectures (Intel/AMD)
        local cpu_family=$(grep "^cpu family" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "0")
        local cpu_model=$(grep "^model" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "0")
        local vendor=$(grep "^vendor_id" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "Unknown")
        
        # Check if it's AMD or Intel
        if [[ "$vendor" == *"AMD"* ]]; then
            # AMD Zen series
            case "$cpu_family:$cpu_model" in
                25:1) # Zen (Family 25, Model 1)
                    echo "ZEN"
                    return 0
                    ;;
                25:17) # Zen+ (Family 25, Model 17)
                    echo "ZEN2"
                    return 0
                    ;;
                25:24) # Zen2 (Family 25, Model 24)
                    echo "ZEN2"
                    return 0
                    ;;
                25:49) # Zen3 (Family 25, Model 49)
                    echo "ZEN3"
                    return 0
                    ;;
                25:80) # Zen4 (Family 25, Model 80)
                    echo "ZEN4"
                    return 0
                    ;;
                *)
                    echo "ZEN"
                    return 0
                    ;;
            esac
        else
            # Intel series
            case "$cpu_family:$cpu_model" in
                6:60) # Sandy Bridge
                    echo "SANDYBRIDGE"
                    return 0
                    ;;
                6:69) # Haswell
                    echo "HASWELL"
                    return 0
                    ;;
                6:70) # Broadwell
                    echo "BROADWELL"
                    return 0
                    ;;
                6:85) # Skylake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:142) # Coffee Lake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:158) # Coffee Lake Refresh
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:165) # Comet Lake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:166) # Comet Lake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:167) # Rocket Lake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:170) # Alder Lake / Meteor Lake
                    echo "HASWELL"
                    return 0
                    ;;
                6:183) # Raptor Lake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                6:186) # Arrow Lake
                    echo "SKYLAKEX"
                    return 0
                    ;;
                *)
                    echo "GENERIC"
                    return 0
                    ;;
            esac
        fi
        
    else
        # Unknown architecture
        echo "UNKNOWN"
        return 1
    fi
}

cd /opt/MagAOX/vendor
log_info "Install OpenBLAS from source"
VERSION=0.3.24
DOWNLOAD_FILE=OpenBLAS-${VERSION}.tar.gz
DOWNLOAD_URL=https://github.com/xianyi/OpenBLAS/releases/download/v${VERSION}/${DOWNLOAD_FILE}

if [[ ! -e $DOWNLOAD_FILE ]]; then
    _cached_fetch $DOWNLOAD_URL $DOWNLOAD_FILE
fi

if [[ ! -d ./OpenBLAS-${VERSION} ]]; then
    tar xf $DOWNLOAD_FILE
fi

cd OpenBLAS-${VERSION}

DETECTED_TARGET=$(detect_cpu_target)

# Log CPU detection results for debugging
log_info "CPU detection completed"
if [[ "$DETECTED_TARGET" != "UNKNOWN" ]]; then
    # Get CPU details for logging
    cpu_family=$(grep "^cpu family" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "Unknown")
    cpu_model=$(grep "^model" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "Unknown")
    cpu_name=$(grep "^model name" /proc/cpuinfo | head -1 | awk -F': ' '{print $2}' 2>/dev/null || echo "Unknown")
    log_info "Detected CPU: $cpu_name (Family: $cpu_family, Model: $cpu_model)"
    log_info "Recommended OpenBLAS target: $DETECTED_TARGET"
else
    log_info "CPU detection failed, will use fallback targets"
fi

ALL_TARGETS=("GENERIC" "HASWELL" "SKYLAKEX" "NEHALEM" "SANDYBRIDGE" "BROADWELL" "ZEN" "ZEN2" "ZEN3" "ZEN4" "CORTEXA53" "CORTEXA55" "CORTEXA57" "CORTEXA72" "CORTEXA76" "CORTEXA77" "CORTEXA78" "CORTEXA710" "CORTEXA510" "CORTEXX1" "CORTEXX2" "CORTEXX3" "ARMV6" "ARMV7" "ARMV8" "POWER8")

if [[ "$DETECTED_TARGET" == "UNKNOWN" ]]; then
    log_info "CPU detection failed, using fallback targets"
    TARGETS=("${ALL_TARGETS[@]}")
else
    log_info "Using detected target: $DETECTED_TARGET"
    TARGETS=("$DETECTED_TARGET" "${ALL_TARGETS[@]}") # fall back to others if first target fails
fi

BUILD_SUCCESS=false
BUILT_TARGET=""

for target in "${TARGETS[@]}"; do
    log_info "Trying to build OpenBLAS with TARGET=$target"
    
    make clean 2>/dev/null || true
    
    log_info "Running: make TARGET=$target USE_OPENMP=1 USE_CBLAS=1 -j$(nproc)"
    if make TARGET=$target USE_OPENMP=1 USE_CBLAS=1 -j$(nproc) 2>&1 | tee /tmp/openblas_build_${target}.log; then
        log_info "OpenBLAS build successful with TARGET=$target"
        BUILD_SUCCESS=true
        BUILT_TARGET=$target
        break
    else
        log_warn "OpenBLAS build failed with TARGET=$target"
        log_warn "Build log saved to /tmp/openblas_build_${target}.log"
        log_warn "Last 20 lines of build output:"
        tail -20 /tmp/openblas_build_${target}.log
        log_warn "Trying next target..."
    fi
done

# If all threaded builds failed, try without threading
if [[ "$BUILD_SUCCESS" == "false" ]]; then
    log_info "All threaded builds failed, trying without threading (USE_THREAD=0)"
    
    for target in "${TARGETS[@]}"; do
        log_info "Trying to build OpenBLAS with TARGET=$target USE_THREAD=0"
        
        make clean 2>/dev/null || true
        
        if make TARGET=$target USE_THREAD=0 USE_CBLAS=1 -j$(nproc); then
            log_info "OpenBLAS build successful with TARGET=$target USE_THREAD=0"
            BUILD_SUCCESS=true
            BUILT_TARGET=$target
            break
        else
            log_warn "OpenBLAS build failed with TARGET=$target USE_THREAD=0, trying next target..."
        fi
    done
fi

if [[ "$BUILD_SUCCESS" == "false" ]]; then
    log_error "All OpenBLAS build attempts failed"
    exit 1
fi

log_info "Build successful with TARGET=$BUILT_TARGET"

# OpenBLAS name convention, library name uses pattern: libopenblas_<target>p-r<version>.a
BUILT_LIB="libopenblas_${BUILT_TARGET,,}p-r${VERSION}.a"

# Verify the expected .a exists 
if [[ ! -f "$BUILT_LIB" ]]; then
    log_error "Expected library $BUILT_LIB not found!"
    log_error "Available libraries:"
    ls -la libopenblas*.a 2>/dev/null || log_error "No libopenblas*.a files found"
    exit 1
fi

# Create installation directories
sudo mkdir -p /opt/MagAOX/vendor/lib
sudo mkdir -p /opt/MagAOX/vendor/include

log_info "Copying library to /opt/MagAOX/vendor/lib/"
sudo cp "$BUILT_LIB" "/opt/MagAOX/vendor/lib/"

# Symbolic links for compatibility, apps use generic name
log_info "Creating symbolic links"
cd /opt/MagAOX/vendor/lib
sudo ln -sf "$(basename "$BUILT_LIB")" "libopenblas.a"
sudo ln -sf "$(basename "$BUILT_LIB")" "libopenblas.so"

# Copy headers
log_info "Copying headers to /opt/MagAOX/vendor/include/"
cd /opt/MagAOX/vendor/OpenBLAS-0.3.24
sudo cp -r include/* /opt/MagAOX/vendor/include/ 2>/dev/null || true

# Copy other header files that are in root
sudo cp *.h /opt/MagAOX/vendor/include/ 2>/dev/null || true

# Copy CBLAS headers as well
log_info "Installing CBLAS headers"
if [[ -d "cblas/include" ]]; then
    sudo mkdir -p /opt/MagAOX/vendor/include/cblas
    sudo cp cblas/include/*.h /opt/MagAOX/vendor/include/cblas/
    # Create compatibility symlink
    sudo ln -sf /opt/MagAOX/vendor/include/cblas/cblas.h /opt/MagAOX/vendor/include/cblas.h
    log_info "CBLAS headers installed"
else
    log_warn "CBLAS headers not found in source, CBLAS support may be limited"
fi

# Create OpenBLAS pkg-config file
log_info "Creating pkg-config file"
sudo mkdir -p /opt/MagAOX/vendor/lib/pkgconfig
cat > /tmp/openblas.pc << EOF
prefix=/opt/MagAOX/vendor
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: OpenBLAS
Description: OpenBLAS is an optimized BLAS library with CBLAS support
Version: ${VERSION}
Libs: -L\${libdir} -lopenblas
Cflags: -I\${includedir} -I\${includedir}/cblas
Requires: 
EOF

sudo cp /tmp/openblas.pc /opt/MagAOX/vendor/lib/pkgconfig/
rm /tmp/openblas.pc

# Create CBLAS pkg-config file
log_info "Creating CBLAS pkg-config file"
cat > /tmp/cblas.pc << EOF
prefix=/opt/MagAOX/vendor
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: CBLAS
Description: CBLAS (C interface to BLAS) from OpenBLAS
Version: ${VERSION}
Libs: -L\${libdir} -lopenblas
Cflags: -I\${includedir}/cblas
Requires: openblas
EOF

sudo cp /tmp/cblas.pc /opt/MagAOX/vendor/lib/pkgconfig/
rm /tmp/cblas.pc

log_info "OpenBLAS installation complete"
