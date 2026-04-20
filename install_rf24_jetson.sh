#!/bin/bash
# install_rf24_jetson.sh
# Installs RF24 + pyRF24 Python bindings on Jetson Orin Nano
# pyRF24 lives inside the RF24 repo at RF24/pyRF24/

set -e

echo "========================================"
echo "  RF24 Install for Jetson Orin Nano"
echo "========================================"

# Step 1 — dependencies
echo ""
echo "Step 1 — Installing dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
    python3-dev \
    python3-setuptools \
    python3-pip \
    git \
    cmake \
    build-essential \
    swig

# Step 2 — clone RF24 repo (contains both C++ lib and pyRF24)
echo ""
echo "Step 2 — Cloning RF24 repo..."
cd ~
if [ -d "RF24" ]; then
    echo "RF24 already exists — pulling latest..."
    cd RF24 && git pull && cd ~
else
    git clone https://github.com/nRF24/RF24.git
fi

# Step 3 — build and install RF24 C++ library
echo ""
echo "Step 3 — Building RF24 C++ library..."
cd ~/RF24
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~

# Step 4 — build pyRF24 Python bindings from RF24/pyRF24/
echo ""
echo "Step 4 — Building pyRF24 Python bindings..."
cd ~/RF24/pyRF24
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~

# Step 5 — verify
echo ""
echo "Step 5 — Verifying installation..."
python3 -c "
from RF24 import RF24, RF24_PA_MAX, RF24_250KBPS, RF24_CRC_16
print('RF24 import OK')
print('RF24_PA_MAX =', RF24_PA_MAX)
print('RF24 installed successfully!')
"

echo "========================================"
echo "  Done — run: python3 clarq_rf_listener.py"
echo "========================================"
