#!/bin/bash

ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" && "$ARCH" != "armv7l" ]]; then
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

sudo apt update
sudo apt install -y build-essential cmake pkg-config git
sudo apt install -y libgtk-4-dev

if ! pkg-config --exists gtk4; then
    sudo add-apt-repository universe -y
    sudo apt update
    sudo apt install -y libgtk-4-dev
    
    if ! pkg-config --exists gtk4; then
    fi
else
    GTK_VERSION=$(pkg-config --modversion gtk4)
fi

sudo usermod -a -G dialout $USER
if groups $USER | grep -q "\bdialout\b"; then
    echo "User '$USER' added to dialout group"
else
    echo "Could not add user to dialout group (may need to log out/in)"
fi

# Create udev rule for consistent USB serial access
echo ""
echo "Creating udev rules for USB serial devices..."
sudo bash -c 'cat > /etc/udev/rules.d/99-usb-serial.rules << EOF
# Allow access to USB serial devices for dialout group
SUBSYSTEM=="tty", GROUP="dialout", MODE="0660"
KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
KERNEL=="ttyACM[0-9]*", GROUP="dialout", MODE="0660"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Building the HMI application"
cd "$(dirname "$0")"

if [ -d "build" ]; then
    echo "Removing old build directory"
    rm -rf build
fi

mkdir -p build
cd build

echo "   Running CMake..."
if cmake ..; then
    echo "CMake configuration successful"
else
    echo "CMake configuration failed!"
    exit 1
fi

echo ""
echo "Compiling (this may take a few minutes)"
if make -j$(nproc); then
    echo "Build successful"
else
    echo "Build failed"
    exit 1
fi

cd ..

if ls /dev/ttyUSB* 2>/dev/null; then
    echo "USB serial devices detected:"
    ls -la /dev/ttyUSB* 2>/dev/null
elif ls /dev/ttyACM* 2>/dev/null; then
    echo "ACM serial devices detected:"
    ls -la /dev/ttyACM* 2>/dev/null
else
    echo "No serial devices detected. Connect your Arduino and check:"
    echo "dmesg | tail -20"
fi
