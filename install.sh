#!/usr/bin/env bash
set -e

if [ "$(id -u)" = "0" ]; then
    echo "Do not run this script as root. It will ask for sudo when needed."
    exit 1
fi

command -v apt >/dev/null 2>&1 || { echo "This script requires a Debian-based system with apt."; exit 1; }

PARENT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== MODEM73 Installer ==="
echo "Install directory: $PARENT_DIR"
echo ""

PACKAGES="git build-essential libncurses-dev g++"

read -rp "Install hamlib for rigctl PTT support? [y/N] " HAMLIB
if [[ "$HAMLIB" =~ ^[Yy]$ ]]; then
    PACKAGES="$PACKAGES libhamlib-dev libhamlib-utils"
fi

read -rp "Install libhidapi-dev for CM108 USB PTT support? [y/N] " CM108
if [[ "$CM108" =~ ^[Yy]$ ]]; then
    PACKAGES="$PACKAGES libhidapi-dev"
fi

echo ""
echo "Installing packages: $PACKAGES"
sudo apt update
sudo apt install -y $PACKAGES

cd "$PARENT_DIR"

for repo in dsp code modem; do
    if [ -d "$repo" ]; then
        echo "$repo/ already exists, skipping clone."
    else
        git clone "https://github.com/aicodix/${repo}.git"
    fi
done

if [ ! -d "modem73" ]; then
    git clone "https://github.com/RFnexus/modem73.git"
fi

cd modem73
make clean 2>/dev/null || true
make -j"$(nproc)" AICODIX_DSP=../dsp AICODIX_CODE=../code MODEM_SRC=../modem

echo ""
echo "Build complete."
echo ""
read -rp "Install modem73 to /usr/local/bin? [y/N] " INSTALL
if [[ "$INSTALL" =~ ^[Yy]$ ]]; then
    sudo make install
    echo ""
    echo "Run: modem73 to launch"
else
    echo ""
    echo "Run: cd $(pwd) && ./modem73 to launch"
fi
