#!/usr/bin/env bash
#
# modem73 update script
# https://github.com/RFnexus/modem73
#

set -e

REPO_URL="https://github.com/RFnexus/modem73.git"
INSTALL_DIR="${MODEM73_DIR:-$HOME/modem73}"
BRANCH="${MODEM73_BRANCH:-master}"

if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    NC='\033[0m'
else
    RED=''
    GREEN=''
    YELLOW=''
    NC=''
fi

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

ask() {
    local prompt="$1" reply=""
    { read -rp "$prompt" reply < /dev/tty; } 2>/dev/null || reply=""
    [[ "$reply" =~ ^[Yy]$ ]]
}

command -v git >/dev/null 2>&1 || error "git is required but not installed"
command -v make >/dev/null 2>&1 || error "make is required but not installed"

if command -v g++ >/dev/null 2>&1; then
    CXX="g++"
elif command -v clang++ >/dev/null 2>&1; then
    CXX="clang++"
else
    error "C++ compiler (g++ or clang++) is required but not installed"
fi

info "modem73 updater"
echo "  Repository: $REPO_URL"
echo "  Branch:     $BRANCH"
echo "  Install to: $INSTALL_DIR"
echo ""

NEEDS_BUILD=1

if [ -d "$INSTALL_DIR/.git" ]; then
    info "Updating existing installation..."
    cd "$INSTALL_DIR"

    if [ -n "$(git status --porcelain)" ]; then
        warn "Local changes detected:"
        git status --short
        echo ""
        if ask "Discard them and update to $BRANCH? [y/N] "; then
            git reset --hard
            git clean -fd
        else
            error "Aborted, local changes kept"
        fi
    fi

    info "Fetching $BRANCH..."
    git fetch --no-tags --quiet origin "$BRANCH" \
        || error "Could not fetch $BRANCH from $REPO_URL"

    CURRENT=$(git rev-parse --abbrev-ref HEAD)
    if [ "$CURRENT" != "$BRANCH" ]; then
        info "Switching from $CURRENT to $BRANCH"
        git checkout --quiet "$BRANCH" 2>/dev/null \
            || git checkout --quiet -b "$BRANCH" FETCH_HEAD
    fi

    if [ "$(git rev-parse HEAD)" = "$(git rev-parse FETCH_HEAD)" ]; then
        info "Already up to date"
        NEEDS_BUILD=0
    elif git merge --ff-only --quiet FETCH_HEAD 2>/dev/null; then
        info "Updated to latest $BRANCH"
    else
        warn "Local branch has diverged from origin/$BRANCH"
        if ask "Reset to origin/$BRANCH? Local commits will be lost. [y/N] "; then
            git reset --hard FETCH_HEAD
        else
            error "Aborted, local commits kept"
        fi
    fi
else
    info "Cloning repository..."
    mkdir -p "$(dirname "$INSTALL_DIR")"
    git clone --branch "$BRANCH" "$REPO_URL" "$INSTALL_DIR"
    cd "$INSTALL_DIR"
fi

if [ "$NEEDS_BUILD" = "1" ] || [ "$1" = "--force" ]; then
    info "Building modem73..."
    make clean >/dev/null 2>&1 || true

    if make -j"$(nproc 2>/dev/null || echo 2)"; then
        info "Build successful"
        echo ""
        echo "  Commit: $(git rev-parse --short HEAD)"
        echo "  Date:   $(git log -1 --format=%ci)"
        echo ""
        if [ -x "$INSTALL_DIR/modem73" ]; then
            info "Binary: $INSTALL_DIR/modem73"
        else
            error "Build reported success but $INSTALL_DIR/modem73 is missing"
        fi
    else
        error "Build failed"
    fi
else
    info "No build needed (use --force to rebuild)"
fi

info "Done!"
