#!/usr/bin/env bash
# =============================================================================
#  jetson_setup.sh  —  One-time setup for both Jetson Orin Nanos
#                       Run on BOTH devices
# =============================================================================

set -euo pipefail

GREEN='\033[0;32m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
log() { echo -e "${CYAN}[setup]${NC} $*"; }
ok()  { echo -e "${GREEN}[done]${NC}  $*"; }

REMOTE_USER="${1:-clarq}"
REMOTE_HOST="${2:-10.42.0.1}"
DROP_DIR="/srv/jetson_drop"

echo -e "\n${BOLD}━━━ Jetson WiFi Transfer — One-Time Setup ━━━━━━━━━━━━━━━━${NC}\n"

# ── Install dependencies ──────────────────────────────────────────────────────
log "Installing dependencies..."
sudo apt-get update -qq
sudo apt-get install -y sshfs inotify-tools zip unzip openssh-server
ok "Dependencies installed"

# ── Create drop directory on THIS machine ─────────────────────────────────────
log "Creating drop directory: $DROP_DIR"
sudo mkdir -p "$DROP_DIR"
sudo chown "$USER:$USER" "$DROP_DIR"
sudo chmod 755 "$DROP_DIR"
ok "Drop directory ready"

# ── SSH key setup (passwordless auth) ────────────────────────────────────────
if [[ ! -f "$HOME/.ssh/id_ed25519" ]]; then
  log "Generating SSH key pair..."
  ssh-keygen -t ed25519 -C "jetson-transfer" -f "$HOME/.ssh/id_ed25519" -N ""
  ok "SSH key generated"
else
  ok "SSH key already exists"
fi

log "Copying SSH key to remote Jetson (${REMOTE_USER}@${REMOTE_HOST})..."
log "You will be prompted for the remote password (one time only):"
ssh-copy-id -i "$HOME/.ssh/id_ed25519.pub" "${REMOTE_USER}@${REMOTE_HOST}"
ok "Passwordless SSH configured"

# ── Verify SSHFS connection ───────────────────────────────────────────────────
log "Testing SSHFS mount..."
MOUNT_POINT="$HOME/jetson_mount"
mkdir -p "$MOUNT_POINT"

sshfs "${REMOTE_USER}@${REMOTE_HOST}:${DROP_DIR}" "$MOUNT_POINT" \
  -o cache=yes,kernel_cache,compression=no,Ciphers=chacha20-poly1305@openssh.com

if mountpoint -q "$MOUNT_POINT"; then
  ok "SSHFS mount works! Unmounting test mount..."
  fusermount -u "$MOUNT_POINT"
fi

# ── Make scripts executable ───────────────────────────────────────────────────
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
chmod +x "${SCRIPT_DIR}/jetson_send.sh" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/jetson_receive.sh" 2>/dev/null || true
ok "Scripts are executable"

# ── Add /etc/fuse.conf setting for non-root SSHFS ────────────────────────────
if ! grep -q "^user_allow_other" /etc/fuse.conf 2>/dev/null; then
  log "Enabling user_allow_other in /etc/fuse.conf..."
  echo "user_allow_other" | sudo tee -a /etc/fuse.conf > /dev/null
  ok "fuse.conf updated"
fi

echo -e "\n${GREEN}${BOLD}✓ Setup complete!${NC}"
echo -e "\nQuick start:"
echo -e "  ${BOLD}Sender Jetson:${NC}   ./jetson_send.sh ~/my_project_folder"
echo -e "  ${BOLD}Receiver Jetson:${NC} ./jetson_receive.sh\n"