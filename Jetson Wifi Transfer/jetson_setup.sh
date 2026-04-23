#!/usr/bin/env bash
# =============================================================================
#  jetson_setup.sh  —  One-time NFS setup for both Jetson Orin Nanos
# =============================================================================
#  Run on the RECEIVER Jetson first, then the SENDER Jetson.
#
#  Usage:
#    On receiver:  ./jetson_setup.sh receiver
#    On sender:    ./jetson_setup.sh sender [remote_host]
#
#  Examples:
#    clarq Jetson:  ./jetson_setup.sh receiver
#    louis Jetson:  ./jetson_setup.sh sender 10.42.0.1
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[setup]${NC} $*"; }
ok()   { echo -e "${GREEN}[done]${NC}  $*"; }
warn() { echo -e "${YELLOW}[warn]${NC}  $*"; }
err()  { echo -e "${RED}[err]${NC}   $*" >&2; exit 1; }

ROLE="${1:-}"
REMOTE_HOST="${2:-10.42.0.1}"
REMOTE_USER="clarq"
DROP_DIR="/srv/jetson_drop"
MOUNT_POINT="$HOME/jetson_mount"

[[ -z "$ROLE" ]] && err "Usage: $0 <receiver|sender> [remote_host]"
[[ "$ROLE" != "receiver" && "$ROLE" != "sender" ]] && err "Role must be 'receiver' or 'sender'"

echo -e "\n${BOLD}━━━ Jetson NFS Setup — ${ROLE^^} ━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"

# =============================================================================
if [[ "$ROLE" == "receiver" ]]; then
# =============================================================================

  log "Installing NFS server + tools..."
  sudo apt-get update -qq
  sudo apt-get install -y nfs-kernel-server inotify-tools zip unzip openssh-server
  ok "Packages installed"

  # Create drop directory
  log "Creating drop directory: $DROP_DIR"
  sudo mkdir -p "$DROP_DIR"
  sudo chown "$USER:$USER" "$DROP_DIR"
  sudo chmod 755 "$DROP_DIR"
  ok "Drop directory ready"

  # Configure NFS export — allow any host on 10.42.0.0/24
  EXPORT_LINE="${DROP_DIR} 10.42.0.0/24(rw,sync,no_subtree_check,no_root_squash)"
  if grep -qF "$DROP_DIR" /etc/exports; then
    warn "/etc/exports already has an entry for $DROP_DIR — skipping"
  else
    log "Adding NFS export to /etc/exports..."
    echo "$EXPORT_LINE" | sudo tee -a /etc/exports > /dev/null
    ok "Export added"
  fi

  # Apply exports and start NFS
  log "Applying NFS exports..."
  sudo exportfs -ra
  sudo systemctl enable --now nfs-kernel-server
  ok "NFS server running"

  # Show active exports
  log "Active NFS exports:"
  sudo exportfs -v

  echo -e "\n${GREEN}${BOLD}✓ Receiver setup complete!${NC}"
  echo -e "Now run ${BOLD}./jetson_setup.sh sender${NC} on the other Jetson.\n"

# =============================================================================
elif [[ "$ROLE" == "sender" ]]; then
# =============================================================================

  log "Installing NFS client + tools..."
  sudo apt-get update -qq
  sudo apt-get install -y nfs-common zip unzip rsync openssh-client
  ok "Packages installed"

  # SSH key for optional remote extraction
  if [[ ! -f "$HOME/.ssh/id_ed25519" ]]; then
    log "Generating SSH key pair..."
    ssh-keygen -t ed25519 -C "jetson-transfer" -f "$HOME/.ssh/id_ed25519" -N ""
    ok "SSH key generated"
  else
    ok "SSH key already exists"
  fi

  log "Copying SSH key to receiver (${REMOTE_USER}@${REMOTE_HOST})..."
  log "Enter the receiver password when prompted (once only):"
  ssh-copy-id -i "$HOME/.ssh/id_ed25519.pub" "${REMOTE_USER}@${REMOTE_HOST}"
  ok "Passwordless SSH configured"

  # Test NFS mount
  log "Testing NFS mount..."
  mkdir -p "$MOUNT_POINT"
  sudo mount -t nfs \
    -o rw,async,noatime,rsize=131072,wsize=131072,hard,intr,nfsvers=4 \
    "${REMOTE_HOST}:${DROP_DIR}" "$MOUNT_POINT"

  if mountpoint -q "$MOUNT_POINT"; then
    ok "NFS mount works! Unmounting test..."
    sudo umount "$MOUNT_POINT"
  fi

  # Make scripts executable
  SCRIPT_DIR="$(dirname "$(realpath "$0")")"
  chmod +x "${SCRIPT_DIR}/jetson_send.sh" 2>/dev/null || true
  chmod +x "${SCRIPT_DIR}/jetson_receive.sh" 2>/dev/null || true
  ok "Scripts are executable"

  echo -e "\n${GREEN}${BOLD}✓ Sender setup complete!${NC}"
  echo -e "\nQuick start:"
  echo -e "  ${BOLD}Send a folder:${NC}      ./jetson_send.sh ~/my_project_folder"
  echo -e "  ${BOLD}Watch for files:${NC}    ./jetson_receive.sh  (run on receiver)\n"

fi