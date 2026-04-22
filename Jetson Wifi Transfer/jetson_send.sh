#!/usr/bin/env bash
# =============================================================================
#  jetson_send.sh  —  Zip and transfer a directory to a remote Jetson Orin Nano
#                      via SSHFS mount
# =============================================================================
#  Usage:
#    ./jetson_send.sh <directory_to_send> [remote_user@host] [remote_drop_path]
#
#  Examples:
#    ./jetson_send.sh ~/my_project
#    ./jetson_send.sh ~/my_project louis@10.42.0.1 /srv/jetson_drop
#
#  Requirements:
#    - sshfs, ssh, zip installed on sender
#    - Remote directory accessible via SSH
# =============================================================================

set -euo pipefail

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()   { echo -e "${GREEN}[OK]${NC}    $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()  { echo -e "${RED}[ERR]${NC}   $*" >&2; exit 1; }

# ── Configuration (override via args or env vars) ─────────────────────────────
SOURCE_DIR="${1:-}"
REMOTE_USER_HOST="${2:-${JETSON_REMOTE:-clarq@10.42.0.1}}"
REMOTE_DROP_PATH="${3:-${JETSON_DROP_PATH:-/srv/jetson_drop}}"
MOUNT_POINT="${JETSON_MOUNT:-$HOME/jetson_mount}"

# ── Validate input ────────────────────────────────────────────────────────────
[[ -z "$SOURCE_DIR" ]] && err "Usage: $0 <directory_to_send> [user@host] [remote_path]"
[[ ! -d "$SOURCE_DIR" ]] && err "Source directory not found: $SOURCE_DIR"

SOURCE_DIR="$(realpath "$SOURCE_DIR")"
DIR_NAME="$(basename "$SOURCE_DIR")"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ZIP_NAME="${DIR_NAME}_${TIMESTAMP}.zip"
TMP_ZIP="/tmp/${ZIP_NAME}"

echo -e "\n${BOLD}━━━ Jetson Transfer ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
log "Source      : $SOURCE_DIR"
log "Remote host : $REMOTE_USER_HOST"
log "Remote path : $REMOTE_DROP_PATH"
log "Archive     : $ZIP_NAME"
echo -e "${BOLD}─────────────────────────────────────────────────────────${NC}\n"

# ── Step 1: Zip the directory ─────────────────────────────────────────────────
log "Compressing directory..."
zip -r "$TMP_ZIP" "$SOURCE_DIR" -x "*.pyc" -x "__pycache__/*" -x ".git/*" \
  && ok "Created archive: $TMP_ZIP ($(du -sh "$TMP_ZIP" | cut -f1))"

# ── Step 2: Ensure SSHFS mount is active ──────────────────────────────────────
mount_sshfs() {
  log "Mounting SSHFS at $MOUNT_POINT ..."
  mkdir -p "$MOUNT_POINT"
  sshfs "${REMOTE_USER_HOST}:${REMOTE_DROP_PATH}" "$MOUNT_POINT" \
    -o cache=yes,kernel_cache,compression=no,Ciphers=chacha20-poly1305@openssh.com \
    && ok "SSHFS mounted at $MOUNT_POINT"
}

if mountpoint -q "$MOUNT_POINT"; then
  ok "SSHFS already mounted at $MOUNT_POINT"
else
  mount_sshfs
fi

# ── Step 3: Copy zip to remote via SSHFS ──────────────────────────────────────
log "Transferring archive over WiFi..."
cp --progress "$TMP_ZIP" "$MOUNT_POINT/" 2>/dev/null \
  || rsync -ah --progress "$TMP_ZIP" "$MOUNT_POINT/"
ok "Transfer complete: $MOUNT_POINT/$ZIP_NAME"

# ── Step 4: Verify the file arrived ──────────────────────────────────────────
REMOTE_SIZE=$(stat -c%s "$MOUNT_POINT/$ZIP_NAME" 2>/dev/null || echo 0)
LOCAL_SIZE=$(stat -c%s "$TMP_ZIP")

if [[ "$REMOTE_SIZE" -eq "$LOCAL_SIZE" ]]; then
  ok "File integrity check passed (${LOCAL_SIZE} bytes)"
else
  warn "Size mismatch — local: ${LOCAL_SIZE}B, remote: ${REMOTE_SIZE}B"
fi

# ── Step 5: Optional remote extraction ───────────────────────────────────────
echo ""
read -rp "$(echo -e "${YELLOW}Auto-extract on remote Jetson? [y/N]:${NC} ")" EXTRACT
if [[ "${EXTRACT,,}" == "y" ]]; then
  log "Extracting on remote Jetson..."
  ssh "$REMOTE_USER_HOST" \
    "cd '${REMOTE_DROP_PATH}' && unzip -o '${ZIP_NAME}' && echo 'Extraction complete'" \
    && ok "Extracted on remote at ${REMOTE_DROP_PATH}/${DIR_NAME}"
fi

# ── Cleanup ───────────────────────────────────────────────────────────────────
rm -f "$TMP_ZIP"
log "Cleaned up local temp file"

echo -e "\n${GREEN}${BOLD}✓ Done!${NC} '${DIR_NAME}' is now on the remote Jetson at ${REMOTE_DROP_PATH}/${ZIP_NAME}\n"
