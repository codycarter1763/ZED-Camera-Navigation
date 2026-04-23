#!/usr/bin/env bash
# =============================================================================
#  jetson_send.sh  —  Zip and transfer a directory to a remote Jetson Orin Nano
#                      via NFS mount
# =============================================================================
#  Usage:
#    ./jetson_send.sh <directory_to_send> [remote_host] [remote_drop_path]
#
#  Examples:
#    ./jetson_send.sh ~/my_project
#    ./jetson_send.sh ~/my_project 10.42.0.1 /srv/jetson_drop
#
#  Requirements:
#    - nfs-common, zip, rsync installed on sender
#    - NFS server running on receiver (use jetson_setup.sh)
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
REMOTE_HOST="${2:-${JETSON_REMOTE_HOST:-10.42.0.1}}"
REMOTE_DROP_PATH="${3:-${JETSON_DROP_PATH:-/srv/jetson_drop}}"
REMOTE_USER="${JETSON_REMOTE_USER:-clarq}"
MOUNT_POINT="${JETSON_MOUNT:-$HOME/jetson_mount}"

# ── Validate input ────────────────────────────────────────────────────────────
[[ -z "$SOURCE_DIR" ]] && err "Usage: $0 <directory_to_send> [remote_host] [remote_path]"
[[ ! -d "$SOURCE_DIR" ]] && err "Source directory not found: $SOURCE_DIR"

SOURCE_DIR="$(realpath "$SOURCE_DIR")"
DIR_NAME="$(basename "$SOURCE_DIR")"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ZIP_NAME="${DIR_NAME}_${TIMESTAMP}.zip"
TMP_ZIP="/tmp/${ZIP_NAME}"

echo -e "\n${BOLD}━━━ Jetson NFS Transfer ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
log "Source      : $SOURCE_DIR"
log "Remote host : $REMOTE_HOST"
log "Remote path : $REMOTE_DROP_PATH"
log "Archive     : $ZIP_NAME"
echo -e "${BOLD}─────────────────────────────────────────────────────────${NC}\n"

# ── Step 1: Zip the directory ─────────────────────────────────────────────────
log "Compressing directory..."
zip -r "$TMP_ZIP" "$SOURCE_DIR" -x "*.pyc" -x "__pycache__/*" -x ".git/*" \
  && ok "Created archive: $TMP_ZIP ($(du -sh "$TMP_ZIP" | cut -f1))"

# ── Step 2: Ensure NFS mount is active ───────────────────────────────────────
mount_nfs() {
  log "Mounting NFS share at $MOUNT_POINT ..."
  mkdir -p "$MOUNT_POINT"
  sudo mount -t nfs \
    -o rw,async,noatime,rsize=131072,wsize=131072,hard,intr,nfsvers=4 \
    "${REMOTE_HOST}:${REMOTE_DROP_PATH}" "$MOUNT_POINT" \
    && ok "NFS mounted at $MOUNT_POINT"
}

if mountpoint -q "$MOUNT_POINT"; then
  ok "NFS already mounted at $MOUNT_POINT"
else
  mount_nfs
fi

# ── Step 3: Copy zip to remote via NFS ───────────────────────────────────────
log "Transferring archive over WiFi (NFS)..."
rsync -ah --progress "$TMP_ZIP" "$MOUNT_POINT/"
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
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    "cd '${REMOTE_DROP_PATH}' && unzip -o '${ZIP_NAME}' && echo 'Extraction complete'" \
    && ok "Extracted on remote at ${REMOTE_DROP_PATH}/${DIR_NAME}"
fi

# ── Cleanup ───────────────────────────────────────────────────────────────────
rm -f "$TMP_ZIP"
log "Cleaned up local temp file"

echo -e "\n${GREEN}${BOLD}✓ Done!${NC} '${DIR_NAME}' is now on the remote Jetson at ${REMOTE_DROP_PATH}/${ZIP_NAME}\n"