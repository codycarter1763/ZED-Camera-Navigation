#!/usr/bin/env bash
# =============================================================================
#  jetson_receive.sh  —  Watch the drop folder and auto-extract incoming zips
#                         Run this on the REMOTE Jetson Orin Nano
# =============================================================================
#  Usage:
#    ./jetson_receive.sh [drop_directory] [extract_directory]
#
#  Examples:
#    ./jetson_receive.sh
#    ./jetson_receive.sh /srv/jetson_drop /home/louis/received
#
#  Requirements:
#    - inotifywait (sudo apt install inotify-tools)
#    - unzip
# =============================================================================

set -euo pipefail

# ── Colours ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'
BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[$(date '+%H:%M:%S')]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date '+%H:%M:%S')] ✓${NC} $*"; }
warn() { echo -e "${YELLOW}[$(date '+%H:%M:%S')] !${NC} $*"; }

# ── Configuration ─────────────────────────────────────────────────────────────
DROP_DIR="${1:-/srv/jetson_drop}"
EXTRACT_DIR="${2:-$HOME/received}"

# ── Preflight checks ──────────────────────────────────────────────────────────
if ! command -v inotifywait &>/dev/null; then
  echo "inotify-tools not found. Installing..."
  sudo apt-get install -y inotify-tools
fi

mkdir -p "$DROP_DIR" "$EXTRACT_DIR"

echo -e "\n${BOLD}━━━ Jetson Receiver ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
log "Watching  : $DROP_DIR"
log "Extracting: $EXTRACT_DIR"
log "Press Ctrl+C to stop"
echo -e "${BOLD}─────────────────────────────────────────────────────────${NC}\n"

# ── Watch loop ────────────────────────────────────────────────────────────────
inotifywait -m -e close_write --format '%f' "$DROP_DIR" | while read -r FILENAME; do

  FILEPATH="${DROP_DIR}/${FILENAME}"

  # Only process zip files
  if [[ "$FILENAME" != *.zip ]]; then
    continue
  fi

  log "New file detected: $FILENAME"

  # Wait briefly to ensure write is fully flushed
  sleep 1

  # Validate zip
  if ! unzip -t "$FILEPATH" &>/dev/null; then
    warn "Invalid or incomplete zip: $FILENAME — skipping"
    continue
  fi

  # Extract
  log "Extracting $FILENAME -> $EXTRACT_DIR ..."
  unzip -o "$FILEPATH" -d "$EXTRACT_DIR"
  ok "Extracted: $FILENAME"

  # Optional: move processed zip to an archive subfolder
  ARCHIVE_DIR="${DROP_DIR}/.processed"
  mkdir -p "$ARCHIVE_DIR"
  mv "$FILEPATH" "${ARCHIVE_DIR}/${FILENAME}"
  log "Archived zip to $ARCHIVE_DIR"

  echo ""
done
