#!/bin/bash
# Backup ATLAS NEXUS: DBs, config, snapshots recientes

BACKUP_DIR="${BACKUP_DIR:-/backups/atlas-nexus}"
DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_PATH="$BACKUP_DIR/backup_$DATE"
ROOT="${1:-.}"

mkdir -p "$BACKUP_PATH"

echo "Starting backup to $BACKUP_PATH (root=$ROOT)..."

# DBs en logs
if [ -d "$ROOT/logs" ]; then
  find "$ROOT/logs" -maxdepth 1 -name "*.sqlite" -exec cp {} "$BACKUP_PATH/" \; 2>/dev/null || true
fi

# Config
if [ -d "$ROOT/config" ]; then
  cp -r "$ROOT/config" "$BACKUP_PATH/" 2>/dev/null || true
fi

# Snapshots (copia y luego borra archivos >7 días en la copia)
if [ -d "$ROOT/snapshots" ]; then
  cp -r "$ROOT/snapshots" "$BACKUP_PATH/" 2>/dev/null || true
  find "$BACKUP_PATH/snapshots" -type f ! -mtime -7 -delete 2>/dev/null || true
fi

# Comprimir
tar -czf "$BACKUP_PATH.tar.gz" -C "$BACKUP_DIR" "backup_$DATE"
rm -rf "$BACKUP_PATH"

# Mantener solo últimos 30 backups
ls -t "$BACKUP_DIR"/*.tar.gz 2>/dev/null | tail -n +31 | xargs -r rm -f

echo "Backup complete: $BACKUP_PATH.tar.gz"
