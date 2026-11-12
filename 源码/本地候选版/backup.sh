#!/usr/bin/env bash
# Snapshot the planner tree before editing anything.
#
#   ./backup.sh                 -> ../backups/s_bend_<timestamp>/
#   ./backup.sh --list          -> show existing snapshots
#   ./backup.sh --restore NAME  -> copy NAME back over this tree
#
# Rule learned the hard way: an edit without a snapshot once truncated
# pairing.py and the only intact copy was on the car. Always snapshot first.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(dirname "$here")"
store="$root/backups"

case "${1:-}" in
  --list)
    ls -1 "$store" 2>/dev/null || echo "no snapshots yet"
    exit 0
    ;;
  --restore)
    name="${2:-}"
    [ -d "$store/$name" ] || { echo "no such snapshot: $name"; exit 1; }
    echo "restoring $name over $here"
    cp -a "$store/$name/." "$here/"
    echo "done - recompile and run --observe before driving"
    exit 0
    ;;
esac

stamp="$(date +%Y%m%d_%H%M)"
target="$store/s_bend_$stamp"
mkdir -p "$target"
( cd "$here" && tar --exclude=__pycache__ -cf - . ) | ( cd "$target" && tar -xf - )
echo "snapshot: $target"
echo "restore with: ./backup.sh --restore s_bend_$stamp"
