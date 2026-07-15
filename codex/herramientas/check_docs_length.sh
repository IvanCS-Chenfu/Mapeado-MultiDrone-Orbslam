#!/usr/bin/env bash
# Simple checker: warns if public MDs exceed recommended limits
set -euo pipefail
ROOT="${PWD}"
MAX_LINES=250
echo "Checking Markdown files under codex/contexto and codex/pipeline..."
find codex/contexto codex/pipeline -name '*.md' -print0 | while IFS= read -r -d '' f; do
  # skip template and short summaries
  base=$(basename "$f")
  if [[ "$base" == "SUMMARY.md" || "$base" == "README_SHORT.md" || "$base" == "05_MAPA_PAQUETES_SHORT.md" ]]; then
    continue
  fi
  lines=$(wc -l < "$f" || echo 0)
  if (( lines > MAX_LINES )); then
    echo "WARN: $f has $lines lines (>$MAX_LINES). Consider adding a short summary file or splitting."
  fi
done
echo "Done."
