#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
POSTERLY_SKILL="${POSTERLY_SKILL:-/Users/xxxwbwxxx/.agents/skills/posterly}"
PYTHON="$POSTERLY_SKILL/.venv/bin/python"

python3 build_posterly_pages.py

for page in 1 2; do
  "$PYTHON" "$POSTERLY_SKILL/tools/run_gates.py" "poster_page${page}.html" \
    --tokens design_tokens.json --report "GATE_REPORT_page${page}.json"
  "$PYTHON" "$POSTERLY_SKILL/tools/render_preview.py" "poster_page${page}.html"
  "$PYTHON" "$POSTERLY_SKILL/tools/poster_check.py" verify-final \
    "poster_page${page}_preview.pdf" --from-html "poster_page${page}.html"
done

pdfunite poster_page1_preview.pdf poster_page2_preview.pdf ../GOAI_ENTERPRISE_POSTER_PRO_ZH.pdf
cp ../GOAI_ENTERPRISE_POSTER_PRO_ZH.pdf ../GOAI_ENTERPRISE_POSTER_ZH.pdf

