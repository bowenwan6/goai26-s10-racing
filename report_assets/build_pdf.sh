#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
pandoc PROJECT_TECHNICAL_ZH.md \
  -f markdown+tex_math_single_backslash -t latex -s \
  --lua-filter=report_assets/layout.lua \
  --include-in-header=report_assets/header.tex \
  -V documentclass=article -V fontsize=10pt -V papersize=a4 \
  -V 'geometry:top=20mm,bottom=19mm,left=20mm,right=20mm' \
  -V mainfont='Times New Roman' -V sansfont='Arial' -V monofont='Menlo' \
  -V CJKmainfont='Songti SC' -V linestretch=1.18 -V colorlinks=true \
  -o report_assets/PROJECT_TECHNICAL_ZH.tex
for pass in 1 2 3; do
  xelatex -interaction=nonstopmode -halt-on-error -output-directory=report_assets \
    report_assets/PROJECT_TECHNICAL_ZH.tex > "report_assets/compile-${pass}.log" 2>&1
done
cp report_assets/PROJECT_TECHNICAL_ZH.pdf PROJECT_TECHNICAL_ZH.pdf
