#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
pandoc PROJECT_TECHNICAL_ZH.md -f markdown+tex_math_single_backslash -t latex -s \
 --lua-filter=academic_assets/layout.lua --include-in-header=academic_assets/header.tex \
 -V documentclass=article -V fontsize=10pt -V papersize=a4 \
 -V 'geometry:top=17mm,bottom=17mm,left=18mm,right=18mm' \
 -V mainfont='Times New Roman' -V sansfont='Arial' -V monofont='Menlo' \
 -V CJKmainfont='Songti SC' -V linestretch=1.10 -V colorlinks=true \
 -o academic_assets/PROJECT_TECHNICAL_ZH.tex
for pass in 1 2; do
 xelatex -interaction=nonstopmode -halt-on-error -output-directory=academic_assets \
  academic_assets/PROJECT_TECHNICAL_ZH.tex > "academic_assets/compile-${pass}.log" 2>&1
done
cp academic_assets/PROJECT_TECHNICAL_ZH.pdf PROJECT_TECHNICAL_ZH.pdf
