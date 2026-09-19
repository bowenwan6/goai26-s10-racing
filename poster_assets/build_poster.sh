#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
for pass in 1 2; do
  xelatex -interaction=nonstopmode -halt-on-error GOAI_ENTERPRISE_POSTER_ZH.tex > "poster-compile-${pass}.log" 2>&1
done
cp GOAI_ENTERPRISE_POSTER_ZH.pdf ../GOAI_ENTERPRISE_POSTER_ZH.pdf
