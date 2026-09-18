# GOAI enterprise poster — Posterly build

The two A3 portrait sheets are authored as self-contained HTML/CSS documents and rendered with Posterly's print-emulated Chromium workflow.

- `poster_page1.html`: competition, course map, control flow, router, RL and verified results
- `poster_page2.html`: enterprise support, field bottlenecks and improvement path
- `build_posterly_pages.py`: shared design system and page content
- `design_tokens.json`: palette and font declarations used by Posterly's style gate
- `GATE_REPORT_page*.json`: preflight, style, measure and polish results
- `assets/`: locally embedded project figures

Build with:

```bash
bash posterly_rebuild/build_final.sh
```

Each page must pass Posterly's hard gates and final PDF verification before the two pages are merged into `GOAI_ENTERPRISE_POSTER_PRO_ZH.pdf`.
