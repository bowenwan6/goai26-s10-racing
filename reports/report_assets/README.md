# 技术报告 PDF 构建与检查

正文来源：`../PROJECT_TECHNICAL_ZH.md`。

- `header.tex`：A4 字体、页眉页脚、标题及目录样式。
- `layout.lua`：表格列宽、代码换行、图表编号和分页规则。
- `figure-1.pdf`、`figure-2.pdf`：PDF 使用的矢量图；分别对应正文架构图和状态机。
- `render_diagrams.cjs`：渲染 Markdown 中的 Mermaid 图。
- `layout_architecture.py`、`print_svg.cjs`：为图 1 的同一组节点和 18 条边提供固定布局。
- `pdf_check.json`：最近一次正文、哈希、字体缺字及页面边界检查结果。

使用本机 Pandoc、XeLaTeX 及 Times New Roman、Arial、Menlo、宋体和黑体执行：

```bash
bash report_assets/build_pdf.sh
```

构建使用已保存的矢量图。修改 Mermaid 内容时，需先重新渲染并同步固定布局。检查脚本依赖 PyMuPDF 与 Pillow；本次运行环境使用 `/tmp/goai-report-pdf-tools/python` 中的 PyMuPDF。
