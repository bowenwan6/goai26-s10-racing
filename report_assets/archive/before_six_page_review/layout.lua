local tables=0
local widths={
 {.24,.43,.33}, {.22,.59,.19}, {.22,.56,.22}, {.22,.12,.66}, {.23,.77},
 {.20,.32,.20,.28}, {.17,.28,.26,.29}, {.52,.48}, {.21,.29,.21,.29}, {.29,.30,.41}, {.20,.43,.37}, {.20,.40,.40}}
local names={'系统软件分层','感知采样配置','策略网络结构','观测向量布局','最终训练入口分布','PPO 发布训练配置','训练、局部评测与部署条件','运行保护条件','连续路线实验结果','局部越障评测结果','补充材料中的多地形研究结果','关键瓶颈与所需技术支持'}
function esc(s)
 return (pandoc.write(pandoc.Pandoc({pandoc.Plain({pandoc.Str(s)})}),'latex'):gsub('\n$',''))
end
function Code(el)
 if #el.text<=32 then
  return pandoc.RawInline('latex','\\mbox{\\ttfamily\\fontsize{8.3}{10.5}\\selectfont '..esc(el.text)..'}')
 end
 return pandoc.RawInline('latex','{\\ttfamily\\fontsize{8.3}{10.5}\\selectfont\\seqsplit{'..esc(el.text)..'}}')
end
function CodeBlock(el)
 return pandoc.RawBlock('latex','\\par\\begin{minipage}{\\linewidth}\n\\begin{ReportCode}\n'..el.text..'\n\\end{ReportCode}\n\\end{minipage}\\par')
end
function Div(el)
 if el.classes:includes('fill-slot') then
  local body=pandoc.write(pandoc.Pandoc(el.content),'latex')
  return pandoc.RawBlock('latex','\\par\\noindent\\fcolorbox{ReportRule}{white}{\\begin{minipage}{\\dimexpr\\linewidth-2\\fboxsep-2\\fboxrule\\relax}\\small\n'..body..'\n\\par\\vspace{7mm}\\end{minipage}}\\par')
 end
end
function Header(el)
 if el.level==1 then
  return pandoc.RawBlock('latex','{\\sffamily\\bfseries\\color{ReportNavy}\\fontsize{19}{25}\\selectfont '..esc(pandoc.utils.stringify(el.content))..'\\par}\\vspace{3pt}')
 end
 el.level=el.level-1
 if el.level==1 and not pandoc.utils.stringify(el.content):match('^1%.') then return {pandoc.RawBlock('latex','\\clearpage'),el} end
 return el
end
function Table(el)
 tables=tables+1
 if widths[tables] then for i,w in ipairs(widths[tables]) do el.colspecs[i][2]=w end end
 return {pandoc.RawBlock('latex','{\\small\\sffamily\\bfseries\\color{ReportGray}表 '..tables..'. '..names[tables]..'\\par}'),el}
end
