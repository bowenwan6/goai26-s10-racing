local figure_index=0
local figure_captions={'系统数据流与关节控制权仲裁。箭头表示状态、命令或控制权消息。','Gate 16 状态机与主要转移条件；交付配置的重试预算为 0。'}
local table_index=0
local table_names={
  '实现模块与职责','ROS 与执行器接口','合成 LiDAR 配置','导航速度调度与场地先验',
  '运动策略网络与部署状态','174D 观测布局','动作解码参数','训练入口分布',
  '最终 PPO 阶段超参数','部署模型 SHA-256','Gate 16 接管入口条件',
  '运行保护与退出条件','完整路线自测结果','局部策略评测结果'}
local widths={
 {.17,.43,.40},{.29,.25,.13,.33},{.28,.72},{.34,.66},
 {.20,.54,.26},{.16,.10,.74},{.19,.43,.38},{.30,.70},
 {.37,.35,.28},{.27,.73},{.56,.44},{.45,.55},{.44,.56},{.18,.25,.29,.28}}
function esc(text)
 return (pandoc.write(pandoc.Pandoc({pandoc.Plain({pandoc.Str(text)})}), 'latex'):gsub('\n$',''))
end
function Code(el)
 return pandoc.RawInline('latex','{\\ttfamily\\small\\seqsplit{'..esc(el.text)..'}}')
end
function CodeBlock(el)
 if el.classes:includes('mermaid') then
  figure_index=figure_index+1
  if figure_index==2 then
   return pandoc.RawBlock('latex','\\begin{center}\n\\includegraphics[width=\\linewidth,height=0.52\\textheight,keepaspectratio]{report_assets/figure-2.pdf}\n\\captionof{figure}{'..figure_captions[2]..'}\n\\end{center}')
  end
  return pandoc.RawBlock('latex','\\begin{figure}[tbp]\n\\centering\n\\includegraphics[width=\\linewidth,height=0.68\\textheight,keepaspectratio]{report_assets/figure-'..figure_index..'.pdf}\n\\caption{'..figure_captions[figure_index]..'}\n\\end{figure}')
 end
 return pandoc.RawBlock('latex','\\par\n\\begin{minipage}{\\linewidth}\n\\begin{ReportCode}\n'..el.text..'\n\\end{ReportCode}\n\\end{minipage}\n\\par')
end
function Table(el)
 table_index=table_index+1
 if widths[table_index] then
  for i,w in ipairs(widths[table_index]) do el.colspecs[i][2]=w end
 end
 if table_index==9 then el.colspecs[2][1]=pandoc.AlignLeft end
 local space=table_index==11 and '185pt' or '6\\baselineskip'
 local cap=pandoc.RawBlock('latex','\\Needspace{'..space..'}\n{\\small\\sffamily\\bfseries\\color{ReportGray}表 '..table_index..'. '..table_names[table_index]..'\\par}\n\\vspace{-4pt}')
 return {cap,el}
end
function Header(el)
 if el.level==1 then return pandoc.RawBlock('latex','\\reporttitle') end
 el.level=el.level-1
 if pandoc.utils.stringify(el.content):match('^1%. ') then
  return {pandoc.RawBlock('latex','\\clearpage\n{\\fontsize{9}{12}\\selectfont\n\\begin{multicols}{2}\n\\tableofcontents\n\\end{multicols}}\n\\clearpage'),el}
 end
 if el.level==1 and pandoc.utils.stringify(el.content):match('^12%. ') then
  return {pandoc.RawBlock('latex','\\clearpage'),el}
 end
 if el.level==1 and pandoc.utils.stringify(el.content):match('^7%. ') then
  return {pandoc.RawBlock('latex','\\Needspace{500pt}'),el}
 end
 if el.level==2 and pandoc.utils.stringify(el.content):match('^6%.7 ') then
  return {pandoc.RawBlock('latex','\\Needspace{165pt}'),el}
 end
 if el.level==2 and (pandoc.utils.stringify(el.content):match('^8%.2 ') or pandoc.utils.stringify(el.content):match('^9%.2 ')) then
  return {pandoc.RawBlock('latex','\\Needspace{260pt}'),el}
 end
 if el.level==1 and (pandoc.utils.stringify(el.content):match('^5%. ') or pandoc.utils.stringify(el.content):match('^10%. ')) then
  return {pandoc.RawBlock('latex','\\Needspace{240pt}'),el}
 end
 if el.level==1 then
  return {pandoc.RawBlock('latex','\\FloatBarrier'),el}
 end
 return el
end

function Para(el)
 local text=pandoc.utils.stringify(el.content)
 if text:match('^图 [12]%.') or text:match('^GOAI LAI／狗來') then return {} end
end
