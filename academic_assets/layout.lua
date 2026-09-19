local tables=0
local widths={{.24,.76},{.18,.82},{.23,.35,.42},{.22,.78}}
local names={'比赛控制层职责','专用策略交接条件','已完成的比赛系统验证','企业支持需求'}
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
 local title=pandoc.utils.stringify(el.content)
 if el.level==1 and (title:match('^2%.') or title:match('^3%.') or title:match('^4%.') or title:match('^5%.') or title:match('^6%.') or title:match('^7%.')) then
  return {pandoc.RawBlock('latex','\\clearpage'),el}
 end
 return el
end
function Table(el)
 tables=tables+1
 if widths[tables] then for i,w in ipairs(widths[tables]) do el.colspecs[i][2]=w end end
 return {pandoc.RawBlock('latex','{\\small\\sffamily\\bfseries\\color{ReportGray}表 '..tables..'. '..names[tables]..'\\par}'),el}
end
