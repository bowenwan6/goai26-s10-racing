from pathlib import Path
import json, re, subprocess, unicodedata
import pymupdf as fitz
from PIL import Image, ImageDraw
root=Path(__file__).resolve().parent.parent
source=root/'PROJECT_TECHNICAL_ZH.md'
pdf=fitz.open(root/'PROJECT_TECHNICAL_ZH.pdf')
raw='\n'.join(p.get_text(sort=True) for p in pdf)
def norm(s):
 return ''.join(c for c in unicodedata.normalize('NFKC',s) if c.isalnum())
normalized=norm(raw)+' '+norm('\n'.join(p.get_text(sort=False) for p in pdf))
ast=json.loads(subprocess.check_output(['pandoc',str(source),'-f','markdown+tex_math_single_backslash','-t','json'],text=True))
missing=[]; count=0
def walk(v):
 global count
 if isinstance(v,dict):
  typ=v.get('t');val=v.get('c')
  if typ=='CodeBlock' and 'mermaid' in val[0][1]:return
  candidate=None
  if typ=='Str':candidate=val
  if typ=='Code':candidate=val[1]
  if candidate and len(norm(candidate))>=8:
   count+=1
   if norm(candidate) not in normalized:missing.append(candidate)
  for child in v.values():walk(child)
 elif isinstance(v,list):
  for child in v:walk(child)
walk(ast['blocks'])
bounds=[]; empty=[]; pageinfo=[];thumbs=[]
for i,page in enumerate(pdf):
 if len(page.get_text().strip())<60:empty.append(i+1)
 for word in page.get_text('words'):
  if word[0]<-1 or word[1]<-1 or word[2]>page.rect.width+1 or word[3]>page.rect.height+1:bounds.append({'page':i+1,'word':word[4]})
 pageinfo.append({'page':i+1,'characters':len(page.get_text()),'width':round(page.rect.width,1),'height':round(page.rect.height,1)})
 pix=page.get_pixmap(matrix=fitz.Matrix(.5,.5),alpha=False)
 im=Image.frombytes('RGB',[pix.width,pix.height],pix.samples)
 frame=Image.new('RGB',(320,450),'#e5e9ec');frame.paste(im,((320-im.width)//2,10));ImageDraw.Draw(frame).text((12,431),f'PAGE {i+1}',fill='black');thumbs.append(frame)
cols=4;sheet=Image.new('RGB',(320*cols,450*((len(thumbs)+cols-1)//cols)),'white')
for i,im in enumerate(thumbs):sheet.paste(im,((i%cols)*320,(i//cols)*450))
sheet.save(root/'academic_assets/page_overview.png')
log=(root/'academic_assets/PROJECT_TECHNICAL_ZH.log').read_text(errors='replace')
issues=re.findall(r'(?:Missing character:.*|Overfull \\[hv]box.*|! .*)',log)
result={'pages':len(pdf),'expected_pages':8,'page_count_valid':len(pdf)==8,'bookmarks':len(pdf.get_toc()),'source_text_fragments_checked':count,'missing_text_fragments':sorted(set(missing)),'missing_glyph_markers':raw.count('\ufffd'),'out_of_page_text':bounds,'blank_pages':empty,'latex_errors_or_overflow':issues,'page_summary':pageinfo}
(root/'academic_assets/pdf_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
(root/'academic_assets/pdf_text.txt').write_text(raw)
print(json.dumps({k:v for k,v in result.items() if k!='page_summary'},ensure_ascii=False,indent=2))
