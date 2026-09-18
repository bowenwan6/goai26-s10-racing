from pathlib import Path
import json, re, subprocess, tempfile, unicodedata
import pdfplumber
from pypdf import PdfReader
from PIL import Image, ImageDraw
root=Path(__file__).resolve().parent.parent
source=root/'PROJECT_TECHNICAL_ZH.md'
pdf_path=root/'PROJECT_TECHNICAL_ZH.pdf'
reader=PdfReader(pdf_path)
with pdfplumber.open(pdf_path) as document:
 raw='\n'.join(p.extract_text(layout=True) or '' for p in document.pages)
def norm(s):
 return ''.join(c for c in unicodedata.normalize('NFKC',s) if c.isalnum())
normalized=norm(raw)+' '+norm('\n'.join(p.extract_text() or '' for p in reader.pages))
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
bounds=[]; empty=[]; pageinfo=[]; thumbs=[]
with pdfplumber.open(pdf_path) as document, tempfile.TemporaryDirectory() as td:
 subprocess.run(['pdftoppm','-png','-r','45',str(pdf_path),str(Path(td)/'page')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 renders=sorted(Path(td).glob('page-*.png'),key=lambda p:int(p.stem.split('-')[-1]))
 for i,(page,raster) in enumerate(zip(document.pages,renders)):
  page_text=page.extract_text() or ''
  if len(page_text.strip())<60: empty.append(i+1)
  for word in page.extract_words():
   if word['x0'] < -1 or word['top'] < -1 or word['x1'] > page.width+1 or word['bottom'] > page.height+1:
    bounds.append({'page':i+1,'word':word['text']})
  pageinfo.append({'page':i+1,'characters':len(page_text),'width':round(page.width,1),'height':round(page.height,1)})
  im=Image.open(raster).convert('RGB'); im.thumbnail((300,420))
  frame=Image.new('RGB',(320,450),'#e5e9ec');frame.paste(im,((320-im.width)//2,10));ImageDraw.Draw(frame).text((12,431),f'PAGE {i+1}',fill='black');thumbs.append(frame)
cols=4;sheet=Image.new('RGB',(320*cols,450*((len(thumbs)+cols-1)//cols)),'white')
for i,im in enumerate(thumbs):sheet.paste(im,((i%cols)*320,(i//cols)*450))
sheet.save(root/'academic_assets/page_overview.png')
log=(root/'academic_assets/PROJECT_TECHNICAL_ZH.log').read_text(errors='replace')
issues=re.findall(r'(?:Missing character:.*|Overfull \\[hv]box.*|! .*)',log)
def count_outline(items):
 return sum(count_outline(x) if isinstance(x,list) else 1 for x in items)
pages=len(reader.pages)
result={'pages':pages,'expected_pages':8,'page_count_valid':pages==8,'bookmarks':count_outline(reader.outline),'source_text_fragments_checked':count,'missing_text_fragments':sorted(set(missing)),'missing_glyph_markers':raw.count('\ufffd'),'out_of_page_text':bounds,'blank_pages':empty,'latex_errors_or_overflow':issues,'page_summary':pageinfo}
(root/'academic_assets/pdf_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
(root/'academic_assets/pdf_text.txt').write_text(raw)
print(json.dumps({k:v for k,v in result.items() if k!='page_summary'},ensure_ascii=False,indent=2))
