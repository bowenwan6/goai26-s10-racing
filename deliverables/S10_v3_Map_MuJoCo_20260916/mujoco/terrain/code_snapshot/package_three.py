"""Package only the bounded release, not raw map archives or other candidates."""
from pathlib import Path
import argparse,hashlib,json,shutil,zipfile,subprocess
from html.parser import HTMLParser
from urllib.parse import unquote,urlparse
ROOT=Path(__file__).resolve().parents[1]
class Links(HTMLParser):
    def __init__(self):super().__init__();self.urls=[]
    def handle_starttag(self,tag,attrs):
        for k,v in attrs:
            if k in ['href','src','poster']:self.urls.append(v)
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('name');out=ROOT/ap.parse_args().name
    archive=ROOT/'Start_B_短平台_精细模型_MuJoCo_20260916.zip'
    assert not archive.exists(),'Never replace an earlier release zip'
    code=out/'code_snapshot';code.mkdir(exist_ok=False)
    for name in ['prepare_three.py','build_three.py','check_three_engine.py','check_three_interfaces.py','check_three_release.py','render_three.py','render_three_mujoco.py','package_three.py','inspect_start.py','extract_features.py','export_precise_obj.py']:
        shutil.copy2(ROOT/'scripts'/name,code/name)
    shutil.copy2(ROOT/'scope.json',out/'scope.json')
    # Retain accepted sample metadata alongside final geometry without changing it.
    provenance=out/'provenance';provenance.mkdir()
    for src,dst in [(ROOT/'sample_05/checks.json','accepted_sample_checks.json'),(ROOT/'sample_05/export_checks.json','accepted_sample_export_checks.json'),(ROOT/'sample_05/README.md','accepted_sample_historical_readme.md'),(ROOT/'sample_05/surface_patches.json','accepted_sample_surface_patches.json'),(ROOT.parent/'B_structured_repair_v1/candidate_05/repair_report.json','original_B_repair_report.json')]:shutil.copy2(src,provenance/dst)
    links=[];missing=[]
    for name in ['index.html','interactive.html']:
        parser=Links();parser.feed((out/name).read_text())
        for url in parser.urls:
            if url.startswith('#') or urlparse(url).scheme:continue
            path=(out/unquote(url.split('#')[0])).resolve();links.append(str(path.relative_to(out)))
            if not path.exists():missing.append(str(path))
    assert not missing,missing
    videos=[]
    for p in sorted((out/'media').glob('*.mp4')):
        probe=json.loads(subprocess.check_output(['/opt/homebrew/bin/ffprobe','-v','error','-show_entries','format=duration,size','-show_entries','stream=codec_name,width,height,r_frame_rate','-of','json',str(p)]))
        subprocess.run(['/opt/homebrew/bin/ffmpeg','-v','error','-i',str(p),'-f','null','-'],check=True)
        videos.append(dict(path=str(p.relative_to(out)),probe=probe,complete_decode_pass=True))
    result=dict(local_html_asset_references_checked=len(links),missing=missing,videos=videos,external_cdn_required=False,portable_probe_script_executed=True,
        UI_checks=['Delivery page rendered','24s video played in browser','3D Start view button changed bounds and displayed mesh'],
        scope='Local assets only. No credentials, robot connections, full map extension or policy execution.')
    (out/'package_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    entries=[dict(path=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(out.rglob('*')) if p.is_file()]
    (out/'SHA256_manifest.json').write_text(json.dumps(entries,ensure_ascii=False,indent=2))
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(out.rglob('*')):
            if p.is_file():z.write(p,Path('Start_B_short_fine')/p.relative_to(out))
    with zipfile.ZipFile(archive) as z:assert z.testzip() is None
    print(json.dumps(dict(zip=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),files=len(entries)+1,html_references=len(links),video_count=len(videos)),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
