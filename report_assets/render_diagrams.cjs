const fs = require('fs');
const path = require('path');
const { chromium } = require('/Users/xxxwbwxxx/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async () => {
 const source=fs.readFileSync('PROJECT_TECHNICAL_ZH.md','utf8');
 const blocks=[...source.matchAll(/```mermaid\n([\s\S]*?)\n```/g)];
 const browser=await chromium.launch({executablePath:'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true});
 const page=await browser.newPage({viewport:{width:1400,height:1600}});
 await page.setContent('<html><head><style>html,body{margin:0;padding:0;background:white;}svg{display:block;}</style></head><body></body></html>');
 await page.addScriptTag({path:'/tmp/goai-report-pdf-tools/node_modules/mermaid/dist/mermaid.min.js'});
 await page.evaluate(()=>mermaid.initialize({startOnLoad:false,securityLevel:'loose',theme:'base',themeVariables:{fontFamily:'Arial',fontSize:'17px',primaryColor:'#eff5f8',primaryBorderColor:'#3d687b',primaryTextColor:'#183040',lineColor:'#6b8291',secondaryColor:'#e6f1ef',tertiaryColor:'#f7f8fa'},flowchart:{htmlLabels:true,curve:'linear',nodeSpacing:20,rankSpacing:32,padding:12},state:{padding:12,nodeSpacing:20,rankSpacing:28}}));
 const dimensions=[];
 for(let i=0;i<blocks.length;i++){
  const svg=await page.evaluate(async ({s,i})=>(await mermaid.render('diagram'+i,s)).svg,{s:blocks[i][1],i});
  fs.writeFileSync(`report_assets/figure-${i+1}.svg`,svg);
  const size=await page.evaluate(svg=>{document.body.innerHTML=svg;const el=document.querySelector('svg');const b=el.viewBox.baseVal;el.style.maxWidth='none';el.setAttribute('width',b.width);el.setAttribute('height',b.height);return {width:Math.ceil(b.width),height:Math.ceil(b.height)}},svg);
  await page.setViewportSize(size);
  await page.pdf({path:`report_assets/figure-${i+1}.pdf`,width:size.width+'px',height:size.height+'px',margin:{top:0,bottom:0,left:0,right:0},printBackground:true});
  await page.screenshot({path:`report_assets/figure-${i+1}.png`,fullPage:true});
  dimensions.push({figure:i+1,...size});
 }
 fs.writeFileSync('report_assets/diagram_dimensions.json',JSON.stringify(dimensions,null,2));
 console.log(JSON.stringify(dimensions));
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1)});
