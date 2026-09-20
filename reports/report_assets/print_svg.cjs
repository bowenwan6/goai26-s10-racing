const fs=require('fs');
const {chromium}=require('/Users/xxxwbwxxx/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{
 const browser=await chromium.launch({executablePath:'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',headless:true});
 const page=await browser.newPage({viewport:{width:770,height:765}});
 await page.setContent('<html><head><style>html,body{margin:0;padding:0}svg{display:block}</style></head><body>'+fs.readFileSync('report_assets/figure-1.svg','utf8')+'</body></html>');
 await page.pdf({path:'report_assets/figure-1.pdf',width:'770px',height:'765px',printBackground:true,margin:{top:0,bottom:0,left:0,right:0}});
 await page.screenshot({path:'report_assets/figure-1.png'});
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1)});
