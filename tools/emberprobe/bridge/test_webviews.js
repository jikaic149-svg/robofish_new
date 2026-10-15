'use strict';
// Exercise the browser transport through native-style handlers; no target is opened.
const assert=require('node:assert/strict'),fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {createWebviews}=require('./webviews');
const root=fs.mkdtempSync(path.join(os.tmpdir(),'robofish-webview-test-'));
const provider={
 _context:{extensionUri:{fsPath:root}},_webviewAssetRootUri:{fsPath:root},
 _livePanels:new Map(),
 resolveWebviewView(view){this.view=view;view.webview.html='<head></head><body>native</body>';view.webview.onDidReceiveMessage(this._workstationDispatch=async message=>view.webview.postMessage({type:'nativeReply',request:message}));},
 openLiveWatchPanel(panel){panel.webview.html='<head></head><body>chart</body>';panel.webview.onDidReceiveMessage(message=>panel.webview.postMessage(message));this._livePanels.set(panel.id,{panel,watchKey:panel.id});},
 _debugSamplingScope:key=>key,
 _displayNamesForSeries:()=>new Map([['counter','Counter']]),
 _samplingArchive:{status:()=>({variables:['counter']}),async exportCsv({outputPath}){fs.writeFileSync(outputPath,'t,counter\n1,42\n');return{seriesCount:1,rows:1};}}
};
const vscode={workspace:{workspaceFolders:[{uri:{fsPath:root}}]},Uri:{parse:value=>value}};
const web=createWebviews(provider,vscode);
(async()=>{
 try{
  await assert.rejects(web.create({origin:'http://example.com/',kind:'sidebar'}),/来源/);
  const sidebar=await web.create({origin:'http://127.0.0.1:18767/',kind:'sidebar'});
  assert.equal(sidebar.id,'sidebar');assert(sidebar.html.includes('native'));
  const future=path.join(root,'late.js'),assetUri=provider.view.webview.asWebviewUri({fsPath:future});
  fs.writeFileSync(future,'native();');assert.equal(web.asset(new URL(assetUri).pathname.split('/').at(-1)).data.toString(),'native();');
  const events=[],unsubscribe=web.events('sidebar',0,event=>events.push(event));
  await web.send('sidebar',{type:'initCheck'});
  assert.deepEqual(events.at(-1).message,{type:'nativeReply',request:{type:'initCheck'}});
  await assert.rejects(web.send('sidebar',{type:'executeCommand',cmd:'mcu-vscode.download'}),/统一烧录/);
  for(let i=0;i<405;i++)await provider.view.webview.postMessage({type:'value',value:i});
  const replay=[];web.events('sidebar',0,event=>replay.push(event))();
  assert.equal(replay[0].message.type,'workstationResync');
  const chart=await web.create({origin:'http://127.0.0.1:18767/',kind:'chart'}),chartEvents=[];
  web.events(chart.id,0,event=>chartEvents.push(event));
  await web.send(chart.id,{type:'exportCsv',source:'archive',names:['counter']});
  const download=chartEvents.find(event=>event.message.type==='workstationDownload').message;
  const file=web.download(download.id);assert.equal(fs.readFileSync(file,'utf8'),'t,counter\n1,42\n');
  assert.equal(chartEvents.at(-1).message.rowCount,1);
  web.completeDownload(download.id);assert(!fs.existsSync(file));
  await assert.rejects(web.send(chart.id,{type:'exportCsv',source:'retained',historyRevision:99}),/历史已变化/);
  assert.equal(chartEvents.at(-1).message.ok,false);
  assert.throws(()=>web.validate('missing'),/不存在/);unsubscribe();
  console.log('PASS: native message roundtrip, event recovery, protected download, CSV failure and cleanup.');
 }finally{
  web.dispose();
  assert(path.resolve(root).startsWith(path.resolve(os.tmpdir())+path.sep));
  fs.rmSync(root,{recursive:true,force:true});
 }
})().catch(error=>{console.error(error);process.exitCode=1;});
