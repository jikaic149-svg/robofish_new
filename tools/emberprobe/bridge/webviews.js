'use strict';
// Reuse EmberProbe's actual webview handlers, renderer and sampling history.
// The transport is the only replacement: private loopback HTTP instead of VS Code UI.
const fs=require('fs'),path=require('path'),crypto=require('crypto');
function createWebviews(provider,vscode){
 const views=new Map(),assets=new Map(),downloads=new Map(),generation=crypto.randomUUID();let serial=0,origin='',sidebar;
 function emit(view,message){
  const item={sequence:++view.sequence,message};
  if(message.type!=='workstationDownload'){
   item.bytes=Buffer.byteLength(JSON.stringify(message));view.events.push(item);view.eventBytes+=item.bytes;
   while(view.events.length>400||view.eventBytes>8*1024*1024){const removed=view.events.shift();view.eventBytes-=removed.bytes;}
  }
  for(const sink of view.sinks)sink(item);
  return Promise.resolve(true);
 }
 function virtual(id){
  const view={id,sequence:0,events:[],eventBytes:0,sinks:new Set(),handler:null,html:'',visible:true,active:true};
  view.webview={options:{},cspSource:origin,
   asWebviewUri(uri){
    // The native renderer registers asset URIs before it writes new chart assets.
    const file=path.resolve(uri.fsPath);
    const roots=[provider._context.extensionUri.fsPath,provider._webviewAssetRootUri.fsPath];
    if(!roots.some(root=>{const relative=path.relative(root,file);return relative!==''&&!relative.startsWith('..'+path.sep)&&relative!=='..'&&!path.isAbsolute(relative);}))throw Error('网页资源须在原生扩展资源目录内');
    if(!['.js','.css','.svg','.png','.woff','.woff2'].includes(path.extname(file).toLowerCase()))throw Error('不允许的网页资源');
    const key=crypto.createHash('sha256').update(file).digest('hex');assets.set(key,file);
    return vscode.Uri.parse(origin+'/api/development/asset/'+key);
   },
   postMessage(message){return emit(view,message);},
   onDidReceiveMessage(handler){view.handler=handler;return {dispose(){}};}
  };
  Object.defineProperty(view.webview,'html',{get:()=>view.html,set:value=>{const changed=view.html&&view.html!==value;view.html=value;if(changed)emit(view,{type:'workstationReload'});}});
  view.onDidChangeViewState=()=>({dispose(){}});
  view.onDidDispose=handler=>{view.disposeHandler=handler;return{dispose(){}};};
  view.reveal=()=>{};
  view.dispose=()=>{view.disposeHandler?.();views.delete(id);for(const sink of view.sinks)sink(null);};
  views.set(id,view);return view;
 }
 // If the original VS Code sidebar is opened too, mirror its outgoing updates.
 const originalResolve=provider.resolveWebviewView.bind(provider);
 provider.resolveWebviewView=function(view){
  if(view!==sidebar){
   const post=view.webview.postMessage.bind(view.webview);
   view.webview.postMessage=message=>{if(sidebar)emit(sidebar,message);return post(message);};
  }
  return originalResolve(view);
 };
 return {
  async create(params){
   const parsed=new URL(params.origin);
   if(parsed.protocol!=='http:'||parsed.hostname!=='127.0.0.1'||parsed.pathname!=='/')throw Error('上位机来源无效');
   if(origin&&origin!==parsed.origin)throw Error('当前宿主已绑定另一上位机地址');
   origin=parsed.origin;
   let view;
   if(params.kind==='sidebar'){
    if(!sidebar){sidebar=virtual('sidebar');provider.resolveWebviewView(sidebar);}
    view=sidebar;
   }else if(params.kind==='chart'){
    if(params.id&&views.has(params.id))view=views.get(params.id);
    else{view=virtual('chart-'+(++serial));provider.openLiveWatchPanel(view);}
   }else throw Error('未知原生网页类型');
   const deadline=Date.now()+8000;
   while(!view.html&&Date.now()<deadline)await new Promise(resolve=>setTimeout(resolve,20));
   if(!view.html)throw Error('原生网页渲染尚未完成');
   return {id:view.id,html:view.html,generation};
  },
  async send(id,message){
   const view=views.get(id);if(!view)throw Error('原生网页尚未初始化');
   if(!message||typeof message.type!=='string'||JSON.stringify(message).length>1024*1024)throw Error('无效原生操作');
   if(message.type==='executeCommand'&&['mcu-vscode.download','mcu-vscode.debug','mcu-vscode.attachDebug'].includes(message.cmd))throw Error('请使用工作站统一烧录与调试入口');
   if(message.type==='executeCommand'&&message.cmd==='mcu-vscode.openLiveWatch')return {openChart:true};
   if(message.type==='exportCsv'){
    const entry=[...provider._livePanels.values()].find(item=>item.panel===view);
    if(!entry)throw Error('图表不存在');
    const directory=path.join(vscode.workspace.workspaceFolders[0].uri.fsPath,'build/workstation/exports');
    fs.mkdirSync(directory,{recursive:true});const id=crypto.randomUUID(),output=path.join(directory,id+'.csv');
    const scope=provider._debugSamplingScope(entry.watchKey);
    const names=message.names||provider._samplingArchive.status(scope).variables;
    const displayNames=provider._displayNamesForSeries(entry.watchKey,names);
    let result;
    try{
    if(message.csv!==undefined){
     if(typeof message.csv!=='string'||message.csv.length>64*1024*1024)throw Error('CSV过大');
     fs.writeFileSync(output,message.csv,'utf8');result={seriesCount:names.length,rows:Math.max(0,message.csv.split('\n').length-2)};
    }else if(message.source&&message.source!=='archive'){
     if((message.historyRevision||0)!==(entry.historyRevision||0))throw Error('图表历史已变化，请重新导出');
     provider._flushChartHistoryAppend(entry);
     result=await provider._chartHistory.request('export',[{scope,outputPath:output,names,displayNames:[...displayNames],fromMs:message.fromMs,toMs:message.toMs,snapshotId:message.source==='snapshot'?message.snapshotId:undefined}]);
    }else{
     result=await provider._samplingArchive.exportCsv({scope,outputPath:output,names,displayNames,fromMs:message.fromMs,toMs:message.toMs});
    }
    if(!fs.existsSync(output))throw Error('原生引擎未生成CSV文件');
    downloads.set(id,{file:output,expires:Date.now()+600000});
    emit(view,{type:'workstationDownload',name:'emberprobe-history.csv',id});
    emit(view,{type:'exportCsvResult',ok:true,seriesCount:result.seriesCount,rowCount:result.rows,historyTruncated:result.limitReached===true});
    return {download:true,rowCount:result.rowCount||result.rows,historyTruncated:result.historyTruncated||result.limitReached||false};
    }catch(error){
     if(fs.existsSync(output))fs.unlinkSync(output);
     emit(view,{type:'exportCsvResult',ok:false,error:error.message});throw error;
    }
   }
   if(id==='sidebar')return provider._workstationDispatch(message);
   return view.handler(message);
  },
  events(id,after,send){
   const view=views.get(id);if(!view)throw Error('原生网页不存在');
   if(view.events.length&&after<view.events[0].sequence-1)send({sequence:view.sequence,message:{type:'workstationResync'}});
   else for(const item of view.events)if(item.sequence>after)send(item);
   view.sinks.add(send);return()=>view.sinks.delete(send);
  },
  validate(id){if(!views.has(id))throw Error('原生网页不存在');},
  asset(id){const file=assets.get(id);if(!file)throw Error('资源未登记');const stat=fs.statSync(file);if(stat.size>8*1024*1024)throw Error('资源过大');return {file,data:fs.readFileSync(file)};},
  download(id){const item=downloads.get(id);if(!item||item.expires<Date.now())throw Error('导出文件已过期，请重新导出');return item.file;},
  completeDownload(id){const item=downloads.get(id);downloads.delete(id);if(item&&fs.existsSync(item.file))fs.unlinkSync(item.file);},
  dispose(){for(const view of views.values())view.dispose();for(const id of downloads.keys())this.completeDownload(id);}
 };
}
module.exports={createWebviews};
