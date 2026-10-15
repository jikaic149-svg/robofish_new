'use strict';
const vscode=require('vscode'),http=require('http'),fs=require('fs'),path=require('path'),crypto=require('crypto');
let server,descriptorFile,publicPointer,busy=false,lastTask=null,provider,webviews;
const memoryPlans=new Map(),consolePlans=new Map();
function nativeDescriptor(root){const pointer=JSON.parse(fs.readFileSync(path.join(root,'.agents/skills/_emberprobe/agent-bridge.json'),'utf8'));const value=pointer.descriptorPath?JSON.parse(fs.readFileSync(pointer.descriptorPath,'utf8')):pointer;if(value.host!=='127.0.0.1'||value.workspace!==root)throw Error('EmberProbe工作区不匹配');return value;}
function nativeCall(root,method,params={}){
 const descriptor=nativeDescriptor(root),body=Buffer.from(JSON.stringify({method,params}));
 return new Promise((resolve,reject)=>{const request=http.request({host:'127.0.0.1',port:descriptor.port,path:'/v1/call',method:'POST',headers:{Authorization:'Bearer '+descriptor.token,'Content-Type':'application/json','Content-Length':body.length},timeout:['debug.status','config.get'].includes(method)?2500:150000},response=>{let buffer='';response.on('data',chunk=>{buffer+=chunk;if(buffer.length>16*1024*1024)request.destroy(Error('响应过大'));});response.on('end',()=>{try{const value=JSON.parse(buffer);value.ok?resolve(value.result):reject(Object.assign(Error(value.error?.message||'EmberProbe请求失败'),value.error));}catch(error){reject(error);}});});request.on('error',reject);request.on('timeout',()=>request.destroy(Error('EmberProbe请求超时，结果未知，请读取状态核对')));request.end(body);});
}
const tasks={
 'connect':async(root,params)=>{
   const status=await nativeCall(root,'debug.status');
   if(['paused','running'].includes(status.state))throw Error('已有调试会话，请先断开');
   const config=await nativeCall(root,'config.get');
   syncScripts(root,config.openocdPath||vscode.workspace.getConfiguration('emberprobe').get('openocdPath'));
   const elf=params.debugElf?path.resolve(root,params.debugElf):config.elf||path.join(root,'stm32f405/build/debug/RoboFishF405.elf');
   const relative=path.relative(root,elf);
   if(params.debugElf&&(relative.startsWith('..')||path.isAbsolute(relative)))throw Error('调试ELF须在当前工程内');
   const link=params.transport==='wifi'?'wifi':'usb';
   const probe=link==='wifi'?'robofish/robofish-dap-wifi.cfg':
    config.debugger&&!config.debugger.endsWith('robofish-dap-wifi.cfg')?config.debugger:'robofish/robofish-dap-usb.cfg';
   if(!fs.existsSync(elf))throw Error('缺少Debug ELF');
   await nativeCall(root,'config.set',{values:{elf,debugger:probe,mcu:!config.mcu||config.mcu==='stm32f4x.cfg'?'robofish/robofish-f405.cfg':config.mcu,transport:config.transport==='auto'?'swd':config.transport||'swd',rtos:config.rtos||'FreeRTOS',adapterSpeedKhz:Number(params.speed)||100}});
   const metadata=path.join(root,'tools/emberprobe/symbols.json');
   const buildRoot=fs.existsSync(metadata)?JSON.parse(fs.readFileSync(metadata,'utf8')).buildWorkspace:root;
   const sourceFileMap={[String(buildRoot).replace(/\\/g,'/')]:root};
   const started=await vscode.debug.startDebugging(vscode.workspace.workspaceFolders[0],{name:'机器鱼附加调试',type:'emberprobe',request:'attach',executable:elf,cwd:root,rtos:'FreeRTOS',sourceFileMap});
   if(!started)throw Error('EmberProbe未建立调试会话，请查看其诊断');
   return {started,attach:true,downloaded:false};
 },
 'open-source':async()=>vscode.commands.executeCommand('workbench.view.debug'),
 'open-waveform':async()=>vscode.commands.executeCommand('mcu-vscode.openLiveWatch'),
 'open-settings':async()=>vscode.commands.executeCommand('workbench.view.extension.mcu-vscode-container'),
 'pause':(root,p={})=>nativeCall(root,'debug.control',{action:'pause',threadId:p.threadId}),
 'continue':(root,p={})=>nativeCall(root,'debug.control',{action:'continue',threadId:p.threadId}),
 'step-over':(root,p={})=>nativeCall(root,'debug.control',{action:'stepOver',threadId:p.threadId}),
 'step-in':(root,p={})=>nativeCall(root,'debug.control',{action:'stepIn',threadId:p.threadId}),
 'step-out':(root,p={})=>nativeCall(root,'debug.control',{action:'stepOut',threadId:p.threadId}),
 'restart':(root)=>nativeCall(root,'debug.control',{action:'restart'}),
 'stop':(root)=>nativeCall(root,'debug.control',{action:'stop'}),
 'threads':(root)=>nativeCall(root,'debug.inspect',{action:'threads'}),
 'stack':(root,p)=>nativeCall(root,'debug.inspect',{action:'stack',threadId:p.threadId,start:p.start,count:p.count}),
 'scopes':(root,p)=>nativeCall(root,'debug.inspect',{action:'scopes',frame:p.frame}),
 'locals':(root,p)=>nativeCall(root,'debug.inspect',{action:'variables',reference:p.reference,start:p.start,count:p.count,filter:p.filter}),
 'list-variables':async root=>{
   const config=await nativeCall(root,'config.get');
   const helper=require(path.join(vscode.extensions.getExtension('BakeSheep.emberprobe').extensionPath,'skills/mcu-variables/scripts/read.js'));
   const snapshot=helper.readStableElf(config.elf||path.join(root,'stm32f405/build/debug/RoboFishF405.elf'));
   return {elf:config.elf,sha256:snapshot.sha256,symbols:helper.parseSymbolsBuffer(snapshot.buffer)};
 },
 'read-variables':(root,p)=>nativeCall(root,'variables.read',{variables:String(p.variables||'').split(',').map(name=>({name:name.trim()})).filter(item=>item.name)}),
 'peripherals':root=>nativeCall(root,'peripherals.list',{}),
 'peripheral-read':(root,p)=>nativeCall(root,'peripherals.read',p),
 'rtos':root=>nativeCall(root,'rtos.snapshot',{includeStackUsage:true}),
 'memory':async root=>{await selectApplication(root);return nativeCall(root,'elf.analyze',{});},
 'breakpoint':(root,p)=>nativeCall(root,'debug.breakpoints.update',p),
 'breakpoints':root=>nativeCall(root,'debug.breakpoints.list'),
 'config':root=>nativeCall(root,'config.get'),
 'configure':(root,p)=>nativeCall(root,'config.set',{values:p.values}),
 'probes':root=>nativeCall(root,'probe.list'),
 'capabilities':root=>nativeCall(root,'capabilities'),
 'settings':async()=>{
   const extension=vscode.extensions.getExtension('BakeSheep.emberprobe'),blocks=extension.packageJSON.contributes.configuration;
   const config=vscode.workspace.getConfiguration('emberprobe'),fields={};
   for(const block of Array.isArray(blocks)?blocks:[blocks])for(const [key,schema] of Object.entries(block.properties||{})){
    if(key.startsWith('emberprobe.experimental.')||key==='emberprobe.prettyPrinterPath')continue;
    const name=key.slice('emberprobe.'.length);fields[name]={...schema,value:config.get(name)};
   }
   return fields;
 },
 'settings-save':async(root,p)=>{
   const status=await nativeCall(root,'debug.status');
   if(['paused','running'].includes(status.state)||engine()._samplingStatus().running)throw Error('请先断开并停止采样，再修改高级设置');
   const schema=await tasks.settings(),config=vscode.workspace.getConfiguration('emberprobe');
   for(const [key,value] of Object.entries(p.values||{})){
    const spec=schema[key];if(!spec)throw Error('不允许的设置：'+key);
    const type=Array.isArray(value)?'array':typeof value;
    if((spec.type==='integer'?!Number.isInteger(value):type!==spec.type)||spec.enum&&!spec.enum.includes(value)||spec.minimum!==undefined&&value<spec.minimum||spec.maximum!==undefined&&value>spec.maximum)throw Error('设置格式不匹配：'+key);
   }
   for(const [key,value] of Object.entries(p.values||{}))await config.update(key,value,vscode.ConfigurationTarget.Workspace);
   return tasks.settings();
 },
 'chip':root=>nativeCall(root,'chip.read',{sections:['identity','debug','runtime']}),
 'fault':root=>nativeCall(root,'fault.read'),
 'variable-write':(root,p)=>nativeCall(root,'variables.write',p),
 'write-permission-reset':root=>nativeCall(root,'variables.write.permission',{action:'reset'}),
 'peripheral-write':(root,p)=>nativeCall(root,'peripherals.write',p),
 'sampling-status':root=>nativeCall(root,'sampling.status'),
 'sampling-start':(root,p)=>nativeCall(root,'sampling.start',p),
 'sampling-stop':root=>nativeCall(root,'sampling.stop'),
 'watch-add':(root,p)=>nativeCall(root,'watch.add',p),
 'samples':(root,p)=>nativeCall(root,'variables.sample',p),
 'session-select':(root,p)=>nativeCall(root,'debug.select',p),
 'verify':(root,p)=>{if(p.transport!=='usb')throw Error('工厂镜像校验使用USB有线维护');return nativeCall(root,'flash.verify',flashPlan(root,p));},
 'launch-debug':async(root,p)=>{
   await tasks.flash(root,p);
   await tasks.connect(root,p);
   return nativeCall(root,'debug.control',{action:'restart'});
 },
 'memory-read':async(root,p)=>{
   const address=memoryAddress(p.address),count=Number(p.count||128);
   if(!Number.isInteger(count)||count<1||count>4096||!readable(address,count))throw Error('读取范围须在F405 Flash/RAM内，最多4096字节');
   const bytes=await engine()._debugBridge.readPausedMemory(address,count);
   return {address:'0x'+address.toString(16),count:bytes.length,hex:Buffer.from(bytes).toString('hex'),status:engine()._debugBridge.agentStatus()};
 },
 'memory-write-plan':async(root,p)=>{
   const address=memoryAddress(p.address),text=String(p.hex||'').replace(/\s/g,'');
   if(!/^(?:[\da-f]{2}){1,256}$/i.test(text))throw Error('写入须为1～256字节的十六进制数据');
   const bytes=Buffer.from(text,'hex');
   if(!ram(address,bytes.length))throw Error('原始写入仅允许F405 RAM，Flash/外设请使用对应入口');
   const before=await engine()._debugBridge.readPausedMemory(address,bytes.length);
   const status=engine()._debugBridge.agentStatus(),id=crypto.randomUUID();
   prunePlans(memoryPlans);memoryPlans.set(id,{address,bytes,before:Buffer.from(before),context:contextKey(status),expires:Date.now()+60000});
   return {confirmationId:id,address:'0x'+address.toString(16),previous:Buffer.from(before).toString('hex'),requested:text,bytes:bytes.length,sessionId:status.session?.id};
 },
 'memory-write':async(root,p)=>{
   const plan=memoryPlans.get(p.confirmationId);memoryPlans.delete(p.confirmationId);
   if(!plan||plan.expires<Date.now()||plan.context!==contextKey(engine()._debugBridge.agentStatus()))throw Error('写入计划已过期或调试上下文变化');
   const before=await engine()._debugBridge.readPausedMemory(plan.address,plan.bytes.length);
   if(!Buffer.from(before).equals(plan.before))throw Error('RAM在确认期间发生变化，未写入，请重新读取');
   await engine()._debugBridge.writePausedMemory(plan.address,plan.bytes);
   const after=Buffer.from(await engine()._debugBridge.readPausedMemory(plan.address,plan.bytes.length));
   if(!after.equals(plan.bytes))throw Error('RAM回读不匹配，写入结果未确认，不自动重试');
   return {verified:true,address:'0x'+plan.address.toString(16),previous:plan.before.toString('hex'),written:plan.bytes.toString('hex'),readBack:after.toString('hex')};
 },
 'console-plan':async(root,p)=>{
   const command=String(p.command||'').trim();
   if(!command||command.length>1024||/[\r\n\0]/.test(command))throw Error('请输入一条GDB命令或表达式');
   engine()._debugBridge.assertPausedAccess();
   const status=engine()._debugBridge.agentStatus(),id=crypto.randomUUID();
   prunePlans(consolePlans);consolePlans.set(id,{command,context:contextKey(status),expires:Date.now()+60000});
   return {confirmationId:id,command,sessionId:status.session?.id,elf:engine()._debugBridge.activeSession?.configuration?.executable};
 },
 'console-run':async(root,p)=>{
   const plan=consolePlans.get(p.confirmationId);consolePlans.delete(p.confirmationId);
   if(!plan||plan.expires<Date.now()||plan.context!==contextKey(engine()._debugBridge.agentStatus()))throw Error('控制台计划已失效');
   engine()._debugBridge.assertPausedAccess();
   const session=engine()._debugBridge.assertUniqueSession();
   return session.customRequest('evaluate',{expression:plan.command,context:'repl'});
 },
 'flash-plan':async(root,params)=>{
   if(params.transport!=='usb')throw Error('含Bootloader的工厂镜像仅允许USB有线维护');
   const factory=path.join(root,'stm32f405/build/debug/RoboFishFactory.elf');
   if(!fs.existsSync(factory))throw Error('工厂镜像不存在，请先构建STM32 Debug');
   return nativeCall(root,'flash.authorize',flashPlan(root,params));
 },
 'flash':async(root,params)=>{
   if(params.transport!=='usb'||typeof params.confirmationId!=='string')throw Error('请先核对工厂镜像烧录计划');
   const result=await nativeCall(root,'flash.execute',{...flashPlan(root,params),confirmationId:params.confirmationId});
   if(result.code!==0)throw Error((result.lines||[]).slice(-8).join('\n')||'EmberProbe烧录失败');
   return result;
 }
};
function flashPlan(root,params={}){
 const elf=params.elf?path.resolve(root,params.elf):path.join(root,'stm32f405/build/debug/RoboFishFactory.elf');
 const relative=path.relative(root,elf);
 if(relative.startsWith('..')||path.isAbsolute(relative)||path.extname(elf)!=='.elf'||!fs.existsSync(elf))throw Error('烧录ELF须在当前工程内');
 return {elf,elfSha256:crypto.createHash('sha256').update(fs.readFileSync(elf)).digest('hex'),
  probe:'robofish/robofish-dap-usb.cfg',target:'robofish/robofish-f405-flash.cfg',transport:'swd',adapterSpeedKhz:100,
  openocd:vscode.workspace.getConfiguration('emberprobe').get('openocdPath')};
}
function syncScripts(root,executable){
 const binary=path.resolve(executable||''),home=path.dirname(path.dirname(binary));
 const scripts=[path.join(home,'share/openocd/scripts'),path.join(home,'scripts')].find(folder=>fs.existsSync(path.join(folder,'target')));
 if(!scripts)throw Error('找不到OpenOCD脚本目录，请先安装工程调试配置');
 for(const [folder,names] of [['interface',['robofish-dap-usb.cfg','robofish-dap-wifi.cfg']],['target',['robofish-f405.cfg','robofish-f405-flash.cfg']]]){
  const destination=path.join(scripts,folder,'robofish');fs.mkdirSync(destination,{recursive:true});
  for(const name of names){const source=path.join(root,'tools/debug',name);fs.copyFileSync(source,path.join(destination,name));}
 }
}
function engine(){if(!provider)throw Error('完整EmberProbe适配尚未就绪，请重载调试宿主');return provider;}
function memoryAddress(value){const text=String(value);if(!/^(?:0x[\da-f]+|\d+)$/i.test(text))throw Error('地址无效');const address=Number(text);if(!Number.isSafeInteger(address)||address<0||address>0xffffffff)throw Error('地址超出范围');return address;}
function ram(address,count){return [[0x20000000,0x20020000],[0x10000000,0x10010000]].some(([a,b])=>address>=a&&address+count<=b);}
function readable(address,count){return ram(address,count)||(address>=0x08000000&&address+count<=0x08100000);}
function contextKey(status){return JSON.stringify([status.state,status.session?.id,status.epoch,provider?._debugBridge.inspectionEpoch]);}
function prunePlans(plans){for(const [id,plan] of plans)if(plan.expires<Date.now())plans.delete(id);if(plans.size>=16)throw Error('待确认计划过多，请先完成或等待过期');}
async function selectApplication(root){
 const elf=path.join(root,'stm32f405/build/debug/RoboFishF405.elf'),config=await nativeCall(root,'config.get');
 if(!config.elf)await nativeCall(root,'config.set',{values:{elf,mcu:path.join(root,'tools/debug/robofish-f405.cfg')}});
}
async function activate(context){
 const root=vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
 if(!root||!fs.existsSync(path.join(root,'common/stm32_boot.h')))return;
 const native=vscode.extensions.getExtension('BakeSheep.emberprobe');await native?.activate();
 provider=native?.exports?.workstationProvider;
 if(provider)webviews=require('./webviews').createWebviews(provider,vscode);
 const token=crypto.randomBytes(24).toString('hex');
 const reply=(response,status,body)=>{const bytes=Buffer.from(JSON.stringify(body));response.writeHead(status,{'Content-Type':'application/json','Content-Length':bytes.length,'Cache-Control':'no-store'});response.end(bytes);};
 server=http.createServer(async(request,response)=>{
  if(request.headers.authorization!=='Bearer '+token||request.headers.host!==`127.0.0.1:${server.address().port}`)return reply(response,403,{error:'仅允许工作站调用'});
  try{
   if(request.method==='GET'&&request.url==='/state'){
    let status=null,configuration=null,error='';
    try{status=await nativeCall(root,'debug.status');configuration=await nativeCall(root,'config.get');}catch(problem){error=problem.message;}
    return reply(response,200,{engine:'EmberProbe',version:vscode.extensions.getExtension('BakeSheep.emberprobe')?.packageJSON.version,available:!!configuration,webReady:!!webviews,revision:3,busy,lastTask,status,chipInfo:provider?._chipInfoService?.info||null,sampling:provider?._samplingStatus(),configuration,error});
   }
   if(request.method==='POST'&&request.url==='/web/create'){
    if(!vscode.workspace.isTrusted)throw Error('请先信任当前VS Code工程');engine();
    let raw='';for await(const chunk of request){raw+=chunk;if(raw.length>8192)throw Error('请求过大');}
    return reply(response,200,await webviews.create(JSON.parse(raw)));
   }
   if(request.method==='POST'&&request.url==='/web/message'){
    if(!vscode.workspace.isTrusted)throw Error('请先信任当前VS Code工程');engine();
    let raw='';for await(const chunk of request){raw+=chunk;if(raw.length>1024*1024)throw Error('请求过大');}
    const body=JSON.parse(raw);await webviews.send(body.id,body.message);return reply(response,200,{accepted:true});
   }
   if(request.method==='GET'&&request.url.startsWith('/web/events?')){
    const query=new URL(request.url,'http://127.0.0.1').searchParams;engine();
    webviews.validate(query.get('id'));
    response.writeHead(200,{'Content-Type':'text/event-stream','Cache-Control':'no-store','Connection':'keep-alive'});
    response.flushHeaders();
    const unsubscribe=webviews.events(query.get('id'),Number(query.get('after')||0),item=>{if(item)response.write('id: '+item.sequence+'\ndata: '+JSON.stringify(item)+'\n\n');else response.end();});
    const timer=setInterval(()=>response.write(': keepalive\n\n'),15000);timer.unref();
    response.on('close',()=>{clearInterval(timer);unsubscribe();});return;
   }
   if(request.method==='GET'&&request.url.startsWith('/web/asset?')){
    engine();const asset=webviews.asset(new URL(request.url,'http://127.0.0.1').searchParams.get('id'));
    response.writeHead(200,{'Content-Type':path.extname(asset.file)==='.css'?'text/css':path.extname(asset.file)==='.svg'?'image/svg+xml':'application/javascript','Content-Length':asset.data.length});response.end(asset.data);return;
   }
   if(request.method==='GET'&&request.url.startsWith('/web/download?')){
    engine();const id=new URL(request.url,'http://127.0.0.1').searchParams.get('id'),file=webviews.download(id);
    response.writeHead(200,{'Content-Type':'text/csv; charset=utf-8','Content-Disposition':'attachment; filename="emberprobe-history.csv"','Content-Length':fs.statSync(file).size});
    const stream=fs.createReadStream(file);response.on('close',()=>stream.destroy());stream.on('close',()=>webviews.completeDownload(id));stream.on('error',()=>response.destroy());stream.pipe(response);return;
   }
   if(request.method!=='POST'||request.url!=='/task')return reply(response,404,{error:'not found'});
   if(busy)return reply(response,409,{error:'调试任务进行中，请等待当前任务结果'});
   let body='';for await(const chunk of request){body+=chunk;if(body.length>65536)throw Error('任务请求过大');}
   const input=JSON.parse(body),handler=tasks[input.task];if(!handler)throw Error('未映射的调试任务');
   if(!vscode.workspace.isTrusted)throw Error('需要在VS Code中信任当前工作区');
   busy=true;lastTask={task:input.task,state:'running',startedAt:Date.now()};
   try{const result=await handler(root,input.params||{});lastTask={...lastTask,state:'returned',finishedAt:Date.now()};reply(response,200,{task:input.task,result:result??null,submitted:true});}
   catch(error){lastTask={...lastTask,state:'error',message:error.message,finishedAt:Date.now()};reply(response,400,{error:error.message,code:error.code||'EMBER_TASK_FAILED'});}
   finally{busy=false;}
  }catch(error){if(response.headersSent)response.destroy();else reply(response,400,{error:error.message});}
 });
 await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 fs.mkdirSync(context.globalStorageUri.fsPath,{recursive:true});
 descriptorFile=path.join(context.globalStorageUri.fsPath,'host-'+crypto.createHash('sha256').update(root).digest('hex').slice(0,16)+'-'+process.pid+'.json');
 fs.writeFileSync(descriptorFile,JSON.stringify({host:'127.0.0.1',port:server.address().port,token,workspace:root}),{mode:0o600});
 publicPointer=path.join(root,'build/workstation/ember-host.json');fs.mkdirSync(path.dirname(publicPointer),{recursive:true});fs.writeFileSync(publicPointer,JSON.stringify({descriptorPath:descriptorFile}));
 context.subscriptions.push({dispose:()=>{webviews?.dispose();server?.close();try{fs.unlinkSync(descriptorFile);}catch(_){} }});
}
function deactivate(){server?.close();}
module.exports={activate,deactivate,tasks};
