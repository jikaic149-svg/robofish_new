'use strict';
window.nativeDevelopment={available:false};
let nativeChipInfo=null;
const dev={busy:false,polling:false,view:'source',diagnostic:'rtos',firmware:'stm32',variablePanel:'values',assets:null,
 source:null,frames:[],frame:null,threads:[],breakpoints:[],charts:[],sidebarLoaded:false,context:'',sourceGeneration:0,sourceRequest:0,settings:null};
const DEV_LABELS={initialize:'启动调试引擎',connect:'连接调试',stop:'断开调试',pause:'暂停',continue:'继续','step-over':'源码单步','step-in':'进入函数','step-out':'跳出函数',restart:'重启',flash:'烧录',verify:'校验','launch-debug':'下载后复位调试',fault:'分析故障',chip:'读取芯片身份'};
function devText(id,value){const element=$(id);if(element)element.textContent=value;}
function devFeedback(message,error=false){devText('dev-task-state',message);document.querySelector('.developer-feedback').dataset.state=error?'error':'normal';}
function devElement(tag,text,className){const element=document.createElement(tag);if(text!==undefined)element.textContent=String(text??'—');if(className)element.className=className;return element;}
async function devCall(task,params={}){const reply=await service('development/task',{task,params,hardware:true});return reply.result??reply;}
function renderNativeValue(element,value){
 element.replaceChildren();const list=devElement('dl',undefined,'plan-grid');
 function add(prefix,item,depth=0){
  if(item&&typeof item==='object'&&depth<5){for(const [key,child] of Object.entries(item).slice(0,60))add(prefix?prefix+'.'+key:key,child,depth+1);}
  else{list.append(devElement('dt',prefix),devElement('dd',item&&typeof item==='object'?JSON.stringify(item):item));}
 }
 add('',value);element.append(list);
}
async function devConfirm(title,description,plan){
 devText('dev-confirm-title',title);devText('dev-confirm-description',description);renderNativeValue($('dev-confirm-plan'),plan);
 const dialog=$('dev-confirm-dialog');dialog.showModal();
 return new Promise(resolve=>{
  let accepted=false;
  $('dev-confirm-allow').onclick=()=>{accepted=true;dialog.close();};$('dev-confirm-cancel').onclick=()=>dialog.close();
  dialog.addEventListener('close',()=>resolve(accepted),{once:true});
 });
}
async function initializeDevelopment(){
 devFeedback('正在启动EmberProbe后台；不会连接或暂停目标。');
 const value=await devCall('initialize');window.nativeDevelopment=value;await pollDevelopment();return value;
}
async function runDevelopment(task,params={}){
 if(dev.busy)return;dev.busy=true;devFeedback((DEV_LABELS[task]||task)+'…');
 try{
  if(task==='initialize'){await initializeDevelopment();devFeedback('调试引擎已就绪，可离线查看ELF。');return;}
  if(!window.nativeDevelopment.webReady)await initializeDevelopment();
  if(['pause','continue','step-over','step-in','step-out'].includes(task)&&$('dev-thread').value)params.threadId=Number($('dev-thread').value);
  if(task==='connect')params={...params,debugElf:$('dev-debug-elf').value,transport:state.transport,host:$('host-input').value.trim(),speed:parseInt($('adapter-speed').value)};
  if(['flash','verify','launch-debug'].includes(task)){
   if(state.transport!=='usb')throw Error('当前工程的STM32镜像维护使用USB有线连接');
   params={...params,transport:'usb',elf:$('dev-flash-elf').value,debugElf:$('dev-debug-elf').value};
   if(task!=='verify'){
    const plan=await devCall('flash-plan',params);renderNativeValue($('dev-flash-results'),plan);
    if(!plan.confirmationId)throw Error('原生引擎未返回有效烧录计划');
    if(!await devConfirm(task==='launch-debug'?'下载后复位调试':'核对本次烧录','先全停机器鱼。核对文件、SHA256、目标和探针后只执行本次；原生授权仍会检查。',plan)){devFeedback('已取消，未执行烧录。');return;}
    params.confirmationId=plan.confirmationId;
   }
  }
  const result=await devCall(task,params);
  if(['flash','verify','launch-debug'].includes(task))renderNativeValue($('dev-flash-results'),result);
  if(task==='fault')renderFault(result);
  if(task==='chip'){nativeChipInfo=result.info||result;syncDeveloperStatus();}
  log('EmberProbe：'+(DEV_LABELS[task]||task)+'返回','ok');devFeedback((DEV_LABELS[task]||task)+'已返回，目标状态以引擎读回为准。');
  if(['continue','step-over','step-in','step-out','restart','stop','connect','launch-debug'].includes(task))invalidateInspection();
  await pollDevelopment();
  if(['pause','step-over','step-in','step-out','restart','connect','launch-debug'].includes(task)&&window.nativeDevelopment.status?.state==='paused')await readDebugContext();
 }catch(error){devFeedback(error.message,true);log('EmberProbe：'+error.message,'error');toast(error.message);}
 finally{dev.busy=false;await pollDevelopment();}
}
async function pollDevelopment(){
 if(dev.polling)return;dev.polling=true;
 try{
  const value=await service('development/state');window.nativeDevelopment=value;
  if(value.chipInfo)nativeChipInfo=value.chipInfo;
  const status=value.status||{},active=['paused','running'].includes(status.state),paused=status.state==='paused';
  const label=({paused:'已暂停',running:'运行中',none:'未连接',conflict:'会话冲突'})[status.state]||'未连接';
  devText('dev-engine-state',value.webReady?`EmberProbe ${value.version} · ${label}`:value.available?'需要重载调试宿主':'调试引擎未启动');
  $('dev-connect').disabled=dev.busy||active;$('dev-stop').disabled=dev.busy||!active;$('dev-initialize').hidden=!!value.webReady;
  document.querySelectorAll('[data-ember-task]').forEach(button=>{
   const task=button.dataset.emberTask;
   button.disabled=dev.busy||(['flash','verify','launch-debug'].includes(task)?active||value.sampling?.running:!active||task==='pause'&&paused||['continue','step-over','step-in','step-out'].includes(task)&&!paused);
  });
  if(active)devText('side-state','STM32 · '+label);
  const key=JSON.stringify([status.state,status.session?.id,status.epoch]);
  const changed=dev.context!==key;
  if(changed)invalidateInspection();dev.context=key;
  if(value.configuration)renderNativeValue($('dev-profile-summary'),{状态:label,ELF:value.configuration.elf,目标:value.configuration.mcu,探针:value.configuration.debugger,协议:value.configuration.transport,时钟_kHz:value.configuration.adapterSpeedKhz,采样模式:value.sampling?.mode});
  if(state.page==='debug')renderDevelopmentSurface();
  syncDeveloperStatus();
  if(changed&&paused&&!dev.busy)await readDebugContext();
 }catch(error){devText('dev-engine-state',error.message);}
 finally{dev.polling=false;}
}
function syncDeveloperStatus(){
 const value=window.nativeDevelopment||{},status=value.status||{},active=['paused','running'].includes(status.state);
 const label=status.state==='paused'?'已暂停':'运行中';
 $('target-read-info').disabled=dev.busy||active||!!value.sampling?.running||!!liveState.session_active;
 devText('target-read-note',active?'调试会话已建立。身份缓存与当前会话状态分别显示；空闲探针时可重新读取身份。':value.sampling?.running?'请先停止变量采样，再读取芯片身份。':'点击“读取芯片身份”读取实际型号、ID和Flash；不会因展开页面自动暂停目标。');
 if(active){
  devText('probe-stat',(state.transport==='usb'?'USB':'Wi-Fi')+' · 调试已连接');
  devText('probe-detail','EmberProbe会话已建立 · '+(state.transport==='usb'?'USB探针':$('host-input').value.trim()));
  devText('target-name',nativeChipInfo?'芯片身份已读取':'调试目标已连接');
  devText('target-run',label);devText('debug-state',label);devText('target-badge',nativeChipInfo?'身份缓存':'身份待读取');
  devText('target-core',value.configuration?.mcu?'工程目标：'+value.configuration.mcu:'实际芯片身份尚未读取');
  devText('target-description','当前状态由EmberProbe报告；身份字段仅显示实际读取结果。');
  devText('latency-stat','未测量');
  $('connect-main').disabled=$('connect-card').disabled=true;
  $('probe-link-toggle').disabled=true;
  devText('pc-link-state','调试通道已连接');devText('pc-link-detail','EmberProbe · '+(state.transport==='usb'?'USB':'Wi-Fi · '+$('host-input').value.trim()));
  devText('stm-link-state',label);devText('stm-link-detail','调试会话状态由EmberProbe报告');
  $('pc-link').dataset.state=$('stm-link').dataset.state='online';
  devText('connection-badge','SWD · '+label);devText('side-state','STM32 · '+label);
 }
 if(nativeChipInfo&&!liveState.connected){
  const info=nativeChipInfo;
  devText('target-name',info.model||info.targetName||info.core||'已读取芯片身份');
  devText('target-stat',info.model||info.targetName||'—');devText('target-id',info.deviceId||'—');
  devText('target-flash',info.flashSize||'—');devText('target-core',info.core||'—');
  devText('target-badge','最近读取的身份');
  devText('target-description',info.uid?'UID '+(Array.isArray(info.uid)?info.uid.join(' '):info.uid):'已缓存实际读取结果；需要更新时重新读取身份。');
 }
}
function invalidateInspection(){
 dev.sourceGeneration++;dev.frames=[];dev.frame=null;devText('dev-stack','调试上下文已变化，暂停后重新读取。');devText('dev-scopes','变量和寄存器需要重新读取。');
 document.querySelectorAll('.code-line.current').forEach(row=>row.classList.remove('current'));
}
async function loadDeveloperAssets(){
 try{dev.assets=await service('development/assets');renderSourceFiles();
  const current=$('dev-flash-elf').value;$('dev-flash-elf').replaceChildren();
  const debugCurrent=$('dev-debug-elf').value;$('dev-debug-elf').replaceChildren();
  for(const file of dev.assets.elfs){for(const id of ['dev-flash-elf','dev-debug-elf']){const option=devElement('option',file.path);option.value=file.path;$(id).append(option);}}
  $('dev-flash-elf').value=current||dev.assets.factory_elf;
  $('dev-debug-elf').value=debugCurrent||dev.assets.default_elf;
 }catch(error){devFeedback(error.message,true);}
}
function renderSourceFiles(){
 if(!dev.assets)return;const select=$('dev-source-file'),current=select.value,query=$('dev-source-search').value.toLowerCase();select.replaceChildren(devElement('option','选择文件'));
 select.firstChild.value='';for(const file of dev.assets.sources.filter(path=>path.toLowerCase().includes(query))){const option=devElement('option',file);option.value=file;select.append(option);}select.value=current;
}
async function openSource(path,line){
 if(!path)return;
 const request=++dev.sourceRequest;
 try{const source=await service('development/source?path='+encodeURIComponent(path));if(request!==dev.sourceRequest)return;dev.source=source;devText('dev-source-name',source.path);$('dev-source-file').value=source.path;renderCode(line);}
 catch(error){devFeedback(error.message,true);}
}
function renderCode(currentLine){
 if(!dev.source)return;const code=$('dev-code');code.replaceChildren();
 const current=Number(currentLine||dev.frame?.line),sameFrame=dev.frame?.source?.path&&normalizeSourcePath(dev.frame.source.path)===normalizeSourcePath(dev.source.absolute_path);
 for(const [index,text] of dev.source.text.split('\n').entries()){
  const line=index+1,row=devElement('div',undefined,'code-line');row.dataset.line=line;
  const breakpoint=dev.breakpoints.find(item=>item.type==='source'&&normalizeSourcePath(item.file)===normalizeSourcePath(dev.source.absolute_path)&&item.line===line);
  if(breakpoint)row.classList.add(breakpoint.enabled?'breakpoint':'disabled-breakpoint');
  if(line===current&&(currentLine||sameFrame))row.classList.add('current');
  const number=devElement('button',line,'line-number');number.title=breakpoint?'点击删除；右键修改条件':'点击添加；右键设置条件';number.onclick=()=>toggleBreakpoint(line,breakpoint);number.oncontextmenu=event=>{event.preventDefault();editBreakpoint(line,breakpoint);};
  row.append(number,devElement('code',text.replace(/\r$/,'')));code.append(row);
 }
 if(currentLine)requestAnimationFrame(()=>code.querySelector(`[data-line="${Number(currentLine)}"]`)?.scrollIntoView({block:'center'}));
 devText('dev-source-position',currentLine?'当前行 '+currentLine:'点击行号设置断点 · 右键设置条件');
}
function normalizeSourcePath(path){return String(path||'').replaceAll('\\','/').toLowerCase();}
async function refreshBreakpoints(){
 try{const result=await devCall('breakpoints');dev.breakpoints=result.breakpoints||result||[];if(!Array.isArray(dev.breakpoints))dev.breakpoints=[];renderCode();renderBreakpointList();}
 catch(error){devFeedback(error.message,true);}
}
async function toggleBreakpoint(line,breakpoint){
 try{if(!window.nativeDevelopment.webReady)await initializeDevelopment();await devCall('breakpoint',{action:breakpoint?'remove':'add',type:'source',file:dev.source.path,line});await refreshBreakpoints();}
 catch(error){devFeedback(error.message,true);}
}
function renderBreakpointList(){
 const box=$('dev-breakpoints');box.replaceChildren();
 if(!dev.breakpoints.length){box.textContent='没有断点。点击源码行号添加。';return;}
 for(const item of dev.breakpoints){const row=devElement('div',undefined,'breakpoint-row'),enabled=devElement('input');enabled.type='checkbox';enabled.checked=item.enabled;enabled.setAttribute('aria-label','启用断点');
  const locator=item.type==='source'?{type:'source',file:item.file,line:item.line,column:item.column}:{type:'function',function:item.function};
  enabled.onchange=async()=>{try{await devCall('breakpoint',{...locator,action:enabled.checked?'enable':'disable'});await refreshBreakpoints();}catch(error){devFeedback(error.message,true);}};
  const label=devElement('span',item.type==='source'?item.file.split(/[\\/]/).at(-1)+':'+item.line:item.function);label.title=[item.condition,item.hitCondition,item.logMessage].filter(Boolean).join('\n');
  const remove=devElement('button','删除','text-button');remove.onclick=async()=>{try{await devCall('breakpoint',{...locator,action:'remove'});await refreshBreakpoints();}catch(error){devFeedback(error.message,true);}};
  row.append(enabled,label,remove);box.append(row);
 }
}
function editBreakpoint(line,item){
 const dialog=$('dev-breakpoint-dialog');devText('dev-bp-location',dev.source.path+':'+line);$('dev-bp-location').value=dev.source.path+':'+line;
 $('dev-bp-condition').value=item?.condition||'';$('dev-bp-hit').value=item?.hitCondition||'';$('dev-bp-log').value=item?.logMessage||'';
 $('dev-bp-cancel').onclick=()=>dialog.close();$('dev-bp-save').onclick=async()=>{
  try{if(!window.nativeDevelopment.webReady)await initializeDevelopment();await devCall('breakpoint',{action:'add',type:'source',file:dev.source.path,line,condition:$('dev-bp-condition').value,hitCondition:$('dev-bp-hit').value,logMessage:$('dev-bp-log').value});dialog.close();await refreshBreakpoints();}
  catch(error){devFeedback(error.message,true);}
 };dialog.showModal();
}
async function readDebugContext(){
 const generation=dev.sourceGeneration;
 try{
  const result=await devCall('threads');if(generation!==dev.sourceGeneration)return;
  dev.threads=result.threads||[];const select=$('dev-thread'),previous=select.value;select.replaceChildren();
  for(const thread of dev.threads){const option=devElement('option',thread.name+' · #'+thread.id);option.value=thread.id;select.append(option);}
  if(dev.threads.some(item=>String(item.id)===previous))select.value=previous;
  await readStack();await refreshBreakpoints();
 }catch(error){devFeedback(error.message,true);}
}
async function readStack(){
 const threadId=Number($('dev-thread').value);if(!threadId)return;const generation=dev.sourceGeneration;
 try{const result=await devCall('stack',{threadId,count:50});if(generation!==dev.sourceGeneration||threadId!==Number($('dev-thread').value))return;dev.frames=result.stackFrames||[];const box=$('dev-stack');box.replaceChildren();
  for(const frame of dev.frames){const button=devElement('button',frame.name+' · '+(frame.source?.name||'汇编')+':'+frame.line);button.onclick=()=>selectStackFrame(frame);box.append(button);}
  if(dev.frames.length)await selectStackFrame(dev.frames[0]);else box.textContent='此线程没有可用栈帧。';
 }catch(error){devFeedback(error.message,true);}
}
async function selectStackFrame(frame){
 dev.frame=frame;const generation=++dev.sourceGeneration;
 [...$('dev-stack').children].forEach((button,index)=>button.classList.toggle('selected',dev.frames[index]===frame));
 if(frame.source?.path)await openSource(frame.source.path,frame.line);
 try{const result=await devCall('scopes',{frame:frame.frame});if(generation!==dev.sourceGeneration)return;const box=$('dev-scopes');box.replaceChildren();box.classList.add('scope-tree');
  for(const scope of result.scopes||[]){const details=devElement('details'),summary=devElement('summary',scope.name),body=devElement('div');details.append(summary,body);details.ontoggle=()=>{if(details.open&&!body.dataset.loaded){body.dataset.loaded='yes';readScope(scope.reference,body,0,generation);}};box.append(details);if(!scope.expensive)details.open=true;}
 }catch(error){devFeedback(error.message,true);}
}
async function readScope(reference,box,start,generation){
 if(!reference)return;
 try{const result=await devCall('locals',{reference,start,count:50});if(generation!==dev.sourceGeneration){box.textContent='调试上下文已变化，请重新读取。';return;}
  for(const variable of result.variables||[]){const row=devElement('div',undefined,'scope-row');row.append(devElement('span',variable.name),devElement('code',variable.value),devElement('small',variable.type));
   if(variable.reference){const details=devElement('details'),summary=devElement('summary'),body=devElement('div');summary.append(row);details.append(summary,body);details.ontoggle=()=>{if(details.open&&!body.dataset.loaded){body.dataset.loaded='yes';readScope(variable.reference,body,0,generation);}};box.append(details);}else box.append(row);
  }
  if((result.variables||[]).length===50){const more=devElement('button','继续展开下一页','text-button');more.onclick=()=>{more.remove();readScope(reference,box,start+50,generation);};box.append(more);}
 }catch(error){box.textContent=error.message;}
}
function switchDevelopmentView(view){dev.view=view;document.querySelectorAll('[data-dev-view]').forEach(button=>button.classList.toggle('selected',button.dataset.devView===view));for(const name of ['source','variables','peripherals','diagnostics','firmware'])$('dev-'+name+'-pane').hidden=name!==view;renderDevelopmentSurface();}
function frameTheme(frame){frame?.contentWindow?.postMessage({type:'rf-ember-theme',theme:document.body.dataset.theme||'light'},location.origin);}
function renderDevelopmentSurface(){
 const section=dev.view==='variables'?'variables':dev.view==='peripherals'?'peripherals':dev.view==='diagnostics'?dev.diagnostic:dev.view==='firmware'&&dev.firmware==='config'?'config':null;
 const nativeNeeded=section&&section!=='fault',ready=!!window.nativeDevelopment.webReady;
 $('dev-native-empty').hidden=!nativeNeeded||ready;$('dev-native-surface').hidden=!nativeNeeded||!ready;
 if(!nativeNeeded||!ready)return;
 const main=$('dev-native-frame'),chartSelected=dev.view==='variables'&&dev.variablePanel!=='values';
 main.hidden=chartSelected;for(const chart of dev.charts)chart.frame.hidden=!(chartSelected&&chart.id===dev.variablePanel);
 if(!dev.sidebarLoaded){dev.sidebarLoaded=true;main.src='/api/development/document?kind=sidebar';}
 main.contentWindow?.postMessage({type:'rf-ember-section',section},location.origin);frameTheme(main);
}
function addChart(){
 if(!window.nativeDevelopment.webReady)return runDevelopment('initialize');
 const id='chart-'+(dev.charts.length+1),frame=devElement('iframe');frame.className='native-frame';frame.title='变量波形 '+(dev.charts.length+1);frame.src='/api/development/document?kind=chart&id='+id;frame.hidden=true;
 $('dev-charts').append(frame);dev.charts.push({id,frame});const button=devElement('button','图表 '+dev.charts.length);button.dataset.variablePanel=id;button.onclick=()=>selectVariablePanel(id);$('dev-chart-tabs').append(button);selectVariablePanel(id);
}
function selectVariablePanel(id){dev.variablePanel=id;document.querySelectorAll('[data-variable-panel]').forEach(button=>button.classList.toggle('selected',button.dataset.variablePanel===id));renderDevelopmentSurface();}
function selectDiagnostic(name){dev.diagnostic=name;document.querySelectorAll('[data-diagnostic]').forEach(button=>button.classList.toggle('selected',button.dataset.diagnostic===name));$('dev-fault-panel').hidden=name!=='fault';renderDevelopmentSurface();}
function selectFirmware(name){dev.firmware=name;document.querySelectorAll('[data-firmware]').forEach(button=>button.classList.toggle('selected',button.dataset.firmware===name));for(const [id,key] of [['dev-stm32-firmware','stm32'],['dev-esp32-firmware','esp32'],['dev-configuration','config']])$(id).hidden=name!==key;if(name==='config')loadSettings();renderDevelopmentSurface();}
function renderFault(result){
 const box=$('dev-fault-results');box.replaceChildren();const grid=devElement('div',undefined,'diagnostic-grid');
 for(const [name,value] of [['故障',result.faultDetected?'检测到异常':'未检测到故障标志'],['异常类型',typeof result.exception==='object'?JSON.stringify(result.exception):result.exception],['内核状态',result.targetState],['PC',result.pc],['LR',result.lr],['SP',result.sp]]){const card=devElement('div',undefined,'diagnostic-item');card.append(devElement('span',name),devElement('strong',value));grid.append(card);}box.append(grid);
 for(const fault of result.faults||[])box.append(devElement('div',typeof fault==='string'?fault:JSON.stringify(fault), 'fault-list'));
 const details=devElement('details'),summary=devElement('summary','寄存器与符号详细结果'),content=devElement('div');renderNativeValue(content,{registers:result.registers,PC对应函数:result.pcSymbol,LR对应函数:result.lrSymbol,xPSR:result.xpsr});details.append(summary,content);box.append(details);
}
const SETTING_LABELS={openocdPath:'OpenOCD路径',armToolchainPath:'ARM工具链目录',gdbPath:'GDB路径',adapterSpeedKhz:'调试时钟(kHz)',probeSerial:'探针序列号',transport:'SWD / JTAG',rtos:'RTOS任务感知',prettyPrintingMode:'C++变量显示',prettyPrinterFiles:'GDB打印脚本列表',sampleFrequencyHz:'波形目标频率(Hz)',chartHistoryMaxMiB:'图表历史上限(MiB)',samplingArchiveMaxMiB:'归档上限(MiB)'};
async function loadSettings(){
 if(!window.nativeDevelopment.webReady)return;
 try{dev.settings=await devCall('settings');const box=$('dev-settings');box.replaceChildren();
  for(const [name,spec] of Object.entries(dev.settings)){const label=devElement('label',SETTING_LABELS[name]||name);let input;
   if(spec.enum||spec.type==='boolean'){input=devElement('select');for(const option of spec.enum||(spec.type==='boolean'?[true,false]:[])){const item=devElement('option',option);item.value=String(option);input.append(item);}input.value=String(spec.value??spec.default);}
   else{input=devElement('input');input.type=['number','integer'].includes(spec.type)?'number':'text';input.value=typeof spec.value==='object'?JSON.stringify(spec.value):String(spec.value??spec.default??'');}
   input.dataset.setting=name;input.setAttribute('aria-label',SETTING_LABELS[name]||name);label.append(input,devElement('small',(spec.description||spec.markdownDescription||'').replace(/`/g,'')));box.append(label);
  }
 }catch(error){devFeedback(error.message,true);}
}
async function uploadDeveloperFile(kind){
 const input=$('dev-upload');input.accept='.'+kind;input.value='';input.onchange=async()=>{
  const file=input.files[0];if(!file)return;
  try{if(file.size>4*1024*1024)throw Error('文件超过4 MiB');const bytes=new Uint8Array(await file.arrayBuffer());let binary='';for(let i=0;i<bytes.length;i+=32768)binary+=String.fromCharCode(...bytes.subarray(i,i+32768));const saved=await service('development/import',{name:file.name,data:btoa(binary)});
   if(!window.nativeDevelopment.webReady)await initializeDevelopment();
   if(kind==='svd')await devCall('configure',{values:{svd:saved.absolute_path}});
   await loadDeveloperAssets();if(kind==='elf')$('dev-flash-elf').value=saved.path;devFeedback('已导入 '+file.name+'，未执行烧录。');
  }catch(error){devFeedback(error.message,true);}
 };input.click();
}
$('dev-connect').onclick=()=>runDevelopment('connect');$('dev-stop').onclick=()=>runDevelopment('stop');$('dev-initialize').onclick=()=>runDevelopment('initialize');
$('target-read-info').onclick=()=>runDevelopment('chip');
document.querySelectorAll('[data-dev-status-go]').forEach(button=>button.onclick=()=>{switchDevelopmentView(button.dataset.devStatusGo);if(button.dataset.devStatusGo==='firmware')selectFirmware('stm32');$('dev-'+button.dataset.devStatusGo+'-pane').scrollIntoView({block:'start',behavior:'smooth'});});
document.querySelectorAll('[data-ember-task]').forEach(button=>button.onclick=()=>runDevelopment(button.dataset.emberTask));
document.querySelectorAll('[data-dev-view]').forEach(button=>button.onclick=()=>switchDevelopmentView(button.dataset.devView));
document.querySelectorAll('[data-diagnostic]').forEach(button=>button.onclick=()=>selectDiagnostic(button.dataset.diagnostic));
document.querySelectorAll('[data-firmware]').forEach(button=>button.onclick=()=>selectFirmware(button.dataset.firmware));
document.querySelector('[data-variable-panel="values"]').onclick=()=>selectVariablePanel('values');$('dev-chart-add').onclick=addChart;
$('dev-source-search').oninput=renderSourceFiles;$('dev-source-file').onchange=()=>openSource($('dev-source-file').value);$('dev-source-refresh').onclick=()=>openSource(dev.source?.path||$('dev-source-file').value);$('dev-read-stack').onclick=readDebugContext;$('dev-thread').onchange=readStack;
$('dev-add-function').onclick=async()=>{const name=prompt('函数名（支持C++限定名）');if(!name)return;try{if(!window.nativeDevelopment.webReady)await initializeDevelopment();await devCall('breakpoint',{action:'add',type:'function',function:name});await refreshBreakpoints();}catch(error){devFeedback(error.message,true);}};
$('dev-fault-read').onclick=()=>runDevelopment('fault');$('dev-svd-import').onclick=()=>uploadDeveloperFile('svd');$('dev-elf-import').onclick=()=>uploadDeveloperFile('elf');
$('dev-memory-read').onclick=async()=>{try{const result=await devCall('memory-read',{address:$('dev-memory-address').value,count:Number($('dev-memory-count').value)});const bytes=result.hex.match(/../g)||[];devText('dev-memory-output',Array.from({length:Math.ceil(bytes.length/16)},(_,row)=>'0x'+(Number(result.address)+row*16).toString(16).padStart(8,'0')+'  '+bytes.slice(row*16,row*16+16).join(' ')).join('\n'));}catch(error){devFeedback(error.message,true);}};
$('dev-memory-write').onclick=async()=>{try{const plan=await devCall('memory-write-plan',{address:$('dev-memory-address').value,hex:$('dev-memory-hex').value});if(await devConfirm('核对本次RAM写入','核对地址、旧值与新值；上下文或内存变化会拒绝执行。',plan)){const result=await devCall('memory-write',{confirmationId:plan.confirmationId});devFeedback('RAM回读校验：'+(result.verified?'一致':'未确认'));$('dev-memory-read').click();}}catch(error){devFeedback(error.message,true);}};
$('dev-console-form').onsubmit=async event=>{event.preventDefault();try{const plan=await devCall('console-plan',{command:$('dev-console-command').value});if(!await devConfirm('执行本次GDB命令','GDB命令可能改变目标状态。确认当前目标与命令后执行一次。',plan))return;const result=await devCall('console-run',{confirmationId:plan.confirmationId});$('dev-console-output').textContent+='\n> '+plan.command+'\n'+String(result.result??result.value??JSON.stringify(result));await pollDevelopment();}catch(error){devFeedback(error.message,true);}};
$('dev-config-refresh').onclick=loadSettings;$('dev-tools-check').onclick=async()=>{try{const tools=await service('tooling');devFeedback(tools.available?tools.version+' · '+tools.path:'工具不可用：'+tools.path);}catch(error){devFeedback(error.message,true);}};
$('dev-settings-save').onclick=async()=>{try{if(!dev.settings)return;const values={};for(const input of document.querySelectorAll('[data-setting]')){const name=input.dataset.setting,spec=dev.settings[name];const value=spec.type==='boolean'?input.value==='true':['integer','number'].includes(spec.type)?Number(input.value):['array','object'].includes(spec.type)?JSON.parse(input.value):input.value;if(JSON.stringify(value)!==JSON.stringify(spec.value??spec.default))values[name]=value;}await devCall('settings-save',{values});await loadSettings();devFeedback('高级设置已保存。');}catch(error){devFeedback(error.message,true);}};
window.addEventListener('message',event=>{
 if(event.origin!==location.origin)return;const allowed=[$('dev-native-frame'),...dev.charts.map(item=>item.frame)];if(!allowed.some(frame=>frame.contentWindow===event.source))return;
 if(event.data?.type==='rf-ember-chart'){switchDevelopmentView('variables');addChart();}
 if(event.data?.type==='rf-ember-notice'){devFeedback(event.data.message,event.data.level==='error');log('EmberProbe：'+event.data.message,event.data.level==='error'?'error':'info');}
 if(event.data?.type==='rf-ember-ready'){renderDevelopmentSurface();for(const frame of allowed)frameTheme(frame);}
 if(event.data?.type==='rf-ember-height'){const frame=allowed.find(frame=>frame.contentWindow===event.source);if(frame)frame.style.height=Math.min(1600,Math.max(420,Number(event.data.height)||700))+'px';}
});
window.addEventListener('keydown',event=>{if(state.page!=='debug'||/INPUT|TEXTAREA|SELECT/.test(event.target.tagName))return;const task=event.key==='F5'?'continue':event.key==='F6'?'pause':event.key==='F10'?'step-over':event.key==='F11'?(event.shiftKey?'step-out':'step-in'):null;if(task){event.preventDefault();runDevelopment(task);}});
window.addEventListener('stationtheme',()=>{for(const frame of [$('dev-native-frame'),...dev.charts.map(item=>item.frame)])frameTheme(frame);});
window.addEventListener('stationpage',event=>{if(event.detail==='debug'){loadDeveloperAssets();pollDevelopment();}});
setInterval(()=>{if(state.page==='debug')pollDevelopment();},1200);
loadDeveloperAssets();pollDevelopment();
