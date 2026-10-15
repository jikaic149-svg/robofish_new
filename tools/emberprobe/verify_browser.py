"""Exercise real native widgets offline, then intercept target-dependent tasks."""
from pathlib import Path
import json
import os
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/workstation'
URL=os.environ.get('ROBOFISH_TEST_URL','http://127.0.0.1:18767')
errors=[]
calls=[]
breakpoints=[]
native_messages=[]
native_assets=[]

with sync_playwright() as p:
    browser=p.chromium.launch(channel='msedge',headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1000})
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.on('response',lambda response:errors.append('Native asset HTTP '+str(response.status)) if '/api/development/asset/' in response.url and response.status>=400 else None)
    page.on('response',lambda response:native_assets.append({'status':response.status,'url':response.url}) if '/api/development/asset/' in response.url else None)
    page.on('request',lambda request:native_messages.append(request.post_data_json.get('message',{}).get('type')) if request.method=='POST' and request.url.endswith('/api/development/web-message') else None)
    page.goto(URL,wait_until='domcontentloaded')
    page.wait_for_function('window.nativeDevelopment?.webReady===true')
    page.click('[data-page="debug"]')
    source='stm32f405/stm32f405/Core/Src/main.cpp'
    page.select_option('#dev-source-file',source)
    page.wait_for_function("document.querySelectorAll('#dev-code .code-line').length>30")
    page.screenshot(path=str(OUT/'developer-desktop.png'),full_page=True)

    page.click('[data-dev-view="variables"]')
    native=page.frame_locator('#dev-native-frame')
    native.locator('#liveValuesSection').wait_for(state='visible')
    for _ in range(100):
        if 'initCheck' in native_messages:break
        page.wait_for_timeout(100)
    assert 'initCheck' in native_messages,{'errors':errors,'assets':native_assets,'messages':native_messages}
    native.locator('.available-row').first.wait_for(state='attached')
    page.click('#dev-chart-add')
    chart=page.frame_locator('#dev-charts iframe')
    chart.locator('canvas').first.wait_for(state='attached')
    page.wait_for_timeout(700)
    assert 'initCheck' in native_messages and 'ready' in native_messages,native_messages
    page.click('[data-dev-view="peripherals"]')
    native.locator('#peripheralSection').wait_for(state='visible')
    page.click('[data-dev-view="diagnostics"]')
    native.locator('#rtosSection').wait_for(state='visible')
    page.click('[data-diagnostic="chip"]')
    native.locator('#chipInfoSection').wait_for(state='visible')
    page.click('[data-diagnostic="memory"]')
    native.locator('#memoryAnalysisSection').wait_for(state='visible')
    page.click('[data-dev-view="firmware"]')
    page.click('[data-firmware="config"]')
    native.locator('#mcuConfigSection').wait_for(state='visible')
    page.wait_for_function("document.querySelectorAll('[data-setting]').length>5")

    # No target is connected: paused frames and writes below are contract fixtures.
    state={'available':True,'webReady':True,'version':'0.8.3','revision':3,
           'status':{'state':'paused','epoch':1,'session':{'id':'browser-contract'}},'configuration':{}}
    page.route('**/api/development/state',lambda route:route.fulfill(json=state))
    def task(route):
        body=route.request.post_data_json;calls.append(body)
        name=body['task'];params=body.get('params',{})
        result={}
        if name=='threads':result={'threads':[{'id':7,'name':'MotionControl'},{'id':8,'name':'RadioPoll'}]}
        elif name=='stack':result={'stackFrames':[{'name':'controlTick','line':20,'frame':'frame-'+str(params['threadId']),'source':{'name':'main.cpp','path':str(ROOT/source)}}]}
        elif name=='scopes':result={'scopes':[{'name':'Locals','reference':'locals-current','expensive':False}]}
        elif name=='locals':result={'variables':[{'name':'value'+str(i),'value':str(i),'type':'int'} for i in range(50)] if not params.get('start') else [{'name':'lastValue','value':'50','type':'int'}]}
        elif name=='breakpoints':result={'breakpoints':breakpoints}
        elif name=='breakpoint':
            if params['action']=='add':breakpoints.append({'type':'source','file':str(ROOT/params['file']),'line':params['line'],'enabled':True})
            else:breakpoints.clear()
        elif name=='memory-read':result={'address':'0x20000000','hex':'0033aaff'}
        elif name=='memory-write-plan':result={'confirmationId':'browser-plan','address':'0x20000000','previous':'0033','requested':'ffff'}
        route.fulfill(json={'task':name,'result':result})
    page.route('**/api/development/task',task)
    page.click('[data-dev-view="source"]')
    page.evaluate('pollDevelopment()')
    page.wait_for_function("document.getElementById('dev-scopes').textContent.includes('value49')")
    page.get_by_role('button',name='继续展开下一页').click()
    page.wait_for_function("document.getElementById('dev-scopes').textContent.includes('lastValue')")
    assert any(x['task']=='locals' and x['params'].get('start')==50 for x in calls)
    page.select_option('#dev-thread','8')
    page.wait_for_function("document.getElementById('dev-thread').value==='8'&&dev.frame?.frame==='frame-8'")
    page.locator('#dev-code [data-line="25"] .line-number').click()
    page.wait_for_function("document.querySelector('#dev-code [data-line=\"25\"]').classList.contains('breakpoint')")
    page.locator('#dev-code [data-line="25"] .line-number').click()
    page.wait_for_function("!document.querySelector('#dev-code [data-line=\"25\"]').classList.contains('breakpoint')")
    page.locator('#dev-source-name').click()
    page.keyboard.press('F10')
    page.wait_for_timeout(250)
    assert any(x['task']=='step-over' and x['params'].get('threadId')==8 for x in calls)
    page.locator('details.source-extra > summary').click()
    page.locator('#dev-memory-read').click()
    page.wait_for_function("document.getElementById('dev-memory-output').textContent.includes('00 33 aa ff')")
    page.fill('#dev-memory-hex','ffff');page.click('#dev-memory-write')
    page.locator('#dev-confirm-dialog').wait_for(state='visible');page.click('#dev-confirm-cancel')
    assert not any(x['task']=='memory-write' for x in calls)

    page.set_viewport_size({'width':390,'height':844})
    for name in ['overview','vision','data','debug']:
        page.click('[data-page="'+name+'"]');page.wait_for_timeout(120)
        assert not page.evaluate('document.documentElement.scrollWidth>innerWidth+2'),name
    for name in ['source','variables','peripherals','diagnostics','firmware']:
        page.click('[data-dev-view="'+name+'"]');page.wait_for_timeout(100)
        assert not page.evaluate('document.documentElement.scrollWidth>innerWidth+2'),name
    page.screenshot(path=str(OUT/'developer-mobile.png'),full_page=True)
    assert not errors,errors
    browser.close()

result={'passed':True,'page_errors':errors,'real_native_widgets':['variables','chart','peripherals','rtos','chip','memory-analysis','configuration'],
        'native_messages':sorted(set(native_messages)),
        'mocked_target_checks':['thread and frame selection','variable pagination','source breakpoints','thread-bound stepping','memory read','cancelled RAM write'],
        'responsive_width':390,'hardware_tested':False}
(OUT/'ember-browser-validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,ensure_ascii=False))
