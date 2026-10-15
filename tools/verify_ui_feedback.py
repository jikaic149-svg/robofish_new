"""Exercise recording and vision layouts without connecting cameras or a target."""
from pathlib import Path
import json
import sys
import threading
from http.server import ThreadingHTTPServer
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'esp32s3/AmphiLink_UI'))
import server
server.STATION.start()
httpd=ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
server.PORT=httpd.server_port
thread=threading.Thread(target=httpd.serve_forever,daemon=True);thread.start()
url=f'http://127.0.0.1:{server.PORT}'
out=ROOT/'build/workstation';out.mkdir(parents=True,exist_ok=True)
errors=[];actions=[];starts=[];folders=[]
vision={'running':False,'ready':False,'source':'','model':'','logs':[]}
native={'available':False,'webReady':False,'status':{'state':'none'}}
image='<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720"><rect width="1280" height="720" fill="#193b46"/><text x="640" y="360" text-anchor="middle" fill="#b6e6dc" font-size="48">Layout test image - no camera</text></svg>'
def screenshot(page,name):
    page.evaluate("window.scrollTo({top:0,behavior:'instant'})")
    page.wait_for_function('window.scrollY===0')
    page.screenshot(path=str(out/name),full_page=True)

try:
    with sync_playwright() as p:
        browser=p.chromium.launch(channel='msedge',headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1000})
        page.on('pageerror',lambda error:errors.append(str(error)))
        page.route('**/api/development/state',lambda route:route.fulfill(json=native))
        page.route('**/api/vision/state',lambda route:route.fulfill(json=vision))
        page.route('**/api/vision/stream?*',lambda route:route.fulfill(content_type='image/svg+xml',body=image))
        page.route('http://192.168.1.42:81/stream',lambda route:route.fulfill(content_type='image/svg+xml',body=image))
        def start(route):
            body=route.request.post_data_json;starts.append(body)
            vision.update(running=True,ready=True,result_ready=True,source=str(body['source']),model=body['model'],process_id=12345,frame_age_ms=5)
            route.fulfill(json=vision)
        def action(route):
            actions.append(route.request.post_data_json['event']);route.fulfill(json={'id':'fixture-action','state':'queued'})
        def folder(route):
            folders.append(route.request.post_data_json);route.fulfill(json={'opened':True,'path':str(ROOT/'build/vision')})
        page.route('**/api/vision/start',start);page.route('**/api/vision/action',action);page.route('**/api/vision/open-output',folder)
        page.goto(url,wait_until='domcontentloaded')
        page.click('[data-page="debug"]')
        for name in ['probe-stat','target-name','pc-link','stm-link']:
            assert page.locator('#'+name).is_visible(),name
            assert page.locator('#'+name).evaluate('(element)=>element.closest("details")===null'),name
        assert page.locator('#legacy-maintenance').get_attribute('open') is None
        screenshot(page,'connection-overview.png')

        page.click('[data-page="overview"]')
        page.select_option('#station-mode','simulation')
        page.wait_for_function("stationMode==='simulation'&&!modePending")
        assert page.locator('#capture-toggle').inner_text()=='记录仿真数据'
        assert '六路输出' in page.locator('#capture-overview-detail').inner_text()
        page.click('#capture-toggle');page.wait_for_function('recorded.recording&&recorded.count>0')
        page.click('#capture-toggle');page.wait_for_function('!recorded.recording')
        page.click('[data-page="data"]')
        with page.expect_download() as download:page.click('#data-export-json')
        captured=json.loads(Path(download.value.path()).read_text(encoding='utf8'))
        row=captured['samples'][0]
        assert row['source']=='simulation'
        assert {'battery_v','power_pct','tail1_deg','tail2_deg','left_fin_deg','right_fin_deg','left_esc_pct','right_esc_pct','owner','safety','reason'}.issubset(row)

        page.click('[data-page="vision"]')
        page.wait_for_function("document.getElementById('vision-model').value==='VISION/models/fish.pt'")
        assert page.locator('#vision-popout').count()==0
        page.click('#vision-config-link');assert page.locator('#vision-settings').get_attribute('open') is not None
        page.fill('#vision-board-host','192.168.1.42')
        page.click('[data-vision-layout="board"]');page.click('#vision-board-toggle')
        page.wait_for_function("document.getElementById('vision-board').naturalWidth===1280&&!document.getElementById('vision-board').hidden")
        assert not page.locator('#vision-control-card').is_visible()
        assert not starts,starts
        page.wait_for_timeout(500)
        assert page.locator('#vision-board').is_visible()

        page.click('[data-vision-layout="control"]')
        assert not page.locator('#vision-board-card').is_visible()
        page.fill('#vision-camera-index','2');page.click('#vision-start')
        page.wait_for_function('visionState.running&&visionState.ready')
        assert starts==[{'source':2,'model':'VISION/models/fish.pt'}],starts
        assert page.locator('#vision-model').is_disabled()
        page.locator('#vision-control-card [data-vision-task="MARKER_ROI"]').click()
        page.locator('#vision-control-card [data-vision-task="RECORD"]').click()
        assert [item['task'] for item in actions[-2:]]==['MARKER_ROI','RECORD']
        assert page.evaluate('recorded.recording') is False
        page.wait_for_function("document.getElementById('vision-result').naturalWidth===1280")
        page.locator('#vision-result').click()
        page.wait_for_timeout(100)
        points=[item for item in actions if item.get('kind') in ('down','up')]
        assert points and all(abs(item['x']-.5)<.02 and abs(item['y']-.5)<.02 for item in points),points
        for box in page.locator('#vision-control-card .vision-actions').first.locator('button').all():
            assert box.bounding_box()['height']>=44
        screenshot(page,'vision-control-preview.png')

        page.click('[data-vision-layout="dual"]')
        assert page.locator('#vision-board-card').is_visible() and page.locator('#vision-control-card').is_visible()
        screenshot(page,'vision-dual-preview.png')
        page.click('#vision-open-output');assert folders==[{}]
        page.click('[data-vision-layout="control"]');page.evaluate("localStorage.setItem('robofish-vision-camera','8')");page.reload(wait_until='domcontentloaded')
        page.wait_for_function("visionLayout==='control'&&visionState.running")
        assert page.locator('#vision-board-card').is_hidden()
        page.wait_for_function("document.getElementById('vision-camera-index').value==='2'")

        page.set_viewport_size({'width':390,'height':844})
        for layout in ['board','control','dual']:
            page.click('[data-vision-layout="'+layout+'"]');page.wait_for_timeout(100)
            assert not page.evaluate('document.documentElement.scrollWidth>innerWidth+2'),layout
        screenshot(page,'vision-mobile-preview.png')
        page.click('[data-page="debug"]');page.wait_for_timeout(150)
        assert page.locator('#target-name').is_visible()
        assert not page.evaluate('document.documentElement.scrollWidth>innerWidth+2'),'debug'
        assert not errors,errors
        browser.close()
finally:
    httpd.shutdown();httpd.server_close();thread.join(3)

result={'passed':True,'page_errors':errors,'checks':['always visible connection/target','recording explanation and real exported sample','board-only view without inference','control-only view','dual view','44px actions','model and calibration tasks','layout persistence','capture-folder task','390px layouts'],
        'camera_and_vision_responses':'intercepted fixtures','hardware_tested':False}
(out/'ui-feedback-validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,ensure_ascii=False))
