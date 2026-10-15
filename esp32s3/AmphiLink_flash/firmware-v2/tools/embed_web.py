from pathlib import Path
import sys
web=Path(sys.argv[1]);out=Path(sys.argv[2])
parts=[]
for name,symbol in [('index.html','web_index'),('app.v3.js','web_js'),('app.v2.css','web_css'),('icon.svg','web_icon'),('robot.js','web_robot_js')]:
    raw=(web/name).read_bytes()
    parts.append('static const unsigned char '+symbol+'[] = {'+','.join(str(b) for b in raw)+'};\n')
out.write_text(''.join(parts),encoding='ascii')
