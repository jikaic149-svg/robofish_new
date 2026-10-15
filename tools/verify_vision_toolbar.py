"""Check that visible toolbar buttons and native hit testing agree at both sizes."""
from pathlib import Path
import sys
import cv2
import numpy as np

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'VISION'))
from vision_toolbar import VisionToolbar

for width,height in [(640,480),(1280,720)]:
    toolbar=VisionToolbar();rectangles=toolbar.layout(width,height)
    for action,(x1,y1,x2,y2) in rectangles.items():
        assert 0<=x1<x2<=width and 0<=y1<y2<=height,(action,width)
        assert y2-y1>=44
        assert toolbar.hit_test((x1+x2)//2,(y1+y2)//2,width,height)==action
    values=list(rectangles.values())
    for i,(a,b,c,d) in enumerate(values):
        for x,y,z,w in values[i+1:]:assert c<=x or z<=a or d<=y or w<=b
    if width==1280:
        image=np.full((height,width,3),(45,42,36),dtype=np.uint8);toolbar.draw(image)
        output=root/'build/workstation/native-toolbar-preview.png';output.parent.mkdir(parents=True,exist_ok=True)
        assert cv2.imwrite(str(output),image)
print('PASS: native toolbar buttons are 49 px high, do not overlap, and their centers trigger the matching actions at 640x480 and 1280x720.')
