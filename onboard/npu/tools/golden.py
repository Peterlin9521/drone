import sys, numpy as np, cv2, onnxruntime as ort
ONNX=sys.argv[1]; SIZE=320
img=cv2.imread("/opt/yolov5/input_data/dog.jpg"); h0,w0=img.shape[:2]
r=min(SIZE/h0,SIZE/w0); nh,nw=round(h0*r),round(w0*r)
canvas=np.full((SIZE,SIZE,3),114,np.uint8)
t,l=(SIZE-nh)//2,(SIZE-nw)//2
canvas[t:t+nh,l:l+nw]=cv2.resize(img,(nw,nh))
rgb=cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB)
blob=rgb.astype(np.float32).transpose(2,0,1)[None]/255.0
s=ort.InferenceSession(ONNX,providers=['CPUExecutionProvider'])
out=s.run(None,{s.get_inputs()[0].name:blob})[0][0]   # (605,2100)
cls=out[4:]
print("onnx 全圖最高類別分數:", round(float(cls.max()),3),
      " 類別idx:", int(cls.max(1).argmax()))
