import subprocess, os, numpy as np, cv2, onnxruntime as ort
NB="/home/orangepi/yolov8-oiv7.nb"; SIZE=320
VPM="/opt/vpm_run/vpm_run"; INP="/home/orangepi/in320.dat"; SAMPLE="/home/orangepi/s8.txt"
O0="/home/orangepi/output_0.txt"; O1="/home/orangepi/output_1.txt"
img=cv2.imread("/opt/yolov5/input_data/dog.jpg"); h0,w0=img.shape[:2]
r=min(SIZE/h0,SIZE/w0); nh,nw=round(h0*r),round(w0*r)
c=np.full((SIZE,SIZE,3),114,np.uint8); t,l=(SIZE-nh)//2,(SIZE-nw)//2
c[t:t+nh,l:l+nw]=cv2.resize(img,(nw,nh))
s=ort.InferenceSession("/home/orangepi/yolov8n-oiv7.onnx",providers=['CPUExecutionProvider'])
blob=c[:,:,::-1].transpose(2,0,1)[None].astype(np.float32)/255
gold=s.run(None,{s.get_inputs()[0].name:blob})[0][0][4:].ravel()
open(SAMPLE,"w").write("[network]\n%s\n[input]\n%s\n"%(NB,INP))
for name,x in {"CHW BGR":c.transpose(2,0,1),"CHW RGB":c[:,:,::-1].transpose(2,0,1)}.items():
    np.ascontiguousarray(x).tofile(INP)
    for f in (O0,O1):
        if os.path.exists(f): os.remove(f)
    subprocess.run([VPM,"-s",SAMPLE,"-l","1","-b","0","--save_txt","1"],cwd="/home/orangepi",capture_output=True)
    a=np.array(open(O0).read().split(),np.float32); b=np.array(open(O1).read().split(),np.float32)
    cls=(b if a.size==8400 else a)
    cos=float(cls@gold/(np.linalg.norm(cls)*np.linalg.norm(gold)))
    print("%s  跟onnx相似度=%.4f  >0.25的格數=%d"%(name,cos,int((cls>0.25).sum())))
print("onnx  >0.25的格數=%d"%int((gold>0.25).sum()))
