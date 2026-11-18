import subprocess, os, numpy as np, cv2
NB="/home/orangepi/yolov8-oiv7.nb"; SIZE=320
VPM="/opt/vpm_run/vpm_run"; INP="/home/orangepi/in320.dat"; SAMPLE="/home/orangepi/s8.txt"
O0="/home/orangepi/output_0.txt"; O1="/home/orangepi/output_1.txt"
img=cv2.imread("/opt/yolov5/input_data/dog.jpg"); h0,w0=img.shape[:2]
r=min(SIZE/h0,SIZE/w0); nh,nw=round(h0*r),round(w0*r)
c=np.full((SIZE,SIZE,3),114,np.uint8); t,l=(SIZE-nh)//2,(SIZE-nw)//2
c[t:t+nh,l:l+nw]=cv2.resize(img,(nw,nh))
open(SAMPLE,"w").write("[network]\n%s\n[input]\n%s\n"%(NB,INP))
variants={
 "HWC BGR": c,
 "HWC RGB": c[:,:,::-1],
 "CHW BGR": c.transpose(2,0,1),
 "CHW RGB": c[:,:,::-1].transpose(2,0,1),
}
for name,x in variants.items():
    np.ascontiguousarray(x).tofile(INP)
    for f in (O0,O1):
        if os.path.exists(f): os.remove(f)
    subprocess.run([VPM,"-s",SAMPLE,"-l","1","-b","0","--save_txt","1"],cwd="/home/orangepi",capture_output=True)
    a=np.array(open(O0).read().split(),np.float32); b=np.array(open(O1).read().split(),np.float32)
    cls=(b if a.size==8400 else a).reshape(601,2100)
    m=cls.max(0); k=int(m.argmax())
    print("%-8s 最高分=%.3f  類別=%d" % (name, m.max(), int(cls[:,k].argmax())))
