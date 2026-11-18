import cv2, time
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(3, 640); cap.set(4, 480); cap.set(cv2.CAP_PROP_FPS, 30)
for _ in range(10): cap.read()
t = time.time(); n = 0
while time.time() - t < 3:
    ok, _ = cap.read(); n += ok
print("純相機: %.1f fps" % (n / 3))
