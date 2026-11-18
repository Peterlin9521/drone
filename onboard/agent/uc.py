import os, sys, time, select, termios, subprocess
dev, baud = "/dev/ttyS2", termios.B57600
sh = lambda a: subprocess.run(["sudo", "systemctl", a, "mavlink-router"])
sh("stop")
try:
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    t = termios.tcgetattr(fd)
    t[0] = t[1] = t[3] = 0
    t[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    t[4] = t[5] = baud
    termios.tcsetattr(fd, termios.TCSANOW, t)
    if "loop" in sys.argv:
        os.write(fd, b"PING123")
    buf, end = b"", time.time() + 15
    while time.time() < end:
        if select.select([fd], [], [], .2)[0]:
            buf += os.read(fd, 4096)
    print("bytes:", len(buf))
    if "loop" in sys.argv:
        print("loopback OK" if b"PING123" in buf else "loopback FAIL")
    else:
        hb = b"\xfd\x09\x00\x00" in buf or b"\xfe\x09" in buf
        print("HEARTBEAT seen: GOOD, link works" if hb else
              "no heartbeat: 0 bytes = no wiring/power; few bytes = noise "
              "(FC TX3 not reaching pin 13); many bytes = wrong baud")
finally:
    sh("start")
