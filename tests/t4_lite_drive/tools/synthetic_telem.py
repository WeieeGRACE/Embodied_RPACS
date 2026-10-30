"""Send synthetic TELEM frames to 127.0.0.1:5001 (logger self-test)."""
import sys
import time
sys.path.insert(0, "/home/HwHiAiUser/t4_tools")
import struct
import t4_lite_drive as T

sock = __import__("socket").socket(__import__("socket").AF_INET, __import__("socket").SOCK_DGRAM)
n = 0
t0 = time.time()
while time.time() - t0 < 4.0:
    enc_l = int((time.time() - t0) * 40)
    enc_r = enc_l
    payload = struct.pack("<iihhhhhBBBH", enc_l, enc_r, 123, 0, 0, 40, 40, 0, n & 0xFF, n & 0xFF, 5)
    frame = T.build_frame(T.T_TELEM, payload)
    sock.sendto(frame, ("127.0.0.1", 5001))
    n += 1
    time.sleep(0.02)
print("sent", n, "frames")
