# test_video_streamer.py
# A simple Python script to stream a local video file over UDP using FFmpeg.
# Usage: python test_video_streamer.py input.mp4 [host] [port]

import sys
import subprocess

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python test_video_streamer.py <input_file> [host] [port]")
        sys.exit(1)

    input_file = sys.argv[1]
    host = sys.argv[2] if len(sys.argv) >= 3 else '127.0.0.1'
    port = sys.argv[3] if len(sys.argv) >= 4 else '5600'

    # FFmpeg command to stream over UDP as MPEG-TS
    cmd = [
        'ffmpeg', '-re',             # read input at native frame rate
        '-stream_loop', '-1',        # loop input indefinitely
        '-i', input_file,           # input file
        '-c:v', 'libx264',          # encode video to H.264
        '-preset', 'veryfast',      # encoding speed
        '-tune', 'zerolatency',     # reduce latency
        '-f', 'mpegts',             # format: MPEG transport stream
        f'udp://{host}:{port}?pkt_size=1316'
    ]

    print("Streaming", input_file, "to udp://%s:%s" % (host, port))
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("FFmpeg exited with error:", e)
        sys.exit(e.returncode)
