

# Streaming from CAMERA
ffmpeg -f rtp rtp://192.168.3.113:9000 -framerate 30 -video_size 640x480 -i /dev/video0

ffmpeg -i /dev/video0 \
-vcodec libx264 -preset ultrafast -tune zerolatency -r 10 -async 1 -an -ar 22050 -bsf:v h264_mp4toannexb \
-maxrate 750k -bufsize 3000k -f mpegts udp://192.168.3.113:9000

# Receiving CAMERA stream
ffmpeg -i udp://0.0.0.0:9000 -f v4l2 /dev/video4

