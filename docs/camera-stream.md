# Drone camera to WebRTC

The dashboard already receives WHEP/WebRTC through the backend and MediaMTX.
The new ROS relay completes the source path:
`/camera/image_raw → FFmpeg H.264 → SRT → MediaMTX → WHEP/WebRTC → dashboard`.

Encoding runs on a separate thread and retains only the newest RGB/BGR frame.
The relay validates buffer dimensions and row padding, rejects replayed acquisition
stamps and stops publishing after one second without a fresh image. It never
publishes a substitute camera image and sends no flight commands.

## Run

Start root compose services `mediamtx`, `backend`, `frontend`. Rebuild the sim
image once with `make -C sim build` to install FFmpeg. Run the camera driver, or
the school simulator and `make -C sim school-bridge`, then:

```sh
make -C sim camera-stream MEDIA_HOST=127.0.0.1 DRONE_ID=drone-001
```

For Docker Desktop bridge networking use `MEDIA_HOST=host.docker.internal`.
For a physical drone use the MediaMTX server's reachable IP/DNS. UDP 8890 must
be reachable. In the dashboard enter the same drone ID and press `영상 연결`.

Other camera topics/rates can use the node directly inside the sourced ROS environment:

```sh
python3 /ros2_ws/src/nv_telemetry/nv_telemetry/camera_stream.py --ros-args \
  -p drone_id:=drone-001 -p srt_host:=host.docker.internal -p fps:=15.0 \
  -r camera/image_raw:=/my_camera/image_raw
```

The default is 5 fps for the school camera. Only rgb8/bgr8 are accepted; depth and
compressed topics need conversion upstream. Frames must have advancing acquisition
timestamps; restart the relay after resetting simulator time. Resolution changes
restart the encoder. Source dropout closes the publisher; fresh images can reopen
it. FFmpeg/network failure terminates the node so its supervisor can restart it.

WebRTC uses TCP 8889 signalling and UDP 8189 media. Configure the actual reachable
server IP/DNS in `MEDIA_WEBRTC_ADDITIONAL_HOSTS` for remote viewers, HTTPS outside
localhost, and your TURN server for restrictive NAT. TURN must be configured on
both MediaMTX and the browser. The default anonymous gateway is local development
configuration; set authentication before exposing it to the Internet.

Upstream contracts: [FFmpeg publishing](https://mediamtx.org/docs/publish/ffmpeg),
[WebRTC reading](https://mediamtx.org/docs/read/webrtc).

## Observed validation, 2026-10-25

Six ROS camera tests passed for padded BGR rows, invalid layouts/encodings and
replayed/reordered frames. A synthetic 320x240 changing-colour ROS source was
relayed through the actual encoder, SRT, MediaMTX, backend WHEP proxy and Chrome.
Chrome decoded 11 video frames and advancing playback time; sampled pixels
matched the test source. This proves transport with synthetic images, not physical
drone capture or a public Internet ICE/TURN route. Evidence is in
`/tmp/nv-stream-browser-report.json` and `/tmp/nv-stream-browser.png` on the test host.
