#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 파일: ~/2026_ROBO_1/scripts/jetson_lidar_web.py
# 실행: python3 jetson_lidar_web.py
# 브라우저: http://127.0.0.1:8765 또는 http://<Jetson-IP>:8765

import json
import math
import threading
import time

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformException, TransformListener

# 0.0.0.0으로 바인딩하여 젯슨 로컬 및 노트북 브라우저 모두 접속 가능
HOST = "0.0.0.0"
PORT = 8765


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class ViewerNode(Node):
    def __init__(self):
        super().__init__("jetson_lidar_web_viewer")

        self.lock = threading.Lock()

        self.scan = None
        self.scan_stamp = None
        self.scan_frame = ""
        self.scan_wall_time = 0.0

        self.map = None
        self.map_seq = 0
        self.map_frame = "map"

        self.pose = None
        self.tf_status = "scan 대기 중"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.on_scan,
            qos_profile_sensor_data
        )

        # Transient Local 지도 구독
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            OccupancyGrid, "/map", self.on_map, map_qos
        )

        # Volatile 퍼블리셔와도 호환되도록 추가 구독
        live_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            OccupancyGrid, "/map", self.on_map, live_qos
        )

        self.create_timer(0.1, self.update_pose)

    def on_scan(self, msg):
        points = []
        angle = msg.angle_min

        for distance in msg.ranges:
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            ):
                points.append([
                    round(distance * math.cos(angle), 3),
                    round(distance * math.sin(angle), 3),
                ])

            angle += msg.angle_increment

        with self.lock:
            self.scan = {
                "points": points,
                "frame": msg.header.frame_id,
                "range_max": (
                    float(msg.range_max)
                    if math.isfinite(msg.range_max)
                    else 8.0
                ),
                "count": len(points),
            }

            self.scan_frame = msg.header.frame_id
            self.scan_stamp = msg.header.stamp
            self.scan_wall_time = time.time()

    def on_map(self, msg):
        info = msg.info
        p = info.origin.position

        new_map = {
            "w": int(info.width),
            "h": int(info.height),
            "res": float(info.resolution),
            "ox": float(p.x),
            "oy": float(p.y),
            "yaw": yaw_from_quaternion(
                info.origin.orientation
            ),
            "frame": msg.header.frame_id or "map",
            "data": list(msg.data),
        }

        with self.lock:
            self.map = new_map
            self.map_frame = new_map["frame"]
            self.map_seq += 1

    def update_pose(self):
        with self.lock:
            frame = self.scan_frame
            target = self.map_frame
            stamp = self.scan_stamp

        if not frame:
            return

        if frame == target:
            with self.lock:
                self.pose = [0.0, 0.0, 0.0]
                self.tf_status = "동일 프레임"
            return

        try:
            if stamp and (stamp.sec or stamp.nanosec):
                requested_time = Time.from_msg(stamp)
            else:
                requested_time = Time()

            try:
                transform = self.tf_buffer.lookup_transform(
                    target, frame, requested_time
                )
                status = "TF 정상 (스캔 시각 기준)"

            except TransformException:
                transform = self.tf_buffer.lookup_transform(
                    target, frame, Time()
                )
                status = "TF 최신값 사용 (시간 정렬 확인 필요)"

            t = transform.transform

            pose = [
                float(t.translation.x),
                float(t.translation.y),
                yaw_from_quaternion(t.rotation),
            ]

        except TransformException as exc:
            pose = None
            status = "TF 없음: " + str(exc)[:160]

        with self.lock:
            self.pose = pose
            self.tf_status = status

    def scan_snapshot(self):
        with self.lock:
            return {
                "scan": self.scan,
                "pose": self.pose,
                "tf_status": self.tf_status,
                "map_available": self.map is not None,
                "scan_age_s": (
                    round(
                        time.time() - self.scan_wall_time, 1
                    )
                    if self.scan is not None
                    else None
                ),
            }

    def map_snapshot(self, since):
        with self.lock:
            seq = self.map_seq

            return {
                "seq": seq,
                "map": (
                    self.map if seq != since else None
                ),
            }


HTML = r'''
<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport"
      content="width=device-width,initial-scale=1">
<title>Jetson 4륜 메카넘 카트 LiDAR / SLAM 모니터</title>

<style>
* { box-sizing: border-box; }

html, body {
    margin: 0;
    width: 100%;
    height: 100%;
    background: #0b1220;
    color: #e2e8f0;
    font: 14px system-ui, -apple-system, sans-serif;
}

main {
    display: flex;
    flex-direction: column;
    height: 100%;
}

header {
    padding: 12px 16px;
    background: #111d33;
    border-bottom: 1px solid #334155;
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 10px;
    flex-wrap: wrap;
}

h1 {
    font-size: 16px;
    font-weight: 600;
    margin: 0;
    color: #38bdf8;
}

#status {
    font-size: 13px;
    color: #94a3b8;
    background: #1e293b;
    padding: 4px 10px;
    border-radius: 6px;
}

canvas {
    flex: 1;
    width: 100%;
    min-height: 100px;
    display: block;
    touch-action: none;
}

footer {
    padding: 8px 16px;
    background: #111d33;
    border-top: 1px solid #334155;
    color: #cbd5e1;
    font-size: 13px;
    display: flex;
    gap: 20px;
}

.legend-item {
    display: inline-flex;
    align-items: center;
    gap: 6px;
}

.dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    display: inline-block;
}
</style>
</head>

<body>
<main>
<header>
    <h1>🛰️ 4륜 메카넘 카트 2D LiDAR · SLAM 실시간 지도 모니터</h1>
    <div id="status">ROS 2 토픽 수신 대기 중...</div>
</header>

<canvas id="view"></canvas>

<footer>
    <div class="legend-item"><span class="dot" style="background:#34d399"></span> 초록선: 라이다 실시간 거리 빔</div>
    <div class="legend-item"><span class="dot" style="background:#f87171"></span> 붉은점: 카트(라이다) 현재 위치</div>
    <div class="legend-item"><span class="dot" style="background:#64748b"></span> 흑백/회색: SLAM 실시간 지도</div>
</footer>
</main>

<script>
const canvas = document.getElementById("view");
const ctx = canvas.getContext("2d");
const status = document.getElementById("status");

let map = null;
let mapSeq = -1;
let mapBitmap = null;
let scan = null;
let pose = null;
let tfStatus = "";
let scanAge = null;
let scanBusy = false;

function resize() {
    const r = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;

    canvas.width = Math.max(1, Math.floor(r.width * dpr));
    canvas.height = Math.max(1, Math.floor(r.height * dpr));

    draw();
}

window.addEventListener("resize", resize);

function buildMap(m) {
    if (!m || m.w < 1 || m.h < 1 ||
        m.data.length !== m.w * m.h) {
        return null;
    }

    const off = document.createElement("canvas");
    off.width = m.w;
    off.height = m.h;

    const c = off.getContext("2d");
    const image = c.createImageData(m.w, m.h);

    for (let y = 0; y < m.h; y++) {
        for (let x = 0; x < m.w; x++) {
            const v = m.data[y * m.w + x];
            const idx = ((m.h - 1 - y) * m.w + x) * 4;

            const a = v < 0 ? 90 : v >= 50 ? 31 : 225;

            image.data[idx] = a;
            image.data[idx + 1] = v < 0 ? 100 : a;
            image.data[idx + 2] = v < 0 ? 117 : a;
            image.data[idx + 3] = 255;
        }
    }

    c.putImageData(image, 0, 0);
    return off;
}

function draw() {
    const w = canvas.width;
    const h = canvas.height;

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = "#101b2b";
    ctx.fillRect(0, 0, w, h);

    if (!scan && !map) return;

    const hasMap = !!(map && mapBitmap);
    const overlay = !!(hasMap && scan && pose);

    let centerX = 0;
    let centerY = 0;
    let scale;

    if (hasMap) {
        const mx = map.w * map.res / 2;
        const my = map.h * map.res / 2;
        const c = Math.cos(map.yaw);
        const s = Math.sin(map.yaw);

        centerX = map.ox + c * mx - s * my;
        centerY = map.oy + s * mx + c * my;

        const ew =
            Math.abs(c) * map.w * map.res +
            Math.abs(s) * map.h * map.res;

        const eh =
            Math.abs(s) * map.w * map.res +
            Math.abs(c) * map.h * map.res;

        scale = Math.min(
            (w - 32) / (ew + 2),
            (h - 32) / (eh + 2)
        );
    } else {
        const maxR = Math.min(
            18, Math.max(4, scan?.range_max || 8)
        );

        scale = Math.min(w, h) / (2.4 * maxR);
    }

    ctx.translate(w / 2, h / 2);
    ctx.scale(scale, -scale);
    ctx.translate(-centerX, -centerY);

    // SLAM 지도
    if (hasMap) {
        ctx.save();
        ctx.translate(map.ox, map.oy);
        ctx.rotate(map.yaw);
        ctx.scale(map.res, -map.res);
        ctx.drawImage(mapBitmap, 0, -map.h);
        ctx.restore();
    }

    // 지도 위에 라이다 빔 중첩
    if (scan && (!hasMap || pose)) {
        const robot = overlay ? pose : [0, 0, 0];
        const [rx, ry, theta] = robot;

        const c = Math.cos(theta);
        const s = Math.sin(theta);

        ctx.strokeStyle = "rgba(52,211,153,0.55)";
        ctx.lineWidth = 1.2 / scale;
        ctx.beginPath();

        for (const [x, y] of scan.points) {
            ctx.moveTo(rx, ry);
            ctx.lineTo(
                rx + c * x - s * y,
                ry + s * x + c * y
            );
        }

        ctx.stroke();

        ctx.fillStyle = "#f87171";
        ctx.beginPath();
        ctx.arc(rx, ry, 6 / scale, 0, 2 * Math.PI);
        ctx.fill();
    }

    // TF 미연결 시 작은 창에서 LiDAR 단독 표시
    if (hasMap && scan && !pose) {
        ctx.setTransform(1, 0, 0, 1, 0, 0);

        const side = Math.min(195, w * .38, h * .50);
        const x = w - side - 12;
        const y = 12;

        ctx.fillStyle = "#182c43";
        ctx.fillRect(x, y, side, side);

        ctx.strokeStyle = "#64748b";
        ctx.strokeRect(x, y, side, side);

        ctx.fillStyle = "#cbd5e1";
        ctx.font = "12px system-ui";
        ctx.fillText(
            "LiDAR (TF 미연결)", x + 8, y + 17
        );

        const r = Math.max(
            1, Math.min(15, scan.range_max || 8)
        );

        const k = side / (2.3 * r);

        ctx.save();
        ctx.translate(x + side / 2, y + side / 2);
        ctx.scale(k, -k);

        ctx.strokeStyle = "rgba(52,211,153,.65)";
        ctx.lineWidth = 1 / k;
        ctx.beginPath();

        for (const [px, py] of scan.points) {
            ctx.moveTo(0, 0);
            ctx.lineTo(px, py);
        }

        ctx.stroke();
        ctx.restore();
    }
}

async function updateScan() {
    if (scanBusy) return;
    scanBusy = true;

    try {
        const r = await fetch(
            "/api/scan", {cache: "no-store"}
        );

        if (!r.ok) throw Error(r.status);

        const data = await r.json();

        scan = data.scan;
        pose = data.pose;
        tfStatus = data.tf_status;
        scanAge = data.scan_age_s;

        const tf = pose ? "TF 정상" : "TF 미연결";

        status.textContent =
            `/scan: ${scan ? scan.count + "점 (10Hz)" : "대기"}` +
            ` | /map: ${map ? "지도 수신 중" : "매핑 대기"}` +
            ` | ${tf}` +
            ` | 최근 수신: ${scanAge ?? "-"}초 전`;

        status.title = tfStatus;
        draw();

    } catch (e) {
        status.textContent =
            "서버 연결 확인: " + e.message;
    } finally {
        scanBusy = false;
    }
}

async function updateMap() {
    try {
        const r = await fetch(
            "/api/map?since=" + mapSeq,
            {cache: "no-store"}
        );

        if (!r.ok) return;

        const data = await r.json();

        if (data.map) {
            map = data.map;
            mapSeq = data.seq;
            mapBitmap = buildMap(map);
            draw();
        }
    } catch (e) {
        console.warn("Map polling:", e);
    }
}

resize();
updateMap();
updateScan();

setInterval(updateScan, 100);
setInterval(updateMap, 1000);
</script>
</body>
</html>
'''


def create_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlsplit(self.path)

            if parsed.path in ("/", "/index.html"):
                body = HTML.encode("utf-8")
                content_type = "text/html; charset=utf-8"

            elif parsed.path == "/api/scan":
                body = json.dumps(
                    node.scan_snapshot(),
                    separators=(",", ":")
                ).encode()

                content_type = "application/json; charset=utf-8"

            elif parsed.path == "/api/map":
                try:
                    since = int(
                        parse_qs(parsed.query)
                        .get("since", ["-1"])[0]
                    )
                except (ValueError, IndexError):
                    since = -1

                body = json.dumps(
                    node.map_snapshot(since),
                    separators=(",", ":")
                ).encode()

                content_type = "application/json; charset=utf-8"

            else:
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_args):
            pass

    return Handler


def main():
    rclpy.init()
    node = ViewerNode()

    server = ThreadingHTTPServer(
        (HOST, PORT),
        create_handler(node)
    )

    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True
    )
    thread.start()

    print(
        f"✅ LiDAR / SLAM Web Viewer 가동 완료: http://127.0.0.1:{PORT}",
        flush=True
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

