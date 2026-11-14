#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TMAP 보행자 길찾기 API 및 지도 처리 모듈
- SK Open API 보행자 경로 탐색 호출
- 경로 좌표(Waypoints) 및 가이드 안내 문구 파싱
- Leaflet / HTML 지도 생성 및 QWebEngine / 캔버스 연동
"""

import json
import urllib.parse
import urllib.request
import math

class TmapEngine:
    def __init__(self, app_key="Y1SXBnTMciaDlRMu3rsyU2TFuN3kXKvC6X0zGFwY"):
        self.app_key = app_key
        self.api_url = "https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1"
        self.route_points = [] # [(lat, lon), ...]
        self.guide_descriptions = []
        self.current_lat = 36.0378  # 기본 좌표 (포항고등학교 인근 예시)
        self.current_lon = 129.3650
        self.target_lat = 36.0390
        self.target_lon = 129.3670
        self.heading_deg = 0.0

    def set_current_position(self, lat, lon, heading=0.0):
        self.current_lat = lat
        self.current_lon = lon
        self.heading_deg = heading

    def set_destination(self, lat, lon):
        self.target_lat = lat
        self.target_lon = lon

    def request_pedestrian_route(self, start_lat=None, start_lon=None, end_lat=None, end_lon=None):
        """SK TMAP 보행자 경로 API 요청"""
        s_lat = start_lat if start_lat else self.current_lat
        s_lon = start_lon if start_lon else self.current_lon
        e_lat = end_lat if end_lat else self.target_lat
        e_lon = end_lon if end_lon else self.target_lon

        headers = {
            "appKey": self.app_key,
            "Content-Type": "application/json"
        }
        payload = {
            "startX": f"{s_lon:.6f}",
            "startY": f"{s_lat:.6f}",
            "endX": f"{e_lon:.6f}",
            "endY": f"{e_lat:.6f}",
            "reqCoordType": "WGS84GEO",
            "resCoordType": "WGS84GEO",
            "startName": urllib.parse.quote("현재위치"),
            "endName": urllib.parse.quote("목적지"),
            "searchOption": "0" # 0: 추천경로
        }

        try:
            req = urllib.request.Request(
                self.api_url, 
                data=json.dumps(payload).encode('utf-8'),
                headers=headers,
                method='POST'
            )
            with urllib.request.urlopen(req, timeout=5) as response:
                if response.status == 200:
                    data = json.loads(response.read().decode('utf-8'))
                    return self.parse_route_response(data)
        except Exception as e:
            print(f"[!] TMAP API 요청 실패 ({e}), 시뮬레이션 경로를 사용합니다.")
            return self.generate_mock_route(s_lat, s_lon, e_lat, e_lon)

    def parse_route_response(self, data):
        self.route_points = []
        self.guide_descriptions = []
        
        for feature in data.get("features", []):
            geom = feature.get("geometry", {})
            props = feature.get("properties", {})
            
            # 경로 좌표 추출
            if geom.get("type") == "Point":
                coords = geom.get("coordinates", [])
                if len(coords) >= 2:
                    self.route_points.append((coords[1], coords[0])) # (lat, lon)
            elif geom.get("type") == "LineString":
                coords = geom.get("coordinates", [])
                for pt in coords:
                    self.route_points.append((pt[1], pt[0])) # (lat, lon)

            # 음성/화면 안내 문구 추출
            desc = props.get("description")
            if desc:
                self.guide_descriptions.append(desc)

        return True

    def generate_mock_route(self, s_lat, s_lon, e_lat, e_lon):
        """오프라인 또는 API 미연결 시 가상 경로 생성"""
        self.route_points = []
        steps = 20
        for i in range(steps + 1):
            t = i / float(steps)
            # 완만한 곡선 경로
            lat = s_lat + (e_lat - s_lat) * t + math.sin(t * math.pi) * 0.0003
            lon = s_lon + (e_lon - s_lon) * t
            self.route_points.append((lat, lon))
        self.guide_descriptions = [
            "경로 안내를 시작합니다.",
            "30m 앞 직진 후 우측 보행로로 이동하세요.",
            "전방 횡단보도에서 대기하세요.",
            "목적지 부근입니다."
        ]
        return True

    def generate_leaflet_html(self):
        """Leaflet 기반 다크 스타일 반응형 내비게이션 HTML 지도 생성"""
        pts_js = json.dumps([[lat, lon] for lat, lon in self.route_points])
        current_js = json.dumps([self.current_lat, self.current_lon])
        target_js = json.dumps([self.target_lat, self.target_lon])
        guide_text = self.guide_descriptions[0] if self.guide_descriptions else "경로를 따라 이동하세요."

        html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body, html {{ margin: 0; padding: 0; width: 100%; height: 100%; background: #0f172a; font-family: 'Segoe UI', sans-serif; overflow: hidden; }}
        #map {{ width: 100%; height: 100%; }}
        .hud-overlay {{
            position: absolute; top: 12px; left: 12px; right: 12px; z-index: 1000;
            background: rgba(15, 23, 42, 0.85); backdrop-filter: blur(8px);
            border: 1px solid rgba(56, 189, 248, 0.4); border-radius: 10px;
            padding: 10px 14px; color: #f8fafc; font-size: 13px;
            display: flex; align-items: center; justify-content: space-between;
            box-shadow: 0 4px 15px rgba(0,0,0,0.5);
        }}
        .hud-badge {{ background: #0284c7; color: white; padding: 3px 8px; border-radius: 6px; font-weight: bold; font-size: 11px; }}
    </style>
</head>
<body>
    <div class="hud-overlay">
        <div><span class="hud-badge">TMAP 안내</span> <b>{guide_text}</b></div>
        <div style="color: #38bdf8;">GPS: {self.current_lat:.5f}, {self.current_lon:.5f}</div>
    </div>
    <div id="map"></div>
    <script>
        var map = L.map('map', {{ zoomControl: false }}).setView({current_js}, 17);
        L.tileLayer('https://{{s}}.basemaps.cartocdn.com/dark_all/{{z}}/{{x}}/{{y}}{{r}}.png', {{
            maxZoom: 19,
            attribution: 'TMAP / OpenStreetMap'
        }}).addTo(map);

        var route = {pts_js};
        if (route.length > 0) {{
            var polyline = L.polyline(route, {{ color: '#00f0ff', weight: 6, opacity: 0.85, dashArray: '8, 8' }}).addTo(map);
            map.fitBounds(polyline.getBounds(), {{ padding: [30, 30] }});
        }}

        // 현재 위치 카트 마커 (방동 펄스 효과)
        var cartIcon = L.divIcon({{
            className: 'cart-marker',
            html: '<div style="width:20px; height:20px; background:#10b981; border:3px solid #ffffff; border-radius:50%; box-shadow:0 0 15px #10b981;"></div>',
            iconSize: [20, 20],
            iconAnchor: [10, 10]
        }});
        var marker = L.marker({current_js}, {{ icon: cartIcon }}).addTo(map).bindPopup("현재 카트 위치");

        // 목적지 마커
        var destIcon = L.divIcon({{
            className: 'dest-marker',
            html: '<div style="width:20px; height:20px; background:#ef4444; border:3px solid #ffffff; border-radius:50%; box-shadow:0 0 15px #ef4444;"></div>',
            iconSize: [20, 20],
            iconAnchor: [10, 10]
        }});
        L.marker({target_js}, {{ icon: destIcon }}).addTo(map).bindPopup("목적지");
    </script>
</body>
</html>"""
        return html
