import sys
import os
import time
import math
import socket
import serial
import threading
import queue

from collections import deque
from serial.tools import list_ports

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
from PyQt5.QtWebEngineCore import QWebEngineUrlSchemeHandler, QWebEngineUrlScheme
from PyQt5.QtCore import QBuffer, QIODevice, QUrl

import pyqtgraph as pg

# =========================
# REGISTER CUSTOM SCHEME
# Must happen before QApplication is created
# =========================

scheme = QWebEngineUrlScheme(b"tile")
scheme.setFlags(
    QWebEngineUrlScheme.SecureScheme |
    QWebEngineUrlScheme.LocalAccessAllowed
)
QWebEngineUrlScheme.registerScheme(scheme)


# =========================
# CONFIG
# =========================

BAUD_RATE  = 115200
MAX_POINTS = 1000

data_queue = queue.Queue()
ser_thread = None


# =========================
# PATH SAFE (PY + EXE)
# =========================

def app_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))

def get_tile_dir():
    return os.path.join(app_dir(), "maptiles")

TILE_DIR = get_tile_dir()


# =========================
# INTERNET CHECK
# =========================

def has_internet():
    try:
        socket.create_connection(("1.1.1.1", 53), timeout=2)
        return True
    except:
        return False


# =========================
# TILE SCHEME HANDLER
# =========================

EMPTY_PNG = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01"
    b"\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00\x00"
    b"\x00\x0cIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00\x05\x18"
    b"\xd8N\x00\x00\x00\x00IEND\xaeB`\x82"
)

class TileSchemeHandler(QWebEngineUrlSchemeHandler):
    def __init__(self, tile_dir, parent=None):
        super().__init__(parent)
        self.tile_dir = tile_dir

    def requestStarted(self, request):
        path = request.requestUrl().path().lstrip("/")
        full = os.path.join(self.tile_dir, path)

        buf = QBuffer(parent=self)
        buf.open(QIODevice.WriteOnly)

        if os.path.exists(full):
            with open(full, "rb") as fh:
                buf.write(fh.read())
        else:
            buf.write(EMPTY_PNG)

        request.reply(b"image/png", buf)


# =========================
# SAFE FLOAT
# =========================

def f(x):
    try:
        return float(x)
    except:
        return 0.0


# =========================
# GPS DISTANCE
# =========================

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) *
         math.sin(dlon / 2) ** 2)
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# =========================
# SERIAL THREAD
# =========================

class SerialThread(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, BAUD_RATE, timeout=1)

    def run(self):
        while True:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    data_queue.put(line)
            except:
                continue


# =========================
# BUFFERS
# =========================

t_buf      = deque(maxlen=MAX_POINTS)
baro_buf   = deque(maxlen=MAX_POINTS)
gpsalt_buf = deque(maxlen=MAX_POINTS)
ax_buf     = deque(maxlen=MAX_POINTS)
ay_buf     = deque(maxlen=MAX_POINTS)
az_buf     = deque(maxlen=MAX_POINTS)
rssi_buf   = deque(maxlen=MAX_POINTS)
snr_buf    = deque(maxlen=MAX_POINTS)
speed_buf  = deque(maxlen=MAX_POINTS)


# =========================
# MAP HTML
# Markers use a pure SVG circle icon so no external image files are needed.
# This works identically online and offline.
# =========================

MAP_HTML = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<style>
html, body { margin:0; height:100%; }
#map { height:100%; }
</style>

__LEAFLET_CSS__
__LEAFLET_JS__

</head>
<body>
<div id="map"></div>
<script>

var map = L.map('map', {minZoom: __MIN_ZOOM__, maxZoom: __MAX_ZOOM__}).setView([52.8154254, -4.1315230], 16);

// SVG circle icon — no external image files needed, works offline
var svgIcon = L.divIcon({
    html: '<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16">'
        + '<circle cx="8" cy="8" r="6" fill="red" stroke="white" stroke-width="2"/>'
        + '</svg>',
    className: '',
    iconSize:   [16, 16],
    iconAnchor: [8, 8],
    popupAnchor:[0, -10]
});

L.marker([52.8154254,-4.131523], {icon: svgIcon})
  .addTo(map)
  .bindPopup("Launch Site");

L.tileLayer("__TILE_URL__", {
    minZoom: __MIN_ZOOM__,
    maxZoom: __MAX_ZOOM__,
    tileSize: 256,
    noWrap:   true,
    errorTileUrl: ''
}).addTo(map);

var path   = L.polyline([], {color: 'red'}).addTo(map);
var marker = null;

function update(lat, lon) {
    var p = [lat, lon];
    path.addLatLng(p);
    if (marker) { map.removeLayer(marker); }
    marker = L.marker(p, {icon: svgIcon}).addTo(map);
    map.panTo(p);
}

</script>
</body>
</html>
"""


def build_map_html(tile_url, online):
    if online:
        min_zoom    = 15
        max_zoom    = 18
        leaflet_css = '<link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>'
        leaflet_js  = '<script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>'
    else:
        min_zoom = 15
        max_zoom = 18
        css_path = os.path.join(app_dir(), "leaflet", "leaflet.css")
        js_path  = os.path.join(app_dir(), "leaflet", "leaflet.js")
        try:
            with open(css_path, "r", encoding="utf-8") as fh:
                css_content = fh.read()
            with open(js_path, "r", encoding="utf-8") as fh:
                js_content = fh.read()
            leaflet_css = f"<style>{css_content}</style>"
            leaflet_js  = f"<script>{js_content}</script>"
        except FileNotFoundError:
            print("WARNING: leaflet/ folder not found, falling back to CDN")
            leaflet_css = '<link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>'
            leaflet_js  = '<script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>'

    return (MAP_HTML
            .replace("__TILE_URL__",    tile_url)
            .replace("__LEAFLET_CSS__", leaflet_css)
            .replace("__LEAFLET_JS__",  leaflet_js)
            .replace("__MIN_ZOOM__",    str(min_zoom))
            .replace("__MAX_ZOOM__",    str(max_zoom)))


# =========================
# DASHBOARD
# =========================

class Dashboard(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("SX1280 GCS")
        self.resize(1600, 900)
        self.setStyleSheet("background:black;")

        main = QtWidgets.QVBoxLayout(self)

        # =========================
        # MAP MODE
        # =========================

        online = has_internet()

        if online:
            tile_url = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        else:
            tile_url = "tile:///{z}/{x}/{y}.png"

            if not os.path.exists(TILE_DIR):
                QtWidgets.QMessageBox.warning(
                    self, "Offline Mode",
                    f"Tile cache not found at:\n{TILE_DIR}\nMap will be blank."
                )

        html = build_map_html(tile_url, online)

        QWebEngineSettings.globalSettings().setAttribute(
            QWebEngineSettings.LocalContentCanAccessFileUrls, True
        )

        # =========================
        # HUD
        # =========================

        top = QtWidgets.QHBoxLayout()

        self.gps     = self.box("GPS FIX")
        self.baro    = self.box("BARO (m)")
        self.gps_alt = self.box("GPS ALT (m)")
        self.temp    = self.box("TEMP (°C)")
        self.rssi    = self.box("RSSI")
        self.speed   = self.box("SPD (m/s)")

        self.ax = self.box("AX (G)")
        self.ay = self.box("AY (G)")
        self.az = self.box("AZ (G)")

        self.sats = self.box("SAT")
        self.lat  = self.box("LAT")
        self.lon  = self.box("LON")
        self.pkt  = self.box("PKT")

        for w in [
            self.gps, self.baro, self.gps_alt, self.temp, self.rssi,
            self.speed, self.ax, self.ay, self.az,
            self.sats, self.lat, self.lon, self.pkt
        ]:
            top.addWidget(w)

        main.addLayout(top)

        # =========================
        # GRAPHS + MAP
        # =========================

        center = QtWidgets.QHBoxLayout()

        self.graph = pg.GraphicsLayoutWidget()

        self.p1 = self.graph.addPlot(title="Altitude (m)")
        self.baro_curve   = self.p1.plot(pen='y', name="Baro")
        self.gpsalt_curve = self.p1.plot(pen='c', name="GPS")

        self.graph.nextRow()

        self.p2 = self.graph.addPlot(title="Acceleration (G)")
        self.ax_curve = self.p2.plot(pen='r', name="X")
        self.ay_curve = self.p2.plot(pen='g', name="Y")
        self.az_curve = self.p2.plot(pen='b', name="Z")

        self.graph.nextRow()

        self.p3 = self.graph.addPlot(title="Link Quality")
        self.rssi_curve = self.p3.plot(pen='m', name="RSSI")
        self.snr_curve  = self.p3.plot(pen='w', name="SNR")

        self.graph.nextRow()

        self.p4 = self.graph.addPlot(title="Speed (m/s)")
        self.speed_curve = self.p4.plot(pen='c')

        center.addWidget(self.graph, 55)

        # =========================
        # NO TX OVERLAY
        # Parented to self.graph so it floats over the plots.
        # Shown when no packet has arrived for more than 1 second.
        # =========================

        self.no_tx_label = QtWidgets.QLabel("No TX", self.graph)
        self.no_tx_label.setStyleSheet(
            "color: red;"
            "font-size: 48px;"
            "font-weight: bold;"
            "background: transparent;"
        )
        self.no_tx_label.setAlignment(QtCore.Qt.AlignCenter)
        self.no_tx_label.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.no_tx_label.hide()

        # Map widget — tile handler installed before setHtml
        self.map = QWebEngineView()

        if not online and os.path.exists(TILE_DIR):
            # Must be stored on self or Python garbage collects it
            self.tile_handler = TileSchemeHandler(TILE_DIR)
            self.map.page().profile().installUrlSchemeHandler(
                b"tile", self.tile_handler
            )

        time.sleep(0.5)
        self.map.setHtml(html, QUrl("tile:///"))

        center.addWidget(self.map, 45)

        main.addLayout(center)

        # =========================
        # STATE
        # =========================

        self.last_lat  = None
        self.last_lon  = None
        self.last_time = time.time()
        self.speed_val = 0

        self.base_pkt = None
        self.last_pkt = None

        self.last_rx_time = time.time()  # updated each time a valid packet arrives

        # =========================
        # TIMER
        # =========================

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)


    # =========================
    # HUD BOX
    # =========================

    def box(self, name):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)

        lbl_name = QtWidgets.QLabel(name)
        lbl_name.setStyleSheet("color:#00ff00;")
        lbl_name.setAlignment(QtCore.Qt.AlignCenter)

        lbl_val = QtWidgets.QLabel("---")
        lbl_val.setStyleSheet("color:#00ff00; font-size:12px;")
        lbl_val.setAlignment(QtCore.Qt.AlignCenter)

        v.addWidget(lbl_name)
        v.addWidget(lbl_val)

        w.val = lbl_val
        return w


    # =========================
    # UPDATE LOOP (NON-BLOCKING)
    # =========================

    def update(self):

        # =========================
        # NO TX OVERLAY CHECK
        # Runs every timer tick (50 ms) regardless of queue state.
        # If more than 1 second has passed since the last good packet,
        # stretch the label to cover the graph widget and show it.
        # =========================

        if (time.time() - self.last_rx_time) > 1.0:
            self.no_tx_label.setGeometry(
                0, 0, self.graph.width(), self.graph.height()
            )
            self.no_tx_label.show()
            self.no_tx_label.raise_()
        else:
            self.no_tx_label.hide()

        if data_queue.empty():
            return

        line = data_queue.get()

        # Drop warning / error lines from RX
        if line.startswith("#"):
            print(line)
            return

        try:
            p = line.split(",")
            if len(p) < 14:
                return

            # CSV columns from RX:
            # 0:counter  1:xAcc(G)  2:yAcc(G)  3:zAcc(G)
            # 4:temp(C)  5:baro(m)  6:gpsAlt(m)
            # 7:fix      8:lat      9:lon      10:sats
            # 11:rssi    12:snr     13:freqErr

            pkt_id = int(f(p[0]))

            # =========================
            # RESET DETECTION
            # =========================

            if self.base_pkt is None:
                self.base_pkt = pkt_id

            rel_pkt = pkt_id - self.base_pkt

            if self.last_pkt is not None:
                if pkt_id < self.last_pkt or (self.last_pkt - pkt_id) > 50:
                    print("TX RESET detected")

                    self.base_pkt = pkt_id
                    rel_pkt = 0

                    for buf in (t_buf, baro_buf, gpsalt_buf, ax_buf,
                                ay_buf, az_buf, rssi_buf, snr_buf, speed_buf):
                        buf.clear()

            self.last_pkt = pkt_id

            # =========================
            # PARSE
            # =========================

            ax      = f(p[1])       # G
            ay      = f(p[2])       # G
            az      = f(p[3])       # G
            temp    = f(p[4])       # °C
            baro    = f(p[5])       # m
            gps_alt = f(p[6])       # m
            fix     = int(f(p[7]))
            lat     = f(p[8])
            lon     = f(p[9])
            sats    = int(f(p[10]))
            rssi    = f(p[11])
            snr     = f(p[12])

            gps_valid = fix >= 3 and sats >= 6

            # =========================
            # SPEED
            # =========================

            now = time.time()
            dt  = now - self.last_time
            self.last_time = now

            if gps_valid and self.last_lat is not None:
                dist = haversine(self.last_lat, self.last_lon, lat, lon)
                self.speed_val = dist / dt if dt > 0 else 0

            if gps_valid:
                self.last_lat = lat
                self.last_lon = lon
                self.map.page().runJavaScript(f"update({lat},{lon});")

            # =========================
            # HUD
            # =========================

            self.gps.val.setText(f"{fix} ({'3D' if fix >= 3 else 'NO FIX'})")
            self.baro.val.setText(f"{baro:.1f}")
            self.gps_alt.val.setText(f"{gps_alt:.1f}")
            self.temp.val.setText(f"{temp:.1f}")
            self.rssi.val.setText(f"{rssi:.1f}")
            self.speed.val.setText(f"{self.speed_val:.2f}")

            self.ax.val.setText(f"{ax:.3f}")
            self.ay.val.setText(f"{ay:.3f}")
            self.az.val.setText(f"{az:.3f}")

            self.sats.val.setText(str(sats))
            self.lat.val.setText(f"{lat:.6f}")
            self.lon.val.setText(f"{lon:.6f}")
            self.pkt.val.setText(str(pkt_id))

            # =========================
            # BUFFERS
            # =========================

            t_buf.append(rel_pkt)
            baro_buf.append(baro)
            gpsalt_buf.append(gps_alt)
            ax_buf.append(ax)
            ay_buf.append(ay)
            az_buf.append(az)
            rssi_buf.append(rssi)
            snr_buf.append(snr)
            speed_buf.append(self.speed_val)

            # =========================
            # GRAPHS
            # =========================

            t_list = list(t_buf)

            self.baro_curve.setData(t_list,   list(baro_buf))
            self.gpsalt_curve.setData(t_list, list(gpsalt_buf))
            self.ax_curve.setData(t_list,     list(ax_buf))
            self.ay_curve.setData(t_list,     list(ay_buf))
            self.az_curve.setData(t_list,     list(az_buf))
            self.rssi_curve.setData(t_list,   list(rssi_buf))
            self.snr_curve.setData(t_list,    list(snr_buf))
            self.speed_curve.setData(t_list,  list(speed_buf))

            # =========================
            # MARK SUCCESSFUL RECEIVE
            # Reset the no-TX timer on every valid packet.
            # =========================

            self.last_rx_time = time.time()

        except Exception as e:
            print("Parse error:", e)


# =========================
# PORT SELECTOR
# =========================

class PortSelector(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Select Receiver")
        self.setFixedSize(320, 160)

        layout = QtWidgets.QVBoxLayout(self)

        self.combo = QtWidgets.QComboBox()

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_refresh = QtWidgets.QPushButton("Reload Ports")
        self.btn_connect = QtWidgets.QPushButton("Connect")
        btn_row.addWidget(self.btn_refresh)
        btn_row.addWidget(self.btn_connect)

        layout.addWidget(self.combo)
        layout.addLayout(btn_row)

        self.btn_refresh.clicked.connect(self.refresh)
        self.btn_connect.clicked.connect(self.connect)

        self.refresh()

    def refresh(self):
        self.combo.clear()
        ports = list_ports.comports()
        for p in ports:
            self.combo.addItem(f"{p.device} - {p.description}", p.device)
        if not ports:
            self.combo.addItem("No ports found", None)

    def connect(self):
        port = self.combo.currentData()
        if port is None:
            QtWidgets.QMessageBox.warning(self, "No Port", "No serial port selected.")
            return

        global ser_thread
        ser_thread = SerialThread(port)
        ser_thread.start()

        self.w = Dashboard()
        self.w.show()
        self.close()


# =========================
# MAIN
# =========================

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = PortSelector()
    w.show()
    sys.exit(app.exec_())
