"""어두운 테마와 색.

플롯 색은 dataviz 기본 팔레트의 다크 모드 1~3번 슬롯(파랑·주황·청록: 세 가지끼리 색약 구분 검증됨),
상태 색은 같은 팔레트의 status 색(good/warning/serious/critical)을 쓴다. 상태 색은 항상 글자와 같이 쓴다.
"""
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPalette, QPixmap
from PySide6.QtCore import Qt

C = {
    "plane": "#0d0d0d",
    "surface": "#1a1a19",
    "panel": "#151514",
    "raised": "#232322",
    "ink": "#f0efec",
    "ink2": "#c3c2b7",
    "muted": "#898781",
    "grid": "#2c2c2a",
    "axis": "#383835",
    "border": "#2f2f2d",
    # 데이터 계열: 실측 / 목표 / 시뮬
    "real": "#3987e5",
    "target": "#d95926",
    "sim": "#199e70",
    # 상태
    "good": "#0ca30c",
    "warning": "#fab219",
    "serious": "#ec835a",
    "critical": "#d03b3b",
    "off": "#4a4a47",
    "uncal": "#9085e9",     # 영점 없음 (보라: 목표 주황·상태 노랑과 겹치지 않게)
}

# 3D 화면 링크 색 (RGBA 0~1)
RGBA = {
    "real_ok": (0.50, 0.66, 0.90, 1.0),
    "sim_ok": (0.45, 0.80, 0.66, 1.0),
    "uncal": (0.60, 0.55, 0.93, 1.0),
    "error": (0.88, 0.26, 0.26, 1.0),
    "offline": (0.55, 0.55, 0.55, 0.16),
    "base": (0.60, 0.61, 0.64, 1.0),
    "ghost_target": (0.92, 0.42, 0.18, 0.30),
    "ghost_real": (0.25, 0.55, 0.95, 0.32),
    "axis_arrow": (1.0, 0.82, 0.25, 1.0),
}


def apply(app):
    app.setStyle("Fusion")
    pal = QPalette()
    roles = {
        QPalette.Window: C["panel"], QPalette.WindowText: C["ink"], QPalette.Base: C["surface"],
        QPalette.AlternateBase: C["raised"], QPalette.ToolTipBase: C["raised"], QPalette.ToolTipText: C["ink"],
        QPalette.Text: C["ink"], QPalette.Button: C["raised"], QPalette.ButtonText: C["ink"],
        QPalette.BrightText: "#ffffff", QPalette.Highlight: "#2a5a9a", QPalette.HighlightedText: "#ffffff",
        QPalette.PlaceholderText: C["muted"], QPalette.Link: C["real"],
    }
    for role, col in roles.items():
        pal.setColor(role, QColor(col))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        pal.setColor(QPalette.Disabled, role, QColor(C["muted"]))
    app.setPalette(pal)
    f = QFont("Malgun Gothic", 9)
    app.setFont(f)
    app.setStyleSheet(f"""
        QToolTip {{ color: {C['ink']}; background: {C['raised']}; border: 1px solid {C['border']}; padding: 4px; }}
        QGroupBox {{ border: 1px solid {C['border']}; border-radius: 6px; margin-top: 14px; padding: 6px; }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {C['ink2']}; }}
        QTabWidget::pane {{ border: 1px solid {C['border']}; border-radius: 4px; }}
        QTabBar::tab {{ padding: 5px 12px; background: {C['panel']}; color: {C['ink2']};
                        border: 1px solid {C['border']}; border-bottom: none; }}
        QTabBar::tab:selected {{ background: {C['raised']}; color: {C['ink']}; }}
        QPushButton {{ padding: 4px 10px; border: 1px solid {C['border']}; border-radius: 4px; background: {C['raised']}; }}
        QPushButton:hover {{ background: #2c2c2a; }}
        QPushButton:pressed {{ background: #333331; }}
        QPushButton:disabled {{ color: {C['muted']}; background: {C['panel']}; }}
        QPushButton#estop {{ background: #8f1f1f; color: white; font-weight: bold; padding: 6px 14px;
                             border: 1px solid #b33; }}
        QPushButton#estop:hover {{ background: #a82626; }}
        QPushButton#primary {{ background: #1f4f8a; border: 1px solid #2f66aa; }}
        QPushButton#primary:hover {{ background: #255da0; }}
        QTableWidget {{ gridline-color: {C['grid']}; }}
        QHeaderView::section {{ background: {C['raised']}; color: {C['ink2']}; padding: 3px;
                                border: none; border-right: 1px solid {C['border']}; }}
        QSplitter::handle {{ background: {C['border']}; }}
        QLabel#muted {{ color: {C['muted']}; }}
        QLabel#value {{ font-family: Consolas, 'Malgun Gothic'; font-size: 10pt; }}
        QLabel#msg_err {{ color: {C['serious']}; }}
        QLabel#msg_ok {{ color: {C['ink2']}; }}
    """)


_icon_cache = {}


def dot_icon(color, size=10):
    key = (color, size)
    if key not in _icon_cache:
        pm = QPixmap(size, size)
        pm.fill(Qt.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QColor(color))
        p.setPen(Qt.NoPen)
        p.drawEllipse(1, 1, size - 2, size - 2)
        p.end()
        _icon_cache[key] = QIcon(pm)
    return _icon_cache[key]
