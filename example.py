import sys
from PyQt5.QtCore import QSize, Qt, QTimer, QPointF, QTime, QLineF
from PyQt5.QtGui import QColor, QPainter, QPalette, QPen
from PyQt5.QtWidgets import QApplication, QGridLayout, QSizePolicy, QWidget
from math import sin, cos
from obj_mov import Track, PolyCurvePoint, Point2, LineSegment, ArcSegment

class TrackWidget(QWidget):
    def __init__(self, track, scale, parent=None):
        super(TrackWidget, self).__init__(parent)

        self.antialiased = False
        self.frameNo = 0

        self.setBackgroundRole(QPalette.Base)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.point_pen = QPen(QColor(0, 0, 0), 4)
        self.segment_pen = QPen(QColor(0, 0, 0), 2)
        self.base_poly_curve_pen = QPen(QColor(127, 127, 127), 1)
        self.current_point_pen = QPen(QColor(200, 0, 0), 5)
        self.tangent_line_pen = QPen(QColor(200, 0, 0), 1)

        self.time = QTime()
        self.time.start()

    def set_track(self, track):
        self.track = track

    def setAntialiased(self, antialiased):
        self.antialiased = antialiased
        self.update()

    def nextAnimationFrame(self):
        self.frameNo += 1
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, self.antialiased)

        for i in range(len(self.track.points) - 1):
            painter.setPen(self.base_poly_curve_pen)
            painter.drawLine(QLineF(self.track.points[i].point.x * scale, self.track.points[i].point.y * scale, 
                                    self.track.points[i + 1].point.x * scale, self.track.points[i + 1].point.y * scale))

        for segment in self.track.segments:
            if type(segment) == LineSegment:
                painter.setPen(self.point_pen)
                painter.drawPoint(QPointF(segment.start.x * scale, segment.start.y * scale))
                painter.drawPoint(QPointF(segment.finish.x * scale, segment.finish.y * scale))

                painter.setPen(self.segment_pen)
                painter.drawLine(segment.start.x * scale, segment.start.y * scale,
                                 segment.finish.x * scale, segment.finish.y * scale)
            elif type(segment) == ArcSegment:
                painter.setPen(self.point_pen)
                painter.drawPoint(QPointF(segment.start.x * scale, segment.start.y * scale))
                painter.drawPoint(QPointF(segment.finish.x * scale, segment.finish.y * scale))
                painter.setPen(self.segment_pen)
                painter.drawEllipse(QPointF(segment.M.x * scale, segment.M.y * scale), segment.smooth_radius * scale, segment.smooth_radius * scale)

            current_time = self.time.elapsed()
            current_point = Track.get_track_point(track, current_time)
            painter.setPen(self.current_point_pen)
            painter.drawPoint(QPointF(current_point.x * scale, current_point.y * scale))

            painter.setPen(self.tangent_line_pen)
            painter.drawLine(QLineF(current_point.x * scale, current_point.y * scale, 
                                    current_point.x * scale + 20 * cos(current_point.angle),
                                    current_point.y * scale + 20 * sin(current_point.angle)))

class Window(QWidget):
    def __init__(self, track, scale = 1.0):
        super(Window, self).__init__()

        layout = QGridLayout()
        timer = QTimer(self)

        w = TrackWidget(track, scale)
        w.setAntialiased(True)
        w.set_track(track)

        timer.timeout.connect(w.nextAnimationFrame)

        layout.addWidget(w, 0, 0)

        timer.start(25)
        self.setLayout(layout)

        self.setWindowTitle("Track")

if __name__ == '__main__':
    points = []
    points.append(PolyCurvePoint(Point2(1.0, 1.0), 0.2))
    points.append(PolyCurvePoint(Point2(5.0, 1.0), 4.0))
    points.append(PolyCurvePoint(Point2(10.0, 1.0), 1.0))
    points.append(PolyCurvePoint(Point2(12.0, 9.0), 2.0))
    points.append(PolyCurvePoint(Point2(1.0, 5.0), 3.0))
    points.append(PolyCurvePoint(Point2(2.0, 15.0), 0.0))
    points.append(PolyCurvePoint(Point2(8.0, 15.0), 0.0))

    # Data preparation
    if len(points) > 0:
        points[0].velocity = abs(points[0].velocity)
        points[-1].velocity = abs(points[-1].velocity)
        for i in range(1, len(points) - 1):
            if points[i].velocity == 0.0:
                points[i].velocity = 1.0
            else:
                points[i].velocity = abs(points[i].velocity)

    track = Track(points)

    app = QApplication(sys.argv)
    scale = 20.0
    window = Window(track, scale)
    x_coordinates = []
    y_coordinates = []
    for p in points:
        x_coordinates.append(p.point.x)
        y_coordinates.append(p.point.y)

    window.resize(max(x_coordinates) * scale + 50.0, max(y_coordinates) * scale + 50.0)
    window.show()
    sys.exit(app.exec_())
