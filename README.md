## 2D polygonal curve velocity-dependent circle smoothing

Simple example:
```
from obj_mov import Track, PolyCurvePoint, Point2, LineSegment, ArcSegment
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
```

example.py(PyQt5 required) result:

![alt text](https://github.com/Farfurix/2D_curve_smoothing/blob/master/README_imgs/TrackWindow.png)


