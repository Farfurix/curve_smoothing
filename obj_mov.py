"""2D polygonal curve velocity-dependent circle smoothing

Smoothed track consists of a combination of line and arc segments.
The line segment movement can be accelerated. The arc segment movement
doesn't have acceleration.

Input data
----------
2D polygonal curve with velocities:
    points = list(PolyCurvePoint).
First and last point velocities are non-negative.
The velocities of the remaining points are strictly positive.
See data preparation in example.

Output data
-----------
Smoothed track:
    Track(points) = list(LineSegment | ArcSegment).
Track point: 
    get_track_point(track, current_time) = TrackPoint2().

Example
--------
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

"""
from math import pi, sqrt, fabs, asin, acos, sin, cos

class ZeroVecsAngle(Exception):
    pass

class CalcVecAngleError(Exception):
    pass

class TrackPoint2(object):
    def __init__(self, x = 0.0, y = 0.0, angle = 0.0):
        self.x = x
        self.y = y
        self.angle = angle

class Point2(object):
    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

class PolyCurvePoint(object):
    def __init__(self, point = Point2(), velocity = 0.0):
        self.point = point
        self.velocity = velocity

class LineSegment(object):
    def __init__(self, start = Point2(), finish = Point2(),
                 start_velocity = 0.0, finish_velocity = 0.0,
                 tangent_angle = 0.0,
                 start_global_time = 0.0, finish_global_time = 0.0,
                 acceleration = 0.0):
        self.start = start
        self.finish = finish
        self.start_velocity = start_velocity # (meter/seconds)
        self.finish_velocity = finish_velocity # (meter/seconds)
        self.tangent_angle = tangent_angle # (radians)
        self.start_global_time = start_global_time
        self.finish_global_time = finish_global_time
        self.acceleration = acceleration

class ArcSegment(object):
    def __init__(self, start = Point2(), finish = Point2(),
                 velocity = 0.0, smooth_radius = 0.0,
                 start_point_angle = 0.0, finish_point_angle = 0.0,
                 M = Point2(), vec_product_sign = 0.0,
                 start_global_time = 0.0, finish_global_time = 0.0):
        self.start = start
        self.finish = finish
        self.velocity = velocity
        self.smooth_radius = smooth_radius
        self.start_point_angle = start_point_angle # rad
        self.finish_point_angle = finish_point_angle # rad
        self.M = M  # Smooth circle center point
        self.vec_product_sign = vec_product_sign
        self.start_global_time = start_global_time
        self.finish_global_time = finish_global_time

class Track():
    '''Smoothed track.

    The speed-smooth radius dependence is descibed in
    __calc_smooth_radius(p0, p1, p2) function and given as relation:
        velocity(meter/seconds) = smooth radius(meter)

    Track(points, id_obj = -1)

    Parameters
    ----------
    points : list(PolyCurvePoint())
        2D polygonal curve with velocities.
    id_obj : int, optional
        Mooving object id (multiple case).

    Class variables
    ---------------
    self.points : list(PolyCurvePoint())
        2D polygonal curve with velocities.
    self.segments : list(segment), segment : LineSegment() or ArcSegment()
        Smoothed track segments.

    Public methods
    --------------
    get_track_point(time)
        Get track point by time.

    '''
    def __init__(self, points, id_obj = -1):
        self.set_data(points)
        self.id_obj = id_obj

    def set_data(self, points):
        """Set data.

        Parameters
        ----------
        points : list(PolyCurvePoint())
            2D polygonal curve with velocities.

        """
        if len(points) == 0:
            return
        self.points = points
        self.segments = []
        self.__calc_segments(points)

    def get_track_point(self, time): # time >= 0
        """Get the track point by time.

        Parameters
        ----------
        time : float
            Time from start in seconds.

        Notes
        -----
        If time >= finish track time, returns the last track point.

        Returns
        -------
        track_point : TrackPoint2()
            Track point.

        """
        time = time / 1000.0 # seconds
        for i in range(len(self.segments)):
            if (time >= self.segments[i].start_global_time) & (time < self.segments[i].finish_global_time):
                segment_time_local = time - self.segments[i].start_global_time
                return self.__calc_segment_point(segment_time_local, self.segments[i])
            elif time >= self.segments[-1].finish_global_time: # Returns the last track point.
                return TrackPoint2(self.segments[-1].finish.x, self.segments[-1].finish.y,
                                   self.__calc_vec_angle(self.segments[-1].start, self.segments[-1].finish))

    def __calc_segments(self, points):
        """Calculate all segments by points and add them to the self.segments list.

        Parameters
        ----------
        points : list(PolyCurvePoint())
           2D polygonal curve with velocities.

        """
        try:
            if len(points) == 2: # line track case
                local_time = self.__calc_segment_time(LineSegment(points[0].point, points[1].point,
                                                                  points[0].velocity, points[1].velocity))
                self.segments.append(LineSegment(points[0].point, points[1].point,
                                     points[0].velocity, points[1].velocity,
                                     self.__calc_vec_angle(points[0].point, points[1].point),
                                     0.0,
                                     local_time,
                                     (points[1].velocity - points[0].velocity) / local_time))
                return

            # First segment
            self.__append_segments(points[0], points[1], points[2], 0.0)

            for i in range(1, len(points) - 2):
                self.__append_segments(PolyCurvePoint(self.segments[-1].finish,
                                                      points[i].velocity),
                                       points[i + 1],
                                       points[i + 2],
                                       self.segments[-1].finish_global_time)
            # Last segment
            local_time = self.__calc_segment_time(LineSegment(self.segments[-1].finish, points[-1].point,
                                                  points[-2].velocity, points[-1].velocity))
            self.segments.append(LineSegment(self.segments[-1].finish, points[-1].point,
                                             points[-2].velocity, points[-1].velocity,
                                             self.__calc_vec_angle(self.segments[-1].finish, points[-1].point),
                                             self.segments[-1].finish_global_time,
                                             self.segments[-1].finish_global_time + local_time,
                                             (points[-1].velocity - points[-2].velocity) / local_time))
        except (ZeroVecsAngle, CalcVecAngleError):
            print('Error')
            self.segments.clear()
            return

    def __append_segments(self, p0, p1, p2, last_global_time):
        """Append 2 segments to the self.segments list by 3 points.

        Possible segments combinations:
            ((LineSegment(), ArcSegment()) or (LineSegment(), LineSegment()))

        Parameters
        ----------
        p0, p1, p2 : 3 * PolyCurvePoint()
            Three-point element.
        last_global_time : float
            Segment last global time (seconds).

        """
        vec1_angle = self.__calc_vec_angle(p1.point, p0.point)

        vecs_angle = self.__calc_vecs_angle(p0.point, p1.point, p2.point)

        if vecs_angle == 0.0:
            raise ZeroVecsAngle
        elif vecs_angle == pi:
            local_time = self.__calc_segment_time(LineSegment(p0.point, p1.point,
                                                              p0.velocity, p1.velocity))
            self.segments.append(LineSegment(p0.point, p1.point,
                                 p0.velocity, p1.velocity,
                                 self.__calc_vec_angle(p0.point, p1.point),
                                 0.0, local_time,
                                 (p1.velocity - p0.velocity) / local_time))
            return

        vecs_halfangle_cos = cos(vecs_angle / 2)

        zprojection_sign = self.__calc_vecs_product_zsign(p0.point, p1.point, p2.point)

        # Bisection
        bis_angle = vec1_angle + zprojection_sign * vecs_angle / 2.0

        # Centre of the circle (M)
        p1_smooth_radius = self.__calc_smooth_radius(p0, p1, p2)
        p1_to_M_length = sqrt(p1_smooth_radius ** 2 / (1 - vecs_halfangle_cos ** 2))
        M = Point2(p1.point.x + p1_to_M_length * cos(bis_angle),
                   p1.point.y + p1_to_M_length * sin(bis_angle))

        # Arc points
        M_to_p1_angle = self.__calc_vec_angle(M, p1.point)
        arc_first_point_angle = M_to_p1_angle + zprojection_sign * (pi - vecs_angle) / 2
        # Check these cases: !
        if arc_first_point_angle > pi * 2:
            arc_first_point_angle = arc_first_point_angle - pi * 2
        arc_last_point_angle = M_to_p1_angle - zprojection_sign * (pi - vecs_angle) / 2
        if arc_last_point_angle > pi * 2:
            arc_last_point_angle = arc_last_point_angle - pi * 2
        arc_first_point = Point2(M.x + p1_smooth_radius * cos(arc_first_point_angle),
                                 M.y + p1_smooth_radius * sin(arc_first_point_angle))
        arc_last_point = Point2(M.x + p1_smooth_radius * cos(arc_last_point_angle),
                                M.y + p1_smooth_radius * sin(arc_last_point_angle))

        local_time = self.__calc_segment_time(LineSegment(p0.point, arc_first_point, p0.velocity, p1.velocity))
        self.segments.append(LineSegment(p0.point, arc_first_point,
                                         p0.velocity, p1.velocity,
                                         self.__calc_vec_angle(p0.point, arc_first_point),
                                         last_global_time, last_global_time + local_time,
                                         (p1.velocity - p0.velocity) / local_time))

        local_time = self.__calc_segment_time(ArcSegment(arc_first_point, arc_last_point, p1.velocity,
                                                         p1_smooth_radius, arc_first_point_angle, arc_last_point_angle))
        self.segments.append(ArcSegment(arc_first_point, arc_last_point, p1.velocity,
                                        p1_smooth_radius, arc_first_point_angle, arc_last_point_angle,
                                        M, zprojection_sign,
                                        self.segments[-1].finish_global_time,
                                        self.segments[-1].finish_global_time + local_time))

    def __calc_vec_angle(self, p0, p1):
        """Calculate the vector(p0, p1) angle in radians.

        1 degree correction: 179 degrees == 180 degrees.

        Parameters
        ----------
        p0, p1 : 2 * Point2()
            Vector points.

        Returns
        -------
        vec_angle : float
            Vector angle in radians.

        """
        # Local coordinate system. Vec = ((p0x = 0.0; p0y = 0.0); (p1x; p1y))
        p1x = p1.x - p0.x
        p1y = p1.y - p0.y

        length = sqrt(p1x ** 2 + p1y ** 2)
        # Length exceptions?

        angle = asin(fabs(p1y) / length)

        if p1x >= 0 and p1y > 0:    # I
            return angle
        elif p1x < 0 and p1y >= 0:  # II
            return pi - angle
        elif p1x <= 0 and p1y < 0:  # III
            return pi + angle
        elif p1x > 0 and p1y <= 0:  # IV
            return pi * 2 - angle

        else:
            raise CalcVecAngleError()

    def __calc_vecs_angle(self, p0, p1, p2):
        """Calculate the angle between two vectors ((p1, p0) and (p1, p2)) in radians.

        1 degree correction: 179 degrees == 180 degrees.

        Parameters
        ----------
        p0, p1, p2 : 3 * Point2()
            Two vectros: ((p1, p0), (p1, p2)).

        Returns
        -------
        vecs_angle : float
            Angle between two vectors ((p1, p0) and (p1, p2)) in radians.

        """
        # Projections
        vec1_x = p0.x - p1.x
        vec1_y = p0.y - p1.y
        vec2_x = p2.x - p1.x
        vec2_y = p2.y - p1.y
        cos_angle = ((vec1_x * vec2_x + vec1_y * vec2_y) /
                     (sqrt(vec1_x ** 2 + vec1_y ** 2) * sqrt(vec2_x ** 2 + vec2_y ** 2)))
        angle = acos(cos_angle)
        # 1 degree correction
        if (pi - angle) < 0.0175: # pi / 180.0 = 0.017453292519943295 ~= 0.0175
            return pi
        return angle

    def __calc_vecs_product_zsign(self, p0, p1, p2):
        """Calculate the Z-progection sign of vectors product.

        It is necessary for determining the direction of movement.

        Parameters
        ----------
        p0, p1, p2 : 3 * PolyCurvePoint()
            Two vectros: ((p1, p0), (p1, p2)).

        Returns
        -------
        Z-progection : -1.0 | 0.0 | 1.0

        """
        zprojection = (p0.x - p1.x) * (p2.y - p1.y) - (p0.y - p1.y) * (p2.x - p1.x)
        if zprojection == 0.0:
            # Considered in __append_segments():
            #     if vecs_angle == 0.0:
            #         raise ZeroVecsAngle
            return 0.0
        elif zprojection > 0.0:
            return 1.0
        else:
            return -1.0

    def __calc_smooth_radius(self, p0, p1, p2):
        """Speed-smooth radius dependence.

        The maximum radius (max_radius) is related to p0-p2 length
        of the three-point element (p0, p1, p2) by expression:
            max_radius = (1 / 2 * (length_p0_to_p2 / 2)).

        Parameters
        ----------
        p0, p1, p2 : 3 * PolyCurvePoint()
            Three-point element.

        Returns
        -------
        smooth_radius : float

        """
        length_p0_to_p2 = sqrt((p2.point.x - p0.point.x) ** 2 + (p2.point.y - p0.point.y) ** 2)
        max_radius = (1 / 2 * (length_p0_to_p2 / 2))

        if p1.velocity <= max_radius:
            return p1.velocity
        else:
            return max_radius

    def __calc_segment_time(self, segment):
        """Calculate the segment time in seconds.

        Returns
        -------
        segment_time : float

        """
        if type(segment) == LineSegment:
            length = sqrt((segment.finish.x - segment.start.x) ** 2 + (segment.finish.y - segment.start.y) ** 2)
            if segment.start_velocity == segment.finish_velocity: # No acceleration
                return length / segment.start_velocity
            
            # Simplification
            # x1 - x0 = v0 * t + a / 2 * t ** 2 | x1 - x0 = s, t -- full time
            # a / 2 * t ** 2 + v0 * t - S = 0 | A = a / 2, B = v0, C = -s
            # D = B ** 2 - 4 * A * C = v0 ** 2 - 4 * a / 2 * (-s)
            # D = v0 ** 2 + 2 * a * s | a = (v1 ** 2 - v0 ** 2) / (2 * s)
            # D = v0 ** 2 + 2 * (v1 ** 2 - v0 ** 2) / (2 * s) * s
            # D = v0 ** 2 + v1 ** 2 - v0 ** 2 = v1 ** 2
            #
            # t1, t2 = (-B +/- sqrt(D)) / (2 * A) | sqrt(D) = v1, v1 >= 0
            # t1, t2 = (-v0 +/- v1) / a | a = (v1 ** 2 - v0 ** 2) / (2 * s)
            # t1, t2 = (2 * s) * (-v0 +/- v1) / (v1 ** 2 - v0 ** 2)
            # There are 2 possible cases: v1 > v0 (a > 0) and v1 < v0 (a < 0).
            #     [!] Note that v0, v1 >= 0 and the case of zero acceleration (a == 0)
            #         is considered above (segment.start_velocity == segment.finish_velocity)
            # 1. v1 > v0 (a > 0)
            #     t1 = (2 * s) * (-v0 + v1) / (v1 ** 2 - v0 ** 2) > 0
            # 2. v1 < v0 (a < 0)
            #     t2 = (2 * s) * (-v0 - v1) / (v1 ** 2 - v0 ** 2) < 0 Impossible case, time can't be negative
            # t = t1 = (-v0 + v1) / a = (2 * s) * (-v0 + v1) / (v1 ** 2 - v0 ** 2)

            return ((2 * length) * (-segment.start_velocity + segment.finish_velocity) / 
                    (segment.finish_velocity ** 2 - segment.start_velocity ** 2))

        elif type(segment) == ArcSegment:
            arc_angle = fabs(segment.finish_point_angle - segment.start_point_angle)
            if arc_angle > pi:
                arc_angle = pi * 2 - arc_angle
            return segment.smooth_radius * arc_angle / (segment.velocity)

    def __calc_segment_point(self, segment_time_local, segment):
        """Calculate the segment point by local time.

        Returns
        -------
        segment_point : TrackPoint2()
            Segment point.

        """
        if type(segment) == LineSegment:
            # Polar coordinate system
            r = segment.start_velocity * segment_time_local + segment.acceleration * (segment_time_local ** 2) / 2

            return TrackPoint2(r * cos(segment.tangent_angle) + segment.start.x, 
                               r * sin(segment.tangent_angle) + segment.start.y,
                               segment.tangent_angle)

        elif type(segment) == ArcSegment:
            curr_angle = segment.start_point_angle - segment.vec_product_sign * segment.velocity / segment.smooth_radius * segment_time_local

            tangent_angle = curr_angle - segment.vec_product_sign * pi / 2

            return TrackPoint2(segment.smooth_radius * cos(curr_angle) + segment.M.x, 
                               segment.smooth_radius * sin(curr_angle) + segment.M.y,
                               tangent_angle)
