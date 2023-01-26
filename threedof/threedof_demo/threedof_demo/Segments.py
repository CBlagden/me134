#!/usr/bin/env python3
#
#   Segments.py
#
#   TO IMPORT, ADD TO YOUR CODE:
#   from Segments import Hold, Stay, GotoCubic, SplineCubic
#
#   This library provides functionality for trajectory segments.  The
#   segment types (classes) are:
#
#      seg = SplineCubic(p0, v0, pf, vf, T, space='Joint')
#      seg = GotoCubic(  p0,     pf,     T, space='Joint')
#      seg = Hold(       p,              T, space='Joint')
#      seg = Stay(       p,                 space='Joint')
#
#   Each object then provides a
#
#      (pos, vel) = seg.evaluate(trelative)
#      status     = seg.completed(trelative)
#      Ttotal     = seg.duration()
#      space      = seg.space()
#
#   That is, each segment can compute the position and velocity for
#   the specified time.  It further reports whether is has completed and
#   what its durection is.  Note it can also report which "space" it
#   wants, but in the end it is the user's responsibility to
#   interpret/use the pos/vel information accordingly.
#
#   THE SEGMENTS ALSO ASSUME A T=0 START TIME.  THAT IS, WHEN CALLING
#   EVALUATE(), PLEASE FIRST SUBTRACT THE START TIME OF THE SEGMENT!
#
#   The p0,pf,v0,vf,a0,af may be NumPy arrays.
#
import math


#
#  Cubic Spline Segment Objects
#
#  These compute/store the 4 spline parameters, along with the
#  duration and given space.  Note the space is purely for information
#  and doesn't change the computation.
#
class SplineCubic:
    # Initialize.
    def __init__(self, p0, v0, pf, vf, T, space="Joint"):
        # Precompute the spline parameters.
        self.T = T
        self.a = p0
        self.b = v0
        self.c = 3 * (pf - p0) / T**2 - vf / T - 2 * v0 / T
        self.d = -2 * (pf - p0) / T**3 + vf / T**2 + v0 / T**2
        # Save the space
        self.usespace = space

    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return self.T

    # Report whether the given (relative) time is past the duration.
    def completed(self, t):
        return t >= self.T

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t + self.c * t**2 + self.d * t**3
        v = self.b + 2 * self.c * t + 3 * self.d * t**2
        return (p, v)


class GotoCubic(SplineCubic):
    # Use zero initial/final velocities (of same size as positions).
    def __init__(self, p0, pf, T, space="Joint"):
        SplineCubic.__init__(self, p0, 0 * p0, pf, 0 * pf, T, space)


#
#  Hold (temporarily) and Stay (permanently) Segment Objects
#
#  Note we re-use the cubic spline to keep things simple.
#
class Hold(GotoCubic):
    # Use the same initial and final positions.
    def __init__(self, p, T, space="Joint"):
        GotoCubic.__init__(self, p, p, T, space)


class Stay(Hold):
    # Use an infinite time (stay forever).
    def __init__(self, p, space="Joint"):
        Hold.__init__(self, p, math.inf, space)


# general segment object (interface class)
class Segment:
    def __init__(self, T, space='Joint'):
        self.T = T
        self.space = space
        pass

    def evaluate(self, t):
        pass

    def get_p0(self):
        (p0, v0) = self.evaluate(0.0)
        return p0

    def get_v0(self):
        (p0, v0) = self.evaluate(0.0)
        return v0

    def get_pf(self):
        (pf, vf) = self.evaluate(self.T)
        return pf

    def get_vf(self):
        (pf, vf) = self.evaluate(self.T)
        return vf

    def get_space(self):
        return self.space

    def get_duration(self):
        return self.T

class QuinticSpline(Segment):
    # Initialize.
    def __init__(self, p0, v0, a0, pf, vf, af, T, space='joint'):
        Segment.__init__(self, T, space)
        # Precompute the six spline parameters.
        self.a = p0
        self.b = v0
        self.c = a0
        self.d = -10 * p0 / T ** 3 - 6 * v0 / T ** 2 - 3 * a0 / T + 10 * pf / T ** 3 - 4 * vf / T ** 2 + 0.5 * af / T
        self.e = 15 * p0 / T ** 4 + 8 * v0 / T ** 3 + 3 * a0 / T ** 2 - 15 * pf / T ** 4 + 7 * vf / T ** 3 - 1 * af / T ** 2
        self.f = -6 * p0 / T ** 5 - 3 * v0 / T ** 4 - 1 * a0 / T ** 3 + 6 * pf / T ** 5 - 3 * vf / T ** 4 + 0.5 * af / T ** 3

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t + self.c * t ** 2 + self.d * t ** 3 + self.e * t ** 4 + self.f * t ** 5
        v = self.b + 2 * self.c * t + 3 * self.d * t ** 2 + 4 * self.e * t ** 3 + 5 * self.f * t ** 4
        return (p, v)

    # Report whether the given (relative) time is past the duration.
    def completed(self, t):
        return t >= self.T


class Goto5(QuinticSpline):
    # Use zero initial/final velocities/accelerations (same size as positions).
    def __init__(self, p0, pf, T, space='koint'):
        QuinticSpline.__init__(self, p0, 0 * p0, 0 * p0, pf, 0 * pf, 0 * pf, T, space)