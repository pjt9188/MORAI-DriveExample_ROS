#! usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)

Revised by Juntae park
"""

import sys 
import math
import numpy as np
import bisect

class CubicSpline1D:
    """
    1D Cubic Spline Interpolation class

    점과 점 사이를 연결해주는 2계 미분 연속인 3차 다항식 보간법
    ex) n개의 data가 있는 경우, n-1개의 미분 연속인 Cubic Spline 생성
    
    참고 : https://seong6496.tistory.com/193
    
    Parameters
    ----------
    s : list
        s coordinates(length of spline) for data points. 
        This S coordinates must be sorted in ascending order.
    x : list
        x coordinates for data points
    """
    def __init__(self, s, x):
        h = np.diff(s)      # list of difference between s coordinates (len = ns-1)
        if np.any(h <= sys.float_info.epsilon):
            raise ValueError(
                "min(h) = {}\ns coordinates must be sorted in ascending order or not equal between s coordinates".format(min(h)))
        
        self.a, self.b, self.c, self.d = [], [], [], []
        self.s = s
        self.x = x
        self.n_s = len(s)               # dimension of s
        self.n_eqns = self.n_s - 1      # number of spline eqauions (ns-1개의 방정식)

        # calc coefficient a (ns-1개의 a 계수)
        self.a = [ix for ix in x[:-1]]

        # calc coefficient c
        H = self.__calc_H(h)
        G = self.__calc_G(self.s, self.x)
        self.c = np.linalg.solve(H, G)      # solve Hc = g

        # calc spline coefficient d
        for i in range(self.n_eqns-1):
            d_i = (self.c[i+1] - self.c[i])/(3.0 * h[i])
            self.d.append(d_i)
        self.d.append(-self.c[self.n_eqns-1]/(3.0 * h[self.n_eqns-1]))

        # calc spline coefficient b
        for i in range(self.n_eqns):
            g_i = (x[i+1] - x[i]) / (s[i+1]-s[i])
            b_i = g_i - self.c[i] * h[i] - self.d[i] * h[i]**2
            self.b.append(b_i)

    def __calc_H(self, h):
        """
        Hc = G를 풀기 위한 n_eqns by n_eqns Matrix
        """
        H = np.zeros((self.n_eqns, self.n_eqns))
        H[0, 0] = 1.0
        H[1, 0] = h[0]

        for i in range(1, self.n_eqns-1):
            H[i, i] = 2*(h[i-1] + h[i])
            H[i, i+1] = h[i]
            H[i+1, i] = h[i]
        
        H[self.n_eqns-1, self.n_eqns-1] = 2*(h[self.n_eqns-2] + h[self.n_eqns-1])
        return H
    
    def __calc_G(self, s, x):
        """
        Hc = G를 풀기 위한 n_eq by 1 vector
        g[i] = i번째
        G = 3 * [0 g[1]-g[0] g[2]-g[1] ... g[n_s-2]-g[n_s-3]]
        """
        G = np.zeros(self.n_eqns)

        # calculate G
        for i in range(1, self.n_eqns):
            G[i] = 3.0 * (x[i+1] - x[i])/(s[i+1] - s[i]) \
                    - 3.0 * (x[i] - x[i-1])/(s[i] - s[i-1])
        return G

    def calc_position(self, s):
        """
        Calc `x` position for given `s`.

        Returns
        -------
        x : float
            x position for given s.
        """
        idx = self.__search_index(s)
        ds = s - self.s[idx]
        position = self.a[idx] + self.b[idx] * ds + self.c[idx] * ds ** 2.0 + self.d[idx] * ds ** 3.0
        return position
    
    def calc_first_derivative(self, s):
        """
        Calc first derivative at given s.

        Returns
        -------
        dx : float
            first derivative for given x.
        """

        idx = self.__search_index(s)
        ds = s - self.s[idx]
        dx = self.b[idx]+ 2.0 * self.c[idx] * ds + 3.0 * self.d[idx] * ds ** 2.0
        return dx

    def calc_second_derivative(self, s):
        """
        Calc second derivative at given s.

        Returns
        -------
        ddx : float
            first derivative for given x.
        """

        idx = self.__search_index(s)
        ds = s - self.s[idx]
        ddx = 2.0 * self.c[idx] + 6.0 * self.d[idx] * ds
        return ddx

    
    def __search_index(self, s):
        """
        search data segment index
        
        if `s` is outside the data point's `s` range, raise ValueError
        """
        # s값의 범위 [s0 , sf) 내에서만 계산하도록 Error 처리
        # __search_index(sf)를 실행하면 spline 방정식이 존재하지 않는 마지막 index를 반환하므로
        # s가 최대값일 때는 포함시키지 않는다.
        if s < self.s[0] or s >= self.s[-1]:    
            raise ValueError("CubicSpline1D - s coordintate (value : {}) must be within the s boundary".format(s))
        return bisect.bisect_right(self.s, s)-1

class CubicSpline2D:
    """
    2D Cubic Spline Interpolation class
    
    각각의 x, y data를 1D spline으로 보간후에 계산하는 클래스

    Parameters
    -----------
    x : list
        x coordinates for data points
    y : list
        y coordintaes for data points
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.cubic_spline_x = CubicSpline1D(self.s, x)
        self.cubic_spline_y = CubicSpline1D(self.s, y)
    
    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, raise ValueErrror

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.cubic_spline_x.calc_position(s)
        y = self.cubic_spline_y.calc_position(s)

        return x,y

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, raise Value Error

        Returns
        -------
        yaw : float
            yaw angle [rad] of the tangent vector of spline(reference path) for given s.
        """
        dx = self.cubic_spline_x.calc_first_derivative(s)
        dy = self.cubic_spline_y.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw

    def calc_curvature(self, s):
        """
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for given s.
        """
        dx = self.cubic_spline_x.calc_first_derivative(s)
        ddx = self.cubic_spline_x.calc_second_derivative(s)
        dy = self.cubic_spline_y.calc_first_derivative(s)
        ddy = self.cubic_spline_y.calc_second_derivative(s)
        k = (dx * ddy - dy * ddx) / ((dx ** 2 + dy ** 2) ** (3/2))
        return k

def calc_spline_course(x, y, ds = 0.1):
    """
    calculate spline course

    get reference x, y data points and make course [x, y, yaw, kappa] with the interval 'ds'

    Parameters
    -----------
    x : list
        x coordinates for data points
    y : list
        y coordintaes for data points

    Returns
    -------
    rx      : list
              x coordinates for data points

    ry      : list
              y coordinates for data points

    ryaw    : list
              yaw angle for datapoints [rad], -pi ~ pi

    rk      : list
              curvature for datapoints [1/m]
              CCW - positive / CW - negative

    s       : list
              s coordinates for data points

    cubic_spline_2d : CubicSpline2D Instance
                      CubicSpline2D Instance initialized by x, y data points
    """
    cubic_spline_2d = CubicSpline2D(x, y)
    s = list(np.arange(start = 0, stop = cubic_spline_2d.s[-1], step = ds))     # 맨 끝에 포함 안됨

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        i_x, i_y = cubic_spline_2d.calc_position(i_s)
        rx.append(i_x)
        ry.append(i_y)
        ryaw.append(cubic_spline_2d.calc_yaw(i_s))
        rk.append(cubic_spline_2d.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s, cubic_spline_2d

def main_1d():
    print("CubicSpline1D test")
    import matplotlib.pyplot as plt
    x = np.arange(5)
    y = [1.7, -6, 5, 6.5, 0.0]
    sp = CubicSpline1D(x, y)
    xi = np.linspace(start = 0.0, stop = 4.0, num = 50, endpoint= False)

    plt.plot(x, y, "xb", label="Data points")
    plt.plot(xi, [sp.calc_position(x) for x in xi], "r",
             label="Cubic spline interpolation")
    plt.grid(True)
    plt.legend()
    plt.show()

def main_2d():
    print("CubicSpline2d test")
    import matplotlib.pyplot as plt
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0]

    cubic_spline_2d = CubicSpline2D(x, y)
    s = np.arange(0, cubic_spline_2d.s[-1], step = 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = cubic_spline_2d.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(cubic_spline_2d.calc_yaw(i_s))
        rk.append(cubic_spline_2d.calc_curvature(i_s))

    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()


if __name__ == '__main__':
    main_1d()
    # main_2d()
