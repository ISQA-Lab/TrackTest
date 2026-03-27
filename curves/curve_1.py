from z3 import And

roadline_left = [-3.285e-05, -0.20423569, 721.71531088]
roadline_middle = [-3.285e-05, -0.20423569, 759.32245373]
roadline_right = [-3.285e-05, -0.20423569, 796.92959659]


def getvalue(y, coefficients=roadline_middle):
    value = 0
    for coefficient in coefficients:
        value = value * y + coefficient
    return value


def getderivative(y, coefficients=roadline_middle):
    a, b, _ = coefficients
    dxdy = 2 * a * y + b
    return [1, dxdy]


def on_left_lane(x, y):
    return And(x > getvalue(y, roadline_left) + 5, x < getvalue(y, roadline_middle) - 5)


def on_right_lane(x, y):
    return And(x > getvalue(y, roadline_middle) + 5, x < getvalue(y, roadline_right) - 5)


def on_road(x, y):
    return And(x > getvalue(y, roadline_left) + 5, x < getvalue(y, roadline_right) - 5)
