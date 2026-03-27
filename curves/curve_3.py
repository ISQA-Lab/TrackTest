from z3 import And

roadline1 = [-4.41767160e-05, -0.31203531, 567.08252299]
roadline2 = [-4.41767160e-05, -0.31203531, 603.08252299]
roadline3 = [-4.41767160e-05, -0.31203531, 603.08252299]
roadline4 = [-4.41767160e-05, -0.31203531, 639.08252299]

lane_margin = 5
lane_margin_relaxed = 3


def getvalue(x, coefficients=roadline1):
    value = 0
    for coefficient in coefficients:
        value = value * x + coefficient
    return value


def getderivative(x, coefficients=roadline1):
    derivative = 0
    degree = len(coefficients) - 1
    for index, coefficient in enumerate(coefficients):
        power = degree - index
        if power == 0:
            continue
        derivative += coefficient * power * (x ** (power - 1))
    return [1, derivative]


def on_upper_lane(x, y):
    return And(y > getvalue(x, roadline1) + lane_margin, y < getvalue(x, roadline2) - lane_margin)


def on_lower_lane(x, y):
    return And(y > getvalue(x, roadline3) + lane_margin, y < getvalue(x, roadline4) - lane_margin)


def on_upper_lane_relaxed(x, y):
    return And(y > getvalue(x, roadline1) + lane_margin_relaxed, y < getvalue(x, roadline2) - lane_margin_relaxed)


def on_lower_lane_relaxed(x, y):
    return And(y > getvalue(x, roadline3) + lane_margin_relaxed, y < getvalue(x, roadline4) - lane_margin_relaxed)


def on_road(x, y):
    return And(y > getvalue(x, roadline1) + lane_margin, y < getvalue(x, roadline4) - lane_margin)
