from z3 import And

roadline1 = [1.05520032e-04, -0.17843184, 162.18427157]
roadline2 = [1.05520032e-04, -0.17843184, 126.33732441]
roadline3 = [1.05520032e-04, -0.17843184, 126.33732441]
roadline4 = [1.05520032e-04, -0.17843184, 90.49037725]

lane_margin = 8
lane_margin_relaxed = 5


def getvalue(y, coefficients=roadline1):
    value = 0
    for coefficient in coefficients:
        value = value * y + coefficient
    return value


def getderivative(y, coefficients=roadline1):
    derivative = 0
    degree = len(coefficients) - 1
    for index, coefficient in enumerate(coefficients):
        power = degree - index
        if power == 0:
            continue
        derivative += coefficient * power * (y ** (power - 1))
    return [1, derivative]


def on_left_lane(x, y):
    return And(x > getvalue(y, roadline2) + lane_margin, x < getvalue(y, roadline1) - lane_margin)


def on_right_lane(x, y):
    return And(x > getvalue(y, roadline4) + lane_margin, x < getvalue(y, roadline3) - lane_margin)


def on_left_lane_relaxed(x, y):
    return And(x > getvalue(y, roadline2) + lane_margin_relaxed, x < getvalue(y, roadline1) - lane_margin_relaxed)


def on_right_lane_relaxed(x, y):
    return And(x > getvalue(y, roadline4) + lane_margin_relaxed, x < getvalue(y, roadline3) - lane_margin_relaxed)


def on_road(x, y):
    return And(x > getvalue(y, roadline4) + lane_margin, x < getvalue(y, roadline1) - lane_margin)
