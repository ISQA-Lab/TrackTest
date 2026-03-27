from z3 import And

roadline1 = [-0.00027316, 1.77483048, -415.03983275]
roadline2 = [-0.00027316, 1.77483048, -349.98962048]
roadline3 = [-0.00027316, 1.77483048, -349.98962048]
roadline4 = [-0.00027316, 1.77483048, -284.93940821]

lane_margin = 10
lane_margin_relaxed = 8


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
