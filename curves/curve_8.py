from z3 import And

roadline1 = [-7.52413988e-05, 1.00229554, -1713.19201233]
roadline2 = [-7.52413988e-05, 1.00229554, -1654.97353614]
roadline3 = [-7.52413988e-05, 1.00229554, -1596.75505995]
roadline4 = [5.39921128e-05, -0.11518844, 865.85852294]

lane_margin = 12
lane_margin_relaxed = 15


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


def on_middle_lane(x, y):
    return And(y > getvalue(x, roadline2) + lane_margin, y < getvalue(x, roadline3) - lane_margin)


def on_lower_lane(x, y):
    return And(y > getvalue(x, roadline3) + lane_margin, y < getvalue(x, roadline4) - lane_margin)


def on_upper_lane_relaxed(x, y):
    return And(y > getvalue(x, roadline1) + lane_margin_relaxed, y < getvalue(x, roadline2) - lane_margin_relaxed)


def on_middle_lane_relaxed(x, y):
    return And(y > getvalue(x, roadline2) + lane_margin_relaxed, y < getvalue(x, roadline3) - lane_margin_relaxed)


def on_lower_lane_relaxed(x, y):
    return And(y > getvalue(x, roadline3) + lane_margin_relaxed, y < getvalue(x, roadline4) - lane_margin_relaxed)


def on_road(x, y):
    return And(y > getvalue(x, roadline1) + lane_margin, y < getvalue(x, roadline4) - lane_margin)
