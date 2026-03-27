roadline1 = [-7.73000000e-05, 9.79225870e-01, 2.57957466e01]
roadline2 = [-6.70800000e-05, 9.73892290e-01, -2.93254943e01]
roadline4 = [-3.17800000e-05, 9.40194630e-01, -7.74732157e01]


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
