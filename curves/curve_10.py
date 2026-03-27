roadline1 = [3.92000000e-06, -5.48437780e-01, 2.09502089e03]
roadline2 = [2.51000000e-06, -5.41161320e-01, 2.01932955e03]
roadline4 = [1.10000000e-06, -5.33884860e-01, 1.94363821e03]

lane_margin = 25
lane_margin_relaxed = 32


def getvalue(x, coefficients):
    value = 0
    for coefficient in coefficients:
        value = value * x + coefficient
    return value


def getderivative(x, coefficients):
    derivative = 0
    degree = len(coefficients) - 1
    for index, coefficient in enumerate(coefficients):
        power = degree - index
        if power == 0:
            continue
        derivative += coefficient * power * (x ** (power - 1))
    return [1, derivative]
