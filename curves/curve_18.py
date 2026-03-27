roadline1 = [1.03834320e-04, 0.671945, 70.299]
roadline2 = [1.01519958e-04, 0.667720, 34.863]
roadline3 = [9.92055955e-05, 0.663494, -0.573]
roadline4 = [9.68912334e-05, 0.659269, -36.009]


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
