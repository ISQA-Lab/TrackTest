roadline1 = [-5.7880000e-05, -2.8303410e-02, 1.6790514e02]
roadline2 = [-5.7880000e-05, -2.8303410e-02, 2.0290514e02]
roadline4 = [-5.7880000e-05, -2.8303410e-02, 2.3790514e02]


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
