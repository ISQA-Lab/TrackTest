roadline1 = [3.55570000e-04, -1.27122763e00, 1.00110595e03]
roadline2 = [3.35820000e-04, -1.23095189e00, 9.38857767e02]
roadline3 = [4.16520000e-04, -1.32936746e00, 9.21344507e02]
roadline4 = [4.97210000e-04, -1.42778303e00, 9.03831247e02]


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
