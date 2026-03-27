from z3 import *

def get_value_of_z3(m,x):
    try:
        return float(m[x].as_fraction())
    except Exception as e:
        str = m[x].as_decimal(10)
        return float(str) if str[-1] != '?' else float(str[:-1])