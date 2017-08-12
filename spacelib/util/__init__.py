import math

def cross_product(a, b):
    return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
            )

def dot_product(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def vector_magnitude(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def vector_angle(a, b, degrees=True):
    dp = dot_product(a, b)
    amag = vector_magnitude(a)
    if amag == 0:
        raise ValueError("magnitude of {} is 0".format(a))
    bmag = vector_magnitude(b)
    if bmag == 0:
        raise ValueError("magnitude of {} is 0".format(b))
    angle = math.acos(dp / (amag * bmag))
    if degrees:
        return math.degrees(angle)
    return angle
