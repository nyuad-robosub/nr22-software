import math
def is_approx_equal(x, y, margin=0.1):
    if abs(x - y) / math.sqrt(x ** 2 + y ** 2) < margin:# (x>(y*(1-margin)) and x<(y*(1+margin))):
        return True
    else:
        return False
