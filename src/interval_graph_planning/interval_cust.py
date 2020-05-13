#!/ust/bin/emv python3


from math import sqrt as mathsqrt

def sqrt(n):
    try:
        return Interval(mathsqrt(n.low), mathsqrt(n.high))
    except:
        return mathsqrt(n)


class Interval:
    """Implements interval arithmetic."""
    def __init__(self, a, b=None):
        # If there's just one parameter, set the second bound to be the same value
        if b == None:
            b = a

        # Make sure we know the low and high bounds, and store them.
        self.low = min(a, b)
        self.high = max(a, b)

    def __repr__(self):
        return '[{0}; {1}]'.format(self.low, self.high)

    def __str__(self):
        return self.__repr__()

    def __contains__(self, n):
        return self.low <= n <= self.high

    def __add__(self, other):
        try:
            return Interval(self.low + other.low, self.high + other.high)
        except:
            return Interval(self.low + other, self.high + other)

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        try:
            return Interval(self.low - other.high, self.high - other.low)
        except:
            return Interval(self.low - other, self.high - other)

    def __rsub__(self, other):
        return -self + other

    def __mul__(self, other):
        try:
            return Interval(min(self.low * other.low, self.low * other.high,
                                self.high * other.low, self.high * other.high),
                                max(self.low * other.low, self.low * other.high,
                                self.high * other.low, self.high * other.high))
        except:
            return Interval(self.low * other, self.high * other)

    def __rmul__(self, other):
        return self * other

    # This does not handle intervals containing zero
    def __rtruediv__(self, other):
        try:
            return other.__mul__(Interval(1 / self.high, 1 / self.low))
        except:
            return 1
            return Interval(other / self.low, other / self.high)

    def __neg__(self):
        return Interval(-self.low, -self.high)

    def __lt__(self, other):
        try:
            return self.high < other.low
        except:
            o = Interval(other)
            return self.high < o.low

    def __gt__(self, other):
        try:
            return self.low > other.high
        except:
            o = Interval(other)
            return self.low > o.high


if __name__ == '__main__':
    i = Interval(1, 2)

