"""Command-history actuator estimate, never a measured wheel-angle report."""
import math
from collections import deque


class SteeringPrediction:
    def __init__(self, delay, tau, now):
        if not (math.isfinite(delay) and 0 <= delay <= .5 and math.isfinite(tau) and .01 <= tau <= 1.):
            raise ValueError('Invalid steering actuator model')
        self.delay, self.tau, self.time = delay, tau, now
        self.angle = self.command = 0.
        self.events = deque()

    def advance(self, now):
        if not math.isfinite(now) or now < self.time:
            raise ValueError('Non-monotonic steering prediction')
        while self.events and self.events[0][0] <= now:
            at, command = self.events.popleft()
            self.angle = self.command + (self.angle - self.command) * math.exp(-(at-self.time)/self.tau)
            self.time, self.command = at, command
        self.angle = self.command + (self.angle - self.command) * math.exp(-(now-self.time)/self.tau)
        self.time = now
        return self.angle

    def sent(self, now, command):
        if not math.isfinite(command):
            raise ValueError('Nonfinite steering command')
        self.advance(now)
        self.events.append((now + self.delay, command))


if __name__ == '__main__':
    p = SteeringPrediction(.1, .2, 0.)
    p.sent(0., .3)
    assert p.advance(.1) == 0.
    assert abs(p.advance(.3) - .3*(1-math.exp(-1))) < 1e-12
    p.sent(.3, -.3)
    assert p.advance(.4) > 0.
    assert abs(p.advance(5.) + .3) < 1e-9
    print('Steering history step/reversal/DC gain checks passed')
