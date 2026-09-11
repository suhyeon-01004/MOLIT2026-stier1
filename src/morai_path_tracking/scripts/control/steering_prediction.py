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
        self.history = deque([(now, 0., 0.)])

    def advance(self, now):
        if not math.isfinite(now) or now < self.time:
            raise ValueError('Non-monotonic steering prediction')
        while self.events and self.events[0][0] <= now:
            at, command = self.events.popleft()
            self.angle = self.command + (self.angle - self.command) * math.exp(-(at-self.time)/self.tau)
            self.time, self.command = at, command
            self.history.append((at, self.angle, command))
        self.angle = self.command + (self.angle - self.command) * math.exp(-(now-self.time)/self.tau)
        self.time = now
        while len(self.history)>1 and self.history[1][0]<now-1.:
            self.history.popleft()
        return self.angle

    def at(self, stamp):
        """Exact past model state; does not rewind the live predictor."""
        if not math.isfinite(stamp) or not self.history[0][0]<=stamp<=self.time:
            raise ValueError('Steering history does not cover observation time')
        at, angle, command = next(s for s in reversed(self.history) if s[0]<=stamp)
        return command+(angle-command)*math.exp(-(stamp-at)/self.tau)

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
    assert p.at(.05)==0.
    assert abs(p.at(.2)-.3*(1-math.exp(-.5)))<1e-12
    assert p.time==.3
    p.sent(.3, -.3)
    assert p.advance(.4) > 0.
    assert abs(p.advance(5.) + .3) < 1e-9
    for stamp in (-1.,6.,float('nan')):
        try:p.at(stamp)
        except ValueError:pass
        else:raise AssertionError('Uncovered history accepted')
    print('Steering history step/reversal/DC gain checks passed')
