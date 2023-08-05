class Encoder:
    '''
    The class representing the robot's encoder sensor

    Attributes
    -----------
    delta: int
        Accumulation of encoder ticks since the robot starts up
    increment: int
        The number of ticks in the sampling period
    ticks_per_rev: int
        The number of encoder ticks per revolution of the wheel
    direction: int (-1 or 1)
        The direction of rotation of the wheel (1 -> Clockwise -1 -> Counter Clockwise)

    Methods
    -------
    update(current_count):
        Updates the encoder attributes when the function is called
    '''
    def __init__(self, ticks_per_rev):
        self.delta = 0
        self.increment = 0
        self._prev_count = 0
        self.ticks_per_rev = ticks_per_rev
        self.direction = 0
        self._set_encoder_wrap(-32768, 32768)

    def _set_encoder_wrap(self, low, high):
        '''
        Wraps the encoder count values to prevent overflow

        Parameters
        ----------
        low: int
            The low count threshold value
        high: int
            The high count threshold value

        Returns
        -------
        None
        '''
        self._range = high - low + 1
        self._low_wrap = low + self._range * 0.3
        self._high_wrap = high + self._range * 0.7

    def update(self, current_count):
        if self._prev_count > self._high_wrap and current_count < self._low_wrap:
            self.increment = current_count + self._range - self._prev_count
        elif self._prev_count < self._low_wrap and current_count > self._high_wrap:
            self.increment = current_count - self._range - self._prev_count
        else:
            self.increment = current_count - self._prev_count

        self.delta += self.increment
        self._prev_count = current_count
