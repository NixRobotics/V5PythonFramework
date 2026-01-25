class MedianFilter3:
    def __init__(self, initial_value = 0.0):
        self.buffer = [initial_value] * 3
        self.a = self.b = self.c = initial_value
        self.latest_value = initial_value

    def _update(self, new_value):
        self.buffer.pop(0)
        self.buffer.append(new_value)
        sorted = self.buffer.copy()
        sorted.sort()
        return sorted[1]

    def _update_fast(self, new_value):
        self.a = self.b
        self.b = self.c
        self.c = new_value

        if self.a > self.b:
            if self.b > self.c: return self.b
            elif self.a > self.c: return self.c
            else: return self.a
        else:
            if self.a > self.c: return self.a
            elif self.b > self.c: return self.c
            else: return self.b

    def update(self, new_value):
        self.latest_value = self._update_fast(new_value)
        return self.latest_value
    
    @property
    def median(self):
        return self.latest_value

