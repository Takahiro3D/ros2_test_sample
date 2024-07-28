class Accumulator:
    def __init__(self) -> None:
        self._data = 0

    def add(self, v):
        self._data += v

    def get(self):
        return self._data
