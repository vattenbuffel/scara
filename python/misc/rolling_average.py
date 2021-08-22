from collections import deque
class RollingAverage:
    def __init__(self, n=10) -> None:
       self.vals = deque(maxlen=n) 
       self.n = 10

    def update(self, data):
        self.vals.append(data)

    def get_avg(self):
        return sum(self.vals)/self.n