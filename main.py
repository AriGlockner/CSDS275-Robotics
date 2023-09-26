import random
from queue import PriorityQueue

pq = PriorityQueue()

length = 10

[pq.put(random.randint(1, 100)) for i in range(length)]
print(pq)
[print(pq.get()) for j in range(length)]
print(pq)
