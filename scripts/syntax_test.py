#!/usr/bin/python

def apply_async(func, args, callback):
    # Compute the result
    result = func(*args)

    # Invoke the callback with the result
    callback(result)


class ResultHandler:

    def __init__(self):
        self.sequence = 0

    def handler(self, result):
        self.sequence += 1
        print('[{}] Got: {}'.format(self.sequence, result))


def add(x, y):
    return x + y


r = ResultHandler()
for i in range(10):
    apply_async(add, (2, 3), callback=r.handler)