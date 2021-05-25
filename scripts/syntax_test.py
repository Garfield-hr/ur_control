#!/usr/bin/python

from moveit_test import MoveGroupPythonInteface

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


def test_func(velocity=()):
    print("test function is running")
    if velocity:
        print("velocity is", velocity)


a = [i/10.0 for i in range(10)]
for i in a:
    print i