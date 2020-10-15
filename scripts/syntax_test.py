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

robot = MoveGroupPythonInteface()
goal = (-0.4, -1.0, 1.8, 0, 1.2, 3.1)
robot.group.set_joint_value_target(goal)
plan = robot.group.plan()
print(type(plan))