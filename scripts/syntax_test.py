class A:
    def __init__(self):
        self.a = 1
        self.b = 2

    def output(self):
        print('a, b is ', self.a, self.b)

a = A()
[a.a, a.b] = [1,2]
a.output()