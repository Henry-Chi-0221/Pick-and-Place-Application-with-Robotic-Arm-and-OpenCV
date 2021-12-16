import sys
class test(object):
    def __init__(self):
        self.t = tuple()
    def value(self):
        self.t = (1,2,3,4,5,6)
if __name__ == "__main__":
    obj = test()
    print(obj.t)
    obj.value()
    