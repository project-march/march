class SoftwareCheck:

    def __init__(self, name, description, timeout = 10000):
        self.name = name
        self.description = description
        self.passed = False
        self.done = False
        self.timeout = timeout

    def reset(self):
        self.passed = False
        self.done = False

    def is_passed(self):
        if self.done:
            return self.passed
        return False

    def perform(self):
        raise NotImplementedError("Please implement method 'perform()' on the subclass")
