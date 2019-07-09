class SoftwareCheck:

    def __init__(self, name, description, timeout=10000, manual_confirmation=False):
        self.name = name
        self.description = description
        self.manual_confirmation = manual_confirmation
        self.timeout = timeout
        self.passed = False
        self.done = False

    def reset(self):
        self.passed = False
        self.done = False

    def is_passed(self):
        if self.done:
            return self.passed
        return False

    def perform(self):
        raise NotImplementedError("Please implement method 'perform()' on the subclass")
