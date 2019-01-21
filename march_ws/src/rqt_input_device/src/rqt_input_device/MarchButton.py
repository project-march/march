from python_qt_binding.QtCore import QSize


class MarchButton:

    default_size = QSize(150, 150)

    def __init__(self, name, text=None, image=None, callback=None, size=None):
        self.name = name
        self.text = text
        if size is not None:
            self.size = size
        else:
            self.size = MarchButton.default_size

        self.image = image
        self.callback = callback

    def execute_callback(self):
        if self.callback is not None:
            self.callback()
