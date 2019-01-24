from python_qt_binding.QtCore import QSize


class MarchButton:
    """Button that can have a custom callback and other configurable properties."""

    """Default size of a button, is used if no other size is given."""
    default_size = QSize(150, 150)

    def __init__(self, name, text="", image="", callback=None, size=None):
        self.name = name
        self.text = text
        self.image = image
        self.callback = callback

        if size is not None:
            self.size = size
        else:
            self.size = MarchButton.default_size
