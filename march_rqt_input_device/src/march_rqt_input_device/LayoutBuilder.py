import os

from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QPushButton
import rospkg

from .MarchButton import MarchButton


class LayoutBuilder:
    """Builds a layout from a 2d array of MarchButtons."""

    def __init__(self, button_layout):
        """Stores the 2d array of MarchButtons."""
        self.button_layout = button_layout

    def build(self):
        """Loops through the 2d array and create a qt_button if the entry is not None."""
        qt_button_layout = QGridLayout()
        for i in range(len(self.button_layout)):
            for j in range(len(self.button_layout[i])):

                march_button = self.button_layout[i][j]
                if march_button is not None:
                    qt_button = self.create_qt_button(march_button)
                    qt_button_layout.addWidget(qt_button, i, j, march_button.row_span, march_button.column_span)
                else:
                    qt_button = self.create_qt_default_button()
                    qt_button_layout.addWidget(qt_button, i, j)

        return qt_button_layout

    def create_qt_button(self, march_button):
        """Creates a qt_button from a MarchButton.

        Sets all the specified callback, text and other properties on the qt_button.
        """
        qt_button = QPushButton(march_button.text)
        if march_button.callback is not None:
            qt_button.clicked.connect(march_button.callback)
        qt_button.setStyleSheet(
            self.create_button_css(self.get_image_path(march_button.image)))
        if march_button.text != '':
            qt_button.setText(march_button.text)
        qt_button.setMinimumSize(march_button.size)

        return qt_button

    def create_qt_default_button(self):
        """Creates an invisible qt_default button to act as placeholder in the grid."""
        qt_button = QPushButton()
        qt_button.setStyleSheet(self.get_empty_css())
        qt_button.setFixedSize(MarchButton.default_size)
        qt_button.setVisible(False)
        return qt_button

    @staticmethod
    def get_empty_css():
        """CSS of a hidden button."""
        return """
        display: hidden;
        """

    @staticmethod
    def create_button_css(img_path):
        """CSS of a button with a background-image."""
        css_base = """
        background: url(<img_path>) no-repeat center center;
        background-color:#1F1E24;
        color: #FFFFFF;
        """
        return css_base.replace('<img_path>', img_path)

    @staticmethod
    def get_image_path(img_name):
        """Create an absolute image path to an image."""
        return os.path.join(
            rospkg.RosPack().get_path('march_rqt_input_device'),
            'resource',
            'img{0}'.format(img_name))
