from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QSize
import sys

class PushButton(QWidget):

    height = 400
    selection = 0
    buttons = []

    def __init__(self):
        super(PushButton,self).__init__()
        self.initUI()

    def initUI(self):

        self.setWindowTitle("PushButton")
        self.setGeometry(400,400,800,600)

        self.B0 = QPushButton(self)
        self.B0.resize(800, self.height)
        self.B0.move(0, 0)

        self.B1, self.B2, self.B3, self.B4 = QPushButton(self), QPushButton(self), QPushButton(self), QPushButton(self)
        self.buttons = [self.B1, self.B2, self.B3, self.B4]

        for i in range(len(self.buttons)):
            self.buttons[i].resize(200, 200)
            self.buttons[i].move(i*200, self.height)

        self.renderButtons()


    def renderButtons(self):
        for i in range(len(self.buttons)):
            if self.selection == i:
                self.buttons[i].setIcon(QIcon("stairs_selected.png"))
            else:
                self.buttons[i].setIcon(QIcon("stairs_unselected.png"))
            self.buttons[i].setIconSize(QSize(200, 200)),


    def incrementSelection(self, channel):
        print("pressed")
        self.selection += 1
        self.selection %= 4
        self.renderButtons()


    def decrementSelection(self, channel):
        self.selection -= 1
        self.selection %= 4
        self.renderButtons()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_1:
            self.decrementSelection(-1)
        elif event.key() == Qt.Key_2:
            self.incrementSelection(-1)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = PushButton()
    # ex.setWindowFlags(Qt.FramelessWindowHint)
    # ex.showFullScreen()
    ex.show()

    sys.exit(app.exec_())