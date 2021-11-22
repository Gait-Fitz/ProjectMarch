#!/usr/bin/python3

from PyQt5.QtCore import QDateTime, Qt, QTimer, QSize, QPoint
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget, QToolTip)
from PyQt5.QtGui import QFont

class WidgetGallery(QDialog):
    def __init__(self, parent=None):
        super(WidgetGallery, self).__init__(parent)

        QApplication.setStyle(QStyleFactory.create('Fusion'))

        launchTypeBox = QComboBox()
        launchTypeBox.addItems(['Simulation', 'Airgait', 'Groundgait'])

        styleLabel = QLabel("Launch type:")
        styleLabel.setBuddy(launchTypeBox)

        # disableWidgetsCheckBox = QCheckBox("&Disable widgets")

        # self.createTopRightGroupBox()
        # self.createBottomLeftTabWidget()
        # self.createBottomRightGroupBox()
        # self.createProgressBar()

        launchTypeBox.activated[str].connect(self.changeLaunchType)
        # self.useStylePaletteCheckBox.toggled.connect(self.changePalette)
        # disableWidgetsCheckBox.toggled.connect(self.topLeftGroupBox.setDisabled)
        # disableWidgetsCheckBox.toggled.connect(self.topRightGroupBox.setDisabled)
        # disableWidgetsCheckBox.toggled.connect(self.bottomLeftTabWidget.setDisabled)
        # disableWidgetsCheckBox.toggled.connect(self.bottomRightGroupBox.setDisabled)

        topLayout = QHBoxLayout()
        topLayout.addWidget(styleLabel)
        topLayout.addWidget(launchTypeBox)
        # topLayout.addStretch(1)
        # topLayout.addWidget(self.useStylePaletteCheckBox)
        # topLayout.addWidget(disableWidgetsCheckBox)

        self.mainLayout = QGridLayout()
        self.mainLayout.addLayout(topLayout, 0, 0, 1, 2)
        self.createSimulationLayout()
        # self.mainLayout.addWidget(self.topLeftGroupBox, 1, 0)
        # mainLayout.addWidget(self.topRightGroupBox, 1, 1)
        # mainLayout.addWidget(self.bottomLeftTabWidget, 2, 0)
        # mainLayout.addWidget(self.bottomRightGroupBox, 2, 1)
        self.mainLayout.setRowStretch(1, 1)
        self.mainLayout.setRowStretch(2, 1)
        self.mainLayout.setColumnStretch(0, 1)
        self.mainLayout.setColumnStretch(1, 1)
        self.setLayout(self.mainLayout)

        # self.setWindowTitle("Styles")
        # self.changeStyle('Windows')

    def changeLaunchType(self, launchType):
        if launchType == 'Simulation':
            self.createSimulationLayout()
        else:
            self.createSimulationLayout()

    def createSimulationLayout(self):
        generalArguments = QGroupBox("General arguments")

        layout = QGridLayout()
        generalArguments.setLayout(layout)    

        # trueFalse = QComboBox()
        # trueFalse.addItems(['true', 'false']) 
        # wah = QLabel("Launch type:")
        # wah.setBuddy(trueFalse)
        # layout.addWidget(wah, 4, 0)
        # layout.addWidget(trueFalse, 4, 1)

        A1 = QLabel("robot")
        I1 = QLineEdit()
        I1.setText('march6')
        I1.setAlignment(Qt.AlignRight)
        layout.addWidget(A1, 0, 1)
        layout.addWidget(I1, 0, 2)

        A2 = QLabel("robot_description")
        I2 = QLineEdit()
        I2.setText('march6')
        I2.setAlignment(Qt.AlignRight)
        layout.addWidget(A2, 1, 1)
        layout.addWidget(I2, 1, 2)

        O3 = QComboBox()
        O3.addItems(['effort_control']) 
        A3 = QLabel("controller_type")
        layout.addWidget(A3, 2, 1)
        layout.addWidget(O3, 2, 2)

        A4 = QLabel("controller_name")
        I4 = QLineEdit()
        I4.setText('march6')
        I4.setAlignment(Qt.AlignRight)
        layout.addWidget(A4, 3, 1)
        layout.addWidget(I4, 3, 2)

        O4 = QComboBox()
        O4.addItems(['false', 'true']) 
        A4 = QLabel("gain_scheduling")
        layout.addWidget(A4, 4, 1)
        layout.addWidget(O4, 4, 2)

        O5 = QComboBox()
        O5.addItems(['false', 'true']) 
        A5 = QLabel("gain_scheduling")
        layout.addWidget(A5, 5, 1)
        layout.addWidget(O5, 5, 2)
        

        self.mainLayout.addWidget(generalArguments, 1, 0)

    def createTopRightGroupBox(self):
        self.topRightGroupBox = QGroupBox("Group 2")

        defaultPushButton = QPushButton("Default Push Button")
        defaultPushButton.setDefault(True)

        togglePushButton = QPushButton("Toggle Push Button")
        togglePushButton.setCheckable(True)
        togglePushButton.setChecked(True)

        flatPushButton = QPushButton("Flat Push Button")
        flatPushButton.setFlat(True)

        layout = QVBoxLayout()
        layout.addWidget(defaultPushButton)
        layout.addWidget(togglePushButton)
        layout.addWidget(flatPushButton)
        layout.addStretch(1)
        self.topRightGroupBox.setLayout(layout)

    def createBottomLeftTabWidget(self):
        self.bottomLeftTabWidget = QTabWidget()
        self.bottomLeftTabWidget.setSizePolicy(QSizePolicy.Preferred,
                QSizePolicy.Ignored)

        tab1 = QWidget()
        tableWidget = QTableWidget(10, 10)

        tab1hbox = QHBoxLayout()
        tab1hbox.setContentsMargins(5, 5, 5, 5)
        tab1hbox.addWidget(tableWidget)
        tab1.setLayout(tab1hbox)

        tab2 = QWidget()
        textEdit = QTextEdit()

        textEdit.setPlainText("Twinkle, twinkle, little star,\n"
                              "How I wonder what you are.\n" 
                              "Up above the world so high,\n"
                              "Like a diamond in the sky.\n"
                              "Twinkle, twinkle, little star,\n" 
                              "How I wonder what you are!\n")

        tab2hbox = QHBoxLayout()
        tab2hbox.setContentsMargins(5, 5, 5, 5)
        tab2hbox.addWidget(textEdit)
        tab2.setLayout(tab2hbox)

        self.bottomLeftTabWidget.addTab(tab1, "&Table")
        self.bottomLeftTabWidget.addTab(tab2, "Text &Edit")

    def createBottomRightGroupBox(self):
        self.bottomRightGroupBox = QGroupBox("Group 3")
        self.bottomRightGroupBox.setCheckable(True)
        self.bottomRightGroupBox.setChecked(True)

        lineEdit = QLineEdit('s3cRe7')
        lineEdit.setEchoMode(QLineEdit.Password)

        spinBox = QSpinBox(self.bottomRightGroupBox)
        spinBox.setValue(50)

        dateTimeEdit = QDateTimeEdit(self.bottomRightGroupBox)
        dateTimeEdit.setDateTime(QDateTime.currentDateTime())

        slider = QSlider(Qt.Horizontal, self.bottomRightGroupBox)
        slider.setValue(40)

        scrollBar = QScrollBar(Qt.Horizontal, self.bottomRightGroupBox)
        scrollBar.setValue(60)

        dial = QDial(self.bottomRightGroupBox)
        dial.setValue(30)
        dial.setNotchesVisible(True)

        layout = QGridLayout()
        layout.addWidget(lineEdit, 0, 0, 1, 2)
        layout.addWidget(spinBox, 1, 0, 1, 2)
        layout.addWidget(dateTimeEdit, 2, 0, 1, 2)
        layout.addWidget(slider, 3, 0)
        layout.addWidget(scrollBar, 4, 0)
        layout.addWidget(dial, 3, 1, 2, 1)
        layout.setRowStretch(5, 1)
        self.bottomRightGroupBox.setLayout(layout)

    def createProgressBar(self):
        self.progressBar = QProgressBar()
        self.progressBar.setRange(0, 10000)
        self.progressBar.setValue(0)

        timer = QTimer(self)
        timer.timeout.connect(self.advanceProgressBar)
        timer.start(1000)


if __name__ == '__main__':

    import sys

    app = QApplication(sys.argv)
    gallery = WidgetGallery()
    gallery.show()
    sys.exit(app.exec_()) 