
import sys

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from colornotes import colornotes

class ui_startScreen(QMainWindow):
    def setupUi(self, MainWindow):
        MainWindow.setWindowTitle("Name")

        # Test Boxes as placeholders for real labels
        testb0 = colornotes("Gray")
        testb1 = colornotes("Red")
        testb2 = colornotes("Blue")
        testb3 = colornotes("Green")
        testb4 = colornotes("Yellow")
        testb5 = colornotes("Orange")
        testb6 = colornotes("Cyan")

        # Labels
        self.labelDAQ = QLabel()
        self.labelDAQ.setText("DAQ")
        font1 = self.labelDAQ.font()
        font1.setPixelSize(18)
        font1.setBold(True)
        self.labelDAQ.setFont(font1)
        self.labelDAQ.setAlignment(Qt.AlignCenter)

        self.labelMap = QLabel()
        self.labelMap.setText("MAP")
        font2 = self.labelMap.font()
        font2.setPixelSize(18)
        font2.setBold(True)
        self.labelMap.setFont(font2)
        self.labelMap.setAlignment(Qt.AlignCenter)

        # Add a button to talk to roscore
        self.btn_startDAQ = QPushButton()
        self.btn_startDAQ.setText("Start")
        
        self.btn_setpathDAQ = QPushButton()
        self.btn_setpathDAQ.setText("Path")


        # Grid Layout
        layoutMain = QGridLayout()

        layoutMain.addWidget(self.labelDAQ, 0,1)
        layoutMain.addWidget(self.labelMap, 0,2)
        
        layoutMain.addWidget(self.btn_startDAQ, 1,1)
        layoutMain.addWidget(testb0, 1,2)
        layoutMain.addWidget(self.btn_setpathDAQ, 2,1)
        layoutMain.addWidget(testb1, 2,2)

        mainWidget = QWidget()
        mainWidget.setLayout(layoutMain)
        MainWindow.setCentralWidget(mainWidget)