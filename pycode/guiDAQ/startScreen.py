# Template for PyQT Code
# V 1.0
# Date Modified - 03/02/2021

import sys

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

# Load UI Layout from other-file which can be independently modified
from ui_startScreen import ui_startScreen

# Definition for a class
class MainWindow(QMainWindow):
    def __init__(self,*args, **kwargs):
        super(MainWindow, self).__init__(*args,**kwargs)    # Boiler-plate code

        # Load UI
        self.ui = ui_startScreen()
        self.ui.setupUi(self)

        self.ui.btn_startDAQ.clicked.connect(whenClicked)

    def whenClicked(self):
        print("Started")        


app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec_()