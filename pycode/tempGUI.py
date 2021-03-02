# Template for PyQT Code
# V 1.0
# Date Modified - 03/02/2021

import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

# Definition for a class
class MainWindow(QMainWindow):
    def __init__(self,*args, **kwargs):
        super(MainWindow, self).__init__(*args,**kwargs)    # Boiler-plate code



app = QApplication(sys.argv)

window = MainWindow()
window.show()

app.exec_()