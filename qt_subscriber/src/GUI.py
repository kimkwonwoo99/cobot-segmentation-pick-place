import sys
import cv2
import threading
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5 import QtCore

form_main = uic.loadUiType('main.ui')[0]
form_choose = uic.loadUiType('choose.ui')[0]
form_poppy = uic.loadUiType('poppy.ui')[0]
form_recommand = uic.loadUiType('recommand.ui')[0]


class WindowClass(QMainWindow, form_main):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.chsButton.clicked.connect(self.chsFunction)
        self.ppButton.clicked.connect(self.ppFunction)
        self.recButton.clicked.connect(self.recFunction)

    def chsFunction(self):
        self.choose = ChooseWindow(self)
        self.choose.show()
        self.hide()

    def ppFunction(self):
        self.poppy = PoppyWindow(self)
        self.poppy.show()
        self.hide()

    def recFunction(self):
        self.recommand = RecommandWindow(self)
        self.recommand.show()
        self.hide()

class ChooseWindow(QMainWindow, form_choose):
    def __init__(self, parent):
        super().__init__()
        self.initUi()
        self.parent = parent
        self.ToMain2.clicked.connect(self.returnToMain)

    def initUi(self):
        self.setupUi(self)

    def returnToMain(self):
        self.close()
        self.parent.show()

class PoppyWindow(QMainWindow, form_poppy):
    def __init__(self, parent):
        super().__init__()
        self.initUi()
        self.parent = parent
        self.ToMain1.clicked.connect(self.returnToMain)

    def initUi(self):
        self.setupUi(self)

    def returnToMain(self):
        self.close()
        self.parent.show()

class RecommandWindow(QMainWindow, form_recommand):
    def __init__(self, parent):
        super().__init__()
        self.initUi()
        self.parent = parent
        self.ToMain3.clicked.connect(self.returnToMain)

    def initUi(self):
        self.setupUi(self)

    def returnToMain(self):
        self.close()
        self.parent.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.exec_()
