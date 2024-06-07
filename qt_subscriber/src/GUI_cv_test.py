import sys
import cv2
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtGui, QtCore, QtWidgets

# Load UI files
form_main = uic.loadUiType('main.ui')[0]
form_choose = uic.loadUiType('choose.ui')[0]
form_poppy = uic.loadUiType('poppy.ui')[0]
form_recommand = uic.loadUiType('recommand.ui')[0]

class CameraThread(QtCore.QThread):
    changePixmap = QtCore.pyqtSignal(QtGui.QPixmap)

    def __init__(self):
        super().__init__()
        self.running = False

    def run(self):
        self.cap = cv2.VideoCapture(0)  # Use the default camera
        if not self.cap.isOpened():
            QtWidgets.QMessageBox.critical(None, "Camera Error", "Cannot open camera.")
            return

        while self.running:
            ret, img = self.cap.read()
            if ret:
                img = cv2.resize(img, (320, 240))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, c = img.shape
                qImg = QtGui.QImage(img.data, w, h, w * c, QtGui.QImage.Format_RGB888)
                pixmap = QtGui.QPixmap.fromImage(qImg)
                self.changePixmap.emit(pixmap)
            else:
                QtWidgets.QMessageBox.critical(None, "Frame Error", "Cannot read frame.")
                break
        self.cap.release()

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

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
        self.setupUi(self)
        self.parent = parent
        self.initUi()
        self.cameraThread = CameraThread()
        self.cameraThread.changePixmap.connect(self.setImage)
        self.ToMain1.clicked.connect(self.returnToMain)

    def initUi(self):
        self.label = self.findChild(QLabel, "label")  # Ensure you have a QLabel named "label" in your UI
        self.btn_start = self.findChild(QPushButton, "btn_start")  # Ensure you have a QPushButton named "btn_start"
        self.btn_stop = self.findChild(QPushButton, "btn_stop")  # Ensure you have a QPushButton named "btn_stop"
        
        self.btn_start.clicked.connect(self.startCamera)
        self.btn_stop.clicked.connect(self.stopCamera)

    @QtCore.pyqtSlot(QtGui.QPixmap)
    def setImage(self, pixmap):
        self.label.setPixmap(pixmap)

    def startCamera(self):
        if not self.cameraThread.isRunning():
            self.cameraThread.running = True
            self.cameraThread.start()

    def stopCamera(self):
        self.cameraThread.stop()
        self.label.clear()

    def closeEvent(self, event):
        self.stopCamera()
        event.accept()

    def returnToMain(self):
        self.close()
        self.parent.show()

class RecommandWindow(QMainWindow, form_recommand):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)
        self.parent = parent
        self.ToMain3.clicked.connect(self.returnToMain)

    def returnToMain(self):
        self.close()
        self.parent.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    sys.exit(app.exec_())
