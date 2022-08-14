from motor_ctrl import *
from motor_group import *
from single_motor  import *
import sys
from PyQt5.QtCore import QThread, Qt
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication
from ui_test1 import Ui_Form   

class AppWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)

       #綁上與點擊事件對應的function，所有東西都在ui底下！！
        self.ui.pushButton.clicked.connect(self.pushButton)
        self.show()

    def pushButton(self):
        self.ui.label.setText("Hello")

app = QApplication(sys.argv)
w = AppWindow()
w.show()
sys.exit(app.exec_())
