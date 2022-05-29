from PyQt5 import QtCore, QtGui, QtWidgets
import requests, threading, time, json

def sendJson(num):
    # Get Request
    host = 'http://0.0.0.0:8000/btn_call/'
    # host = 'http://192.168.12.253:8000/btn_call/'
    # host = 'http://192.168.12.253:8000/btn_call/'

    r = requests.post(host, json={"call_id": num})
    print('Status code : ', r.status_code)
    print(r.json())

def pWidgets(wdgt, func=None, font=None):
    if not font:
        font = QtGui.QFont("Arial", 48, QtGui.QFont.Bold)

    p = wdgt
    p.setFont(font)

    method = getattr(p, "setAlignment", None)
    if callable(method):
        p.setAlignment(QtCore.Qt.AlignCenter)
        
    method = getattr(p, "setSizePolicy", None)
    if callable(method):
        p.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

    if func:
        p.clicked.connect(func)
    return p

class PageWindow(QtWidgets.QMainWindow):
    gotoSignal = QtCore.pyqtSignal(str)

    def __init__(self):
        name = 'PageWindow'
        print(str(name + " 1"))
        super().__init__()
        print(str(name + " 2"))
        self.w = 640
        self.h = 480
        frm = QtWidgets.QFrame()
        self.setCentralWidget(frm)
        # self.setFixedSize(self.w, self.h)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

    def goto(self, name):
        self.gotoSignal.emit(name)

class MainWindow(PageWindow):
    def __init__(self, f, parent):
        super().__init__()
        self.font = f
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        name = 'MainWindow'
        print(str(name + " 1"))
        super().__init__()
        print(str(name + " 2"))
        self.initUI()
        self.setWindowTitle("MainWindow")
        self.feet_ip = '0.0.0.0:8000'
        self.bThread = True
        self.threadStatus = threading.Thread(target=self.getStatusThreadRun)
        self.threadStatus.start()
        
    def __del__(self):
        print('MainWindow del')
        self.bThread = False
        self.threadStatus.join()

    def getStatusThreadRun(self):
        # Format Headers
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        print(self.headers)

        while self.bThread:
            _host = 'http://' + self.feet_ip + '/robotstate'
            get_status = requests.get(_host, headers=self.headers)
            #print(get_status)
            parsed = json.loads(get_status.content)            
            res = str(parsed['message'])
            #print('res : ' + res)
            self.robotstateLabel.setText(res)
            time.sleep(1.0)

    def initUI(self):
        self.UiComponents()

    def getSize(self):
        print('getSize')
        size = self.size()
        print('{}, {}'.format(size.width(), size.height()))


    def UiComponents(self):
        btn_call = QtWidgets.QPushButton("CALL")
        btn_call.setStyleSheet("background-color: green")

        btn_1 = QtWidgets.QPushButton("GOTO_2")
        btn_1.setStyleSheet("background-color: blue")

        btn_2 = QtWidgets.QPushButton("GOTO_3")
        btn_2.setStyleSheet("background-color: yellow")
        
        self.robotstateLabel = QtWidgets.QLabel("Robot Status : READY")

        hLayout = QtWidgets.QHBoxLayout()
        hLayout.addWidget(pWidgets(btn_1, lambda: sendJson(2)), 1)
        hLayout.addWidget(pWidgets(btn_2, lambda: sendJson(3)), 1)

        vLayout = QtWidgets.QVBoxLayout()
        vLayout.addWidget(pWidgets(btn_call, lambda: sendJson(1)), 1)
        vLayout.addLayout(hLayout, 1)
        vLayout.addWidget(pWidgets(self.robotstateLabel), 1)

        wdg = QtWidgets.QWidget()
        wdg.setLayout(vLayout)
        self.setCentralWidget(wdg)

    def refresh(self):
        print("MainWindow refresh.")

    def make_handleButton(self, button):
        def handleButton():
            print('handleButton')
        return handleButton

class Window(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__()
        self.font = QtGui.QFont("Arial", 7, QtGui.QFont.Bold)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        self.stacked_widget = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        self.m_pages = {}

        # self.register(FirstWindow(self.printer, self.font), "first")
        self.register(MainWindow(self.font, self), "main")
        # self.register(SearchWindow(self.printer, self.font), "search")
        # self.register(LaserWindow(self.printer, self.font), "laser")
        # self.register(BaseWindow(self.printer, self.font), "base")
        # #self.register(ParamWindow(self.printer, self.font), "param")
        # self.register(ProcessWindow(self.printer, self.font), "process")
        # self.register(ManualWindow(self.printer, self.font), "man")

        self.goto("main")

    def resizeEvent(self, event):
        # print("resize")
        QtWidgets.QMainWindow.resizeEvent(self, event)
        # self.getSize(event)

    def getSize(self, event):
        print('getSize')
        size = event.size()
        print('{}, {}'.format(size.width(), size.height()))

    def register(self, widget, name):
        self.m_pages[name] = widget
        self.stacked_widget.addWidget(widget)
        if isinstance(widget, PageWindow):
            widget.gotoSignal.connect(self.goto)

    @QtCore.pyqtSlot(str)
    def goto(self, name):
        if name in self.m_pages:
            widget = self.m_pages[name]
            self.stacked_widget.setCurrentWidget(widget)
            self.setWindowTitle(widget.windowTitle())
            widget.refresh()

if __name__ == "__main__":
    import sys
    
    app = QtWidgets.QApplication(sys.argv)
    
    w = Window(app)
    w.show()

    sys.exit(app.exec_())
