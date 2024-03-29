# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'c:\Users\sng\GitHub\role-playing-robots\src\qtwizard\rpwizard.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_mainWindow(object):
    def setupUi(self, mainWindow):
        mainWindow.setObjectName("mainWindow")
        mainWindow.setWindowModality(QtCore.Qt.WindowModal)
        mainWindow.resize(1000, 710)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed
        )
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(mainWindow.sizePolicy().hasHeightForWidth())
        mainWindow.setSizePolicy(sizePolicy)
        mainWindow.setMinimumSize(QtCore.QSize(1000, 710))
        mainWindow.setMaximumSize(QtCore.QSize(1000, 710))
        self.centralwidget = QtWidgets.QWidget(mainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.scenarios = QtWidgets.QTabWidget(self.centralwidget)
        self.scenarios.setGeometry(QtCore.QRect(20, 120, 801, 571))
        self.scenarios.setObjectName("scenarios")
        self.intro = QtWidgets.QWidget()
        self.intro.setObjectName("intro")
        self.bPhase4_5 = QtWidgets.QGroupBox(self.intro)
        self.bPhase4_5.setGeometry(QtCore.QRect(30, 140, 441, 61))
        self.bPhase4_5.setTitle("")
        self.bPhase4_5.setObjectName("bPhase4_5")
        self.cond1 = QtWidgets.QRadioButton(self.bPhase4_5)
        self.cond1.setGeometry(QtCore.QRect(10, 20, 212, 40))
        self.cond1.setChecked(True)
        self.cond1.setObjectName("cond1")
        self.cond2 = QtWidgets.QRadioButton(self.bPhase4_5)
        self.cond2.setGeometry(QtCore.QRect(110, 20, 161, 40))
        self.cond2.setObjectName("cond2")
        self.cond3 = QtWidgets.QRadioButton(self.bPhase4_5)
        self.cond3.setGeometry(QtCore.QRect(300, 20, 151, 40))
        self.cond3.setObjectName("cond3")
        self.hintB4ErrorCb_5 = QtWidgets.QCheckBox(self.bPhase4_5)
        self.hintB4ErrorCb_5.setGeometry(QtCore.QRect(60, 410, 561, 40))
        self.hintB4ErrorCb_5.setObjectName("hintB4ErrorCb_5")
        self.label_7 = QtWidgets.QLabel(self.bPhase4_5)
        self.label_7.setGeometry(QtCore.QRect(10, -10, 121, 51))
        self.label_7.setObjectName("label_7")
        self.participantName = QtWidgets.QPlainTextEdit(self.intro)
        self.participantName.setGeometry(QtCore.QRect(140, 20, 261, 51))
        self.participantName.setPlainText("")
        self.participantName.setObjectName("participantName")
        self.label_4 = QtWidgets.QLabel(self.intro)
        self.label_4.setGeometry(QtCore.QRect(10, 20, 121, 51))
        self.label_4.setObjectName("label_4")
        self.startBtn = QtWidgets.QPushButton(self.intro)
        self.startBtn.setGeometry(QtCore.QRect(490, 10, 170, 81))
        self.startBtn.setObjectName("startBtn")
        self.label_5 = QtWidgets.QLabel(self.intro)
        self.label_5.setGeometry(QtCore.QRect(20, 70, 101, 51))
        self.label_5.setObjectName("label_5")
        self.participantId = QtWidgets.QPlainTextEdit(self.intro)
        self.participantId.setGeometry(QtCore.QRect(140, 80, 261, 51))
        self.participantId.setPlainText("")
        self.participantId.setObjectName("participantId")
        self.scenarios.addTab(self.intro, "")
        self.scenarioA = QtWidgets.QWidget()
        self.scenarioA.setObjectName("scenarioA")
        self.sampleHint = QtWidgets.QPushButton(self.scenarioA)
        self.sampleHint.setGeometry(QtCore.QRect(550, 580, 191, 71))
        self.sampleHint.setObjectName("sampleHint")
        self.introB = QtWidgets.QPushButton(self.scenarioA)
        self.introB.setGeometry(QtCore.QRect(550, 680, 191, 61))
        self.introB.setObjectName("introB")
        self.afterOpenB = QtWidgets.QPushButton(self.scenarioA)
        self.afterOpenB.setGeometry(QtCore.QRect(760, 670, 191, 71))
        self.afterOpenB.setObjectName("afterOpenB")
        self.correctHideout = QtWidgets.QPushButton(self.scenarioA)
        self.correctHideout.setGeometry(QtCore.QRect(1000, 560, 271, 101))
        self.correctHideout.setObjectName("correctHideout")
        self.incorrectBtnA = QtWidgets.QPushButton(self.scenarioA)
        self.incorrectBtnA.setGeometry(QtCore.QRect(580, 80, 131, 71))
        self.incorrectBtnA.setObjectName("incorrectBtnA")
        self.goBtnA = QtWidgets.QPushButton(self.scenarioA)
        self.goBtnA.setGeometry(QtCore.QRect(570, 10, 151, 61))
        self.goBtnA.setObjectName("goBtnA")
        self.bPhase1 = QtWidgets.QGroupBox(self.scenarioA)
        self.bPhase1.setGeometry(QtCore.QRect(10, 70, 541, 411))
        self.bPhase1.setTitle("")
        self.bPhase1.setFlat(False)
        self.bPhase1.setCheckable(False)
        self.bPhase1.setObjectName("bPhase1")
        self.a1_c = QtWidgets.QRadioButton(self.bPhase1)
        self.a1_c.setGeometry(QtCore.QRect(20, 40, 212, 40))
        self.a1_c.setChecked(True)
        self.a1_c.setObjectName("a1_c")
        self.buttonGroup = QtWidgets.QButtonGroup(mainWindow)
        self.buttonGroup.setObjectName("buttonGroup")
        self.buttonGroup.addButton(self.a1_c)
        self.a1_n1 = QtWidgets.QRadioButton(self.bPhase1)
        self.a1_n1.setGeometry(QtCore.QRect(160, 40, 161, 40))
        self.a1_n1.setObjectName("a1_n1")
        self.buttonGroup.addButton(self.a1_n1)
        self.a2_n1 = QtWidgets.QRadioButton(self.bPhase1)
        self.a2_n1.setGeometry(QtCore.QRect(160, 90, 391, 40))
        self.a2_n1.setObjectName("a2_n1")
        self.buttonGroup.addButton(self.a2_n1)
        self.a3_n1a = QtWidgets.QRadioButton(self.bPhase1)
        self.a3_n1a.setGeometry(QtCore.QRect(160, 170, 131, 40))
        self.a3_n1a.setObjectName("a3_n1a")
        self.buttonGroup.addButton(self.a3_n1a)
        self.a3_n1b = QtWidgets.QRadioButton(self.bPhase1)
        self.a3_n1b.setGeometry(QtCore.QRect(260, 170, 131, 40))
        self.a3_n1b.setObjectName("a3_n1b")
        self.buttonGroup.addButton(self.a3_n1b)
        self.a3_n1c = QtWidgets.QRadioButton(self.bPhase1)
        self.a3_n1c.setGeometry(QtCore.QRect(370, 170, 131, 40))
        self.a3_n1c.setObjectName("a3_n1c")
        self.buttonGroup.addButton(self.a3_n1c)
        self.a2_c = QtWidgets.QRadioButton(self.bPhase1)
        self.a2_c.setGeometry(QtCore.QRect(20, 90, 141, 40))
        self.a2_c.setObjectName("a2_c")
        self.buttonGroup.addButton(self.a2_c)
        self.a3_c = QtWidgets.QRadioButton(self.bPhase1)
        self.a3_c.setGeometry(QtCore.QRect(20, 140, 121, 40))
        self.a3_c.setObjectName("a3_c")
        self.buttonGroup.addButton(self.a3_c)
        self.a1_n2 = QtWidgets.QRadioButton(self.bPhase1)
        self.a1_n2.setGeometry(QtCore.QRect(160, 60, 161, 40))
        self.a1_n2.setObjectName("a1_n2")
        self.buttonGroup.addButton(self.a1_n2)
        self.a2_n2 = QtWidgets.QRadioButton(self.bPhase1)
        self.a2_n2.setGeometry(QtCore.QRect(160, 110, 391, 40))
        self.a2_n2.setObjectName("a2_n2")
        self.buttonGroup.addButton(self.a2_n2)
        self.a3_n1 = QtWidgets.QRadioButton(self.bPhase1)
        self.a3_n1.setGeometry(QtCore.QRect(160, 140, 151, 40))
        self.a3_n1.setObjectName("a3_n1")
        self.buttonGroup.addButton(self.a3_n1)
        self.a4_c = QtWidgets.QRadioButton(self.bPhase1)
        self.a4_c.setGeometry(QtCore.QRect(20, 200, 121, 40))
        self.a4_c.setObjectName("a4_c")
        self.buttonGroup.addButton(self.a4_c)
        self.a4_g1 = QtWidgets.QRadioButton(self.bPhase1)
        self.a4_g1.setGeometry(QtCore.QRect(160, 200, 121, 40))
        self.a4_g1.setObjectName("a4_g1")
        self.buttonGroup.addButton(self.a4_g1)
        self.a4_g2 = QtWidgets.QRadioButton(self.bPhase1)
        self.a4_g2.setGeometry(QtCore.QRect(160, 220, 171, 40))
        self.a4_g2.setObjectName("a4_g2")
        self.buttonGroup.addButton(self.a4_g2)
        self.a4_g3 = QtWidgets.QRadioButton(self.bPhase1)
        self.a4_g3.setGeometry(QtCore.QRect(160, 240, 121, 40))
        self.a4_g3.setObjectName("a4_g3")
        self.buttonGroup.addButton(self.a4_g3)
        self.cmdBtn = QtWidgets.QPushButton(self.scenarioA)
        self.cmdBtn.setGeometry(QtCore.QRect(180, 640, 271, 101))
        self.cmdBtn.setObjectName("cmdBtn")
        self.startABtn = QtWidgets.QPushButton(self.scenarioA)
        self.startABtn.setGeometry(QtCore.QRect(560, 250, 151, 61))
        self.startABtn.setObjectName("startABtn")
        self.scenarios.addTab(self.scenarioA, "")
        self.scenarioB = QtWidgets.QWidget()
        self.scenarioB.setObjectName("scenarioB")
        self.correctWeapon = QtWidgets.QPushButton(self.scenarioB)
        self.correctWeapon.setGeometry(QtCore.QRect(1090, 420, 191, 101))
        self.correctWeapon.setObjectName("correctWeapon")
        self.incorrectProgressC = QtWidgets.QPushButton(self.scenarioB)
        self.incorrectProgressC.setGeometry(QtCore.QRect(1090, 240, 191, 101))
        self.incorrectProgressC.setObjectName("incorrectProgressC")
        self.introC = QtWidgets.QPushButton(self.scenarioB)
        self.introC.setGeometry(QtCore.QRect(970, 660, 121, 71))
        self.introC.setObjectName("introC")
        self.correctProgressC = QtWidgets.QPushButton(self.scenarioB)
        self.correctProgressC.setGeometry(QtCore.QRect(1090, 130, 191, 101))
        self.correctProgressC.setObjectName("correctProgressC")
        self.afterOpenC = QtWidgets.QPushButton(self.scenarioB)
        self.afterOpenC.setGeometry(QtCore.QRect(1100, 660, 191, 71))
        self.afterOpenC.setObjectName("afterOpenC")
        self.giveHintC = QtWidgets.QPushButton(self.scenarioB)
        self.giveHintC.setGeometry(QtCore.QRect(960, 540, 141, 81))
        self.giveHintC.setObjectName("giveHintC")
        self.bPhase1_2 = QtWidgets.QGroupBox(self.scenarioB)
        self.bPhase1_2.setGeometry(QtCore.QRect(10, 10, 541, 511))
        self.bPhase1_2.setTitle("")
        self.bPhase1_2.setObjectName("bPhase1_2")
        self.b1_c = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_c.setGeometry(QtCore.QRect(20, 40, 212, 40))
        self.b1_c.setChecked(True)
        self.b1_c.setObjectName("b1_c")
        self.buttonGroup_2 = QtWidgets.QButtonGroup(mainWindow)
        self.buttonGroup_2.setObjectName("buttonGroup_2")
        self.buttonGroup_2.addButton(self.b1_c)
        self.b1_n1 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_n1.setGeometry(QtCore.QRect(180, 40, 161, 40))
        self.b1_n1.setObjectName("b1_n1")
        self.buttonGroup_2.addButton(self.b1_n1)
        self.b1_n2 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_n2.setGeometry(QtCore.QRect(180, 90, 391, 40))
        self.b1_n2.setObjectName("b1_n2")
        self.buttonGroup_2.addButton(self.b1_n2)
        self.b2_n1a = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b2_n1a.setGeometry(QtCore.QRect(180, 170, 131, 40))
        self.b2_n1a.setObjectName("b2_n1a")
        self.buttonGroup_2.addButton(self.b2_n1a)
        self.b2_n1b = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b2_n1b.setGeometry(QtCore.QRect(270, 170, 131, 40))
        self.b2_n1b.setObjectName("b2_n1b")
        self.buttonGroup_2.addButton(self.b2_n1b)
        self.b2_n1c = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b2_n1c.setGeometry(QtCore.QRect(390, 170, 131, 40))
        self.b2_n1c.setObjectName("b2_n1c")
        self.buttonGroup_2.addButton(self.b2_n1c)
        self.b2_c = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b2_c.setGeometry(QtCore.QRect(20, 140, 121, 40))
        self.b2_c.setObjectName("b2_c")
        self.buttonGroup_2.addButton(self.b2_c)
        self.b1_n3 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_n3.setGeometry(QtCore.QRect(180, 110, 391, 40))
        self.b1_n3.setObjectName("b1_n3")
        self.buttonGroup_2.addButton(self.b1_n3)
        self.b2_n1 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b2_n1.setGeometry(QtCore.QRect(180, 140, 151, 40))
        self.b2_n1.setObjectName("b2_n1")
        self.buttonGroup_2.addButton(self.b2_n1)
        self.b3_c = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_c.setGeometry(QtCore.QRect(20, 220, 121, 40))
        self.b3_c.setObjectName("b3_c")
        self.buttonGroup_2.addButton(self.b3_c)
        self.b3_g1 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g1.setGeometry(QtCore.QRect(180, 220, 121, 40))
        self.b3_g1.setObjectName("b3_g1")
        self.buttonGroup_2.addButton(self.b3_g1)
        self.b3_g2 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g2.setGeometry(QtCore.QRect(180, 240, 121, 40))
        self.b3_g2.setObjectName("b3_g2")
        self.buttonGroup_2.addButton(self.b3_g2)
        self.b3_g3 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g3.setGeometry(QtCore.QRect(180, 260, 121, 40))
        self.b3_g3.setObjectName("b3_g3")
        self.buttonGroup_2.addButton(self.b3_g3)
        self.b1_n1b = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_n1b.setGeometry(QtCore.QRect(280, 60, 131, 40))
        self.b1_n1b.setObjectName("b1_n1b")
        self.buttonGroup_2.addButton(self.b1_n1b)
        self.b1_n1a = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_n1a.setGeometry(QtCore.QRect(180, 60, 131, 40))
        self.b1_n1a.setObjectName("b1_n1a")
        self.buttonGroup_2.addButton(self.b1_n1a)
        self.b1_n1c = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b1_n1c.setGeometry(QtCore.QRect(390, 60, 131, 40))
        self.b1_n1c.setObjectName("b1_n1c")
        self.buttonGroup_2.addButton(self.b1_n1c)
        self.b3_g4 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g4.setGeometry(QtCore.QRect(180, 280, 121, 40))
        self.b3_g4.setObjectName("b3_g4")
        self.buttonGroup_2.addButton(self.b3_g4)
        self.b3_g5 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g5.setGeometry(QtCore.QRect(180, 300, 121, 40))
        self.b3_g5.setObjectName("b3_g5")
        self.buttonGroup_2.addButton(self.b3_g5)
        self.b3_g6 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g6.setGeometry(QtCore.QRect(180, 320, 181, 40))
        self.b3_g6.setObjectName("b3_g6")
        self.buttonGroup_2.addButton(self.b3_g6)
        self.b3_g7 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g7.setGeometry(QtCore.QRect(180, 340, 181, 40))
        self.b3_g7.setObjectName("b3_g7")
        self.buttonGroup_2.addButton(self.b3_g7)
        self.b3_g8 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g8.setGeometry(QtCore.QRect(180, 360, 181, 40))
        self.b3_g8.setObjectName("b3_g8")
        self.buttonGroup_2.addButton(self.b3_g8)
        self.b3_g9 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g9.setGeometry(QtCore.QRect(180, 380, 181, 40))
        self.b3_g9.setObjectName("b3_g9")
        self.buttonGroup_2.addButton(self.b3_g9)
        self.b3_g10 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b3_g10.setGeometry(QtCore.QRect(180, 400, 181, 40))
        self.b3_g10.setObjectName("b3_g10")
        self.buttonGroup_2.addButton(self.b3_g10)
        self.b4_c = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b4_c.setGeometry(QtCore.QRect(30, 420, 121, 40))
        self.b4_c.setObjectName("b4_c")
        self.buttonGroup_2.addButton(self.b4_c)
        self.b4_n1 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b4_n1.setGeometry(QtCore.QRect(180, 420, 161, 40))
        self.b4_n1.setObjectName("b4_n1")
        self.buttonGroup_2.addButton(self.b4_n1)
        self.b4_n2 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b4_n2.setGeometry(QtCore.QRect(180, 440, 161, 40))
        self.b4_n2.setObjectName("b4_n2")
        self.buttonGroup_2.addButton(self.b4_n2)
        self.b2_n2 = QtWidgets.QRadioButton(self.bPhase1_2)
        self.b2_n2.setGeometry(QtCore.QRect(180, 190, 151, 40))
        self.b2_n2.setObjectName("b2_n2")
        self.buttonGroup_2.addButton(self.b2_n2)
        self.goBtnB = QtWidgets.QPushButton(self.scenarioB)
        self.goBtnB.setGeometry(QtCore.QRect(610, 20, 151, 61))
        self.goBtnB.setObjectName("goBtnB")
        self.incorrectBtnB = QtWidgets.QPushButton(self.scenarioB)
        self.incorrectBtnB.setGeometry(QtCore.QRect(620, 90, 131, 71))
        self.incorrectBtnB.setObjectName("incorrectBtnB")
        self.scenarios.addTab(self.scenarioB, "")
        self.scenarioC = QtWidgets.QWidget()
        self.scenarioC.setObjectName("scenarioC")
        self.finishBtn = QtWidgets.QPushButton(self.scenarioC)
        self.finishBtn.setGeometry(QtCore.QRect(590, 30, 170, 51))
        self.finishBtn.setObjectName("finishBtn")
        self.bPhase1_3 = QtWidgets.QGroupBox(self.scenarioC)
        self.bPhase1_3.setGeometry(QtCore.QRect(30, 20, 311, 511))
        self.bPhase1_3.setTitle("")
        self.bPhase1_3.setObjectName("bPhase1_3")
        self.c0 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c0.setGeometry(QtCore.QRect(20, 40, 212, 40))
        self.c0.setChecked(True)
        self.c0.setObjectName("c0")
        self.buttonGroup_3 = QtWidgets.QButtonGroup(mainWindow)
        self.buttonGroup_3.setObjectName("buttonGroup_3")
        self.buttonGroup_3.addButton(self.c0)
        self.c1 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c1.setGeometry(QtCore.QRect(20, 60, 121, 40))
        self.c1.setObjectName("c1")
        self.buttonGroup_3.addButton(self.c1)
        self.c2 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c2.setGeometry(QtCore.QRect(20, 90, 241, 40))
        self.c2.setObjectName("c2")
        self.buttonGroup_3.addButton(self.c2)
        self.c3 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c3.setGeometry(QtCore.QRect(20, 110, 241, 40))
        self.c3.setObjectName("c3")
        self.buttonGroup_3.addButton(self.c3)
        self.c5 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c5.setGeometry(QtCore.QRect(20, 150, 241, 40))
        self.c5.setObjectName("c5")
        self.buttonGroup_3.addButton(self.c5)
        self.c5a = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c5a.setGeometry(QtCore.QRect(20, 200, 241, 40))
        self.c5a.setObjectName("c5a")
        self.buttonGroup_3.addButton(self.c5a)
        self.c5b = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c5b.setGeometry(QtCore.QRect(150, 200, 241, 40))
        self.c5b.setObjectName("c5b")
        self.buttonGroup_3.addButton(self.c5b)
        self.c6 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c6.setGeometry(QtCore.QRect(20, 230, 241, 40))
        self.c6.setObjectName("c6")
        self.buttonGroup_3.addButton(self.c6)
        self.c55 = QtWidgets.QRadioButton(self.bPhase1_3)
        self.c55.setGeometry(QtCore.QRect(20, 180, 241, 40))
        self.c55.setObjectName("c55")
        self.buttonGroup_3.addButton(self.c55)
        self.goBtnC = QtWidgets.QPushButton(self.scenarioC)
        self.goBtnC.setGeometry(QtCore.QRect(370, 30, 151, 61))
        self.goBtnC.setObjectName("goBtnC")
        self.incorrectBtnC = QtWidgets.QPushButton(self.scenarioC)
        self.incorrectBtnC.setGeometry(QtCore.QRect(380, 100, 131, 51))
        self.incorrectBtnC.setObjectName("incorrectBtnC")
        self.invalidBtnC = QtWidgets.QPushButton(self.scenarioC)
        self.invalidBtnC.setGeometry(QtCore.QRect(380, 160, 131, 51))
        self.invalidBtnC.setObjectName("invalidBtnC")
        self.scenarios.addTab(self.scenarioC, "")
        self.timeLabel = QtWidgets.QLabel(self.centralwidget)
        self.timeLabel.setGeometry(QtCore.QRect(820, 130, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.timeLabel.setFont(font)
        self.timeLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.timeLabel.setObjectName("timeLabel")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(820, 170, 121, 71))
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.mistyField = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.mistyField.setGeometry(QtCore.QRect(20, 50, 331, 61))
        self.mistyField.setPlainText("")
        self.mistyField.setObjectName("mistyField")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(20, 0, 241, 51))
        self.label_3.setObjectName("label_3")
        self.mistyBtn = QtWidgets.QPushButton(self.centralwidget)
        self.mistyBtn.setGeometry(QtCore.QRect(360, 60, 121, 51))
        self.mistyBtn.setObjectName("mistyBtn")
        self.repeatButtonMisty = QtWidgets.QPushButton(self.centralwidget)
        self.repeatButtonMisty.setGeometry(QtCore.QRect(360, 10, 121, 41))
        self.repeatButtonMisty.setObjectName("repeatButtonMisty")
        self.vectorField = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.vectorField.setGeometry(QtCore.QRect(500, 50, 311, 61))
        self.vectorField.setPlainText("")
        self.vectorField.setObjectName("vectorField")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(500, 0, 241, 51))
        self.label_6.setObjectName("label_6")
        self.vectorBtn = QtWidgets.QPushButton(self.centralwidget)
        self.vectorBtn.setGeometry(QtCore.QRect(820, 60, 131, 51))
        self.vectorBtn.setObjectName("vectorBtn")
        self.repeatButtonVector = QtWidgets.QPushButton(self.centralwidget)
        self.repeatButtonVector.setGeometry(QtCore.QRect(820, 10, 131, 41))
        self.repeatButtonVector.setObjectName("repeatButtonVector")
        mainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(mainWindow)
        self.scenarios.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(mainWindow)

    def retranslateUi(self, mainWindow):
        _translate = QtCore.QCoreApplication.translate
        mainWindow.setWindowTitle(
            _translate("mainWindow", "Role Playing Robots Wizard")
        )
        self.cond1.setText(_translate("mainWindow", "Control"))
        self.cond2.setText(_translate("mainWindow", "Narrative Agency"))
        self.cond3.setText(_translate("mainWindow", "Game Agency"))
        self.hintB4ErrorCb_5.setText(_translate("mainWindow", "Error"))
        self.label_7.setText(_translate("mainWindow", "Condition"))
        self.participantName.setPlaceholderText(
            _translate("mainWindow", "Enter text here")
        )
        self.label_4.setText(_translate("mainWindow", "Participant Name"))
        self.startBtn.setText(_translate("mainWindow", "Start/Reset"))
        self.label_5.setText(_translate("mainWindow", "Participant #"))
        self.participantId.setPlaceholderText(
            _translate("mainWindow", "Enter text here")
        )
        self.scenarios.setTabText(
            self.scenarios.indexOf(self.intro), _translate("mainWindow", "Intro")
        )
        self.sampleHint.setText(_translate("mainWindow", "Sample Hint"))
        self.introB.setText(_translate("mainWindow", "Intro"))
        self.afterOpenB.setText(_translate("mainWindow", "After\n" "Opening B"))
        self.correctHideout.setText(
            _translate("mainWindow", "FINISH\n" "Correct Hideout")
        )
        self.incorrectBtnA.setText(_translate("mainWindow", "Incorrect Response"))
        self.goBtnA.setText(_translate("mainWindow", "GO / Correct Response"))
        self.a1_c.setText(_translate("mainWindow", "A1_C: Agent Lee Intro"))
        self.a1_n1.setText(_translate("mainWindow", "A1_N1: Participant Info"))
        self.a2_n1.setText(
            _translate("mainWindow", "A2_N1: Participant popular questions")
        )
        self.a3_n1a.setText(_translate("mainWindow", "Millenium Park"))
        self.a3_n1b.setText(_translate("mainWindow", "Magnificent Mile"))
        self.a3_n1c.setText(_translate("mainWindow", "Navy Pier"))
        self.a2_c.setText(_translate("mainWindow", "A2_C: Popular locations"))
        self.a3_c.setText(_translate("mainWindow", "A3_C: Location Choice"))
        self.a1_n2.setText(_translate("mainWindow", "A1_N2: Lee look forward"))
        self.a2_n2.setText(_translate("mainWindow", "A2_N2: Good intuition"))
        self.a3_n1.setText(_translate("mainWindow", "A3_N1: Location Choice"))
        self.a4_c.setText(_translate("mainWindow", "A4_C: Top of scene"))
        self.a4_g1.setText(_translate("mainWindow", "A4_G1: Top of scene"))
        self.a4_g2.setText(_translate("mainWindow", "A4_G2: Alphabet symbols"))
        self.a4_g3.setText(_translate("mainWindow", "A4_G3: Answer"))
        self.cmdBtn.setText(_translate("mainWindow", "Speak phase"))
        self.startABtn.setText(_translate("mainWindow", "Start timer"))
        self.scenarios.setTabText(
            self.scenarios.indexOf(self.scenarioA),
            _translate("mainWindow", "Scenario A"),
        )
        self.correctWeapon.setText(
            _translate("mainWindow", "FINISH\n" "Correct Weapon")
        )
        self.incorrectProgressC.setText(
            _translate("mainWindow", "Incorrect\n" "Progress")
        )
        self.introC.setText(_translate("mainWindow", "Intro"))
        self.correctProgressC.setText(_translate("mainWindow", "Correct\n" "Progress"))
        self.afterOpenC.setText(_translate("mainWindow", "After\n" "Opening C"))
        self.giveHintC.setText(_translate("mainWindow", "Give Hint"))
        self.b1_c.setText(_translate("mainWindow", "B1_C: Flyer Intro + China"))
        self.b1_n1.setText(_translate("mainWindow", "B1_N1: Flyer + Crime location"))
        self.b1_n2.setText(_translate("mainWindow", "B1_N2: Restaurants (Y/N)"))
        self.b2_n1a.setText(_translate("mainWindow", "Kitchen"))
        self.b2_n1b.setText(_translate("mainWindow", "Dining Room"))
        self.b2_n1c.setText(_translate("mainWindow", "Cleaning Closet"))
        self.b2_c.setText(_translate("mainWindow", "B2_C: Explore restaurant"))
        self.b1_n3.setText(_translate("mainWindow", "B1_N3: Recommendations"))
        self.b2_n1.setText(_translate("mainWindow", "B2_N1: Choices"))
        self.b3_c.setText(_translate("mainWindow", "B3_C: Top of scene"))
        self.b3_g1.setText(_translate("mainWindow", "B3_G1: Top of Scene"))
        self.b3_g2.setText(_translate("mainWindow", "B3_G2: Initial hunch"))
        self.b3_g3.setText(_translate("mainWindow", "B3_G3: Ingredient matching"))
        self.b1_n1b.setText(_translate("mainWindow", "Pilsen"))
        self.b1_n1a.setText(_translate("mainWindow", "Chinatown"))
        self.b1_n1c.setText(_translate("mainWindow", "Greektown"))
        self.b3_g4.setText(_translate("mainWindow", "B3_G4: Receipt Matches"))
        self.b3_g5.setText(_translate("mainWindow", "B3_G5: Numbers"))
        self.b3_g6.setText(_translate("mainWindow", "B3_G6: Modify Numbers done"))
        self.b3_g7.setText(
            _translate("mainWindow", "B3_G7: Convert num to letter hint")
        )
        self.b3_g8.setText(_translate("mainWindow", "B3_G8: Num to letter check in"))
        self.b3_g9.setText(_translate("mainWindow", "B3_G9: Order hint"))
        self.b3_g10.setText(_translate("mainWindow", "B3_G10: Final ingredient"))
        self.b4_c.setText(_translate("mainWindow", "B4_C: Note writing"))
        self.b4_n1.setText(_translate("mainWindow", "B4_N1: Top of scene"))
        self.b4_n2.setText(_translate("mainWindow", "B4_N2: End of scene"))
        self.b2_n2.setText(_translate("mainWindow", "B2_N2: Last choice (Y/N)"))
        self.goBtnB.setText(_translate("mainWindow", "GO / Correct Response"))
        self.incorrectBtnB.setText(_translate("mainWindow", "Incorrect Response"))
        self.scenarios.setTabText(
            self.scenarios.indexOf(self.scenarioB),
            _translate("mainWindow", "Scenario B"),
        )
        self.finishBtn.setText(_translate("mainWindow", "Finish / Export Data"))
        self.c0.setText(_translate("mainWindow", "C0: Top of scene"))
        self.c1.setText(_translate("mainWindow", "C1: Call to action"))
        self.c2.setText(
            _translate("mainWindow", "C2: Bomb answering (no auto-advance!)")
        )
        self.c3.setText(
            _translate("mainWindow", "C3: Bomb final answer (no auto-advance!)")
        )
        self.c5.setText(_translate("mainWindow", "C5: Top of debate"))
        self.c5a.setText(_translate("mainWindow", "C5A: Get rid of bomb"))
        self.c5b.setText(_translate("mainWindow", "C5B: Plant bomb"))
        self.c6.setText(_translate("mainWindow", "C6: Conclusion"))
        self.c55.setText(_translate("mainWindow", "C5.5: Why?"))
        self.goBtnC.setText(_translate("mainWindow", "GO / Positive Response"))
        self.incorrectBtnC.setText(_translate("mainWindow", "Negative Response"))
        self.invalidBtnC.setText(_translate("mainWindow", "Invalid response"))
        self.scenarios.setTabText(
            self.scenarios.indexOf(self.scenarioC),
            _translate("mainWindow", "Scenario C"),
        )
        self.timeLabel.setText(_translate("mainWindow", "5:00"))
        self.label_2.setText(_translate("mainWindow", "Time elapsed"))
        self.mistyField.setPlaceholderText(_translate("mainWindow", "Enter text here"))
        self.label_3.setText(_translate("mainWindow", "Agent Jay Speak"))
        self.mistyBtn.setText(_translate("mainWindow", "Speak"))
        self.repeatButtonMisty.setText(_translate("mainWindow", "Repeat"))
        self.vectorField.setPlaceholderText(_translate("mainWindow", "Enter text here"))
        self.label_6.setText(_translate("mainWindow", "Agent Lee Speak"))
        self.vectorBtn.setText(_translate("mainWindow", "Speak"))
        self.repeatButtonVector.setText(_translate("mainWindow", "Repeat"))
