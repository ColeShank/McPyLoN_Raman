'''
Known Issues:
- Homing doesn't always work, idk why :(

Planned Future Work:
- Update manual

Note: Any changes to the code will require repackaging into an executable again.
'''

## Developed 2024 for the Reznik lab at the Univerisity of Colorado at Boulder by Cole Shank and Hope Whitelock
vers_no = "3.0"

## imports
import sys
import os
import configparser, time, serial
import datetime as dt
import PyQt5.QtGui as QtG
import PyQt5.QtCore as QtC
import PyQt5.QtWidgets as QtW
import pyqtgraph as pg
from tkinter.filedialog import askdirectory
import pylablib as pll
pll.par["devices/dlls/picam"] = "path/to/dlls"
from pylablib.devices import PrincetonInstruments
from matplotlib import pyplot as plt
import numpy as np
import ctypes
import webbrowser
myappid = 'ramanControl.reznik.01' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
plt.ion()

def setup():
    ## connects to monochromator and ccd
    config = configparser.RawConfigParser()
    config.read('mono.cfg')
    sys = config.get('Mono_settings', 'mono_system')
    global localuser
    global comport
    if sys == "single":
        localuser = "nickp"
        comport = "COM3"
    elif sys == "triple_single":
        localuser = "Reznik Lab"
        comport = "COM1"
    elif sys == "triple_triple":
        localuser = "Reznik Lab"
        comport = "COM1"
    global Mono1
    print("Connecting to monochromator ...")
    Mono1 = Monochromator()
    Mono1.sendcommand(' ')
    print('Connected.')
    global cam
    print("Connecting to CCD ...")
    cam = PrincetonInstruments.PicamCamera()
    print('Connected.')
    global Window
    Window = MainWindow()
    Window.show()
    
def initialize():
    ## initialize values and settings
    config = configparser.RawConfigParser()
    config.read('mono.cfg')
    mono_system = config.get('Mono_settings', 'mono_system')
    systemUpdate(mono_system)
    monoWL = config.get('Mono_settings', 'current_wavelength')
    monoWL = round(float(monoWL),2)
    Window.currentMonoWavelengthLabel.setText(str(monoWL))
    global nm_per_px
    nm_per_px = float(config.get('Laser_settings', 'nm_per_px'))
    laserWL = config.get('Laser_settings', 'laser_wavelength')
    Window.laserMenu.setCurrentText(str(laserWL))
    laserUpdate()
    Window.shiftExcitationInput.setText(str(laserWL))
    Window.calWLInput.setText(str(laserWL))
    grating = config.get('Mono_settings', 'current_grating')
    Window.gratingMenu.setCurrentText(str(grating))
    Window.exposureTimeInput.setText("0.1")
    cam.set_attribute_value("Exposure Time", 1000*float(Window.exposureTimeInput.text()))
    global time_out
    time_out = 1000
    Window.camButton.setEnabled(True)
    Window.camButton2D.setEnabled(True)
    global liveopen
    liveopen = False
    Window.live10msButton.setEnabled(True)
    Window.live100msButton.setEnabled(False)
    Window.live1sButton.setEnabled(True)
    global liveRate
    liveRate = 100
    Window.liveMedian.setCheckState(0)
    Window.currentDir.setText('C:/Users/'+localuser+'/Desktop')
    global path
    path = os.path.join(Window.currentDir.text())
    Window.statusBar().showMessage('',1)

def systemUpdate(sys):
    global localuser
    global comport

    if sys == "single":
        Window.singleButton.setEnabled(False)
        Window.tripleSingleButton.setEnabled(True)
        Window.tripleButton.setEnabled(True)
        localuser = "nickp"
        comport = "COM3"
    elif sys == "triple_single":
        Window.tripleSingleButton.setEnabled(False)
        Window.singleButton.setEnabled(True)
        Window.tripleButton.setEnabled(True)
        localuser = "Reznik Lab"
        comport = "COM1"
    elif sys == "triple_triple":
        Window.tripleButton.setEnabled(False)
        Window.singleButton.setEnabled(True)
        Window.tripleSingleButton.setEnabled(True)
        localuser = "Reznik Lab"
        comport = "COM1"

    config = configparser.RawConfigParser()
    config.read('mono.cfg')
    config.set('Mono_settings', 'mono_system', str(sys))
    f = open('mono.cfg','w')
    config.write(f)

def laserUpdate():
    ## updates laser wavelength
    global laser
    wavl = Window.laserMenu.currentText()
    if wavl == "":
        laser = laser
    elif wavl == ".":
        laser = laser
    elif wavl == 0:
        laser = laser
    else:
        laser = float(wavl)
        config = configparser.RawConfigParser()
        config.read('mono.cfg')
        config.set('Laser_settings', 'laser_wavelength', str(laser))
        f = open('mono.cfg',"w")
        config.write(f)
        Window.windowRange()
        Window.calWLInput.setText(str(laser))
        Window.shiftExcitationInput.setText(str(laser))

def wavToNM(las,wavn):
    # converts shift "wavn" [1/cm] to nm
    if round(float(las),3) == 0:
        Window.statusBar().showMessage("Error: Zero Division", 3000)
        nm = 0
    elif round(float(wavn),3) == 0:
        Window.statusBar().showMessage("Error: Zero Division", 3000)
        nm = 0
    else:
        nm = 1/(1/float(las) - float(wavn)/float(1e+7))
    return nm

def nmToWav(las,wavl):
    # calculates shift "wav" [1/cm]
    if round(float(las),3) == 0:
        Window.statusBar().showMessage("Error: Zero Division", 3000)
        wav = 0
    elif round(float(wavl),3) == 0:
        Window.statusBar().showMessage("Error: Zero Division", 3000)
        wav = 0
    else:
        wav = (1/float(las) - 1/float(wavl))*float(1e+7)
    return wav

def setTimeout(time):
    ## prevents CCD from timing out for long exposures
    global time_out
    time_out = time

def imageCCD(pos, mode = "1D", nacq = 1, cal = False):
    ## disable buttons
    Window.camButton.setEnabled(False)
    Window.camButton2D.setEnabled(False)
    app.processEvents()

    cam.set_attribute_value("Correct Pixel Bias", True)
    if mode=="1D":
        cam.set_roi(vbin = 100)
    elif mode=="2D":
        cam.set_roi(vbin = 1)
    else:
        pass
       
    global data
    global wavelen
    global wavenum
    data = []
    wavelen = []
    wavenum = []
    signal = []

    ## calculate x-axis
    pxc = 670.5 # center pixel
    pixel = range(0,1340)
    for i in range(0,1340):
        wav = float(pos) - nm_per_px*(pixel[i]-pxc)
        wavNum = nmToWav(laser,wav)
        wavelen.append(wav)
        wavenum.append(wavNum)
    
    wavelen = wavelen * nacq
    wavenum = wavenum * nacq
    if mode == "2D":
        wavelen = wavelen * 100
        wavenum = wavenum * 100
    else:
        pass
    
    for n in range(0,nacq):
        int_time_out = int(round(time_out))
        Window.statusBar().showMessage('Acquisition underway: ('+str(n+1)+'/'+str(nacq)+')',1000*(int_time_out-5))
        app.processEvents()
        try: ## Mutes most errors, including invalid binning and acquisition not in progress 
            img = np.array(cam.grab(nframes=1, frame_timeout=time_out))
        except:
            try:
                img = np.array(cam.grab(nframes=1, frame_timeout=time_out))
            except:
                img = np.array(cam.grab(nframes=1, frame_timeout=time_out))
        if mode == "1D":
            signal = img.reshape(-1)
            data = np.concatenate((data,signal))
            plt.plot(wavenum[0:1340], data[1340*n:1340*(n+1)])
            plt.show()
        elif mode == "2D":
            ## This may or may not be the most sensible way to package the 2D data into a 1D array
            img = np.squeeze(img)
            img1D = np.ravel(img)
            signal = img1D.tolist()
            data = data + signal
            plt.imshow(img,aspect='auto')
    app.processEvents()
    autoSave(cal)
    
    ## re-enable buttons
    Window.camButton.setEnabled(True)
    Window.camButton2D.setEnabled(True)

def autoSave(cal=False):
    ## saves backup data to the Documents repositories
    now = dt.datetime.now()
    now_str = now.strftime('%y%m%d%H%M%S')
    docpath = f'C:/Users/{localuser}/Documents'
    if cal == True:
        tempname = 'Cal_'+now_str
        autopath = os.path.join(docpath,'Raman calibration files',str(tempname))
    elif cal == False:
        tempname = 'Raman_'+now_str
        autopath = os.path.join(docpath,'Raman data files',str(tempname))
    file = open(autopath,'w')
    file.write('Wavelength(nm), Raman shift(cm^-1), Intensity(counts) \n')
    for line in range(len(data)):
        stringToWrite = str(wavelen[line])+','+str(wavenum[line])+','+str(data[line])+'\n'
        file.write(stringToWrite)
    file.close()
    autopath_txt = os.path.join(autopath+'.txt')
    os.rename(autopath,autopath_txt)

def saveData(fname):
    ## save data to the current directory and filename
    fpath = os.path.join(path,fname)
    bool = os.path.isfile(os.path.join(path,fname+'.txt'))
    if bool == True:
        Window.statusBar().showMessage("Error: file already exists", 3000)
    else:
        file = open(fpath,'w')
        file.write('Wavelength(nm), Raman shift(cm^-1), Intensity(counts) \n')
        for line in range(len(data)):
            stringToWrite = str(wavelen[line])+','+str(wavenum[line])+','+str(data[line])+'\n' 
            file.write(stringToWrite)
        file.close()
        fpath_txt = os.path.join(path,fname+'.txt')
        os.rename(fpath,fpath_txt)
        Window.statusBar().showMessage("File saved.", 3000)

class Monochromator(object):
    ## Initializes a serial port
    def __init__(self):
        self.mono = serial.Serial(comport, timeout=1, baudrate=9600, xonxoff=1, stopbits=1)
        self.config = configparser.RawConfigParser()
        self.config.read('mono.cfg')
        self.current_wavelength = self.config.get('Mono_settings', 'current_wavelength')
        self.speed = self.config.get('Mono_settings', 'speed')
        self.approach_speed = self.config.get('Mono_settings', 'approach_speed')
        self.offset = self.config.get('Mono_settings', 'offset')
        self.nm_per_revolution = self.config.get('Mono_settings', 'nm_per_revolution')
        self.steps_per_revolution = self.config.get('Mono_settings', 'steps_per_revolution')

    ## sends ascii commands to the serial port and pauses for half a second afterwards
    def sendcommand(self,command):
        self.mono.flushInput()
        self.mono.flushOutput()
        if (command != "^"):
            print('Send command: ' + command)
        self.mono.write(bytearray(command + '\r\n','ascii'))
        time.sleep(0.5) 
        
    ## reads ascii text from serial port + formatting
    def readout(self):
        #time.sleep(0.5)
        #self.mono.flushInput()
        value = self.mono.readline().decode("utf-8")
        return str(value.rstrip().lstrip())
        
    ## sets the ramp speed
    def setRampspeed(self, rampspeed):
        self.sendcommand('K ' + str(rampspeed))
        
    ## sets the initial velocity
    def setInitialVelocity(self,initspeed): 
        self.sendcommand('I ' + str(initspeed))
        
    ## sets the velocity   
    def setVelocity(self,velocity):
        self.sendcommand('V ' + str(velocity))
        
    ## checks if the Monochromator is moving (returns True or False) 
    def moving(self):
        self.sendcommand('^')
        a = self.readout()
        if a[3:].lstrip() == "0":
            print("Mono is not moving \r")
            return False
        else:
            print("Mono is moving \r")
            return True
			
    def checkfortimeout(self):
        try:
            self.sendcommand('X')
            if self.readout() == None:
                print('Timeout occurred')
        except:
            print('Timeout occurred')
            
    def checkLimitSwitches(self):
        self.sendcommand("]")
        a = self.readout()
        if a[3:].lstrip() == "64":
            return "Upper"
        if a[3:].lstrip() == "128":
            return "Lower"
        else:
            return False
        
    def checkHOMEstatus(self):
        self.sendcommand("]")
        value = self.mono.readline().decode("utf-8")
        print("HOME Status: " + value[3:])
        return str(value[3:].rstrip().lstrip())
		
    def getHomePosition(self): 

        ## This function performs the homing procedure of the monochromator
        ## See the mono manual for information about the separate parameters
        
        ## This doesn't work reliably for some reason and is not currently in use
        
        self.approachWL(float(435))
        
        while(self.moving()):
            self.moving()

        ## begin homing procedure

        self.sendcommand("A8")
        if(self.checkHOMEstatus() == "32"):
            self.sendcommand("M+23000")
            while(self.checkHOMEstatus() != "2"):
                time.sleep(0.8)
                self.checkHOMEstatus()

            self.sendcommand("@")
            self.sendcommand("-108000")
		
            while(self.moving()):
                self.moving()
				
            self.sendcommand("+72000")

            while(self.moving()):
                self.moving()
				
            self.sendcommand("A24")
	            
            while(self.moving()):
                self.moving()
            
            n1=dt.datetime.now()
			
            self.sendcommand("F1000,0")

            while(self.moving()):
                self.moving()
                n2=dt.datetime.now()
                if (((n2.microsecond-n1.microsecond)/1e6) >= 300):
                    self.sendcommand("@")
                    print("timeout, stopping")
                    break
				
            self.sendcommand("A0")
            self.config.set('Mono_settings', 'current_wavelength', '440.067')
            print("Homing done, setting current wavelength now to 440.067 nm")
            f = open('mono.cfg',"w")
            self.config.write(f)
            Window.currentMonoWavelengthLabel.setText("440.067 nm")
		
    def approachWL(self, approach_wavelength):
        Window.approachButton.setEnabled(False)
        self.config = configparser.RawConfigParser()
        self.config.read('mono.cfg')
        self.nm_per_revolution = self.config.get('Mono_settings', 'nm_per_revolution')
        self.steps_per_revolution = self.config.get('Mono_settings', 'steps_per_revolution')
        if isinstance(approach_wavelength, float):
            nm_difference = float(approach_wavelength) - float(self.current_wavelength)
            step_difference = round(((float(nm_difference) / float(self.nm_per_revolution)) * float(self.steps_per_revolution))+ float(self.offset))
            time_needed_sec = round(abs(step_difference / int(self.speed)) + abs(int(self.offset)/int(self.approach_speed)),2)
            print("Wavelength to approach: " + str(approach_wavelength) + " nm")
            print("Difference in nm: " + str(nm_difference))
            print("Difference in steps: " + str(step_difference))
            print("Time needed for operation: " + str(time_needed_sec) + " s")
            Window.statusBar().showMessage("Moving monochromator . . .  (est. "+str(time_needed_sec)+" seconds)",3000)
            time_delay_for_progressbar = (time_needed_sec + 1) / 100 # with 1 second buffer
            self.sendcommand("V" + str(self.speed))
            self.sendcommand(str(format(step_difference, '+')))
            self.sendcommand("V" + str(self.approach_speed))
            self.sendcommand("-" + str(self.offset))
            while True:
                time.sleep(time_delay_for_progressbar)
                value = Window.progressBar.value() + 1
                Window.progressBar.setValue(value)
                QtW.qApp.processEvents()
                if (value >= Window.progressBar.maximum()):
                    Window.approachWavelengthInput.clear()
                    Window.approachButton.setEnabled(True)
                    Window.progressBar.setValue(0)
                    self.config.set('Mono_settings', 'current_wavelength', approach_wavelength)
                    self.current_wavelength = approach_wavelength
                    Window.currentMonoWavelengthLabel.setText(str(round(self.current_wavelength,2)))
                    Window.windowRange()
                    f = open('mono.cfg',"w")
                    self.config.write(f)
                    break
        else:
            print("Input is not numeric")
            MessageBox = QtG.QMessageBox.warning(Window,"Error:","Input is not numeric") 
            Window.approachButton.setEnabled(True)
   
    def disconnect(self):
        self.mono.flushInput()
        self.mono.flushOutput()
        self.mono.close()


class MainWindow(QtW.QMainWindow):
    def __init__(self):
        super().__init__()

        ## all of this to make the tabs span the window width
        class TabBar(QtW.QTabBar):
            def __init__(self):
                super().__init__()
            def sizeHint(self):
                return QtC.QSize(600,20)

        ## lay out application
        self.setWindowTitle("Raman Control")
        self.setFixedSize(600, 400)
        self.statusBar().setSizeGripEnabled(False)
        self.tabs = QtW.QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tab_bar = TabBar()
        self.tabs.setTabBar(self.tab_bar)     
        tab1 = QtW.QWidget()
        tab2 = QtW.QWidget()
        tab3 = QtW.QWidget()
        tab4 = QtW.QWidget()
        tab5 = QtW.QWidget()
        p1_vertical = QtW.QFormLayout(tab1)
        p2_vertical = QtW.QFormLayout(tab2)
        p3_vertical = QtW.QFormLayout(tab3)
        p4_vertical = QtW.QFormLayout(tab4)
        p5_vertical = QtW.QFormLayout(tab5)
        self.tabs.addTab(tab1, "Spectrometer Control")
        self.tabs.addTab(tab2, "CCD Control")
        self.tabs.addTab(tab3, "File Saving")
        self.tabs.addTab(tab4, "Shift Calculator")
        self.tabs.addTab(tab5, "Configuration")
        
        ## update loop for CCD temperature
        self.update_timer = QtC.QTimer(self)
        self.update_timer.start()
        self.update_timer.setInterval(1000) # milliseconds
        self.update_timer.setSingleShot(False)
        self.update_timer.timeout.connect(lambda: self.camTempLabel.setText(str(cam.get_attribute_value('Sensor Temperature Reading')) + " C"))
        self.update_timer.timeout.connect(lambda: app.processEvents())
        
        ### TAB 1
        self.systemHeader = QtW.QLabel(self)
        self.systemHeader.setText("System Settings")
        self.systemHeader.setStyleSheet("font-weight: bold")
        
        self.laserMenu = QtW.QComboBox(self)
        self.laserMenu.setEditable(True)
        self.laserMenu.addItems(["530.79","532","670.94"])
        self.laserMenu.currentTextChanged.connect(lambda: laserUpdate())

        self.gratingMenu = QtW.QComboBox(self)
        self.gratingMenu.setEditable(False)
        self.gratingMenu.addItems(["50","150","300","600","1200","1800","2400","3600"])
        self.gratingMenu.currentIndexChanged.connect(lambda: self.updateGrating())

        self.currentMonoWavelengthLabel = QtW.QLabel(self)
        self.currentMonoWavelengthLabel.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)

        self.rangeLabelNM = QtW.QLabel(self)
        self.rangeLabelNM.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)

        self.rangeLabelWN = QtW.QLabel(self)
        self.rangeLabelWN.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)

        self.posHeader = QtW.QLabel(self)
        self.posHeader.setText("Monochromator Position")
        self.posHeader.setStyleSheet("font-weight: bold")

        self.approachWavelengthInput = QtW.QLineEdit(self)
        self.approachWavelengthInput.setMaxLength(5)
        self.approachWavelengthInput.setInputMask("999.9")
        self.approachWavelengthInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.approachWavelengthInput.textChanged.emit(self.approachWavelengthInput.text())

        self.approachButton = QtW.QPushButton(self)
        self.approachButton.setObjectName("approachButton")
        self.approachButton.clicked.connect(lambda: Mono1.approachWL(float(self.approachWavelengthInput.text())))
        self.approachButton.setText("Move to Position")

        self.progressBar = QtW.QProgressBar(self)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setMaximum(101)
		
        '''
        ##  Broken currently, left for posterity
        self.homeButton = QtW.QPushButton(self)
        self.homeButton.setObjectName("homeButton")
        self.homeButton.clicked.connect(lambda: Mono1.getHomePosition())
        self.homeButton.clicked.connect(lambda: self.statusBar().showMessage("Homing monochromator . . . ",10000))
        self.homeButton.setText("HOME Monochromator")
        '''

        p1_vertical.addRow(self.systemHeader)
        p1_vertical.addRow("Laser Wavelength (nm)", self.laserMenu)
        p1_vertical.addRow("Grating (G/mm)", self.gratingMenu)
        p1_vertical.addRow(self.posHeader)
        p1_vertical.addRow("Current Mono Wavelength (nm)", self.currentMonoWavelengthLabel)
        p1_vertical.addRow("Window range (nm)", self.rangeLabelNM)
        p1_vertical.addRow("                        (1/cm)", self.rangeLabelWN)
        p1_vertical.addRow(self.approachWavelengthInput, self.approachButton)
        p1_vertical.addRow(self.progressBar)
        #p1_vertical.addRow("Home counter location: 06601", self.homeButton) # Doesn't work RIP

        ### TAB 2
        self.ccdHeader = QtW.QLabel(self)
        self.ccdHeader.setText("CCD Settings")
        self.ccdHeader.setStyleSheet("font-weight: bold")

        self.camTempLabel = QtW.QLabel(self)
        self.camTempLabel.setAlignment(QtC.Qt.AlignRight)
        self.camTempLabel.setText(str(cam.get_attribute_value('Sensor Temperature Reading')) + " C")

        self.exposureTimeInput = QtW.QLineEdit(self)
        self.exposureTimeInput.setMaxLength(7)
        self.exposureTimeInput.setInputMask("9999.99")
        self.exposureTimeInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.exposureTimeInput.textChanged.connect(lambda: cam.set_attribute_value("Exposure Time", 1000*float(self.exposureTimeInput.text())) if self.exposureTimeInput.text()!='.' else None)
        self.exposureTimeInput.textChanged.connect(lambda: setTimeout(5.0+float(self.exposureTimeInput.text())) if self.exposureTimeInput.text()!='.' else None)
        self.exposureTimeInput.textChanged.connect(lambda: self.statusBar().showMessage('Error: must input exposure time', 3000) if self.exposureTimeInput.text()=='.' else None)

        self.nacqsInput = QtW.QSpinBox(self)
        self.nacqsInput.setRange(1,199)
        self.nacqsInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)

        self.snapshotHeader = QtW.QLabel(self)
        self.snapshotHeader.setText("Data Collection")
        self.snapshotHeader.setStyleSheet("font-weight: bold")

        self.camButton = QtW.QPushButton(self)
        self.camButton.setObjectName("camButton")
        self.camButton.clicked.connect(lambda: imageCCD(float(Mono1.current_wavelength), "1D", self.nacqsInput.value()))
        self.camButton.setText("Take 1D spectrum")

        self.camButton2D = QtW.QPushButton(self)
        self.camButton2D.setObjectName("camButton2D")
        self.camButton2D.clicked.connect(lambda: imageCCD(float(Mono1.current_wavelength), "2D", self.nacqsInput.value()))
        self.camButton2D.setText("Take 2D image")

        self.liveHeader = QtW.QLabel(self)
        self.liveHeader.setText("Live Acquisition")
        self.liveHeader.setStyleSheet("font-weight: bold")

        self.liveStartButton = QtW.QPushButton(self)
        self.liveStartButton.setText("Start Acquisition")
        self.liveStartButton.clicked.connect(lambda: self.liveAcquire(go=True))
        self.liveStartButton.clicked.connect(lambda: self.liveStartButton.setEnabled(False))
        self.liveStartButton.clicked.connect(lambda: self.liveStopButton.setEnabled(True))
        
        self.liveStopButton = QtW.QPushButton(self)
        self.liveStopButton.setText("Stop Acquisition")
        self.liveStopButton.setEnabled(False)
        self.liveStopButton.clicked.connect(lambda: self.liveWindow.close())
        self.liveStopButton.clicked.connect(lambda: self.liveStartButton.setEnabled(True))
        self.liveStopButton.clicked.connect(lambda: self.liveStopButton.setEnabled(False))

        self.liveRateHeader = QtW.QLabel(self)
        self.liveRateHeader.setText("     Exposure/Rate")
        self.liveRateHeader.setStyleSheet("font-weight: bold")
        
        self.live10msButton = QtW.QPushButton(self)
        self.live10msButton.setText("10 ms")
        self.live10msButton.clicked.connect(lambda: self.live10msButton.setEnabled(False))
        self.live10msButton.clicked.connect(lambda: self.live100msButton.setEnabled(True))
        self.live10msButton.clicked.connect(lambda: self.live1sButton.setEnabled(True))
        self.live10msButton.clicked.connect(lambda: self.liveAcquire(rate=10))
        
        self.live100msButton = QtW.QPushButton(self)
        self.live100msButton.setText("0.1 s")
        self.live100msButton.clicked.connect(lambda: self.live10msButton.setEnabled(True))
        self.live100msButton.clicked.connect(lambda: self.live100msButton.setEnabled(False))
        self.live100msButton.clicked.connect(lambda: self.live1sButton.setEnabled(True))
        self.live100msButton.clicked.connect(lambda: self.liveAcquire(rate=100))

        self.live1sButton = QtW.QPushButton(self)
        self.live1sButton.setText("1 s")
        self.live1sButton.clicked.connect(lambda: self.live10msButton.setEnabled(True))
        self.live1sButton.clicked.connect(lambda: self.live100msButton.setEnabled(True))
        self.live1sButton.clicked.connect(lambda: self.live1sButton.setEnabled(False))
        self.live1sButton.clicked.connect(lambda: self.liveAcquire(rate=1000))

        self.liveMedian = QtW.QCheckBox(self)

        p2_vertical.addRow(self.ccdHeader)
        p2_vertical.addRow("Current temp", self.camTempLabel)
        p2_vertical.addRow("Exposure time (s)", self.exposureTimeInput)
        p2_vertical.addRow("Number of acquisitions:", self.nacqsInput)
        p2_vertical.addRow(self.snapshotHeader)
        p2_vertical.addRow("Take 1D snapshot", self.camButton)
        p2_vertical.addRow("Take 2D snapshot", self.camButton2D)
        p2_vertical.addRow(self.liveHeader)
        p2_vertical.addRow(self.liveStartButton)
        p2_vertical.addRow(self.liveStopButton)        
        p2_vertical.addRow(self.liveRateHeader)
        p2_vertical.addRow(self.live10msButton)
        p2_vertical.addRow(self.live100msButton)
        p2_vertical.addRow(self.live1sButton)
        p2_vertical.addRow("     Persistence?",self.liveMedian)
 
        ### TAB 3
        self.currentDir = QtW.QLabel(self)
        self.currentDir.setAlignment(QtC.Qt.AlignRight)
        
        self.dirButton = QtW.QPushButton(self)
        self.dirButton.setObjectName("dirButton")
        self.dirButton.clicked.connect(lambda: self.currentDir.setText(askdirectory(title='Select folder')))
        self.dirButton.clicked.connect(lambda: self.pathUpdate())
        self.dirButton.setText("...")
     
        self.fname = QtW.QLineEdit(self)
        self.fname.setMaxLength(80)
        self.fname.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.fname.textChanged.emit(self.fname.text())

        self.saveButton = QtW.QPushButton(self)
        self.saveButton.setObjectName("saveButton")
        self.saveButton.clicked.connect(lambda: saveData(self.fname.text()))
        self.saveButton.setText("Save most recent data")
        
        p3_vertical.addRow("Active folder", self.currentDir)
        p3_vertical.addRow("Select new folder", self.dirButton)
        p3_vertical.addRow("File name", self.fname)
        p3_vertical.addRow(self.saveButton)
     
        ### TAB 4
        self.shiftExcitationInput = QtW.QLineEdit(self)
        self.shiftExcitationInput.setMaxLength(7)
        self.shiftExcitationInput.setInputMask("9999.99")
        self.shiftExcitationInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        self.shiftWNHeader = QtW.QLabel(self)
        self.shiftWNHeader.setText("Enter Wavelength")
        self.shiftWNHeader.setStyleSheet("font-weight: bold")
        
        self.shiftResponseInput = QtW.QLineEdit(self)
        self.shiftResponseInput.setMaxLength(7)
        self.shiftResponseInput.setInputMask("9999.99")
        self.shiftResponseInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.shiftResponseInput.textChanged.connect(lambda: self.calculateShift())

        self.shiftWN = QtW.QLabel(self)

        self.shiftNMHeader = QtW.QLabel(self)
        self.shiftNMHeader.setText("Or, Enter Shift")
        self.shiftNMHeader.setStyleSheet("font-weight: bold")
        
        self.shiftInputWN = QtW.QLineEdit(self)
        self.shiftInputWN.setObjectName("shiftInputWN")
        self.shiftInputWN.setMaxLength(7)
        self.shiftInputWN.setInputMask("#9999.9")
        self.shiftInputWN.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.shiftInputWN.textChanged.connect(lambda: self.calculateShift())
        
        self.absoluteShift = QtW.QLabel(self)
        self.absoluteShift.setObjectName("absoluteShift")
        
        self.relativeShift = QtW.QLabel(self)
        self.relativeShift.setObjectName("relativeShift")

        p4_vertical.addRow("Excitation Wavelength (nm)", self.shiftExcitationInput)
        p4_vertical.addRow(self.shiftWNHeader)
        p4_vertical.addRow("Response Wavelength (nm)", self.shiftResponseInput)
        p4_vertical.addRow("Raman Shift (1/cm)", self.shiftWN)
        p4_vertical.addRow(self.shiftNMHeader)
        p4_vertical.addRow("Raman shift (1/cm)", self.shiftInputWN)
        p4_vertical.addRow("Absolute wavelength (nm)", self.absoluteShift)
        p4_vertical.addRow("Relative wavelength (nm)", self.relativeShift)
        
        ### TAB 5
        
        '''
        ## Left for easy feature testing/debugging
        self.testButton = QtW.QPushButton(self)
        self.testButton.setText("TEST")
        ## update the following line to test whatever functionality is desired:
        self.testButton.clicked.connect(lambda: self.testFunction())        
        p5_vertical.addRow(self.testButton)
        '''

        self.versionHeader = QtW.QLabel(self)
        self.versionHeader.setText("Code Version")
        self.versionHeader.setStyleSheet("font-weight: bold")
        
        self.versionLabel = QtW.QLabel(self)
        self.versionLabel.setObjectName("versionLabel")
        self.versionLabel.setText(str(vers_no))
        self.versionLabel.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)

        self.calHeader = QtW.QLabel(self)
        self.calHeader.setText("Calibration")
        self.calHeader.setStyleSheet("font-weight: bold")

        self.currentCounterInput = QtW.QLineEdit(self)
        self.currentCounterInput.setMaxLength(5)
        self.currentCounterInput.setInputMask("99999")
        self.currentCounterInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        self.calButton = QtW.QPushButton(self)
        self.calButton.setObjectName("calButton")
        self.calButton.clicked.connect(lambda: self.calibrateMono())
        self.calButton.setText("Calibrate monochromator to counter position")
        
        self.calWLInput = QtW.QLineEdit(self)
        self.calWLInput.setMaxLength(8)
        self.calWLInput.setInputMask("9999.999")
        self.calWLInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        self.ccdCalButton = QtW.QPushButton(self)
        self.ccdCalButton.setObjectName("ccdCalButton")
        self.ccdCalButton.clicked.connect(lambda: self.calibrateCCD())
        self.ccdCalButton.setText("Calibrate CCD to known source (nm)")
        
        self.monoSelectHeader = QtW.QLabel(self)
        self.monoSelectHeader.setText("Raman System")
        self.monoSelectHeader.setStyleSheet("font-weight: bold")

        self.singleButton = QtW.QPushButton(self)
        self.singleButton.setText("Single Monochromator System")
        self.singleButton.clicked.connect(lambda: systemUpdate("single"))

        self.tripleSingleButton = QtW.QPushButton(self)
        self.tripleSingleButton.setText("Triple Monochromator System in Single Mode")
        self.tripleSingleButton.clicked.connect(lambda: systemUpdate("triple_single"))

        self.tripleButton = QtW.QPushButton(self)
        self.tripleButton.setText("Triple Monochromator System in Triple Mode")
        self.tripleButton.clicked.connect(lambda: systemUpdate("triple_triple"))

        self.linksHeader = QtW.QLabel(self)
        self.linksHeader.setText("Useful Links")
        self.linksHeader.setStyleSheet("font-weight: bold")

        self.gitButton = QtW.QPushButton(self)
        self.gitButton.setObjectName("gitButton")
        self.gitButton.setText("Open Link")
        self.gitButton.clicked.connect(lambda: webbrowser.open('github.com/ColeShank/McPyLoN_Raman'))

        self.cfgButton = QtW.QPushButton(self)
        self.cfgButton.setObjectName("cfgButton")
        self.cfgButton.setText("Open mono.cfg")
        self.cfgButton.clicked.connect(lambda: webbrowser.open('mono.cfg'))

        self.docButton = QtW.QPushButton(self)
        self.docButton.setObjectName("docButton")
        self.docButton.setText("Open Software Manual")
        self.docButton.clicked.connect(lambda: webbrowser.open('Software Manual.pdf'))
        
        p5_vertical.addRow(self.versionHeader)
        p5_vertical.addRow(self.versionLabel)
        p5_vertical.addRow(self.calHeader)
        p5_vertical.addRow(self.currentCounterInput, self.calButton)
        p5_vertical.addRow(self.calWLInput, self.ccdCalButton)        
        p5_vertical.addRow(self.monoSelectHeader)
        p5_vertical.addRow("", self.singleButton)
        p5_vertical.addRow("", self.tripleSingleButton)
        p5_vertical.addRow("", self.tripleButton)
        p5_vertical.addRow(self.linksHeader)
        p5_vertical.addRow("GitHub Repository", self.gitButton)
        p5_vertical.addRow("Configuration file", self.cfgButton)
        p5_vertical.addRow("Documentation", self.docButton)

    def calibrateMono(self):
        ## updates the monochromator position based to the counter
        if self.currentCounterInput.text() == '':
            self.statusBar().showMessage("Error: no counter value input",3000)
        else:
            grating = float(self.gratingMenu.currentText())
            calWL = round((0.1)*(1200/grating)*float(self.currentCounterInput.text()),2)
            self.config = configparser.RawConfigParser()
            self.config.read('mono.cfg')
            self.config.set('Mono_settings', 'current_wavelength', str(calWL))
            f = open('mono.cfg',"w")
            self.config.write(f)
            self.currentMonoWavelengthLabel.setText(str(calWL))
            Mono1.current_wavelength = str(calWL)
            self.currentCounterInput.clear()

    def calibrateCCD(self):      
        ## set a very short exposure time to ensure CCD is not blown out
        cam.set_attribute_value("Exposure Time", float(0.01))
        
        wavl = float(self.calWLInput.text())
        
        ## place laser on high end of detector range and take image
        Mono1.approachWL(wavl+3)
        img1 = cam.snap()
        signal1 = []
        for i in range(1340):
            signal1.append(sum(img1[:, i]))
        max1 = max(signal1)
        px1 = signal1.index(max1)
        
        ## place laser on low end of detector range and take image
        Mono1.approachWL(wavl-3)
        img2 = cam.snap()
        signal2 = []
        for i in range(1340):
            signal2.append(sum(img2[:,i]))
        max2 = max(signal2)
        px2 = signal2.index(max2)
        
        ## calculate nm/px and center detector at laser line
        global nm_per_px
        nm_per_px = 6/(px1 - px2)
        print("nm per px: "+str(nm_per_px))

        px_offset = 670.5 - px2
        nm_offset = px_offset*nm_per_px
        calWL = float(wavl-3 + nm_offset)
        print("pixel offset: " + str(px_offset))
        print("nm offset: " + str(nm_offset))
        Mono1.approachWL(calWL)
        
        ## update configuration file and such
        config = configparser.RawConfigParser()
        config.read('mono.cfg')
        config.set('Mono_settings', 'current_wavelength', str(wavl))
        config.set('Laser_settings', 'nm_per_px', str(nm_per_px))
        f = open('mono.cfg',"w")
        config.write(f)
        self.currentMonoWavelengthLabel.setText(str(wavl))
        Mono1.current_wavelength = str(wavl)
        Window.windowRange()
        
        ## take and save image to verify calibration
        imageCCD(wavl, "1D", cal = True)
        
        cam.set_attribute_value("Exposure Time", 1000*float(self.exposureTimeInput.text()))

    def updateGrating(self):
        ## updates grating selection
        grating = self.gratingMenu.currentText()
        global nm_per_revolution
        nm_per_revolution = str(32*float(150)/float(grating))
        self.config = configparser.RawConfigParser()
        self.config.read('mono.cfg')
        self.config.set('Mono_settings', 'current_grating', grating)
        self.config.set('Mono_settings', 'nm_per_revolution', nm_per_revolution)
        f = open('mono.cfg',"w")
        self.config.write(f)

    def windowRange(self):
        ## calculates range of the CCD window
        rangemin_nm = round(float(self.currentMonoWavelengthLabel.text()) - 670*nm_per_px,1)
        rangemax_nm = round(float(self.currentMonoWavelengthLabel.text()) + 670*nm_per_px,1)
        Window.rangeLabelNM.setText('( '+str(rangemin_nm)+' , '+str(rangemax_nm)+' )')
        rangemin_wn = round(nmToWav(self.laserMenu.currentText(),rangemin_nm),1)
        rangemax_wn = round(nmToWav(self.laserMenu.currentText(),rangemax_nm),1)
        Window.rangeLabelWN.setText('( '+str(rangemin_wn)+' , '+str(rangemax_wn)+' )')

    def pathUpdate(self):
        ## sets path for filesaving
        global path
        path = os.path.join(self.currentDir.text())

    def liveAcquire(self,rate=None,go=False):
        ## sets framerate for live window and opens it
        global liveRate
        if rate==None:
            pass
        else:
            liveRate = rate
            LiveWindow.updateFramerate(self)
        if go:
            self.liveWindow = LiveWindow()
            self.liveWindow.setGeometry(QtC.QRect(400,300,1000,600))
            self.liveWindow.show()

    def calculateShift(self):
        ## handles live-updating shift calculator
        if self.shiftResponseInput.text()+self.shiftInputWN.text() == '..':
            self.absoluteShift.setText('')
            self.relativeShift.setText('')
            self.shiftWN.setText('')
        elif self.shiftInputWN.text() == '-':
            pass
        elif self.shiftInputWN.text()!='.' and self.shiftResponseInput.text()!='.':
            wav = round(nmToWav(self.shiftExcitationInput.text(),self.shiftResponseInput.text()),2)
            self.shiftWN.setText(str(wav))
            nm = round(wavToNM(self.shiftExcitationInput.text(),self.shiftInputWN.text()),2)
            self.absoluteShift.setText(str(nm))
            self.relativeShift.setText(str(round(nm - float(self.shiftExcitationInput.text()),2)))
        elif self.shiftInputWN.text()!='.':
            nm = round(wavToNM(self.shiftExcitationInput.text(),self.shiftInputWN.text()),2)
            self.absoluteShift.setText(str(nm))
            self.relativeShift.setText(str(round(nm - float(self.shiftExcitationInput.text()),2)))
            self.shiftWN.setText('')
        elif self.shiftResponseInput.text()!='.':
            wav = round(nmToWav(self.shiftExcitationInput.text(),self.shiftResponseInput.text()),2)
            self.shiftWN.setText(str(wav))
            self.absoluteShift.setText('')
            self.relativeShift.setText('')
        else:
            pass

    def testFunction(self):
        ## put whatever is desired in here
        print("test")

    def closeEvent(self,event):
        ## disconnects from instruments when X button is clicked
        Mono1.disconnect()
        print("Terminated connection with monochromator.")
        cam.close()
        print("Disconnected from camera.")
        event.accept()
        sys.exit(0)


class LiveWindow(QtW.QMainWindow):
    ## pop out window for live graphing
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Acquisition")
        
        global liveopen
        liveopen = True

        Window.camButton.setEnabled(False)
        Window.camButton2D.setEnabled(False)

        ## x-axis calculation
        global live_wavenum
        live_wavenum = []
        pos = float(Window.currentMonoWavelengthLabel.text())
        pxc = 670.5
        pixel = range(0,1340)
        for i in range(0,1340):
            wav = float(pos) - nm_per_px*(pixel[i]-pxc)
            wavNum = nmToWav(laser,wav)
            live_wavenum.append(wavNum)       
        
        global liveRate
        global livetime
        livetime = int(liveRate)
        self.updateFramerate()

        ## update loop for live image
        global liveTimer
        liveTimer = QtC.QTimer(self)
        liveTimer.start()
        liveTimer.setInterval(livetime)
        
        ## creates live plot
        global data_buffer
        global plot_graph
        global curve
        data_buffer = np.zeros((1340,10))
        plot_graph = pg.PlotWidget()
        curve = plot_graph.plot()
        self.setCentralWidget(plot_graph)
        cam.set_roi(vbin = 100)
        liveTimer.timeout.connect(lambda: self.update())

    def updateFramerate(self):
        ## updates framerate of the live window if it is open
        global liveRate
        rate = liveRate
        if liveopen:
            cam.set_attribute_value("Exposure Time", rate)
        global livetime
        livetime = int(rate)

    def update(self):
        ## take most recent data and plot
        global curve
        global data_buffer
        global livetime
        try:
            img = cam.snap(timeout = livetime)
        except:
            img = cam.snap(timeout = livetime)
        live_data = []
        live_data = img.reshape(-1)
        if not(Window.liveMedian.isChecked()):
            curve.setData(live_wavenum,live_data)
        else:
            data_buffer[:,9] = live_data
            data_buffer = np.roll(data_buffer,1,axis=1)
            median_data = np.median(data_buffer,axis=1)
            curve.setData(live_wavenum,median_data)
        app.processEvents()

    def closeEvent(self,event):
        global liveTimer
        liveTimer.stop()
        cam.stop_acquisition()
        cam.set_attribute_value("Exposure Time", 1000*float(Window.exposureTimeInput.text()))
        Window.camButton.setEnabled(True)
        Window.camButton2D.setEnabled(True)
        Window.liveStartButton.setEnabled(True)
        Window.liveStopButton.setEnabled(False)    
        global liveopen
        liveopen = False
        event.accept()

def main():
    global app
    app = QtW.QApplication(sys.argv)
    pixmap = QtG.QPixmap('icon.png')
    splash = QtW.QSplashScreen(pixmap)
    splash.show()
    setup()
    initialize()
    app.setWindowIcon(QtG.QIcon('icon.png'))
    splash.finish(Window)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()