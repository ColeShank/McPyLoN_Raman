'''
Future Additions:
-# of acquisitions option

Known Issues:
-Homing doesn't always work, idk why :(

For the next editor:
-Paths are specific to nickp user and the raman system in the corner
-Only validated for 1800/mm grating, other gratings may require some edits

Note: Any changes to the code will require repackaging into an executable again.
'''

## Developed 2024 for the Reznik lab at the Univerisity of Colorado at Boulder by Cole Shank and Hope Whitelock
vers_no = "1.3"


## imports
import sys
import os
from datetime import date
import logging, configparser, time, serial
import datetime as dt
import PyQt5.QtGui as QtG
import PyQt5.QtCore as QtC
import PyQt5.QtWidgets as QtW
import pyqtgraph as pg
from tkinter.filedialog import askdirectory
import pylablib as pll
pll.par["devices/dlls/picam"] = "path/to/dlls"
from pylablib.devices import PrincetonInstruments
cam = PrincetonInstruments.PicamCamera()
import matplotlib
from matplotlib import pyplot as plt
import numpy as np
import ctypes
import webbrowser
myappid = 'reznik.ramanControl.tony.01' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
plt.ion()


global laser
global nm_per_px
global time_out
laser = 532 # pre-filled for 532 nm
nm_per_px = 0.012484 # pre-filled for 1800/mm grating
time_out = 1000 # 1 second

def initialize():
    config = configparser.RawConfigParser()
    config.read('mono.cfg')
    
    laserWL = config.get('Laser_settings', 'laser_wavelength')
    Window.currentLaserWavelengthInput.setText(str(laserWL))
    laserUpdate(float(laserWL))
    
    mono_system = config.get('Mono_settings', 'mono_system')
    systemUpdate(mono_system)
   
    grating = config.get('Mono_settings', 'current_grating')
    Window.gratingMenu.setCurrentIndex(Window.gratingMenu.findText(str(grating)))
    
    monoWL = config.get('Mono_settings', 'current_wavelength')
    Window.currentMonoWavelengthLabel.setText(str(monoWL))

    Window.exposureTimeInput.setText("0.1")
    cam.set_attribute_value("Exposure Time", 1000*float(Window.exposureTimeInput.text()))
    
    Window.live10msButton.setEnabled(True)
    Window.live100msButton.setEnabled(False)
    global liveRate
    liveRate = 100
    Window.live1sButton.setEnabled(True)

    global xmin, xmax, ymin, ymax
    xmin = 0
    xmax = 0
    ymin = 0
    ymax = 0

    Window.currentDir.setText('C:/')
    global path
    path = os.path.join(Window.currentDir.text())

def systemUpdate(sys):
    if sys == "single":
        Window.singleButton.setEnabled(False)
        Window.tripleSingleButton.setEnabled(True)
        Window.tripleButton.setEnabled(True)

    elif sys == "triple_single":
        Window.tripleSingleButton.setEnabled(False)
        Window.singleButton.setEnabled(True)
        Window.tripleButton.setEnabled(True)

    elif sys == "triple_triple":
        Window.tripleButton.setEnabled(False)
        Window.singleButton.setEnabled(True)
        Window.tripleSingleButton.setEnabled(True)

    config = configparser.RawConfigParser()
    config.read('mono.cfg')
    config.set('Mono_settings', 'mono_system', str(sys))
    f = open('mono.cfg','w')
    config.write(f)

def laserUpdate(wavl):
    ## updates laser wavelength
    global laser
    if wavl == "":
        laser = laser
    elif wavl == 0:
        laser = laser
    else:
        laser = float(wavl)
        config = configparser.RawConfigParser()
        config.read('mono.cfg')
        laserWL = config.set('Laser_settings', 'laser_wavelength', str(laser))
        f = open('mono.cfg',"w")
        config.write(f)

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

def imageCCD(pos, mode = "1D", cal = False):
    ## disable buttons
    Window.camButton.setEnabled(False)
    Window.camButton2D.setEnabled(False)
    Window.ramanButton.setEnabled(False)

    img = cam.snap(timeout = time_out)


    global data
    global wavelen
    global wavenum

    data = []
    wavelen = []
    wavenum = []
    signal = []
    
    ## calculate x-axes
    pxc = 670.5 # center pixel
    pixel = range(0,1340)
    for i in range(0,1340):
        wav = float(pos) - nm_per_px*(pixel[i]-pxc)
        wavNum = nmToWav(laser,wav)
        wavelen.append(wav)
        wavenum.append(wavNum)

    cam.set_attribute_value("Correct Pixel Bias", True)

    if mode == "1D":
        for i in range(0,1340): 
            signal = sum(img[:, i])
            data.append(signal)
        plt.plot(wavenum, data)
        plt.show()

    elif mode == "2D":
        ## This may or may not be the most sensible way to package the data into a 1D array...
        ## In MATLAB, this requires a reshape and a rotate to display the proper orientation, which doesn't seem the most ideal but I don't know how else to do it
        img1D = img.reshape(-1)
        data = img1D.tolist()
        wavelen = wavelen * 100
        wavenum = wavenum * 100
        plt.imshow(img,aspect='auto')
        
    autoSave(cal)
    
    ## re-enable buttons
    Window.camButton.setEnabled(True)
    Window.camButton2D.setEnabled(True)
    Window.ramanButton.setEnabled(True)

def takeSpectrum(start, stop):
    if start == '':
        Window.statusBar().showMessage("Error: Start value required",3000)
        Window.camButton.setEnabled(True)
        Window.camButton2D.setEnabled(True)
        Window.ramanButton.setEnabled(True)
    elif stop == '':
        Window.statusBar().showMessage("Error: Stop value required",3000)
        Window.camButton.setEnabled(True)
        Window.camButton2D.setEnabled(True)
        Window.ramanButton.setEnabled(True)
    else:
        ## pixel bias compensation leads to a weird background
        cam.set_attribute_value("Correct Pixel Bias", False)

        ## start by converting start and stop to nm
        nmStart = wavToNM(laser,start)
        nmStop = wavToNM(laser,stop)
        deltaL = 1340*nm_per_px #detector wavelength range
        numSnaps = int(np.ceil(abs(nmStart - nmStop)/deltaL)) #number of detector width spectra required
        Window.statusBar().showMessage("Estimated time needed: "+str((1000/60)*float(cam.cav["Exposure Time"])*(0.5*float(stop)/1000 + 0.5*numSnaps))+" minutes.",5000)
        time.sleep(5)
        Window.statusBar().showMessage("Pausing 60 seconds before data collection...", 3000)
        
        ## sleep for 1 minute to give time to leave the room
        time.sleep(60)

        global data
        global wavelen
        global wavenum
        data = []
        wavelen =[]
        wavenum = []
        signal = []

        for pos in np.linspace(float(nmStart - (0.5*deltaL) + deltaL*(numSnaps-0.5)), float(nmStart), numSnaps):
            
            ## position is detector center
            pos = pos + (0.5*deltaL)
            Mono1.approachWL(float(pos))

            img = cam.snap(timeout = time_out)

            pxc = 670.5 # center pixel
            pixel = range(0,1340)
            for i in range(0,1340):
                wav = pos - nm_per_px*(pixel[i]-pxc)
                wavNum = nmToWav(laser,wav)
                signal = sum(img[:, i])
                wavelen.append(wav)
                wavenum.append(wavNum)
                data.append(signal)

        autoSave()
        print('Spectrum complete.')
        Window.camButton.setEnabled(True)
        Window.camButton2D.setEnabled(True)
        Window.ramanButton.setEnabled(True)
        plt.plot(wavenum, data)
        plt.show()

def autoSave(cal=False):
    now = dt.datetime.now()
    now_str = now.strftime('%y%m%d%H%M%S')

    if cal == True:
        tempname = 'Cal_'+now_str
        autopath = os.path.join('C:/Users/nickp/Documents/Raman calibration files',str(tempname))

    elif cal == False:
        tempname = 'Raman_'+now_str
        autopath = os.path.join('C:/Users/nickp/Documents/Raman data files',str(tempname))

    file = open(autopath,'w')
    for line in np.arange(len(data)):
        stringToWrite = str(wavelen[line])+','+str(wavenum[line])+','+str(data[line])+'\n' 
        file.write(stringToWrite)

    file.close()
    autopath_txt = os.path.join(autopath+'.txt')
    os.rename(autopath,autopath_txt)

def saveData(fname):
    fpath = os.path.join(path,fname)
    bool = os.path.isfile(os.path.join(path,fname+'.txt'))
    if bool == True:
        Window.statusBar().showMessage("Error: file already exists", 3000)
    else:
        file = open(fpath,'w')
        file.write('Wavelength(nm), Raman shift(cm^-1), Intensity(arb) \n')
        for line in np.arange(len(data)):
            stringToWrite = str(wavelen[line])+','+str(wavenum[line])+','+str(data[line])+'\n' 
            file.write(stringToWrite)

        file.close()
        fpath_txt = os.path.join(path,fname+'.txt')
        os.rename(fpath,fpath_txt)

class Monochromator(object):
    ## Initializes a serial port
    def __init__(self):
	
        self.config = configparser.RawConfigParser()
        self.config.read('mono.cfg')
        self.comport = self.config.get('Mono_settings', 'com_port')
        self.mono = serial.Serial(self.comport, timeout=1, baudrate=9600, xonxoff=1, stopbits=1)

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
        #logging.debug('Send command: ' + command)
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
        
        ## This doesn't work reliably for some reason
        
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
            print("Homing done, setting current wavelength now to 440.067 nm (verify counter: 660.1)")
            f = open('mono.cfg',"w")
            self.config.write(f)
            Window.currentMonoWavelengthLabel.setText("440.067 nm")
		
    def approachWL(self, approach_wavelength):
        Window.approachButton.setEnabled(False)
        if isinstance(approach_wavelength, float):
            print("Wavelength to approach: " + str(approach_wavelength) + " nm")
            nm_difference = float(approach_wavelength) - float(self.current_wavelength)
            print("Difference in nm: " + str(nm_difference))
            step_difference = round(((float(nm_difference) / float(self.nm_per_revolution)) * float(self.steps_per_revolution))+ float(self.offset))
            print("Difference in steps: " + str(step_difference))
            time_needed_sec = round(abs(step_difference / int(self.speed)) + abs(int(self.offset)/int(self.approach_speed)),2)
            print("Time needed for operation: " + str(time_needed_sec) + " s")
            Window.statusBar().showMessage("Moving monochromator . . .  (est. "+str(time_needed_sec)+" seconds)",3000)
            time_delay_for_progressbar = time_needed_sec / 100
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
                    Window.approachButton.setEnabled(True)
                    Window.progressBar.setValue(0)
                    self.config.set('Mono_settings', 'current_wavelength', approach_wavelength)
                    self.current_wavelength = approach_wavelength
                    Window.currentMonoWavelengthLabel.setText(str(round(self.current_wavelength,2)) + " nm")
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

        self.setWindowTitle("Raman Control")
        tab_widget = QtW.QTabWidget()
        tab1 = QtW.QWidget()
        tab2 = QtW.QWidget()
        tab3 = QtW.QWidget()
        tab4 = QtW.QWidget()
        tab5 = QtW.QWidget()
        tab6 = QtW.QWidget()
        p1_vertical = QtW.QFormLayout(tab1)
        p2_vertical = QtW.QFormLayout(tab2)
        p3_vertical = QtW.QFormLayout(tab3)
        p4_vertical = QtW.QFormLayout(tab4)
        p5_vertical = QtW.QFormLayout(tab5)
        p6_vertical = QtW.QFormLayout(tab6)
        tab_widget.addTab(tab1, "Spectrometer Control")
        tab_widget.addTab(tab2, "CCD Control")
        tab_widget.addTab(tab3, "Live Acquisition")
        tab_widget.addTab(tab4, "File Saving")
        tab_widget.addTab(tab5, "Shift Calculator")
        tab_widget.addTab(tab6, "About")      
        self.setCentralWidget(tab_widget)
        
        ## update loop for CCD temperature
        self.update_timer = QtC.QTimer(self)
        self.update_timer.start()
        self.update_timer.setInterval(1000) # milliseconds
        self.update_timer.setSingleShot(False)
        self.update_timer.timeout.connect(lambda: self.camTempLabel.setText(str(cam.get_attribute_value('Sensor Temperature Reading')) + " C"))
        
        ### TAB 1
        self.systemHeader = QtW.QLabel(self)
        self.systemHeader.setText("System Settings")
        self.systemHeader.setStyleSheet("font-weight: bold")
        
        self.currentLaserWavelengthInput = QtW.QLineEdit(self)
        self.currentLaserWavelengthInput.setMaxLength(3)
        self.currentLaserWavelengthInput.setInputMask("999")
        self.currentLaserWavelengthInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.currentLaserWavelengthInput.textChanged.connect(lambda: laserUpdate(self.currentLaserWavelengthInput.text()))
        self.currentLaserWavelengthInput.textChanged.connect(lambda: self.shiftExcitationInput.setText(self.currentLaserWavelengthInput.text()))

        self.gratingMenu = QtW.QComboBox(self)
        self.gratingMenu.setEditable(False)
        self.gratingMenu.addItems(["50","150","300","600","1200","1800","2400","3600"])
        self.gratingMenu.currentIndexChanged.connect(lambda: self.updateGrating())

        self.calHeader = QtW.QLabel(self)
        self.calHeader.setText("Calibrate Monochromator Position")
        self.calHeader.setStyleSheet("font-weight: bold")

        self.currentCounterInput = QtW.QLineEdit(self)
        self.currentCounterInput.setMaxLength(5)
        self.currentCounterInput.setInputMask("99999")
        self.currentCounterInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        self.calButton = QtW.QPushButton(self)
        self.calButton.setObjectName("calButton")
        self.calButton.clicked.connect(lambda: self.calibrateMono())
        self.calButton.setText("Calibrate monochromator position")

        self.currentMonoWavelengthLabel = QtW.QLabel(self)
        self.currentMonoWavelengthLabel.setAlignment(QtC.Qt.AlignRight)

        self.moveHeader = QtW.QLabel(self)
        self.moveHeader.setText("Move Monochromator Position")
        self.moveHeader.setStyleSheet("font-weight: bold")

        self.approachWavelengthInput = QtW.QLineEdit(self)
        self.approachWavelengthInput.setMaxLength(5)
        self.approachWavelengthInput.setInputMask("999.9")
        self.approachWavelengthInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.approachWavelengthInput.textChanged.emit(self.approachWavelengthInput.text())

        self.approachButton = QtW.QPushButton(self)
        self.approachButton.setObjectName("approachButton")
        self.approachButton.clicked.connect(lambda: Mono1.approachWL(float(self.approachWavelengthInput.text())))
        self.approachButton.setText("Approach")

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
        p1_vertical.addRow("Current Laser Wavelength (nm)", self.currentLaserWavelengthInput)
        p1_vertical.addRow("Grating (G/mm)", self.gratingMenu)
        p1_vertical.addRow(self.calHeader)
        p1_vertical.addRow("Calibration, enter current counter setting", self.currentCounterInput)
        p1_vertical.addRow(self.calButton)
        p1_vertical.addRow("Current Mono Wavelength (nm)", self.currentMonoWavelengthLabel)
        p1_vertical.addRow(self.moveHeader)
        p1_vertical.addRow("Approach Mono Wavelength", self.approachWavelengthInput)
        p1_vertical.addRow(self.progressBar, self.approachButton)
        #p1_vertical.addRow("Home counter location: 660.1", self.homeButton) # Doesn't work RIP

        ### TAB 2
        self.ccdHeader = QtW.QLabel(self)
        self.ccdHeader.setText("CCD Settings")
        self.ccdHeader.setStyleSheet("font-weight: bold")

        self.camTempLabel = QtW.QLabel(self)
        self.camTempLabel.setAlignment(QtC.Qt.AlignRight)
        self.camTempLabel.setText(str(cam.get_attribute_value('Sensor Temperature Reading')) + " C")
        
        self.ccdCalButton = QtW.QPushButton(self)
        self.ccdCalButton.setObjectName("ccdCalButton")
        self.ccdCalButton.clicked.connect(lambda: self.calibrateCCD())
        self.ccdCalButton.setText("Calibrate")

        self.exposureTimeInput = QtW.QLineEdit(self)
        self.exposureTimeInput.setMaxLength(7)
        self.exposureTimeInput.setInputMask("9999.99")
        self.exposureTimeInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.exposureTimeInput.textChanged.connect(lambda: cam.set_attribute_value("Exposure Time", 1000*float(self.exposureTimeInput.text())))
        self.exposureTimeInput.textChanged.connect(lambda: setTimeout(5000+float(self.exposureTimeInput.text())))

        self.snapshotHeader = QtW.QLabel(self)
        self.snapshotHeader.setText("Single Spectrum")
        self.snapshotHeader.setStyleSheet("font-weight: bold")

        self.camButton = QtW.QPushButton(self)
        self.camButton.setObjectName("camButton")
        self.camButton.clicked.connect(lambda: self.camButton.setEnabled(False))
        self.camButton.clicked.connect(lambda: self.camButton2D.setEnabled(False))
        self.camButton.clicked.connect(lambda: self.ramanButton.setEnabled(False))
        self.camButton.clicked.connect(lambda: imageCCD(float(Mono1.current_wavelength), "1D"))
        self.camButton.setText("Take 1D spectrum")

        self.camButton2D = QtW.QPushButton(self)
        self.camButton2D.setObjectName("camButton2D")
        self.camButton2D.clicked.connect(lambda: self.camButton.setEnabled(False))
        self.camButton2D.clicked.connect(lambda: self.camButton2D.setEnabled(False))
        self.camButton2D.clicked.connect(lambda: self.ramanButton.setEnabled(False))
        self.camButton2D.clicked.connect(lambda: imageCCD(float(Mono1.current_wavelength), "2D"))
        self.camButton2D.setText("Take 2D image")
        
        self.ramanHeader = QtW.QLabel(self)
        self.ramanHeader.setText("Wideband Raman Spectrum")
        self.ramanHeader.setStyleSheet("font-weight: bold")
        
        self.startInput = QtW.QLineEdit(self)
        self.startInput.setMaxLength(4)
        self.startInput.setInputMask("9999")
        self.startInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.startInput.textChanged.emit(self.startInput.text())

        self.stopInput = QtW.QLineEdit(self)
        self.stopInput.setMaxLength(4)
        self.stopInput.setInputMask("9999")
        self.stopInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.stopInput.textChanged.emit(self.stopInput.text())

        self.ramanButton = QtW.QPushButton(self)
        self.ramanButton.setObjectName("ramanButton")
        self.ramanButton.clicked.connect(lambda: self.camButton.setEnabled(False))
        self.ramanButton.clicked.connect(lambda: self.camButton2D.setEnabled(False))
        self.ramanButton.clicked.connect(lambda: self.ramanButton.setEnabled(False))
        self.ramanButton.clicked.connect(lambda: takeSpectrum(self.startInput.text(), self.stopInput.text()))
        self.ramanButton.setText("Take Raman spectrum")

        p2_vertical.addRow(self.ccdHeader)
        p2_vertical.addRow("Current temp", self.camTempLabel)
        p2_vertical.addRow("Calibrate CCD", self.ccdCalButton)
        p2_vertical.addRow("Exposure time (s)", self.exposureTimeInput)
        p2_vertical.addRow(self.snapshotHeader)
        p2_vertical.addRow("Take 1D snapshot", self.camButton)
        p2_vertical.addRow("Take 2D snapshot", self.camButton2D)
        p2_vertical.addRow(self.ramanHeader)
        p2_vertical.addRow("Scan Start (1/cm)", self.startInput)
        p2_vertical.addRow("Scan Stop (1/cm)", self.stopInput)
        p2_vertical.addRow(self.ramanButton)

        ### TAB 3
        
        self.liveStartStopHeader = QtW.QLabel(self)
        self.liveStartStopHeader.setText("Live Acquisition")
        self.liveStartStopHeader.setStyleSheet("font-weight: bold")
        
        self.liveStartButton = QtW.QPushButton(self)
        self.liveStartButton.setText("Start Acquisition")
        self.liveStartButton.clicked.connect(lambda: self.liveAcquire(rate=None,go=True))
        self.liveStartButton.clicked.connect(lambda: self.liveStartButton.setEnabled(False))
        self.liveStartButton.clicked.connect(lambda: self.liveStopButton.setEnabled(True))
        
        self.liveStopButton = QtW.QPushButton(self)
        self.liveStopButton.setText("Stop Acquisition")
        self.liveStopButton.setEnabled(False)
        self.liveStopButton.clicked.connect(lambda: self.liveWindow.close())
        self.liveStopButton.clicked.connect(lambda: self.liveStartButton.setEnabled(True))
        self.liveStopButton.clicked.connect(lambda: self.liveStopButton.setEnabled(False))
        
        self.liveRateHeader = QtW.QLabel(self)
        self.liveRateHeader.setText("Framerate/Exposure Controls")
        self.liveRateHeader.setStyleSheet("font-weight: bold")
        
        self.live10msButton = QtW.QPushButton(self)
        self.live10msButton.setText("10 ms")
        self.live10msButton.clicked.connect(lambda: self.liveAcquire(10))
        self.live10msButton.clicked.connect(lambda: self.live10msButton.setEnabled(False))
        self.live10msButton.clicked.connect(lambda: self.live100msButton.setEnabled(True))
        self.live10msButton.clicked.connect(lambda: self.live1sButton.setEnabled(True))
        
        self.live100msButton = QtW.QPushButton(self)
        self.live100msButton.setText("0.1 s")
        self.live100msButton.clicked.connect(lambda: self.liveAcquire(100))
        self.live100msButton.clicked.connect(lambda: self.live10msButton.setEnabled(True))
        self.live100msButton.clicked.connect(lambda: self.live100msButton.setEnabled(False))
        self.live100msButton.clicked.connect(lambda: self.live1sButton.setEnabled(True))
        
        self.live1sButton = QtW.QPushButton(self)
        self.live1sButton.setText("1 s")
        self.live1sButton.clicked.connect(lambda: self.liveAcquire(1000))
        self.live1sButton.clicked.connect(lambda: self.live10msButton.setEnabled(True))
        self.live1sButton.clicked.connect(lambda: self.live100msButton.setEnabled(True))
        self.live1sButton.clicked.connect(lambda: self.live1sButton.setEnabled(False))
        
        self.liveViewHeader = QtW.QLabel(self)
        self.liveViewHeader.setText("View Controls")
        self.liveViewHeader.setStyleSheet("font-weight: bold")

        self.xminLabel = QtW.QLabel(self)
        self.xminLabel.setText("xmin")        
        self.xminInput = QtW.QLineEdit(self)
        self.xminInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.xminInput.textChanged.connect(lambda: LiveWindow.updateLimits(self))

        self.xmaxLabel = QtW.QLabel(self)
        self.xmaxLabel.setText("xmax")        
        self.xmaxInput = QtW.QLineEdit(self)
        self.xmaxInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.xmaxInput.textChanged.connect(lambda: LiveWindow.updateLimits(self))

        self.yminLabel = QtW.QLabel(self)
        self.yminLabel.setText("ymin")
        self.yminInput = QtW.QLineEdit(self)
        self.yminInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.yminInput.textChanged.connect(lambda: LiveWindow.updateLimits(self))

        self.ymaxLabel = QtW.QLabel(self)
        self.ymaxLabel.setText("ymax")
        self.ymaxInput = QtW.QLineEdit(self)
        self.ymaxInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.ymaxInput.textChanged.connect(lambda: LiveWindow.updateLimits(self))

        self.clearLimitsButton = QtW.QPushButton(self)
        self.clearLimitsButton.setText("Clear Limits")
        self.clearLimitsButton.clicked.connect(lambda: LiveWindow.autorange(self))
        
        p3_vertical.addRow(self.liveStartStopHeader)
        p3_vertical.addRow(self.liveStartButton)
        p3_vertical.addRow(self.liveStopButton)        
        p3_vertical.addRow(self.liveRateHeader)
        p3_vertical.addRow(self.live10msButton)
        p3_vertical.addRow(self.live100msButton)
        p3_vertical.addRow(self.live1sButton)
        p3_vertical.addRow(self.liveViewHeader)
        p3_vertical.addRow(self.xminLabel, self.xminInput)
        p3_vertical.addRow(self.xmaxLabel, self.xmaxInput)
        p3_vertical.addRow(self.yminLabel, self.yminInput)
        p3_vertical.addRow(self.ymaxLabel, self.ymaxInput)
        p3_vertical.addRow(self.clearLimitsButton)
   
        ### TAB 4
        self.currentDir = QtW.QLabel(self)
        self.currentDir.setAlignment(QtC.Qt.AlignRight)
        
        self.dirButton = QtW.QPushButton(self)
        self.dirButton.setObjectName("dirButton")
        self.dirButton.clicked.connect(lambda: self.currentDir.setText(askdirectory(title='Select folder')))
        self.dirButton.clicked.connect(lambda: self.pathUpdate())
        self.dirButton.setText("...")
     
        self.fname = QtW.QLineEdit(self)
        self.fname.setMaxLength(50)
        self.fname.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.fname.textChanged.emit(self.fname.text())

        self.saveButton = QtW.QPushButton(self)
        self.saveButton.setObjectName("saveButton")
        self.saveButton.clicked.connect(lambda: saveData(self.fname.text()))
        self.saveButton.setText("Save most recent data")
        
        p4_vertical.addRow("Active folder", self.currentDir)
        p4_vertical.addRow("Select new folder", self.dirButton)
        p4_vertical.addRow("File name", self.fname)
        p4_vertical.addRow(self.saveButton)
     
        ### TAB 5     
        self.shiftExcitationInput = QtW.QLineEdit(self)
        self.shiftExcitationInput.setMaxLength(3)
        self.shiftExcitationInput.setInputMask("999")
        self.shiftExcitationInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        self.shiftWNHeader = QtW.QLabel(self)
        self.shiftWNHeader.setText("Enter Wavelength")
        self.shiftWNHeader.setStyleSheet("font-weight: bold")
        
        self.shiftResponseInput = QtW.QLineEdit(self)
        self.shiftResponseInput.setMaxLength(6)
        self.shiftResponseInput.setInputMask("999.99")
        self.shiftResponseInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.shiftResponseInput.textChanged.connect(lambda: self.calculateShift())

        self.shiftWN = QtW.QLabel(self)

        self.shiftNMHeader = QtW.QLabel(self)
        self.shiftNMHeader.setText("Or, Enter Shift")
        self.shiftNMHeader.setStyleSheet("font-weight: bold")
        
        self.shiftInputWN = QtW.QLineEdit(self)
        self.shiftInputWN.setObjectName("shiftInputWN")
        self.shiftInputWN.setMaxLength(4)
        self.shiftInputWN.setInputMask("#9999")
        self.shiftInputWN.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.shiftInputWN.textChanged.connect(lambda: self.calculateShift())
        
        self.absoluteShift = QtW.QLabel(self)
        self.absoluteShift.setObjectName("absoluteShift")
        
        self.relativeShift = QtW.QLabel(self)
        self.relativeShift.setObjectName("relativeShift")

        p5_vertical.addRow("Excitation Wavelength (nm)", self.shiftExcitationInput)
        p5_vertical.addRow(self.shiftWNHeader)
        p5_vertical.addRow("Response Wavelength (nm)", self.shiftResponseInput)
        p5_vertical.addRow("Raman Shift (1/cm)", self.shiftWN)
        p5_vertical.addRow(self.shiftNMHeader)
        p5_vertical.addRow("Raman shift (1/cm)", self.shiftInputWN)
        p5_vertical.addRow("Absolute wavelength (nm)", self.absoluteShift)
        p5_vertical.addRow("Relative wavelength (nm)", self.relativeShift)
        
        ### TAB 6
        self.versionHeader = QtW.QLabel(self)
        self.versionHeader.setText("Code Version")
        self.versionHeader.setStyleSheet("font-weight: bold")
        
        self.versionLabel = QtW.QLabel(self)
        self.versionLabel.setObjectName("versionLabel")
        self.versionLabel.setText(str(vers_no))
        self.versionLabel.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
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
        
        p6_vertical.addRow(self.versionHeader)
        p6_vertical.addRow(self.versionLabel)
        p6_vertical.addRow(self.monoSelectHeader)
        p6_vertical.addRow("", self.singleButton)
        p6_vertical.addRow("", self.tripleSingleButton)
        p6_vertical.addRow("", self.tripleButton)
        p6_vertical.addRow(self.linksHeader)
        p6_vertical.addRow("GitHub Repository", self.gitButton)
        p6_vertical.addRow("Configuration file", self.cfgButton)
        p6_vertical.addRow("Documentation", self.docButton)

    def calibrateMono(self):
        ## updates the monochromator position in the program based on the counter
        if self.currentCounterInput.text() == '.':
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
        
        las = float(self.currentLaserWavelengthInput.text())
        
        ## place laser on high end of detector range and take image
        Mono1.approachWL(las+3)
        img1 = cam.snap()
        signal1 = []
        pixel = range(1340)
        for i in range(1340):
            signal1.append(sum(img1[:, i]))
        max1 = max(signal1)
        px1 = signal1.index(max1)
        
        ## place laser on low end of detector range and take image
        Mono1.approachWL(las-3)
        img2 = cam.snap()
        signal2 = []
        pixel = range(1340)
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
        calWL = float(las-3 + nm_offset)
        print("pixel offset: " + str(px_offset))
        print("nm offset: " + str(nm_offset))
        Mono1.approachWL(calWL)
        
        ## update configuration file
        config = configparser.RawConfigParser()
        config.read('mono.cfg')
        config.set('Mono_settings', 'current_wavelength', str(las))
        f = open('mono.cfg',"w")
        config.write(f)
        self.currentMonoWavelengthLabel.setText(str(las))
        Mono1.current_wavelength = str(las)
        
        ## take and save image to verify calibration
        imageCCD(las, "1D", cal = True)

    def updateGrating(self):
        grating = self.gratingMenu.currentText()
        global nm_per_revolution
        nm_per_revolution = str(32*float(150)/float(grating))
        self.config = configparser.RawConfigParser()
        self.config.read('mono.cfg')
        self.config.set('Mono_settings', 'current_grating', grating)
        self.config.set('Mono_settings', 'nm_per_revolution', nm_per_revolution)
        f = open('mono.cfg',"w")
        self.config.write(f)

    def pathUpdate(self):
        global path
        path = os.path.join(self.currentDir.text())

    def liveAcquire(self,rate=None,go=False):
        global liveRate
        if rate==None:
            liveRate = liveRate
        else:
            LiveWindow.updateFramerate(self,rate)

        if go:
            self.liveWindow = LiveWindow()
            self.liveWindow.setGeometry(QtC.QRect(400,300,1000,600))
            self.liveWindow.show()

    def calculateShift(self):
        if self.shiftResponseInput.text()+self.shiftInputWN.text() == '.':
            self.statusBar().showMessage("Error: must enter at least one input",2000)
            
        elif self.shiftInputWN.text() == '-':
            pass

        elif self.shiftResponseInput.text() == '.':
            nm = round(wavToNM(self.shiftExcitationInput.text(),self.shiftInputWN.text()),2)
            self.absoluteShift.setText(str(nm))
            self.relativeShift.setText(str(round(nm - float(self.shiftExcitationInput.text()),2)))
            
        elif self.shiftInputWN.text() == '':
            wav = round(nmToWav(self.shiftExcitationInput.text(),self.shiftResponseInput.text()),2)
            self.shiftWN.setText(str(wav))
        
        else:       
            wav = round(nmToWav(self.shiftExcitationInput.text(),self.shiftResponseInput.text()),2)
            self.shiftWN.setText(str(wav))
            nm = round(wavToNM(self.shiftExcitationInput.text(),self.shiftInputWN.text()),2)
            self.absoluteShift.setText(str(nm))
            self.relativeShift.setText(str(round(nm - float(self.shiftExcitationInput.text()),2)))

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
        rate = liveRate
        global livetime
        livetime = int(rate)
        self.updateFramerate(rate)

        ## update loop for live image
        global liveTimer
        liveTimer = QtC.QTimer(self)
        liveTimer.start()
        liveTimer.setInterval(livetime)
        
        global plot_graph
        plot_graph = pg.PlotWidget()
        global curve
        curve = plot_graph.plot()
        self.setCentralWidget(plot_graph)
        cam.set_roi(vbin = 100)
        liveTimer.timeout.connect(lambda: self.update())

    def updateFramerate(self,rate):
        global liveRate
        if rate == liveRate:
            pass
        else:
            cam.set_attribute_value("Exposure Time", rate)
            global livetime
            livetime = int(rate)
            liveRate = rate

    def autorange(self):
        global plot_graph
        Window.xminInput.clear()
        Window.xmaxInput.clear()
        Window.yminInput.clear()
        Window.ymaxInput.clear()
        try:
            plot_graph
        except:
            pass
        else: # Doesn't work RIP
            plot_graph.autoRange(item=curve)
            plot_graph.enableAutoRange()
            
    def updateLimits(self):
        global xmin, xmax, ymin, ymax, plot_graph
        if Window.xminInput.text()+Window.xmaxInput.text()+Window.yminInput.text()+Window.ymaxInput.text()=="":
            xmin = 0
            xmax = 0
            ymin = 0
            ymax = 0
        else:
            if Window.xminInput.text()=='':
                pass
            elif xmin == float(Window.xminInput.text()):
                pass
            else:
                xmin = float(Window.xminInput.text())
                plot_graph.setLimits(xMin=xmin)

            if Window.xmaxInput.text()=='':
                pass
            elif xmax == float(Window.xmaxInput.text()):
                pass
            else:
                xmax = float(Window.xmaxInput.text())
                plot_graph.setLimits(xMax=xmax)            
            if Window.yminInput.text()=='':
                pass
            elif ymin == float(Window.yminInput.text()):
                pass
            else:
                ymin = float(Window.yminInput.text())
                plot_graph.setLimits(yMin=ymin)
                        
            if Window.ymaxInput.text()=='':
                pass
            elif ymax == float(Window.ymaxInput.text()):
                pass
            else:
                ymax = float(Window.ymaxInput.text())
                plot_graph.setLimits(yMax=ymax)

    def update(self):
        ## take most recent data and plot
        global livetime
        img = cam.snap(timeout = livetime)

        live_data = []
        live_data = img.reshape(-1)

        global curve
        curve.setData(live_wavenum,live_data)
        
        QtG.QGuiApplication.processEvents()

    def closeEvent(self,event):
        global liveTimer
        liveTimer.stop()
        cam.stop_acquisition()
        cam.set_attribute_value("Exposure Time", 1000*float(Window.exposureTimeInput.text()))
        Window.liveStartButton.setEnabled(True)
        Window.liveStopButton.setEnabled(False)        
        event.accept()

def main():        
    app = QtW.QApplication(sys.argv)
    pixmap = QtG.QPixmap('icon.png')
    splash = QtW.QSplashScreen(pixmap)
    splash.show()
    print("Connecting to monochromator ...")
    global Mono1
    Mono1 = Monochromator()
    Mono1.sendcommand(' ')  
    app.processEvents()
    global Window
    Window = MainWindow()
    initialize()
    Window.resize(550,400)
    Window.show()
    app.setWindowIcon(QtG.QIcon('icon.png'))
    splash.finish(Window)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()