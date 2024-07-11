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
vers_no = 1.2


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
        cam.set_roi(vbin=100)
        cam.setup_acquisition(mode="snap",nframes=10)
        cam.start_acquisition()
        cam.wait_for_frame()
        img = cam.snap(timeout = time_out)

        data = img.reshape(-1)
        
        plt.plot(wavenum, data)
        plt.show()
    
    elif mode == "2D":
        cam.set_roi(vbin=1)
        cam.setup_acquisition(mode="snap",nframes=10)
        cam.start_acquisition()
        cam.wait_for_frame()
        img = cam.snap(timeout = time_out)
        
        data = img.reshape(-1)
        plt.imshow(np.flip(img,1),aspect='auto')
        
        wavelen = wavelen * 100
        wavenum = wavenum * 100
        
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
    
    Window.liveFastButton.setEnabled(False)
    Window.liveSlowButton.setEnabled(True)
    
    Window.currentDir.setText('C:/')
    global path
    path = os.path.join(Window.currentDir.text())


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

        tab_widget = QtW.QTabWidget()
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
        tab_widget.addTab(tab1, "Spectrometer Control")
        tab_widget.addTab(tab2, "CCD Control")
        tab_widget.addTab(tab3, "File Saving")
        tab_widget.addTab(tab4, "Shift Calculator")
        tab_widget.addTab(tab5, "About")
        
        
        ## update loop for CCD temperature
        self.update_timer = QtC.QTimer(self)
        self.update_timer.start()
        self.update_timer.setInterval(1000) # milliseconds
        self.update_timer.setSingleShot(False)
        self.update_timer.timeout.connect(lambda: self.camTempLabel.setText(str(cam.get_attribute_value('Sensor Temperature Reading')) + " C"))
        
        
        ## create header for system settings
        self.systemHeader = QtW.QLabel(self)
        self.systemHeader.setText("System Settings")
        self.systemHeader.setStyleSheet("font-weight: bold")
        
        ## create input field for current laser wavelength
        self.currentLaserWavelengthInput = QtW.QLineEdit(self)
        self.currentLaserWavelengthInput.setMaxLength(3)
        self.currentLaserWavelengthInput.setInputMask("999")
        self.currentLaserWavelengthInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.currentLaserWavelengthInput.textChanged.connect(lambda: laserUpdate(self.currentLaserWavelengthInput.text()))
        self.currentLaserWavelengthInput.textChanged.connect(lambda: self.shiftExcitationInput.setText(self.currentLaserWavelengthInput.text()))

        ## create dropdown menu for grating selection
        self.gratingMenu = QtW.QComboBox(self)
        self.gratingMenu.setEditable(False)
        self.gratingMenu.addItems(["50","150","300","600","1200","1800","2400","3600"])
        self.gratingMenu.currentIndexChanged.connect(lambda: self.updateGrating())

        ## create header for calibration
        self.calHeader = QtW.QLabel(self)
        self.calHeader.setText("Calibrate Monochromator Position")
        self.calHeader.setStyleSheet("font-weight: bold")

        ## create input field for counter
        self.currentCounterInput = QtW.QLineEdit(self)
        self.currentCounterInput.setMaxLength(5)
        self.currentCounterInput.setInputMask("99999")
        self.currentCounterInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        ## create button to calibrate mono
        self.calButton = QtW.QPushButton(self)
        self.calButton.setObjectName("calButton")
        self.calButton.clicked.connect(lambda: self.calibrateMono())
        self.calButton.setText("Calibrate monochromator position")

        ## create label for current mono wavelength
        self.currentMonoWavelengthLabel = QtW.QLabel(self)
        self.currentMonoWavelengthLabel.setAlignment(QtC.Qt.AlignRight)

        ## create header for moving mono
        self.moveHeader = QtW.QLabel(self)
        self.moveHeader.setText("Move Monochromator Position")
        self.moveHeader.setStyleSheet("font-weight: bold")

        ## create input field for wavelength to approach
        self.approachWavelengthInput = QtW.QLineEdit(self)
        self.approachWavelengthInput.setMaxLength(5)
        self.approachWavelengthInput.setInputMask("999.9")
        self.approachWavelengthInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.approachWavelengthInput.textChanged.emit(self.approachWavelengthInput.text())

        ## create button to start the mono movement
        self.approachButton = QtW.QPushButton(self)
        self.approachButton.setObjectName("approachButton")
        self.approachButton.clicked.connect(lambda: Mono1.approachWL(float(self.approachWavelengthInput.text())))
        self.approachButton.setText("Approach")

		## create progress bar for mono movement progress indication
        self.progressBar = QtW.QProgressBar(self)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setMaximum(101)
		
        ## create button for mono homing procedure
        '''
            Broken currently, left for posterity
        '''
        self.homeButton = QtW.QPushButton(self)
        self.homeButton.setObjectName("homeButton")
        self.homeButton.clicked.connect(lambda: Mono1.getHomePosition())
        self.homeButton.clicked.connect(lambda: self.statusBar().showMessage("Homing monochromator . . . ",10000))
        self.homeButton.setText("HOME Monochromator")


        ## create header for CCD settings
        self.ccdHeader = QtW.QLabel(self)
        self.ccdHeader.setText("CCD Settings")
        self.ccdHeader.setStyleSheet("font-weight: bold")

        ## create label for current camera temp
        self.camTempLabel = QtW.QLabel(self)
        self.camTempLabel.setAlignment(QtC.Qt.AlignRight)
        self.camTempLabel.setText(str(cam.get_attribute_value('Sensor Temperature Reading')) + " C")
        
        ## create button for CCD calibration
        self.ccdCalButton = QtW.QPushButton(self)
        self.ccdCalButton.setObjectName("ccdCalButton")
        self.ccdCalButton.clicked.connect(lambda: self.calibrateCCD())
        self.ccdCalButton.setText("Calibrate")

        ## create exposure time input
        self.exposureTimeInput = QtW.QLineEdit(self)
        self.exposureTimeInput.setMaxLength(7)
        self.exposureTimeInput.setInputMask("9999.99")
        self.exposureTimeInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.exposureTimeInput.textChanged.connect(lambda: cam.set_attribute_value("Exposure Time", 1000*float(self.exposureTimeInput.text())))
        self.exposureTimeInput.textChanged.connect(lambda: setTimeout(5000+float(self.exposureTimeInput.text())))

        ## create header for snapshot
        self.snapshotHeader = QtW.QLabel(self)
        self.snapshotHeader.setText("Single Spectrum")
        self.snapshotHeader.setStyleSheet("font-weight: bold")

        ## create picture button
        self.camButton = QtW.QPushButton(self)
        self.camButton.setObjectName("camButton")
        self.camButton.clicked.connect(lambda: self.camButton.setEnabled(False))
        self.camButton.clicked.connect(lambda: self.camButton2D.setEnabled(False))
        self.camButton.clicked.connect(lambda: self.ramanButton.setEnabled(False))
        self.camButton.clicked.connect(lambda: imageCCD(float(Mono1.current_wavelength), "1D"))
        self.camButton.setText("Take 1D spectrum")

        ## create 2D button
        self.camButton2D = QtW.QPushButton(self)
        self.camButton2D.setObjectName("camButton2D")
        self.camButton2D.clicked.connect(lambda: self.camButton.setEnabled(False))
        self.camButton2D.clicked.connect(lambda: self.camButton2D.setEnabled(False))
        self.camButton2D.clicked.connect(lambda: self.ramanButton.setEnabled(False))
        self.camButton2D.clicked.connect(lambda: imageCCD(float(Mono1.current_wavelength), "2D"))
        self.camButton2D.setText("Take 2D image")
        
        ## create header for Raman
        self.ramanHeader = QtW.QLabel(self)
        self.ramanHeader.setText("Wideband Raman Spectrum")
        self.ramanHeader.setStyleSheet("font-weight: bold")
        
        ## create start input
        self.startInput = QtW.QLineEdit(self)
        self.startInput.setMaxLength(4)
        self.startInput.setInputMask("9999")
        self.startInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.startInput.textChanged.emit(self.startInput.text())

        ## create stop input
        self.stopInput = QtW.QLineEdit(self)
        self.stopInput.setMaxLength(4)
        self.stopInput.setInputMask("9999")
        self.stopInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.stopInput.textChanged.emit(self.stopInput.text())

        ## create take spectrum button
        self.ramanButton = QtW.QPushButton(self)
        self.ramanButton.setObjectName("ramanButton")
        self.ramanButton.clicked.connect(lambda: self.camButton.setEnabled(False))
        self.ramanButton.clicked.connect(lambda: self.camButton2D.setEnabled(False))
        self.ramanButton.clicked.connect(lambda: self.ramanButton.setEnabled(False))
        self.ramanButton.clicked.connect(lambda: takeSpectrum(self.startInput.text(), self.stopInput.text()))
        self.ramanButton.setText("Take Raman spectrum")
        
        ## create header for live window
        self.liveHeader = QtW.QLabel(self)
        self.liveHeader.setText("Live Acquisition")
        self.liveHeader.setStyleSheet("font-weight: bold")

        ## create button for quick exposure
        self.liveFastButton = QtW.QPushButton(self)
        self.liveFastButton.setText("0.1 s")
        self.liveFastButton.clicked.connect(lambda: LiveWindow.updateFramerate(self))
        self.liveFastButton.clicked.connect(lambda: self.liveSlowButton.setEnabled(True))
        self.liveFastButton.clicked.connect(lambda: self.liveFastButton.setEnabled(False))
        
        ## create button for long exposure
        self.liveSlowButton = QtW.QPushButton(self)
        self.liveSlowButton.setText("1 s")
        self.liveSlowButton.clicked.connect(lambda: LiveWindow.updateFramerate(self))
        self.liveSlowButton.clicked.connect(lambda: self.liveFastButton.setEnabled(True))
        self.liveSlowButton.clicked.connect(lambda: self.liveSlowButton.setEnabled(False))
        
        ## create button for 1D live mode
        self.live1DButton = QtW.QPushButton(self)
        self.live1DButton.setText("Live spectrum acquisition")
        self.live1DButton.clicked.connect(lambda: self.liveAcquire("1D"))
        
        ## create button for 2D live mode
        self.live2DButton = QtW.QPushButton(self)
        self.live2DButton.setText("Live video acquisition")
        self.live2DButton.clicked.connect(lambda: self.liveAcquire("2D"))
                
        
        ## create label for current directory
        self.currentDir = QtW.QLabel(self)
        self.currentDir.setAlignment(QtC.Qt.AlignRight)
        
        ## create directory button
        self.dirButton = QtW.QPushButton(self)
        self.dirButton.setObjectName("dirButton")
        self.dirButton.clicked.connect(lambda: self.currentDir.setText(askdirectory(title='Select folder')))
        self.dirButton.clicked.connect(lambda: self.pathUpdate())
        self.dirButton.setText("...")
     
        ## create file name input
        self.fname = QtW.QLineEdit(self)
        self.fname.setMaxLength(50)
        self.fname.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.fname.textChanged.emit(self.fname.text())

        ## create save file button
        self.saveButton = QtW.QPushButton(self)
        self.saveButton.setObjectName("saveButton")
        self.saveButton.clicked.connect(lambda: saveData(self.fname.text()))
        self.saveButton.setText("Save most recent data")
        
     
        ## create excitation wavelength input
        self.shiftExcitationInput = QtW.QLineEdit(self)
        self.shiftExcitationInput.setMaxLength(3)
        self.shiftExcitationInput.setInputMask("999")
        self.shiftExcitationInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        ## create header for wavenumber shift
        self.shiftWNHeader = QtW.QLabel(self)
        self.shiftWNHeader.setText("Enter Wavelength")
        self.shiftWNHeader.setStyleSheet("font-weight: bold")
        
        ## create response wavelength input
        self.shiftResponseInput = QtW.QLineEdit(self)
        self.shiftResponseInput.setMaxLength(6)
        self.shiftResponseInput.setInputMask("999.99")
        self.shiftResponseInput.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.shiftResponseInput.textChanged.connect(lambda: self.calculateShift())

        ## create shift wavenumber label
        self.shiftWN = QtW.QLabel(self)

        ## create header for wavelength shift
        self.shiftNMHeader = QtW.QLabel(self)
        self.shiftNMHeader.setText("Or, Enter Shift")
        self.shiftNMHeader.setStyleSheet("font-weight: bold")
        
        ## create shift input
        self.shiftInputWN = QtW.QLineEdit(self)
        self.shiftInputWN.setObjectName("shiftInputWN")
        self.shiftInputWN.setMaxLength(4)
        self.shiftInputWN.setInputMask("9999")
        self.shiftInputWN.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        self.shiftInputWN.textChanged.connect(lambda: self.calculateShift())
        
        ## create absolute nm label
        self.absoluteShift = QtW.QLabel(self)
        self.absoluteShift.setObjectName("absoluteShift")
        
        ## create relative nm label
        self.relativeShift = QtW.QLabel(self)
        self.relativeShift.setObjectName("relativeShift")
        

        ## create header for version
        self.versionHeader = QtW.QLabel(self)
        self.versionHeader.setText("Code Version")
        self.versionHeader.setStyleSheet("font-weight: bold")
        
        ## create version label
        self.versionLabel = QtW.QLabel(self)
        self.versionLabel.setObjectName("versionLabel")
        self.versionLabel.setText(str(vers_no))
        self.versionLabel.setAlignment(QtC.Qt.AlignRight|QtC.Qt.AlignTrailing|QtC.Qt.AlignVCenter)
        
        ## create header for system selection
        self.monoSelectHeader = QtW.QLabel(self)
        self.monoSelectHeader.setText("Raman System")
        self.monoSelectHeader.setStyleSheet("font-weight: bold")

        ## create button for single system
        self.singleButton = QtW.QPushButton(self)
        self.singleButton.setText("Single Monochromator System")
        self.singleButton.clicked.connect(lambda: systemUpdate("single"))

        ## create button for triple system in single configuration
        self.tripleSingleButton = QtW.QPushButton(self)
        self.tripleSingleButton.setText("Triple Monochromator System in Single Mode")
        self.tripleSingleButton.clicked.connect(lambda: systemUpdate("triple_single"))

        ## create button for triple system in triple configuration
        self.tripleButton = QtW.QPushButton(self)
        self.tripleButton.setText("Triple Monochromator System in Triple Mode")
        self.tripleButton.clicked.connect(lambda: systemUpdate("triple_triple"))

        ## create header for links
        self.linksHeader = QtW.QLabel(self)
        self.linksHeader.setText("Useful Links")
        self.linksHeader.setStyleSheet("font-weight: bold")

        ## create button to open GitHub repository
        self.gitButton = QtW.QPushButton(self)
        self.gitButton.setObjectName("gitButton")
        self.gitButton.setText("Open Link")
        self.gitButton.clicked.connect(lambda: webbrowser.open('github.com/ColeShank/McPyLoN_Raman'))

        ## create button to open configuration file
        self.cfgButton = QtW.QPushButton(self)
        self.cfgButton.setObjectName("cfgButton")
        self.cfgButton.setText("Open mono.cfg")
        self.cfgButton.clicked.connect(lambda: webbrowser.open('mono.cfg'))

        ## create button to open manual
        self.docButton = QtW.QPushButton(self)
        self.docButton.setObjectName("docButton")
        self.docButton.setText("Open Software Manual")
        self.docButton.clicked.connect(lambda: webbrowser.open('Software Manual.pdf'))
        
        
        ## put widgets into the QFormLayout of tab1
        p1_vertical.addRow(self.systemHeader)
        p1_vertical.addRow("Current Laser Wavelength (nm)   ", self.currentLaserWavelengthInput)
        p1_vertical.addRow("Grating (G/mm)   ", self.gratingMenu)
        p1_vertical.addRow(self.calHeader)
        p1_vertical.addRow("Calibration, enter current counter setting   ", self.currentCounterInput)
        p1_vertical.addRow(self.calButton)
        p1_vertical.addRow("Current Mono Wavelength   ", self.currentMonoWavelengthLabel)
        p1_vertical.addRow(self.moveHeader)
        p1_vertical.addRow("Approach Mono Wavelength   ", self.approachWavelengthInput)
        p1_vertical.addRow(self.progressBar, self.approachButton)
        #p1_vertical.addRow("Home counter location: 660.1", self.homeButton) # Doesn't work RIP

        ## put widgets into the QFormLayout of tab2
        p2_vertical.addRow(self.ccdHeader)
        p2_vertical.addRow("Current temp   ", self.camTempLabel)
        p2_vertical.addRow("Calibrate CCD   ", self.ccdCalButton)
        p2_vertical.addRow("Exposure time (s)   ", self.exposureTimeInput)
        p2_vertical.addRow(self.liveHeader)
        p2_vertical.addRow("Timing", self.liveFastButton)
        p2_vertical.addRow("", self.liveSlowButton)
        p2_vertical.addRow("Live acquisition", self.live1DButton)
        #p2_vertical.addRow(self.live2DButton) # Doesn't work RIP
        p2_vertical.addRow(self.snapshotHeader)
        p2_vertical.addRow("Take 1D snapshot   ", self.camButton)
        p2_vertical.addRow("Take 2D snapshot   ", self.camButton2D)
        p2_vertical.addRow(self.ramanHeader)
        p2_vertical.addRow("Scan Start (1/cm)   ", self.startInput)
        p2_vertical.addRow("Scan Stop (1/cm)   ", self.stopInput)
        p2_vertical.addRow(self.ramanButton)

        ## put widgets into the QFormLayout of tab3
        p3_vertical.addRow("Active folder   ", self.currentDir)
        p3_vertical.addRow("Select new folder   ", self.dirButton)
        p3_vertical.addRow("File name   ", self.fname)
        p3_vertical.addRow(self.saveButton)
        
        ## put widgets into the QFormLayout of tab4
        p4_vertical.addRow("Excitation Wavelength (nm)   ", self.shiftExcitationInput)
        p4_vertical.addRow(self.shiftWNHeader)
        p4_vertical.addRow("Response Wavelength (nm)   ", self.shiftResponseInput)
        p4_vertical.addRow("Raman Shift (1/cm)   ", self.shiftWN)
        p4_vertical.addRow(self.shiftNMHeader)
        p4_vertical.addRow("Raman shift (1/cm)   ", self.shiftInputWN)
        p4_vertical.addRow("Absolute wavelength (nm)   ", self.absoluteShift)
        p4_vertical.addRow("Relative wavelength (nm)   ", self.relativeShift)
        
        ## put widgets into the QFormLayout of tab5
        p5_vertical.addRow(self.versionHeader)
        p5_vertical.addRow(self.versionLabel)
        p5_vertical.addRow(self.monoSelectHeader)
        p5_vertical.addRow("", self.singleButton)
        p5_vertical.addRow("", self.tripleSingleButton)
        p5_vertical.addRow("", self.tripleButton)
        p5_vertical.addRow(self.linksHeader)
        p5_vertical.addRow("GitHub Repository   ", self.gitButton)
        p5_vertical.addRow("Configuration file   ", self.cfgButton)
        p5_vertical.addRow("Documentation   ", self.docButton)
        

        ## set window title and add tab widget to main window
        self.setWindowTitle("Raman Control")
        self.setCentralWidget(tab_widget)

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

    def liveAcquire(self,mode):
        global livemode
        if mode == "1D":
            livemode = "1D"
        elif mode == "2D":
            livemode = "2D"
        self.liveWindow = LiveWindow()
        self.liveWindow.setGeometry(QtC.QRect(400,300,1000,600))
        self.liveWindow.show()

    def calculateShift(self):
        if self.shiftResponseInput.text()+self.shiftInputWN.text() == '.':
            self.statusBar().showMessage("Error: must enter at least one input",2000)
            
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
        QtW.QWidget.__init__(self)
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

        self.updateFramerate(start=True)

        ## update loop for live image
        global liveTimer
        liveTimer = QtC.QTimer(self)
        liveTimer.start()
        liveTimer.setInterval(livetime)
        
        ## live video or spectrum
        global livemode
        if  livemode == "1D":
            self.plot_graph = pg.PlotWidget()
            global curve
            curve = self.plot_graph.plot()
            self.setCentralWidget(self.plot_graph)
            cam.set_roi(vbin = 100)
            liveTimer.timeout.connect(lambda: self.update1D())
        elif livemode == "2D":  ## currently non-functional
            global image
            image = pg.RawImageWidget()
            self.setCentralWidget(image)
            cam.set_roi(vbin = 1)
            liveTimer.timeout.connect(lambda: self.update2D())
        
    def updateFramerate(self, start=False):
        ## This whole start boolean business is awful but makes it work correctly for some reason
        global livetime
        if start == True:
            if not(Window.liveFastButton.isEnabled()):
                cam.set_attribute_value('Exposure Time', 100)
                livetime = 100
            elif not(Window.liveSlowButton.isEnabled()):
                cam.set_attribute_value('Exposure Time', 1000)
                livetime = 1000
            else:
                cam.set_attribute_value('Exposure Time', 100)
                livetime = 100
        else:
            if Window.liveFastButton.isEnabled():
                cam.set_attribute_value('Exposure Time', 100)
                livetime = 100
            elif Window.liveSlowButton.isEnabled():
                cam.set_attribute_value('Exposure Time', 1000)
                livetime = 1000
            else:
                cam.set_attribute_value('Exposure Time', 100)
                livetime = 100

    def update1D(self):
        ## take most recent data and plot
        global livetime
        img = cam.snap(timeout = livetime)

        live_data = []
        live_data = img.reshape(-1)

        global curve
        curve.setData(live_wavenum,live_data)
        
        QtG.QGuiApplication.processEvents()

    def update2D(self):
        ## take most recent data and plot
        global livetime
        img = cam.snap(timeout = livetime)
        img = np.rot90(img)
        
        global image
        image.setImage(img)
        
        QtG.QGuiApplication.processEvents()

    def closeEvent(self,event):
        global liveTimer
        liveTimer.stop()
        cam.stop_acquisition()
        cam.set_attribute_value("Exposure Time", 1000*float(Window.exposureTimeInput.text()))
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
    Window.resize(450,350)
    Window.show()
    #Window.initialize()
    app.setWindowIcon(QtG.QIcon('icon.png'))
    splash.finish(Window)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()