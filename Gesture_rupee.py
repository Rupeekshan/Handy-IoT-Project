import serial #Serial imported for Serial communication
import time #Required to use delay functions
import pyautogui   #Required to to perform actions

ArduinoSerial = serial.Serial('com5',9600) #Create Serial port object called arduinoSerialData
time.sleep(2) #wait for 2 seconds for the communication to get established

while 1:
    incoming = str (ArduinoSerial.readline()) #read the serial data and print it as line
    print incoming
    
    if 'Play/Pause' in incoming:
        pyautogui.typewrite(['space'], 0.2)

    if 'Rewind 3sec' in incoming:
        pyautogui.hotkey('shift', 'left')

    if 'Rewind 10sec' in incoming:
        pyautogui.hotkey('alt', 'left')

    if 'Rewind 1min' in incoming:
        pyautogui.hotkey('ctrl', 'left')  

    if 'Forward 3sec' in incoming:
        pyautogui.hotkey('shift', 'right')

    if 'Forward 10sec' in incoming:
        pyautogui.hotkey('alt', 'right')

    if 'Forward 1min' in incoming:
        pyautogui.hotkey('ctrl', 'right') 

    if 'Volume Increased' in incoming:
        pyautogui.hotkey('ctrl', 'up')
        
    if 'Volume Decreased' in incoming:
        pyautogui.hotkey('ctrl', 'down')

    incoming = "";
