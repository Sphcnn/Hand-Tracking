import cv2
import time
import numpy as np
import HandTrackingModule as htm
import math
import screen_brightness_control as sbc
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from PIL import ImageGrab
import pygetwindow as pg
import uuid




################
wCam, hCam = 1280, 720
################

cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)
pTime = 0

detector = htm.handDetector(detectionCon=0.7)

devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(
    IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
# volume.GetMute()
# volume.GetMasterVolumeLevel()
volRange = volume.GetVolumeRange()
minVol = volRange[0]
maxVol = volRange[1]
vol = 0
volBar = 400
volPer = 0
minBrightness = 0
maxBrightness = 100
last_image_taken = time.time()

def screen_shot(save_path="./"):
    global last_image_taken

    if time.time() - last_image_taken < 10:
        return
    
    screenshot = ImageGrab.grab()
    screenshot.save(save_path + str(uuid.uuid4()) + ".png")
    print("Ekran Görüntüsü Alındı.")
    last_image_taken = time.time()




def set_brightness(brightness):

    if brightness < minBrightness or brightness > maxBrightness:
        raise ValueError("Parlaklık değeri 0 ile 100 arasında olmalıdır.")
    
    sbc.set_brightness(int(brightness))


"""def adjust_brightness(image, brightness=1.0):
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv[:, :, 2] = hsv[:, :, 2] * brightness
    hsv[:, :, 2][hsv[:, :, 2] > 255] = 255
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)"""


while True:
    success, img = cap.read()
    if not success:
        continue

    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img, draw=False)

    if len(lmList) >= 12:  # lmList'in yeterli uzunlukta olup olmadığını kontrol edin
        
    
        
        x1, y1 = lmList[4][1],lmList[4][2]
        x2, y2 = lmList[8][1],lmList[8][2]
        x3,y3 = lmList[20][1], lmList[20][2]
        x4,y4=lmList[9][1],lmList[9][2]
        cx, cy = (x1+x2)//2 , (y1+y2)//2     
        cx1,cx2=(x1+x3)//2, (y1+y3)//2
        cv2.circle(img,(x1,y1),7,(255,150,150),cv2.FILLED)
        cv2.circle(img,(x2,y2),7,(255,255,255),cv2.FILLED)
        cv2.circle(img,(x3,y3),7,(255,0,0),cv2.FILLED)
        cv2.line(img, (x1,y1),(x2,y2),(255,2555,255),thickness=3)
        cv2.circle(img,(cx,cy),7 ,(255,255,255),cv2.FILLED)
        cv2.line(img, (x1,y1),(x3,y3),(255,2555,255),thickness=3)

        
        lengthVol = math.hypot(x2-x1,y2-y1)
        lengthBright = math.hypot(x3-x1,y3-y1)

        vol = np.interp(lengthVol,[50,200],[minVol,maxVol])
        volume.SetMasterVolumeLevel(vol,None)
        
        lengthsscreenshot1= math.hypot(x4-x1,y4-y1)
        lengthsscreenshot2= math.hypot(x4-x2,y4-y2)
        lengthsscreenshot3= math.hypot(x4-x3,y4-y3)

        brightness = np.interp(lengthBright, [50, 200], [0,100])  
        set_brightness(int(brightness))
        #img = adjust_brightness(img, brightness)


        if lengthVol<50:
           cv2.circle(img,(cx,cy), 7,(0,0,0),cv2.FILLED)

        if lengthBright< 60:
            cv2.circle(img,(cx1,cx2),7,(0,0,0),cv2.FILLED)
        
        if lengthsscreenshot1<100 and lengthsscreenshot2 < 100 and lengthsscreenshot3 <100:
            screen_shot()
        
  
    else:
        print("Yeterli veri sağlanamadı:", len(lmList))
        time.sleep(0)
    
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, f"FPS: {int(fps)}", (40, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 3)

    cv2.imshow("Img", img)
    cv2.waitKey(1)
