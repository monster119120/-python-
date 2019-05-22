# -- coding: utf-8 --
# 得到手机摄像头图像
# 还未进行拉伸矫正



import os
# os.system("adb shell am start com.meizu.media.camera/com.meizu.media.camera.CameraLauncher")
def get_phone_pic():
    os.system('adb shell input tap 532 1780')
    os.system('adb shell input tap 532 1000')
    os.system("adb shell screencap -p /sdcard/screen.png")
    os.system('adb shell input tap 80 136')
    os.system("adb pull /sdcard/screen.png C:\Users\monster\Desktop")