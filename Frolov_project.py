import tkinter as tk
from tkinter import messagebox, filedialog
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage, ImageTk
import threading
from datetime import datetime
import math
import json
import csv

#инициация ноды
rospy.init_node('flight_control_gui')

#подключение сервисов
get_tlm = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
nav_glo = rospy.ServiceProxy("navigate_global", srv.NavigateGlobal)
lnd = rospy.ServiceProxy("land", Trigger)
nav_loc = rospy.ServiceProxy("navigate", srv.Navigate)

#
bridge = CvBridge()
fullbody_cascade = cv2.CascadeClassifier('haarcascade_fullbody.xml')

latest_img = None
lock = threading.Lock()
video_writer = None
running_mode = None


start = get_tlm()
home = [start.lat, start.lon]

#Окно
window = tk.Tk()
window.title("Пульт управления дроном")
window.geometry("1080x550")
window.resizable(False, False)

#Функция ожидания
def ar_wait(tolerance=0.5):
    while not rospy.is_shutdown():
        tlm = get_tlm(frame_id="navigate_target")
        if math.sqrt(tlm.x**2+tlm.y**2+tlm.z**2) < tolerance:# 
            break
        rospy.sleep(0.2)

#  Взлет
def vzlet():
    try:
        if not get_tlm("navigate_target").armed:#Проверка запущены двигатели или нет
            threading.Thread(target=nav_loc, args=(0, 0, 2, 0, 0, "body", True)).start()
            ar_wait()
            info_label.config(text="Взлетел", fg="green")
        else:
            info_label.config(text="В воздухе", fg="red")
            
    except Exception as e:#ветвление для диагностики ошибок
        info_label.config(text=f"Ошибка: {e}", fg="red")

# Посадка
def posadka():
    try:
        if get_tlm("navigate_target").armed:
            threading.Thread(target=lnd).start()
            info_label.config(text="Приземлился", fg="green")
        else:
            info_label.config(text="Уже на земле", fg="red")
    except:
        info_label.config(text="Ошибка", fg="red")

# Показ телеметрии
def show_tlm():
    tlm = get_tlm()
    info_label.config(text=f"Телеметрия: X: {tlm.x}, Y:{tlm.y}, Z:{tlm.z}, широта={tlm.lat}, долгота {tlm.lon}", fg="black")

# Вперед
def fly_vpered_button():
    if get_tlm("navigate_target").armed:
        threading.Thread(target=nav_loc, args=(1, 0, 0, 0, 1, "body",False)).start()
        ar_wait()
        info_label.config(text="Вперед на 1м", fg="green")
    else: 
        info_label.config(text="Двигатель не запущен", fg="red")

# Влево
def fly_vlevo_button():
    if get_tlm("navigate_target").armed:
        threading.Thread(target=nav_loc, args=(0, 1, 0, 0, 1, "body",False)).start()
        ar_wait()
        info_label.config(text="Влево на 1м", fg="green")
    else: 
        info_label.config(text="Двигатель не запущен", fg="red")

# Вправо
def fly_vpravo_button():
    if get_tlm("navigate_target").armed:
        threading.Thread(target=nav_loc, args=(0, -1, 0, 0, 1, "body",False)).start()
        ar_wait()
        info_label.config(text="Вправо на 1м", fg="green")
    else: 
        info_label.config(text="Двигатель не запущен", fg="red")
        
# Назад
def fly_nazad_button():
    if get_tlm("navigate_target").armed:
        threading.Thread(target=nav_loc, args=(-1, 0, 0, 0, 1, "body",False)).start()
        ar_wait()
        info_label.config(text="Назад на 1м", fg="green")
    else: 
        info_label.config(text="Двигатель не запущен", fg="red")
# Полет домой
def fly_home():
    global home
    if home[0] is None or home[1] is None:  
        info_label.config(text="Ошибка: нет координат взлета", fg="red")
        return
    if get_tlm("navigate_target").armed:
        lat, lon = home[0], home[1]
        threading.Thread(target=nav_glo, args=(lat, lon, 2, 0, 1, "map", False)).start()
        info_label.config(text="Летим домой",fg="green")
        ar_wait()
        info_label.config(text="Мы дома", fg="green")
    else:
        info_label.config(text="Нужно взлететь", fg="red")

# Получение изображения с камеры
def camera_img(msg):
    global latest_img
    with lock:
        latest_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
img_sub = rospy.Subscriber('main_camera/image_raw', Image, camera_img, queue_size=1) #подписка на топик камеры

# Обновление изображения в интерфейсе
def update_img():
    if latest_img is not None:
        with lock:
            img_rgb = cv2.cvtColor(latest_img, cv2.COLOR_BGR2RGB) 
            img_pil = PILImage.fromarray(img_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)

            camera_label.config(image=img_tk)
            camera_label.image = img_tk
    
    window.after(100, update_img)

# Полет по локальным координатам
def fly_local():
    try:
        if get_tlm("navigate_target").armed:
            if entry_x.get():
                x = float(entry_x.get())
            else:
                x = 0
            if entry_y.get():
                y = float(entry_y.get())
            else:
                y = 0
            if entry_z.get():
                z = float(entry_z.get())
            else:
                if z != 0:
                    z = float(0)
            threading.Thread(target=nav_loc, args=(x, y, z, 1, 0, "body", False)).start()
            ar_wait()
            info_label.config(text=f"Дрон на точке Х: {x}, Y: {y} и на высоте Z: {z}",fg="green")
        else:
            info_label.config(text="Сначала взлетите ", fg="red")
    except :
        info_label.config(text=f"Ошибка, введите числовые значения x,y,z",fg="red")

# Полет по глобальным координатам
def fly_global():
    try:
        if get_tlm("navigate_target").armed:
            lat = float(entry_lat.get())
            lon = float(entry_lon.get())
            z = float(entry_z.get())
            if entry_z.get():
                z = float(entry_z.get())
            else:
                z = 2
            threading.Thread(target=nav_glo, args=(lat, lon, z, 1, 0, "body", False)).start()
            ar_wait()
            info_label.config(text=f"Дрон на точки широта: {lat}, долгота: {lon} и на высоте Z: {z}", fg="green")
        else:
            info_label.config(text="Сначала взлетите ", fg="red")
    except:
        info_label.config(text="Ошибка, введите числовые значения для широты, долготы и высоты", fg="red")

#Поля ввода и текст
info_label = tk.Label(window, text="Информационная панель", fg="black")
info_label.grid(row=7, column=0, columnspan=6,padx=20)
camera_label = tk.Label(window)
camera_label.grid(row=0, column=1, rowspan=5,columnspan=5, pady=20)

tk.Label(window, text="X (м): ").grid(row=0, column=6, pady=10)
entry_x = tk.Entry(window, width=10)
entry_x.grid(row=0, column=7)

tk.Label(window, text=" Y (м): ").grid(row=1, column=6 )
entry_y = tk.Entry(window, width=10)
entry_y.grid(row=1, column=7)

tk.Label(window, text="Z (м): ").grid(row=2, column=6)
entry_z = tk.Entry(window, width=10)
entry_z.grid(row=2, column=7)

tk.Label(window, text="Широта: ").grid(row=3, column=6)
entry_lat = tk.Entry(window, width=10)
entry_lat.grid(row=3, column=7)

tk.Label(window, text="Долгота: ").grid(row=4, column=6)
entry_lon = tk.Entry(window, width=10)
entry_lon.grid(row=4, column=7)

#кнопки
vzlet_button = tk.Button(window, text="Взлет", width=20, bg="green", fg="white", relief='sunken', activebackground="silver",command=vzlet).grid(row=0, column=0, padx=20, pady=10)
posadka_button = tk.Button(window, text="Посадка", width=20, bg="red", fg="white", relief='sunken', activebackground="silver",command=posadka).grid(row=1, column=0, padx=20 )
vpered_button = tk.Button(window, text="Вперед", width=20, bg="blue", fg="white", relief='sunken', activebackground="silver", command=fly_vpered_button).grid(row=8, column=2 )
vlevo_button = tk.Button(window, text="Влево", width=20, bg="blue", fg="white", relief='sunken', activebackground="silver",command=fly_vlevo_button).grid(row=9, column=1 )
vpavo_button = tk.Button(window, text="Вправо", width=20, bg="blue", fg="white", relief='sunken', activebackground="silver",command=fly_vpravo_button).grid(row=9, column=3 )
nazad_button = tk.Button(window, text="Назад", width=20, bg="blue", fg="white", relief='sunken', activebackground="silver",command=fly_nazad_button).grid(row=9, column=2 )
tlm_button = tk.Button(window, text="Телеметрия", width=20, relief="solid", activebackground="silver",command=show_tlm).grid(row=9, column=0, padx=20)
fly_home_button = tk.Button(window, text="Полет домой", width=20, relief="solid", activebackground="silver",command=fly_home).grid(row=8, column=0, padx=20)
fly_loc_button = tk.Button(window, text="Полет по лк.координатам", width=20, relief="solid", activebackground="silver",command=fly_local).grid(row=3, column=0)
fly_glo_button = tk.Button(window, text="Полет по гл.координатам", width=20, relief="solid", activebackground="silver", command=fly_global).grid(row=4, column=0)
image_flag='flag.jpg'
image_pil=PILImage.open(image_flag)
image_pil=image_pil.resize((20,10))
image_tk=ImageTk.PhotoImage(image_pil)
tk.Label(window,image=image_tk).place(x=25, y=30)#левый флаг на кнопке взлет
tk.Label(window,image=image_tk).place(x=180, y=30)#правый флаг на кнопке взлет
tk.Label(window,image=image_tk).place(x=25, y=95)#левый флаг на кнопке взлет
tk.Label(window,image=image_tk).place(x=180, y=95)#правый флаг на кнопке взлет
tk.Label(window,image=image_tk).place(x=25, y=313)#левый флаг на кнопке взлет
tk.Label(window,image=image_tk).place(x=180, y=313)#правый флаг на кнопке взлет
window.after(50, update_img)
window.mainloop()