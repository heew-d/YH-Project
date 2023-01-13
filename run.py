# Main Code

import cv2
from CVlib import Orchestrator
import asyncio
from bleak import BleakClient
import numpy as np
import os
import time

from keras.models import load_model

MAC_ADDRESS = "98:DA:60:03:B8:F8"
UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

state = 't'
flag_list = []

async def run(path_in, address):
    async with BleakClient(address) as client:
            await asyncio.sleep(20.0)
            print('Connected')
            services = await client.get_services()
            for service in services:
                for characteristic in service.characteristics:
                    if characteristic.uuid == UUID:
                        if 'write' in characteristic.properties:
                            global state

                            src = cv2.VideoCapture(path_in, cv2.CAP_DSHOW)

                            # src.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                            # src.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

                            # frame_size = (int(src.get(cv2.CAP_PROP_FRAME_WIDTH)),
                            #             int(src.get(cv2.CAP_PROP_FRAME_HEIGHT)))

                            # print('frame_size =', frame_size)

                            if src.isOpened() == False:
                                print('Unable to read Camera.')

                            model = load_model('keras_model.h5')
                            labels = open('labels.txt', 'r').readlines()
                            cnt = 0

                            while True:
                                ret, frame = src.read()
                                dst, flag = Orchestrator(frame)
                                
                                resized = cv2.resize(dst, (1280, 720))
                                cv2.imshow('Video streaming', resized)
                                # cv2.imshow('Video streaming', cv2.pyrDown(dst))

                                image = cv2.resize(frame, (224, 224), interpolation=cv2.INTER_AREA)
                                image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
                                image = (image / 127.5) - 1

                                probabilities = model.predict(image)
                                print(labels[np.argmax(probabilities)])
                                val = (labels[np.argmax(probabilities)])
                                intVal = int(val) 

                                if intVal == 1:
                                    cnt = cnt + 1
                                    if cnt == 3:
                                        if state != 't':
                                            await client.write_gatt_char(characteristic, bytes(b't'))
                                            print(' *** *** *** State: Stop *** *** *** ')
                                            state = 't'
                                            cnt = 0

                                    print('direct: ', state)

                                else:
                                    if flag != None:
                                        flag_list.append(flag)
                                        
                                    if len(flag_list) == 5:
                                        print("flag_list: ", flag_list)
                                        print('flag: ', flag)

                                        s = flag_list.count('Straight')
                                        e = flag_list.count('M_Right')
                                        q = flag_list.count('M_Left')
                                        d = flag_list.count('Back')
                                        r = flag_list.count('S_Right')
                                        l = flag_list.count('S_Left')

                                        if max(s, e, q, d, r, l) == s:
                                            flag = 'Straight'
                                            if state != 'w':
                                                await client.write_gatt_char(characteristic, bytes(b'w'))
                                                print(' *** *** *** State: Forward *** *** *** ')
                                                state = 'w'
                                                
                                        if max(s, e, q, d, r, l) == e:
                                            flag = 'M_Right'
                                            if state != 'e':
                                                await client.write_gatt_char(characteristic, bytes(b'e'))
                                                print(' *** *** *** State: M_Right *** *** *** ')
                                                state = 'e'
                                                
                                        if max(s, e, q, d, r, l) == q:
                                            flag = 'M_Left'
                                            if state != 'q':
                                                await client.write_gatt_char(characteristic, bytes(b'q'))
                                                print(' *** *** *** State: M_Left *** *** *** ')
                                                state = 'q'
                                                
                                        if max(s, e, q, d, r, l) == d:
                                            flag = 'Back'
                                            if state != 'd':
                                                await client.write_gatt_char(characteristic, bytes(b'd'))
                                                print(' *** *** *** State: Back *** *** *** ')
                                                state = 'd'

                                        if max(s, e, q, d, r, l) == r:
                                            flag = 'S_Right'
                                            if state != 'r':
                                                await client.write_gatt_char(characteristic, bytes(b'r'))
                                                print(' *** *** *** State: S_Right *** *** *** ')
                                                state = 'r'

                                        if max(s, e, q, d, r, l) == l:
                                            flag = 'S_Left'
                                            if state != 'l':
                                                await client.write_gatt_char(characteristic, bytes(b'l'))
                                                print(' *** *** *** State: S_Left *** *** *** ')
                                                state = 'l'

                                        flag_list.clear()

                                    # os.system('cls')

                                    if cv2.waitKey(1) == 27:
                                        break
            
                            src.release()
                            cv2.destroyAllWindows()

            print('Disconnected')

loop = asyncio.get_event_loop()
loop.run_until_complete(run(0, MAC_ADDRESS))
loop.close()