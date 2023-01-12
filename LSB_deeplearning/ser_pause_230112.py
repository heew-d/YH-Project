#필요한 모듈
import cv2                          #OPENCV
import numpy as np                  #넘파이
from keras.models import load_model #딥러닝한 모델사용
import serial                       #아두이노와 통신


def pause_stop():                               #함수로지정

    #아두이노 포트값과 전송속도 입력                        
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    # 학습시킨 모델을 불러온다(keras_model.h5)
    model = load_model('keras_model.h5')
    #opencv의 videocapture 기능사용, 0번은 웹캠
    camera = cv2.VideoCapture(0)
    # 텍스트 파일에 있는 것을 토대로 라벨을 만든다
    labels = open('labels.txt', 'r').readlines()
    #cnt라는 변수생성
    cnt = 0

    #while 문 안에 반복할 동작 작성
    while True:

        #웹캠이미지를 읽어온다
        ret, image = camera.read()
        #224x224 사이즈로 리사이즈, 축소시 디테일이 사라질 수 있으므로 interpolation=cv2.INTER_AREA 사용
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
        #화면으로 송출
        cv2.imshow('Webcam Image', image)
        # 이미지 배열을 넘파이로 처리하고 다시 만들어 다루기 쉽게 변형시킴        
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
        # 1은 1장의 이미지를 의미, 224개의 픽셀을 3채널로 구조시킨 1개의 이미지
        # 이미지 배열을 일반화 시키기
        # 이미지의 각 픽셀값은 RGB를 나타낼때 사용되는 0~255 사이의 정수값으로 표현된다.
        # 그 정수 값이 아래식을 통하면 -1부터 ~ 1까지의 값을 얻을 수 있다.   
        image = (image / 127.5) - 1
        # 모델이 현재이미지를 예측하도록 한다 Model.predict
        # 백분율로 배열을 반환시킨다. Ex) [0.2,0.8]
        # 그중 높은것을 채택함
        probabilities = model.predict(image)
        # 가장 높은 확률로 예측한 모델값 송출
        print(labels[np.argmax(probabilities)])
        # val의 값은 str으로 들어옴
        val = (labels[np.argmax(probabilities)])
        #사용하기 용이하게 int로 바꿈
        intVal = int(val)
        #intVal의 값이 1이 나오면 cnt에 1씩 더한다
        if intVal == 1:         
            cnt = cnt+1
            print(cnt)
            #cnt가 해당값(25)에 달하면 t\n 을 아두이노에게 시리얼통신한다
            #Stop~!!! 이라고 송출후 cnt를 0으로 초기화한다
            if cnt == 25:
                val = 't\n'
                ser.write((val+'\n').encode('utf-8'))
                print("Stop~!!!")
                cnt = 0
        #나머지는 cnt를 0으로 하고 w\n 을 아두이노에게 보낸다
        #Go!! 라고 송출한다
        else:
            cnt = 0
            val = 'w\n'
            ser.write((val+'\n').encode('utf-8'))
            print("Go!!")
        #키보드의 입력을 대기
        keyboard_input = cv2.waitKey(33)
        # 27번인 esc 가 눌리면 종료
        if keyboard_input == 27:
            break
    #카메라 종료
    camera.release()
    #창 종료
    cv2.destroyAllWindows()
#지정한 함수 실행
pause_stop()