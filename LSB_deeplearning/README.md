# 인공지능 & 딥러닝
- Keras Model을 사용하여 Arduino에 적용
    
    - Arduino의 Servo를 사용
    - Webcam을 통해 들어오는 정보가 Keras 모델을 거쳐, 물체의 변화에 따른 학습된 값을 송출하는 것을 확인
    - 그 값에 따라 Servo의 Angle 값 변화 확인

- Webcam을 통해 학습한 물체가 바뀔때마다 다른값을 보내는 것을 확인

 ![servo_webc_1](https://user-images.githubusercontent.com/109050683/212274572-699648e0-3955-4a42-8216-99b506c413df.gif)

- 값에 따라 Servo의 Angle값이 바뀌는 것을 확인

![servo_webc_2](https://user-images.githubusercontent.com/109050683/212274960-7ffd2371-5db8-4960-8de7-2c93b289a07d.gif)

- 값에 따라 Mobility가 물체를 보고 정지 또는 일시정지하고, 물체가 사라지면 다시 출발하는 코드 작성


# 파일 설명
- 1.mp4, 2.mp4
    * 딥러닝을 통한 아두이노 서보모터 동작 영상

- converted_keras를 제외한 압축파일
    * 딥러닝 모델을 얻기위한 사진 샘플들

- 각 jpg 파일들
    * 동일 압축파일에 들어있는 사진샘플중 1장

- converted_keras 압축파일
    * 학습시킨 모델(0번 wall은 박스를 제외한 나머지,
    1번 obj는 웹캠 박스)

- ser_pause_230112.py
    * 코드내에 설명 주석처리함

    