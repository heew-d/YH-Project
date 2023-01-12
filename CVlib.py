# Import libraries
import numpy as np  # 다차원 배열 처리 모듈
import cv2      # 비디오/이미지 처리 모듈


# 노란색 차선 검출
def color_hsv(frame):

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # HSV에서 노란색 범위 정의
    lower_yellow = np.array([24,20,20]) 
    upper_yellow = np.array([79,255,255])          

    # 노란색만 얻기 위해 HSV 이미지의 임계값 지정
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND 연산을 통해 mask 영역에서 서로 공통으로 겹치는 부분 추출
    res = cv2.bitwise_and(frame, frame, mask=mask)

    return res

# 노이즈 제거
def denoise_frame(frame): 
    
    denoised_frame = cv2.GaussianBlur(frame, (7,7),0)   # 가우시안 블러 적용하여 노이즈 제거
    
    return denoised_frame

# 에지 검출
def detect_edges(frame):
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # frame을 그레이스케일로 변환
    canny_edges = cv2.Canny(gray, 100, 300)  # thresh 비율 1:3, Canny를 사용하여 엣지 검출

    return canny_edges  # Return edged frame

# 차선 검출할 영역 지정
def region_of_interest(frame):
    
    height, width = frame.shape
    mask = np.zeros_like(frame)

    # 화면 아래쪽 2/3 정도를 관심 영역으로 잡음
    polygon = np.array([[
        (int(width*0.0), height),               # 왼쪽 아래
        (int(width*0.0),  int(height*0.10)),    # 왼쪽 위
        (int(width), int(height*0.10)),         # 오른쪽 위
        (int(width), height),                   # 오른쪽 아래
    ]], np.int32)
    
    # 위에 지정한 크기의 다각형 내면을 255로 채우기
    cv2.fillPoly(mask, polygon, 255)
    # bitwise_and 함수로 255인 부분만 출력
    roi = cv2.bitwise_and(frame, mask)
    
    return roi

# Hough Lines Polar를 통한 라인 감지
def detect_lines(frame):
    
    # 직선(차선) 검출
    # HoughLinesP(검출 이미지, 거리, 각도, 임계값, 최소 선 길이, 최대 선 간격)
    line_segments = cv2.HoughLinesP(frame, 1, np.pi/180 , 100, 
                                minLineLength=80, maxLineGap=10)

    return line_segments


# 라인 최적화 및 도로에 하나의 실선 출력
def optimize_lines(frame, lines):

    if lines is not None:   
        # 줄 구분을 위한 변수 초기화
        lane_lines = [] # 양쪽 라인
        left_fit = []   # 왼쪽 라인
        right_fit = []  # 오른쪽 라인
        
        for line in lines:  
            x1, y1, x2, y2 = line.reshape(4)    # 라인을 좌표로 얻기
            parameters = np.polyfit((x1, x2), (y1, y2), 1)  # 얻은 좌표들을 parameters에 저장
            slope = parameters[0]       # 첫번째 parameter는 기울기
            intercept = parameters[1]   # 두번째 parameter는 절편

            if slope < 0:   # 기울기 확인하여 리스트에 저장
                left_fit.append((slope, intercept))
            else:   
                right_fit.append((slope, intercept))

        if len(left_fit) > 0:       # 왼쪽 라인인지 확인
            left_fit_average = np.average(left_fit, axis=0)     # 왼쪽 선에 대한 평균 얻기
            lane_lines.append(map_coordinates(frame, left_fit_average)) # lane_lines 목록에 왼쪽 선에 대한 평균 결과 추가
            
        if len(right_fit) > 0:        # 오른쪽 라인인지 확인
            right_fit_average = np.average(right_fit, axis=0)   # 오른쪽 선에 대한 평균 얻기
            lane_lines.append(map_coordinates(frame, right_fit_average))    # lane_lines 목록에 오른쪽 선에 대한 평균 결과 추가
        
    else:
        lane_lines = None

    return lane_lines       # 최적화된 라인 반환

# 라인 구성을 위해 주어진 매개변수를 매핑
def map_coordinates(frame, parameters):
    
    height, width, _ = frame.shape  # 프레임 크기 가져오기
    slope, intercept = parameters   # 주어진 매개변수에서 기울기 및 절편 가져오기
    
    if slope == 0:      # Check whether the slope is 0
        slope = 0.1     # handle it for reducing Divisiob by Zero error
    
    y1 = height             # 프레임 하단 포인트
    y2 = int(height*0.10)  # 프레임 중앙에서 아래쪽으로 점 찍기
    x1 = int((y1 - intercept) / slope)  # (y1절편)/기울기 공식으로 x1을 계산
    x2 = int((y2 - intercept) / slope)  # (y2절편)/기울기 공식으로 x2을 계산
    
    return [[x1, y1, x2, y2]]   # 배열로 반환

# 화면에 라인 표시
def display_lines(frame, lines):

    height, width, _ = frame.shape

    mask = np.zeros_like(frame)   # 프레임과 동일한 차원을 사용하여 0으로 배열 만들기
    cv2.line(mask, ((width//2), height), ((width//2), (height//2)), (200,200,200), 2)   # 이미지의 중심선 표시

    if lines is not None:       # 기존 라인이 있는지 확인
        for line in lines:
            for x1, y1, x2, y2 in line: # line을 좌표로 풀기

                # 선 감지 안될 때
                if x1<-10000 or y1<-10000 or x2<-10000 or y2<-10000 or x1>10000 or y1>10000 or x2>10000 or y2>10000:
                    print('fail_line: ', line)

                else:
                    cv2.line(mask, (x1, y1), (x2, y2), (200, 200, 200), 5)
 
    # 원본 프레임과 마스크 병합
    mask = cv2.addWeighted(frame, 0.8, mask, 1, 1) 
    
    return mask

# 선의 x좌표를 이용해 방향 구하기
def direction_func(frame, lane_lines):

    height, width, _ = frame.shape

    # 기준이 될 고정 x좌표
    a1 = int(width*0.4)     # 첫번째 고정 좌표 (파란 사각형의 왼쪽 위 꼭짓점)
    a2 = int(width*0.6)     # 두번째 고정 좌표 (파란 사각형의 오른쪽 위 꼭짓점)

    # 고정 좌표를 눈으로 볼 수 있게 사각형 그리기
    cv2.rectangle(frame, (int(width*0.4), int(height*0.2)), (int(width*0.6), int(height*0.3)), (255,100,0), -1)

    # 방향 초기화
    Direction = 'Straight'
    
    # 양쪽 선이 감지될 때
    if len(lane_lines) == 2: 
        left_x1, _, left_x2, _ = lane_lines[0][0]   # left line
        right_x1, _, right_x2, _ = lane_lines[1][0] # right line

        if a1 < left_x2 < a2:    # left line
            Direction = 'S_Right'
        elif left_x2 > a2:
            Direction = 'M_Right'
        else:
            Direction = 'Straight'

        if a2 > right_x2 > a1:    # right line
            Direction = 'S_Left'
        elif right_x2 < a1:
            Direction = 'M_Left'
        else:
            Direction = 'Straight'

    # 한쪽 선만 감지될 때
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]   

        # 왼쪽선인지 오른쪽선인지 구분
        if x1 < x2:             # left line
            if a1 < x2 < a2:
                Direction = 'S_Right'
            elif x2 > a2:
                Direction = 'M_Right'
            else:
                Direction = 'Straight'
                
        else:                   # right line
            if a2 > x2 > a1:
                Direction = 'S_Left'
            elif x2 < a1:
                Direction = 'M_Left'
            else:
                Direction = 'Straight'

    # 선이 감지 안됐을 때
    else:
        Direction = 'Back'

    cv2.putText(frame, Direction, (int(width*0.05), int(height*0.10)), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    return Direction

# 실행
def Orchestrator(frame):

    hsv_color = color_hsv(frame)
    
    denoised_frame = denoise_frame(hsv_color)   # 노이즈 제거

    canny_edges = detect_edges(denoised_frame)  # 프레임에서 가장자리 찾기

    roi_frame = region_of_interest(canny_edges)   # 관심영역 그리기

    lines = detect_lines(roi_frame)                 # 프레임에서 차선 감지

    lane_lines = optimize_lines(frame, lines)       # 라인 최적화
    
    # 직선 검출 됐을 때
    if lane_lines != None:
    
        direction = direction_func(frame, lane_lines)   # 방향 구하기
             
        final_frame = display_lines(frame, lane_lines)  # 화면에 선 표시

        return final_frame, direction

    else:
        return frame, 'Straight'
