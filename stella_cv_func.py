# import로 모듈 가져오기
import math     # 수학 모듈
import numpy as np  # 다차원 배열 처리 모듈
import cv2      # 비디오/이미지 처리 모듈

prev_angle = []

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

    return canny_edges

# 차선 검출할 관심 영역 지정
def region_of_interest(frame):

    height, width = frame.shape
    mask = np.zeros_like(frame)

    # 화면 아래쪽 2/3 정도를 관심 영역으로 잡음
    polygon = np.array([[
        (int(width*0.0), height),              # 왼쪽 아래
        (int(width*0.0),  int(height*0.30)),   # 왼쪽 위
        (int(width), int(height*0.30)),        # 오른쪽 위
        (int(width), height),                  # 오른쪽 아래
    ]], np.int32)
    
    # 위에 지정한 크기의 다각형 내면을 255로 채우기
    cv2.fillPoly(mask, polygon, 255)
    # bitwise_and 함수로 255인 부분만 출력
    roi = cv2.bitwise_and(frame, mask)
    
    return roi

# 스카이 뷰 각도로 처리
def warp_perspective(frame):
    """ 스카이뷰 각도에서 처리하기 위한 프레임 뒤틀림 기능 """
    
    height, width = frame.shape    # 프레임 크기 가져오기
    offset = 200     # 프레임 비율을 위한 오프셋

    # 뒤틀릴 시점의 좌표
    source_points = np.float32([[int(width*0.20), int(height*0.50)], # 왼쪽 위
                      [int(width*0.90), int(height*0.50)],           # 오른쪽 위
                      [int(width*0.0), height],                      # 왼쪽 아래
                      [int(width), height]])                         # 오른쪽 아래
    
    # 변경할 이미지 좌표
    destination_points = np.float32([[offset, 0],           # 왼쪽 위
                      [width-2*offset, 0],                  # 오른쪽 위
                      [offset, height],                     # 왼쪽 아래
                      [width-2*offset, height]])            # 오른쪽 아래
    
    # getPerspectiveTransform(원본 이미지 좌표, 변경할 이미지 좌표)
    matrix = cv2.getPerspectiveTransform(source_points, destination_points)    # 스카이뷰 이미지를 왜곡하는 매트릭스
    skyview = cv2.warpPerspective(frame, matrix, (width, height))    # 최종 warping 

    return skyview, matrix

# 히스토그램 이용해서 x축 왼쪽, 오른쪽 베이스 좌표 찾기
def histogram(frame):
    """ leftx 및 rightx 기준을 찾기 위한 히스토그램 """
    
    histogram = np.sum(frame, axis=0)   # 히스토그램 구축
    midpoint = np.int(histogram.shape[0]/2)     # 히스토그램에서 중간점 찾기

    # np.argmax(): 가장 큰 원소 인덱스 반환
    left_x_base = np.argmax(histogram[:midpoint])    # 왼쪽 최대 픽셀 계산
    right_x_base = np.argmax(histogram[midpoint:]) + midpoint    # 오른쪽 최대 픽셀 계산

    return left_x_base, right_x_base


# Hough Lines Polar를 통한 라인 감지
def detect_lines(frame):    

    # 직선(차선) 검출
    # HoughLinesP(검출 이미지, 거리(0~1 실수), 각도(0 ~ 180 정수), 임계값, 최소 선 길이, 최대 선 간격)
    # 임계값(threshold) : 만나는 점의 기준, 숫자가 작으면 많은 선이 검출되지만 정확도가 떨어지고, 숫자가 크면 정확도가 올라감
    line_segments = cv2.HoughLinesP(frame, 1, np.pi/180 , 50, 
                                minLineLength=10, maxLineGap=50)
    return line_segments

# 라인 구성을 위해 주어진 매개변수를 매핑
def map_coordinates(frame, parameters): 

    height, width, _ = frame.shape  # 프레임 크기 가져오기
    slope, intercept = parameters   # 주어진 매개변수에서 기울기 및 절편 가져오기
    
    if slope == 0:      # 기울기가 0인지 확인
        slope = 0.1     # Divisiob by Zero 오류를 줄이기 위한 처리
    
    y1 = height             # 프레임 하단 포인트
    y2 = int(height*0.40)  # 프레임 중앙에서 아래쪽으로 점 찍기
    x1 = int((y1 - intercept) / slope)  # (y1절편)/기울기 공식으로 x1을 계산
    x2 = int((y2 - intercept) / slope)  # (y2절편)/기울기 공식으로 x2을 계산
    
    return [[x1, y1, x2, y2]]   # 배열로 반환


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



# 화면에 라인 표시
def display_lines(frame, lines):

    height, width, _ = frame.shape
    cv2.line(frame, ((width//2), height), ((width//2), (height//2)), (255,0,0), 2) # 이미지의 중심선 표시

    mask = np.zeros_like(frame)   # 프레임과 동일한 차원을 사용하여 0으로 배열 만들기
    if lines is not None:       # 기존 라인이 있는지 확인
        for line in lines:      # 줄 목록을 반복
            for x1, y1, x2, y2 in line: # line을 좌표로 풀기
                # 선 감지 안될 때
                if x1<-10000 or y1<-10000 or x2<-10000 or y2<-10000 or x1>10000 or y1>10000 or x2>10000 or y2>10000:
                    print('fail_line: ', line)
                    pass
                else:
                    cv2.line(mask, (x1, y1), (x2, y2), (0, 255, 0), 2)  
    # 원본 프레임과 마스크 병합
    mask = cv2.addWeighted(frame, 0.8, mask, 1, 1)   
    
    return mask


# 조향각 계산
def get_floating_center(frame, lane_lines):

    height, width, _ = frame.shape
    
    if len(lane_lines) == 2:    # 2개의 줄이 감지되었는지 확인
        left_x1, _, left_x2, _ = lane_lines[0][0]   # 왼쪽 줄
        right_x1, _, right_x2, _ = lane_lines[1][0] # 오른쪽 줄
        
        # 중간선의 위,아래 좌표
        low_mid = (right_x1 + left_x1) / 2  # 하단 중간 지점의 상대 위치 계산
        up_mid = (right_x2 + left_x2) / 2   # 상단 중간 지점의 상대 위치 계산
    else:
        up_mid = int(width*1.9)
        low_mid = int(width*1.9)

    return up_mid, low_mid

# 화면에 중앙선 표시
def display_heading_line(frame, up_center, low_center):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    x1 = int(low_center)
    y1 = height
    x2 = int(up_center)
    y2 = int(height*0.40)
    
    # 선 검출 안될 때
    if x1<0 or x2<0 or x1>10000 or x2>10000:
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    else:
        cv2.line(heading_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    
    return heading_image


# 각도 구하고 회전 방향 출력
def add_text(frame, up_center, low_center, lane_lines):

    height, width, _ = frame.shape
    
    left_x1, _, left_x2, _ = lane_lines[0][0] 
    
    # image center 와 line center 사이 각도 구하기
    if up_center < 1000:
        if up_center > low_center:
            tan_theta = ((up_center-width/2)-(up_center-low_center))/(height/2)
        elif up_center < low_center:
            tan_theta = ((low_center-width/2)-(low_center-up_center))/(height/2)
        else :
            tan_theta = 0
    else:
        tan_theta = ((width/2-left_x1)-(left_x2-left_x1))/(height/2)

    theta = np.arctan(tan_theta)
    rotate_angle = theta *180//math.pi

    if rotate_angle >= -10 and rotate_angle <= 23:
        text = "Straight"
        memory_text = text
    elif rotate_angle < -10:
        text = "Smooth Left"
        memory_text = text
    elif rotate_angle > 23:
        text = "Smooth Right"
        memory_text = text
    else:
        text = memory_text
    
    cv2.putText(frame, text, (int(width*0.10), int(height*0.20)), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA) # Draw direction
    
    return frame, rotate_angle

# 평균필터 사용하여 각도의 평균 구하기
def weight_moving_average(w1,w2,w3,w4):   #w1= 0.5 //  w2 = 0.3 // w3 = 0.15 // w4 = 0.05

    global prev_angle

    if len(prev_angle) <4:
        WMA_angle = prev_angle[0]

    elif len(prev_angle)>=4:    
        WMA_angle = prev_angle[0]*w4+prev_angle[1]*w3+prev_angle[2]*w2+prev_angle[3]*w1 
        
        prev_angle.pop(0)
        # print('WMA_angle: ',WMA_angle)

    return WMA_angle

# 프로그램 실행
def Orchestrator(frame):

    hsv_color = color_hsv(frame)
    
    denoised_frame = denoise_frame(hsv_color)   # 노이즈 제거
    
    canny_edges = detect_edges(denoised_frame)  # 프레임에서 가장자리 찾기
    
    roi_frame = region_of_interest(canny_edges)   # 관심영역 그리기
    lines = detect_lines(roi_frame)                 # 프레임에서 차선 감지

    lane_lines = optimize_lines(frame, lines)       # 라인 최적화
    
    # 직선 검출 됐을 때
    if lane_lines!=None:
        height, width, _ = frame.shape
             
        lane_lines_image = display_lines(frame, lane_lines) # 선 표시
        
        up_center, low_center = get_floating_center(frame, lane_lines) # 두 선 사이의 중심 계산

        heading_line = display_heading_line(lane_lines_image, up_center, low_center)

        final_frame, rotate_angle = add_text(heading_line, up_center, low_center, lane_lines)
        
        prev_angle.append(rotate_angle)

        weight_angle = weight_moving_average(w1=0.5,w2=0.3,w3=0.15,w4=0.05)
        cv2.putText(final_frame, str(weight_angle), (int(width*0.10), int(height*0.30)), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA) # Draw direction

        return final_frame, weight_angle
    else:
        return frame, -1
