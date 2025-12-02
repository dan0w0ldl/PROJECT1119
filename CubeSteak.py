#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

# --- [기본 설정] ---
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
grab_motor = Motor(Port.B)

left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S4)
jcolor = ColorSensor(Port.S2) 
ultra = UltrasonicSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, 55.5, 104)

# [튜닝 값]
THRESHOLD = 50 
KP = 1.2
DRIVE_SPEED = 120 
TURN_OFFSET = 10
DETECT_DIST = 120 

# 방향 상수
DIR_N, DIR_E, DIR_S, DIR_W = 0, 1, 2, 3

# 맵 설정
MAP_SIZE = 3
current_pos = (0, 0)
current_facing = DIR_E 

# -----------------------------------------------------------
# 1. 라인트레이싱 함수들
# -----------------------------------------------------------
def line_follow_left_step(speed, kp):
    # '오른센서_흰바탕' (True) -> 왼쪽 센서가 검정 라인을 탐
    reflection = left_color.reflection()
    error = reflection - THRESHOLD
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def line_follow_right_step(speed, kp):
    # '왼센서_흰바탕' (False) -> 오른쪽 센서가 검정 라인을 탐
    reflection = right_color.reflection()
    error = reflection - THRESHOLD
    turn_rate = -kp * error 
    robot.drive(speed, turn_rate)

# -----------------------------------------------------------
# 2. 회전 및 이동
# -----------------------------------------------------------
def turn_to_direction_smart(target_dir):
    global current_facing
    diff = (target_dir - current_facing) % 4
    if diff == 0: return

    robot.straight(TURN_OFFSET) 
    if diff == 1: robot.turn(90)
    elif diff == 2: robot.turn(180)
    elif diff == 3: robot.turn(-90)
    
    current_facing = target_dir
    wait(100)

def get_trace_mode(r, c, facing):
    """
    사용자 요청에 따라 좌표(x,y)를 스왑하여 규칙 적용
    True: 오른센서_흰바탕 (왼쪽 라인추적)
    False: 왼센서_흰바탕 (오른쪽 라인추적)
    """
    rules = {
        # 원래 (0,0) -> (0,0) 그대로
        (0,0): {DIR_E: True,  DIR_W: False, DIR_S: False, DIR_N: True},
        
        # 원래 (0,1) -> (1,0)으로 이동
        (1,0): {DIR_E: True,  DIR_W: False, DIR_S: False, DIR_N: True},
        
        # 원래 (0,2) -> (2,0)으로 이동
        (2,0): {DIR_E: False, DIR_W: True,  DIR_S: False, DIR_N: True},
        
        # 원래 (1,0) -> (0,1)으로 이동
        (0,1): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        
        # 원래 (1,1) -> (1,1) 그대로
        (1,1): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        
        # 원래 (1,2) -> (2,1)으로 이동
        (2,1): {DIR_E: False, DIR_W: True,  DIR_S: True,  DIR_N: False},
        
        # 원래 (2,0) -> (0,2)으로 이동
        (0,2): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        
        # 원래 (2,1) -> (1,2)으로 이동
        (1,2): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        
        # 원래 (2,2) -> (2,2) 그대로
        (2,2): {DIR_E: False, DIR_W: True,  DIR_S: True,  DIR_N: False}
    }
    
    if (r, c) in rules:
        return rules[(r, c)].get(facing, True) 
    return True

# 탐색용 (물체 감지 ON)
def n_move_grid_smart(n):
    global current_pos
    INTERSECTION_THRESHOLD = 30  
    
    for _ in range(n):
        robot.drive(DRIVE_SPEED, 0)
        
        r, c = current_pos 
        is_trace_left = get_trace_mode(r, c, current_facing)
        
        # 탈출 로직
        if not is_trace_left:
            while left_color.reflection() < 40:
                if ultra.distance() < DETECT_DIST:
                     robot.stop()
                     if handle_object_encounter(): return True
                     robot.drive(DRIVE_SPEED, 0)
                wait(5)
        else:
            while right_color.reflection() < 40:
                if ultra.distance() < DETECT_DIST:
                     robot.stop()
                     if handle_object_encounter(): return True
                     robot.drive(DRIVE_SPEED, 0)
                wait(5)

        # 주행 및 감지
        while True:
            if ultra.distance() < DETECT_DIST:
                robot.stop()
                ev3.speaker.beep()
                if handle_object_encounter(): return True 
                robot.drive(DRIVE_SPEED, 0)

            if not is_trace_left: 
                line_follow_right_step(DRIVE_SPEED, KP)
                if left_color.reflection() < INTERSECTION_THRESHOLD: break 
            else:
                line_follow_left_step(DRIVE_SPEED, KP)
                if right_color.reflection() < INTERSECTION_THRESHOLD: break 
            wait(5)
            
        robot.stop()
        ev3.speaker.beep()
        robot.drive(DRIVE_SPEED, 0)
        
        if not is_trace_left:
            while left_color.reflection() < 40: wait(5)
        else:
            while right_color.reflection() < 40: wait(5)
        wait(100) 
        robot.stop()

        if current_facing == DIR_N: r -= 1
        elif current_facing == DIR_E: c += 1
        elif current_facing == DIR_S: r += 1
        elif current_facing == DIR_W: c -= 1
        current_pos = (r, c)
    
    return False

# 복귀용 (물체 감지 OFF)
def n_move_grid_blind(n):
    global current_pos
    INTERSECTION_THRESHOLD = 30
    
    for _ in range(n):
        robot.drive(DRIVE_SPEED, 0)
        
        r, c = current_pos 
        is_trace_left = get_trace_mode(r, c, current_facing)
        
        if not is_trace_left:
            while left_color.reflection() < 40: wait(5)
        else:
            while right_color.reflection() < 40: wait(5)

        while True:
            if not is_trace_left: 
                line_follow_right_step(DRIVE_SPEED, KP)
                if left_color.reflection() < INTERSECTION_THRESHOLD: break 
            else:
                line_follow_left_step(DRIVE_SPEED, KP)
                if right_color.reflection() < INTERSECTION_THRESHOLD: break 
            wait(5)
            
        robot.stop()
        robot.drive(DRIVE_SPEED, 0)
        if not is_trace_left:
            while left_color.reflection() < 40: wait(5)
        else:
            while right_color.reflection() < 40: wait(5)
        wait(100) 
        robot.stop()
        
        if current_facing == DIR_N: r -= 1
        elif current_facing == DIR_E: c += 1
        elif current_facing == DIR_S: r += 1
        elif current_facing == DIR_W: c -= 1
        current_pos = (r, c)

# -----------------------------------------------------------
# 3. 배달 및 복귀
# -----------------------------------------------------------
def n_move_delivery(n, direction="right"):
    for _ in range(n):
        if direction == "right":
            while right_color.reflection() > THRESHOLD:
                line_follow_left_step(100, KP)
                wait(10)
            while right_color.reflection() <= THRESHOLD:
                line_follow_right_step(100, KP)
                wait(10)
        elif direction == "left":
            while left_color.reflection() > THRESHOLD:
                line_follow_right_step(100, KP)
                wait(10)
            while left_color.reflection() <= THRESHOLD:
                line_follow_left_step(100, KP)
                wait(10)
    robot.stop()

def execute_delivery_and_return(rgb_color):
    global current_facing

    r, g, b = rgb_color
    is_red = (r > b + 3) and (r > g + 3)
    is_blue = (b > r + 3) and (b > g + 3)

    if is_red: 
        print("RED ROUTE")
        robot.straight(30)
        n_move_delivery(1, direction="left")
        robot.straight(50)
        robot.turn(-90)
        robot.straight(30)
        n_move_delivery(1, direction="right")
        robot.turn(90)
        robot.straight(30)
        n_move_delivery(1, direction="right")
        
        # 물건 놓기
        grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100) 
        wait(500)
        
        # 복귀 (Reverse)
        # 왔다갔다 하는 불필요한 straight(150) 제거 (시간 단축 및 오차 방지)
        robot.turn(180) # 서쪽 -> 동쪽으로 회전
        
        n_move_delivery(1, direction='left')
        robot.straight(30)
        robot.turn(-90)
        n_move_delivery(1, direction='right')
        robot.straight(30)
        robot.turn(90)
        n_move_delivery(1, direction='left')
        robot.straight(30)

    elif is_blue: 
        print("BLUE ROUTE")
        n_move_delivery(1, direction="right")
        robot.turn(-90)
        n_move_delivery(2, direction="left")
        robot.turn(90)
        n_move_delivery(1, direction="left")
        
        # 물건 놓기
        grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100) 
        wait(500)
        
        # 복귀 (Reverse)
        # ★ 수정됨: 여기서 180도 한번만 돌면 동쪽을 보게 됩니다.
        # 기존 코드에 있던 두 번째 turn(180)을 삭제했습니다.
        robot.turn(180) 
        
        n_move_delivery(1, direction='left')
        robot.straight(30)
        robot.turn(-90)
        n_move_delivery(2, direction='right')
        robot.straight(30)
        robot.turn(90)
        n_move_delivery(1, direction='left')
        robot.straight(30)

    # 이제 물리적으로 동쪽을 보고 있으므로, 논리적 방향도 동쪽으로 설정
    current_facing = DIR_E
    print("At (0,0), Facing East (Reset Done)")

# -----------------------------------------------------------
# 4. 경로 계산 (BFS)
# -----------------------------------------------------------
def bfs_path(start, end):
    queue = [(start, [start])]
    visited = {start}
    moves = [(-1,0), (0,1), (1,0), (0,-1)]
    while queue:
        (curr, path) = queue.pop(0)
        if curr == end: return path
        r, c = curr
        for dr, dc in moves:
            nr, nc = r+dr, c+dc
            if 0 <= nr < MAP_SIZE and 0 <= nc < MAP_SIZE:
                if (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append(((nr, nc), path + [(nr, nc)]))
    return []

def get_direction(curr, next_node):
    cr, cc = curr; nr, nc = next_node
    if nr < cr: return DIR_N
    if nr > cr: return DIR_S
    if nc > cc: return DIR_E
    if nc < cc: return DIR_W
    return current_facing

def run_grid_path(path_list):
    if not path_list: return False
    for i in range(1, len(path_list)):
        next_node = path_list[i]
        target_dir = get_direction(current_pos, next_node)
        turn_to_direction_smart(target_dir)
        wait(100)
        interrupted = n_move_grid_smart(1)
        if interrupted: return True
        wait(100)
    return False

def run_grid_path_blind(path_list):
    if not path_list: return
    for i in range(1, len(path_list)):
        next_node = path_list[i]
        target_dir = get_direction(current_pos, next_node)
        turn_to_direction_smart(target_dir)
        wait(100)
        n_move_grid_blind(1)
        wait(100)

# -----------------------------------------------------------
# 5. 감지 -> 집기 -> 색상판별
# -----------------------------------------------------------
def handle_object_encounter():
    global current_pos
    
    # 1. 접근
    robot.drive(80, 0)
    while ultra.distance() > 40: 
        wait(10)
    robot.stop()
    
    robot.straight(50) 

    # 2. 집기
    ev3.speaker.say("Grab")
    grab_motor.run_until_stalled(1000, then=Stop.HOLD, duty_limit=100)
    robot.straight(100)

    # 3. 색상 판별
    rgb = jcolor.rgb()
    print("Grabbed RGB:", rgb)
    
    detected_color = None
    r, g, b = rgb[0], rgb[1], rgb[2]
    
    if r > g and r > b:
        print("-> RED")
        detected_color = rgb
    elif b > r and b > g:
        print("-> BLUE")
        detected_color = rgb
    else:
        print("-> Unknown")
        ev3.speaker.say("Unknown")
        grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100)
        robot.straight(-50)
        wait(500)
        return False

    # ★ 집은 후 다시 원래 교차로 중심으로 복귀!
    wait(200)
    robot.straight(-50)

    # 4. 좌표 업데이트
    r, c = current_pos
    if current_facing == DIR_N: r -= 1
    elif current_facing == DIR_E: c += 1
    elif current_facing == DIR_S: r += 1
    elif current_facing == DIR_W: c -= 1
    current_pos = (r, c)
    
    saved_pos = current_pos
    print("Coordinates Updated to {}. Start Delivery.".format(saved_pos))

    # 베이스 복귀 -> 배달 -> 원위치
    path_to_base = bfs_path(current_pos, (0, 0))
    run_grid_path_blind(path_to_base)
    
    turn_to_direction_smart(DIR_W)
    wait(200)
    execute_delivery_and_return(detected_color)
    print("Returning to {}".format(saved_pos))
    current_pos = (0, 0)
    
    path_back = bfs_path((0, 0), saved_pos)
    run_grid_path_blind(path_back)
    
    return True 

# -----------------------------------------------------------
# 6. 메인 실행
# -----------------------------------------------------------
def start_exploration():
    global current_pos
    grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100)

    TARGET_ORDER = [
        (0,1), (0,2),       
        (1,2), (1,1), (1,0), 
        (2,0), (2,1), (2,2)  
    ]
    
    i = 0
    while i < len(TARGET_ORDER):
        target = TARGET_ORDER[i]
        path = bfs_path(current_pos, target)
        
        interrupted = run_grid_path(path)
        
        if interrupted:
            print("Interrupted! Retry same target...")
            continue
        
        i += 1
        
    ev3.speaker.say("Finish")

start_exploration()