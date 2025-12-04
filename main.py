#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

# — [기본 설정] —
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
grab_motor = Motor(Port.B)

left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S4)
jcolor = ColorSensor(Port.S2) 
ultra = UltrasonicSensor(Port.S3)

# ★중요★: 바퀴 지름(55.5)과 축간거리(104) 설정
robot = DriveBase(left_motor, right_motor, 55.5, 104)

# [튜닝 값]
THRESHOLD = 50 
KP = 1.2
DRIVE_SPEED = 140 
TURN_OFFSET = 50  # 회전 전 약간 앞으로 가는 거리 (축 정렬용)
DETECT_DIST = 120 

# [상태 변수]
IS_CARRYING = False # False: 빈손, True: 물건 잡음

# 방향 상수
DIR_N, DIR_E, DIR_S, DIR_W = 0, 1, 2, 3
DIR_NAMES = ["North", "East", "South", "West"]

# 맵 설정
MAP_SIZE = 3
current_pos = (0, 0)
current_facing = DIR_E 

# 상태 관리
visited_nodes = set()
found_objects = 0       # 찾은 물건 개수
TOTAL_GRID_OBJECTS = 4  # 목표 개수

# —————————————————————————————
# [Helper] 위치 안내
# —————————————————————————————
def announce_position():
    r, c = current_pos
    direction_str = DIR_NAMES[current_facing]
    print(">>> STATUS: Pos({}, {}) Facing {}".format(r, c, direction_str))

# -----------------------------------------------------------
# 1. 라인트레이싱 및 회전 함수들
# -----------------------------------------------------------
def line_follow_left_step(speed, kp):
    # 왼쪽 센서가 라인을 타도록 (라인의 왼쪽 가장자리 or 라인 위)
    reflection = left_color.reflection()
    error = reflection - THRESHOLD
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def line_follow_right_step(speed, kp):
    # 오른쪽 센서가 라인을 타도록
    reflection = right_color.reflection()
    error = reflection - THRESHOLD
    turn_rate = -kp * error 
    robot.drive(speed, turn_rate)

def turn_to_direction_smart(target_dir):
    global current_facing
    diff = (target_dir - current_facing) % 4
    if diff == 0: return

    # 교차로 중앙에 바퀴 축을 맞추기 위해 약간 전진
    robot.straight(TURN_OFFSET) 
    
    # --- 회전 각도 설정 (무게 보정) ---
    if IS_CARRYING:
        turn_90 = 110   # 무거울 때 90도
        turn_180 = 200  # 무거울 때 180도
    else:
        turn_90 = 100   # 평소 90도
        turn_180 = 190  # 평소 180도

    if diff == 1:   # 90도 (우회전)
        robot.turn(turn_90)
    elif diff == 2: # 180도 (뒤로)
        robot.turn(turn_180)
    elif diff == 3: # -90도 (좌회전)
        robot.turn(-turn_90)
    
    current_facing = target_dir
    wait(5)

def turn_and_update(angle):
    global current_facing
    
    execute_angle = angle
    
    # 배달 중 회전 각도 보정
    if IS_CARRYING:
        if angle == 90: execute_angle = 105
        elif angle == -90: execute_angle = -105
        elif angle == 180: execute_angle = 200
    else:
        if angle == 90: execute_angle = 95
        elif angle == -90: execute_angle = -95
        elif angle == 180: execute_angle = 185
        
    robot.turn(execute_angle)
    
    if angle == 90:
        current_facing = (current_facing + 1) % 4
    elif angle == -90:
        current_facing = (current_facing - 1) % 4
    elif angle == 180:
        current_facing = (current_facing + 2) % 4

# ★★★ [복구됨] 트레이싱 모드 결정 함수 ★★★
def get_trace_mode(r, c, facing):
    """
    True: 오른센서_흰바탕 (왼쪽 라인추적 - line_follow_left_step)
    False: 왼센서_흰바탕 (오른쪽 라인추적 - line_follow_right_step)
    """
    rules = {
        # (0,0) -> (0,0)
        (0,0): {DIR_E: True,  DIR_W: False, DIR_S: False, DIR_N: True},
        # (0,1) -> (1,0)
        (1,0): {DIR_E: True,  DIR_W: False, DIR_S: False, DIR_N: True},
        # (0,2) -> (2,0)
        (2,0): {DIR_E: False, DIR_W: True,  DIR_S: False, DIR_N: True},
        
        # (1,0) -> (0,1)
        (0,1): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        # (1,1) -> (1,1)
        (1,1): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        # (1,2) -> (2,1)
        (2,1): {DIR_E: False, DIR_W: True,  DIR_S: True,  DIR_N: False},
        
        # (2,0) -> (0,2)
        (0,2): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        # (2,1) -> (1,2)
        (1,2): {DIR_E: True,  DIR_W: False, DIR_S: True,  DIR_N: False},
        # (2,2) -> (2,2)
        (2,2): {DIR_E: False, DIR_W: True,  DIR_S: True,  DIR_N: False}
    }
    
    if (r, c) in rules:
        return rules[(r, c)].get(facing, True)
    return True

# -----------------------------------------------------------
# 2. 경로 계산 (BFS)
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

def bfs_path_safe(start, end):
    queue = [(start, [start])]
    checked = {start}
    moves = [(-1,0), (0,1), (1,0), (0,-1)]
    while queue:
        (curr, path) = queue.pop(0)
        if curr == end: return path
        r, c = curr
        for dr, dc in moves:
            nr, nc = r+dr, c+dc
            if 0 <= nr < MAP_SIZE and 0 <= nc < MAP_SIZE:
                if ((nr, nc) in visited_nodes) and ((nr, nc) not in checked):
                    checked.add((nr, nc))
                    queue.append(((nr, nc), path + [(nr, nc)]))
    return [] 

def get_direction(curr, next_node):
    cr, cc = curr; nr, nc = next_node
    if nr < cr: return DIR_N
    if nr > cr: return DIR_S
    if nc > cc: return DIR_E
    if nc < cc: return DIR_W
    return current_facing

# -----------------------------------------------------------
# 3. 배달 및 복귀 로직
# -----------------------------------------------------------
def n_move_delivery(n, direction="right"):
    for _ in range(n):
        if direction == "right":
            while right_color.reflection() > THRESHOLD:
                line_follow_left_step(DRIVE_SPEED, KP)
                wait(5)
            while right_color.reflection() <= THRESHOLD:
                line_follow_right_step(DRIVE_SPEED, KP)
                wait(5)
        elif direction == "left":
            while left_color.reflection() > THRESHOLD:
                line_follow_right_step(DRIVE_SPEED, KP)
                wait(5)
            while left_color.reflection() <= THRESHOLD:
                line_follow_left_step(DRIVE_SPEED, KP)
                wait(5)
    robot.stop()

def drop_and_turn(return_angle):
    global IS_CARRYING
    
    # 1. 물건 내려놓기
    grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100) 
    IS_CARRYING = False 
    
    # 2. 복귀 각도에 따른 분기
    if abs(return_angle) == 180:
        robot.straight(100)
        robot.straight(-135)
        turn_and_update(180)
    else:
        robot.straight(TURN_OFFSET)
        turn_and_update(return_angle)

def execute_delivery_and_return(rgb_color, is_last_object=False):
    global current_facing, current_pos
    r, g, b = rgb_color
    is_red = (r > b + 3) and (r > g + 3)
    is_blue = (b > r + 3) and (b > g + 3)

    # -------------------------------------------------
    # RED ROUTE
    # -------------------------------------------------
    if is_red: 
        print("RED ROUTE")
        robot.straight(30)
        n_move_delivery(1, direction="left")
        robot.straight(50)
        turn_and_update(-95)
        n_move_delivery(1, direction="right")
        turn_and_update(95)
        n_move_delivery(1, direction="right")
        
        # 하차
        return_angle = 180 
        drop_and_turn(return_angle)
        robot.straight(30)
        
        # [복귀 로직 분기]
        if not is_last_object:
            # 평소 복귀 (0,0 으로)
            n_move_delivery(1, direction='right') 
            turn_and_update(-90)
            n_move_delivery(1, direction='right')
            turn_and_update(90)
            n_move_delivery(1, direction='right')
        else:
            # [4번째 물건] -> (0, -2) 이동 (사용자 요청 코드 적용)
            print("LAST OBJECT: Moving to (0, -2) via RED path")
            n_move_delivery(1, direction='right') 
            turn_and_update(-90)
            n_move_delivery(1, direction='right')
            turn_and_update(-90)
            n_move_delivery(1, direction='right')
            # 도착: (0, -2), 서쪽(West)

    # -------------------------------------------------
    # BLUE ROUTE
    # -------------------------------------------------
    elif is_blue: 
        print("BLUE ROUTE")
        robot.straight(30)
        n_move_delivery(1, direction="left")
        robot.straight(30)
        turn_and_update(-95)
        n_move_delivery(2, direction="right")
        turn_and_update(95)
        robot.straight(200)        
        
        # 하차
        return_angle = 180
        drop_and_turn(return_angle)
        robot.straight(30)

        # [복귀 로직 분기]
        if not is_last_object:
            # 평소 복귀 (0,0 으로)
            n_move_delivery(1, direction='left')
            turn_and_update(-90)
            n_move_delivery(2, direction='right')
            turn_and_update(90)
            n_move_delivery(1, direction='right')
        else:
            # [4번째 물건] -> (0, -2) 이동 (사용자 요청 코드 적용)
            print("LAST OBJECT: Moving to (0, -2) via BLUE path")
            n_move_delivery(1, direction='left')
            turn_and_update(-90)
            n_move_delivery(2, direction='right')
            turn_and_update(-90)
            n_move_delivery(1, direction='right')
            # 도착: (0, -2), 서쪽(West)
    
    else:
        print("Unknown Color")

    # 4번째 물건인 경우 좌표 및 방향 업데이트 (Bonus Phase 시작점)
    if is_last_object:
        current_pos = (0, -2) 
        current_facing = DIR_W # 서쪽
        
        
def run_delivery_sequence(detected_color, return_pos):
    global current_pos, found_objects
    
    # 1. 베이스로 복귀 (물건 들고 있음)
    path_to_base = bfs_path_safe(current_pos, (0, 0))
    run_grid_path_blind(path_to_base)
    
    # 2. 물건 하차
    turn_to_direction_smart(DIR_W)
    
    # ★ 4번째 물건인지 확인 (found_objects는 handle_object_encounter에서 이미 +1 된 상태)
    is_last = (found_objects >= TOTAL_GRID_OBJECTS) 
    
    execute_delivery_and_return(detected_color, is_last_object=is_last)
    
    # 3. 복귀 (마지막 물건이 아닐 때만 작동)
    if not is_last and return_pos is not None:
        current_pos = (0, 0)
        path_back = bfs_path_safe((0, 0), return_pos)
        ev3.speaker.beep()
        run_grid_path_blind(path_back)
# -----------------------------------------------------------
# 4. 물체 감지 및 잡기
# -----------------------------------------------------------
def handle_object_encounter():
    global current_pos, current_facing, visited_nodes, IS_CARRYING, found_objects
    
    # 1. 접근
    robot.drive(60, 0)
    while ultra.distance() > 50: 
        wait(5)
    robot.stop()
    robot.straight(30)

    # 2. 물건 집기
    print("Grabbing Object...")
    grab_motor.run_until_stalled(1000, then=Stop.HOLD, duty_limit=100)
    IS_CARRYING = True 
    wait(20)

    # 3. 색상 판별
    rgb = jcolor.rgb()
    r, g, b = rgb[0], rgb[1], rgb[2]
    detected_color = None
    
    if (r > g + 3) and (r > b + 3): detected_color = rgb
    elif (b > r + 3) and (b > g + 3): detected_color = rgb
    else:
        # 색상 인식이 안 되면 물건이 아니라고 판단 -> 개수 증가 안 함
        grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100)
        IS_CARRYING = False 
        robot.straight(-50)
        return False

    robot.straight(55) 

    # 4. 좌표 업데이트
    prev_r, prev_c = current_pos
    target_r, target_c = prev_r, prev_c
    
    if current_facing == DIR_N: target_r -= 1
    elif current_facing == DIR_E: target_c += 1
    elif current_facing == DIR_S: target_r += 1
    elif current_facing == DIR_W: target_c -= 1
    
    current_pos = (target_r, target_c)
    visited_nodes.add(current_pos)
    
    print("DEBUG: Object Captured at {}.".format(current_pos))
    
    # 5. 찾은 개수 증가
    found_objects += 1
    print("Status: Found {}/{}".format(found_objects, TOTAL_GRID_OBJECTS))
    
    # 6. 배달 수행 (마지막이면 여기서 (0, -2)로 감)
    run_delivery_sequence(detected_color, None)
    
    # 마지막 물건이 아니라면 (0,0)으로 리셋 
    # (마지막 물건이면 (0, -2)에 있으므로 리셋하면 안 됨)
    if found_objects < TOTAL_GRID_OBJECTS:
        current_pos = (0, 0)
        current_facing = DIR_E 
    
    return True


# [Helper] 그리드 이동 (Trace Mode 적용)
def n_move_grid_smart(n):
    global current_pos
    INTERSECTION_THRESHOLD = 25
    ESCAPE_DISTANCE = 45 

    for _ in range(n):
        robot.straight(ESCAPE_DISTANCE)
        
        # ★ 복구된 규칙 적용
        r, c = current_pos
        is_trace_left = get_trace_mode(r, c, current_facing)
        print("DEBUG TRACE: pos=({},{}) facing={} trace_left={}".format(r, c, DIR_NAMES[current_facing], is_trace_left))
        
        while True:
            if ultra.distance() < DETECT_DIST:
                robot.stop()
                ev3.speaker.beep()
                if handle_object_encounter(): return True 
                robot.drive(DRIVE_SPEED, 0)

            # Trace Mode에 따라 분기 
            if not is_trace_left: 
                # False: 오른쪽 라인 타기 (왼쪽 센서가 흰색인지 확인)
                line_follow_right_step(DRIVE_SPEED, KP)
                if left_color.reflection() < INTERSECTION_THRESHOLD: break 
            else:
                # True: 왼쪽 라인 타기 (오른쪽 센서가 흰색인지 확인)
                line_follow_left_step(DRIVE_SPEED, KP)
                if right_color.reflection() < INTERSECTION_THRESHOLD: break 
            wait(5)
            
        robot.stop()
        
        if current_facing == DIR_N: r -= 1
        elif current_facing == DIR_E: c += 1
        elif current_facing == DIR_S: r += 1
        elif current_facing == DIR_W: c -= 1
        current_pos = (r, c)
        announce_position()
    return False

def n_move_grid_blind(n):
    global current_pos
    INTERSECTION_THRESHOLD = 30
    ESCAPE_DISTANCE = 45 

    for _ in range(n):
        robot.straight(ESCAPE_DISTANCE)
        
        r, c = current_pos
        is_trace_left = get_trace_mode(r, c, current_facing)

        while True:
            if not is_trace_left: 
                line_follow_right_step(DRIVE_SPEED, KP)
                if left_color.reflection() < INTERSECTION_THRESHOLD: break 
            else:
                line_follow_left_step(DRIVE_SPEED, KP)
                if right_color.reflection() < INTERSECTION_THRESHOLD: break 
            wait(5)
        robot.stop()
        
        # 좌표 업데이트
        if current_facing == DIR_N: r -= 1
        elif current_facing == DIR_E: c += 1
        elif current_facing == DIR_S: r += 1
        elif current_facing == DIR_W: c -= 1
        current_pos = (r, c)
        announce_position()

def run_grid_path(path_list):
    if not path_list: return False
    for i in range(1, len(path_list)):
        next_node = path_list[i]
        target_dir = get_direction(current_pos, next_node)
        turn_to_direction_smart(target_dir)
        interrupted = n_move_grid_smart(1)
        if interrupted: return True
    return False

def run_grid_path_blind(path_list):
    if not path_list: return
    for i in range(1, len(path_list)):
        next_node = path_list[i]
        target_dir = get_direction(current_pos, next_node)
        turn_to_direction_smart(target_dir)
        n_move_grid_blind(1)

# —————————————————————————————
# 5. BFS 및 보너스 페이즈 (수정됨)
# —————————————————————————————
def find_nearest_unvisited():
    queue = [current_pos]
    checked = {current_pos} 
    moves = [(0,1), (1,0), (0,-1), (-1,0)]
    
    if len(visited_nodes) >= MAP_SIZE * MAP_SIZE:
        return None
    
    while queue:
        curr = queue.pop(0)
        if curr not in visited_nodes: return curr
        r, c = curr
        for dr, dc in moves:
            nr, nc = r+dr, c+dc
            if 0 <= nr < MAP_SIZE and 0 <= nc < MAP_SIZE:
                if (nr, nc) not in checked:
                    checked.add((nr, nc))
                    queue.append((nr, nc))
    return None

def run_bonus_phase():
    global current_pos, current_facing, IS_CARRYING
    
    print("=== BONUS PHASE START ===")
    
    # 현재 상태: (0, -2) 도착 완료, 서쪽(DIR_W)을 보고 있음
    # Mission: (0, -2) -> (0, 0) 으로 이동 (2칸 직진)
    
    print("Turning East towards (0,0)...")
    turn_to_direction_smart(DIR_E) # 서쪽 -> 동쪽 회전
    
    print("Moving 2 blocks to (0,0) with Left Sensor Line Tracing...")
    
    # 배달존 라인 간섭 피하기 위해 살짝 전진
    robot.straight(100)
    
    # "오른쪽 하얀 배경, 왼쪽 검은 선 트레이싱" = line_follow_left_step
    # (0,0) 교차로(Start 지점)를 만날 때까지 이동
    robot.reset()
    while True:
        # 왼쪽 센서가 라인을 타고 감
        line_follow_left_step(DRIVE_SPEED, KP)
        
        # (0,0) 도착 조건: 양쪽 센서가 모두 검은색(30이하)이면 정지 
        # (Start 지점은 보통 십자거나 T자이므로 양쪽 검출이 안전)
        if left_color.reflection() < 30 and right_color.reflection() < 30:
            break
        wait(5)
    
    robot.stop()
    print("Arrived at (0,0) Intersection.")
    
    # 정렬
    robot.straight(55)
    current_pos = (0, 0)
    current_facing = DIR_E
    ev3.speaker.beep()

    # ---------------------------------------------------------
    # (0, 2)로 이동 및 찌르기 (기존 로직 유지)
    # ---------------------------------------------------------
    path = bfs_path_safe(current_pos, (0, 2))
    run_grid_path_blind(path)
    turn_to_direction_smart(DIR_E)
    
    print("Start Poking...")
    robot.straight(300)
    turn_to_direction_smart(DIR_S)
    
    print("Scanning South (max 1000mm)...")
    robot.reset()
    object_detected = False
    
    while robot.distance() < 1000:
        if ultra.distance() < 50:
            robot.stop()
            object_detected = True
            break
        robot.drive(150, 0)
    
    robot.stop()
    moved_distance = robot.distance()
    
    if object_detected:
        print("Bonus Object Detected!")
        grab_motor.run_until_stalled(1000, then=Stop.HOLD, duty_limit=100)
        IS_CARRYING = True
        wait(50)
        rgb = jcolor.rgb()
        
        print("Reversing {} mm".format(moved_distance))
        robot.straight(-moved_distance)
        
        turn_to_direction_smart(DIR_W)
        
        print("Finding (0,2) line...")
        robot.drive(150, 0)
        while left_color.reflection() > 30 and right_color.reflection() > 30:
            wait(10)
        robot.stop()
        
        robot.straight(55)
        current_pos = (0, 2)
        current_facing = DIR_W 
        
        print("Starting Delivery.")
        run_delivery_sequence(rgb, None)
        print("Bonus Mission Complete.")
        
    else:
        print("Void was Empty.")
        robot.straight(-moved_distance)
        turn_to_direction_smart(DIR_W)
        robot.drive(150, 0)
        while left_color.reflection() > 30 and right_color.reflection() > 30:
            wait(10)
        robot.stop()
        robot.straight(55)

    print("Mission End.")
# -----------------------------------------------------------
# 6. 메인 실행
# -----------------------------------------------------------
def go_to_origin():
    global current_pos, current_facing, found_objects, IS_CARRYING
    print("Initializing... Go to (0,0)")
    
    robot.drive(150, 0)
    
    print("Searching for 1st line...")
    while right_color.reflection() > 30:
        line_follow_left_step(150, KP)
        wait(5)
    
    ev3.speaker.beep()
    print("1st Line Detected")
    
    while right_color.reflection() < 40:
        line_follow_left_step(150, KP)
        wait(5)
    
    print("Approaching 2nd Line (Goal: (0,0))...")

    while True:
        line_follow_left_step(150, KP) 
        
        if ultra.distance() < 35: 
            robot.stop()
            print("Object Detected at Start")
            
            current_pos = (0, 0)
            current_facing = DIR_E
            
            robot.straight(25) 
            grab_motor.run_until_stalled(1000, then=Stop.HOLD, duty_limit=100)
            IS_CARRYING = True 
            wait(50)
            rgb = jcolor.rgb()
            
            # 찾은 개수 증가
            found_objects += 1
            print("Status: Found {}/{}".format(found_objects, TOTAL_GRID_OBJECTS))

            # 배달
            run_delivery_sequence(rgb, (0, 0))
            
            # ★ 수정된 부분: 만약 이게 4번째(마지막) 물건이었다면, 
            # 로봇은 현재 (0, -2)에 있습니다. (0,0)으로 재정렬하면 안 됩니다.
            if found_objects >= TOTAL_GRID_OBJECTS:
                print("Last object found at origin. Staying at (0, -2).")
                # current_pos는 execute_delivery_and_return 안에서 이미 (0, -2)로 설정됨
                announce_position()
                return

            # 마지막 물건이 아닐 때만 (0,0) 재정렬 수행
            current_pos = (0, 0)
            current_facing = DIR_E
            announce_position()
            return

        if right_color.reflection() < 30:
            robot.stop()
            ev3.speaker.beep()
            print("Arrived at (0,0)")
            
            current_pos = (0, 0)
            current_facing = DIR_E
            print("DEBUG: Before straight(55)")
            robot.straight(55)
            print("DEBUG: After straight(55)")
            announce_position()
            break
        wait(5)


def start_exploration():
    global current_pos, found_objects
    
    grab_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=100)
    go_to_origin()
    
    # go_to_origin에서 이미 4개를 다 찾았을 수도 있으므로 체크
    if found_objects >= TOTAL_GRID_OBJECTS:
        print("Grid Cleared at Origin. Switching to Bonus Phase.")
        run_bonus_phase()
        return

    visited_nodes.add(current_pos)
    
    while True:
        # 루프 시작 시점 체크
        if found_objects >= TOTAL_GRID_OBJECTS:
            print("Grid Cleared. Switching to Bonus Phase.")
            break
            
        next_target = find_nearest_unvisited()
        
        if next_target is None:
            print("Grid Clear - No Unvisited Nodes")
            break
        
        print("Next:", next_target)
        
        path = bfs_path(current_pos, next_target)
        interrupted = run_grid_path(path)
        
        if interrupted:
            print("Objects Found: {}/{}".format(found_objects, TOTAL_GRID_OBJECTS))
            
            # ★ 핵심 수정: 물건을 찾았으면, 목표 개수를 채웠는지 즉시 확인하고 탈출
            if found_objects >= TOTAL_GRID_OBJECTS:
                print("Target Reached! Breaking loop.")
                break 
                
            continue 

        visited_nodes.add(current_pos)
        
    run_bonus_phase()

while True:
    if Button.CENTER in ev3.buttons.pressed():
        ev3.speaker.beep()
        print("Start")
        start_exploration()              
        break
    wait(10)