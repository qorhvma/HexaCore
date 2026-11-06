from math import *
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

'''
대한민국 표준 26, 16 (넓이, 폭)
지형을 고른다. (경계 값은 불가능)
x 좌표
0~16 : 0단
16~32 : 1단
...
y 좌표
0 : 1단
6 : 2단
...


몸체의 위치는 femur 관절의 위치로 한다.
'''


stairs_H = 8
stairs_W = 16


tibia = 12
min_tibia_angle = -100; max_tibia_angle=80      # 임의 값 측정 필요
femur = 7.7
min_femur_angle = -150; max_femur_angle=150      # 임의 값 측정 필요
betweenD = 13.7

robot_W = 21
robot_H = 5.6


def getLegPoints(bodyPoint):
    result = [set(), set(), set()]
   
    for t in range(min_tibia_angle, max_tibia_angle, 2):
        t = radians(t)
        for f in range(min_femur_angle, max_femur_angle, 2):
            f = radians(f)+t-pi/2
            t -= pi/2
            blocked = 0
            for i, b in enumerate([-betweenD, 0, betweenD]):
                dx = tibia*cos(t) + femur*cos(f)+bodyPoint[0]+b
                dy = tibia*sin(t) + femur*sin(f)+bodyPoint[1]+b*stairs_H/stairs_W
                dx = round(dx, 0)
                dy = round(dy, 0)
                dan = dy//stairs_H+1
                # print(dx, dy)
                if (dy%stairs_H != 0 or (dan-1)*16 >= dx or dx >= dan*16): continue
                if (fabs(dx-bodyPoint[0]-b)>=10): continue

                # coxa 위치
                cx = bodyPoint[0] + b
                cy = bodyPoint[1] + b*stairs_H/stairs_W
                # 무릎 위치
                kx = cx + femur*cos(f)
                ky = cy + femur*sin(f)

                ok = True
                for s in np.linspace(0, 1, 8):
                    if inside_stairs(cx+s*(kx-cx), cy+s*(ky-cy)): ok = False
                    if inside_stairs(kx+s*(dx-kx), ky+s*(dy-ky)): ok = False
                if not ok: 
                    blocked += 1
                    continue
                result[i].add((dx, dy))
            # print("Helloworld",blocked)


    return result

def inside_stairs(x, y):
    dan = x // stairs_W          # x로 단을 구함
    return y < dan * stairs_H    # 그 단 표면보다 아래면 안쪽     # 그 높이에서 계단이 차지한 영역
# print(inside_stairs(20, 2))
# exit()
def stair_hits_body(w, h):
    for dan in range(0, 5):
        nx, ny = dan * stairs_W, dan * stairs_H      # 계단 코 좌표
        if (w - robot_W/2 < nx < w + robot_W/2 and
            h - robot_H/2 < ny < h + robot_H/2):
            return True
    return False

V = {}

# VT = [[[] for i in range(1005)] for i in range(1005)]
for h in range(8, stairs_H*5):
    for w in range(0, 48):
        vi = w + (h-8)*48              # 위치로 인덱스 계산
        if (inside_stairs((w-robot_W/2), (h-robot_H/2))
         or inside_stairs((w+robot_W/2), (h-robot_H/2))
         or inside_stairs((w-robot_W/2), (h+robot_H/2))
         or inside_stairs((w+robot_W/2), (h+robot_H/2))
         or stair_hits_body(w, h)):
            V[vi] = ([set(), set(), set()], w, h)   # 빈 노드
        else:
            V[vi] = (getLegPoints((w, h)), w, h)
            print(w, h)

for m in range(3):
    print(m, len(V[0][0][m]))          # 시작점 발판 개수
    print('  공통', len(V[0][0][m] & V[1][0][m]))   # 옆칸과 공통

# 위치 끼리는 묶여 있음
# 각도만 같으면 위치끼리도 묶인다.


# if (vi):
#     for i in range(vi-1):
#         for j in range(i, vi):
#             rchk = False
#             for m in range(0, 3):
#                 cnt = 0
#                 cnt = len(V[i][0][m] & V[j][0][m])
#                 if (cnt>=2): rchk = True
#             if (rchk):
#                 VT[i].append(j)
#                 VT[j].append(i)
           
# else: print("없어요...")

# queue = [0]
# chk = [0 for i in range(vi)]
# chk[0]=1
# dist = np.full((22, 48), np.nan)
# dist[V[0][2]-8][V[0][1]] = chk[0]

# while (queue):
#     x = queue.pop(0)

#     for i in VT[x]:
#         if (chk[i] or V[i][1]-V[x][1] <= 0 and V[i][2]-V[x][2]<=0): continue
#         queue.append(i)
#         chk[i]=chk[x]+1
#         dist[V[i][2]-8][V[i][1]] = chk[i]
        
# BFS2
par = {}
def connected(a, b):          # a, b = V의 인덱스
    for m in range(3):
        if len(V[a][0][m] & V[b][0][m]) < 2:
            return False
    return True

def idx(w, hi): return w + hi*48

dist = np.full((stairs_H*5, 48), np.nan)
start = (0, 0)                    # (w, h_index)
dist[0][0] = 0
queue = deque([start])

DX = (0, 1, 0, -1); DY = (1, 0, -1, 0)

while queue:
    w, hi = queue.popleft()
    for k in range(4):
        nw, nh = w + DX[k], hi + DY[k]
        if nw < 0 or nh < 0 or nw >= 48 or nh >= 22: continue
        if not np.isnan(dist[nh][nw]): continue
        if not connected(idx(w, hi), idx(nw, nh)): continue
        dist[nh][nw] = dist[hi][w] + 1
        queue.append((nw, nh))
        par[(nw, nh)] = (w, hi)

def get_path(goal):
    path = [goal]
    while path[-1] in par:
        path.append(par[path[-1]])
    return path[::-1]             # 뒤집어서 시작→끝

cands = [(w, hi) for hi in range(stairs_H*5) for w in range(48)
         if not np.isnan(dist[hi][w]) and w >= 46]
goal = cands[0]  
path = get_path(goal)         # 원하는 도착점
for w, hi in path:
    print(w, hi + 8)
    plt.scatter(w, hi, 5, 'r')

for a, b in zip(path, path[1:]):
    ia, ib = idx(*a), idx(*b)
    for m in range(3):
        shared = V[ia][0][m] & V[ib][0][m]
        print(m, sorted(shared)[:3])

        # BFS 돌면서 dist[h][w] = 깊이
plt.imshow(dist, origin='lower', cmap='viridis')
plt.colorbar()

for a, b in zip(path, path[1:]):
    ia, ib = idx(*a), idx(*b)
    for m in range(3):
        print(len(V[ia][0][m] & V[ib][0][m]), end=' ')
    print()




# tmp = getLegPoints((20, 15))
# plt.scatter([i[0] for i in tmp], [i[1] for i in tmp], 1)
# plt.scatter([20+b for b in [-betweenD, 0, betweenD]], [15+b*stairs_H/stairs_W for b in [-betweenD, 0, betweenD]], 5, 'r')
# plt.title("Tottenham Players Goal and Assists")
# plt.xlabel("Goals")
# plt.ylabel("Assists")
# plt.xlim(0, 50)
# plt.ylim(0, 50)
plt.show()