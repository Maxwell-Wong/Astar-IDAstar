#coding=utf-8
import copy
import heapq
import numpy as np
import time


# 最终状态
end_state = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 0]]
node_num  = 0

# 初始状态测例集
init_state = [
       [[1, 15, 7, 10],[9, 14, 4, 11], [8, 5,0 , 6],[13, 3, 2, 12]],
     [[1, 7, 8, 10],[6, 9, 15, 14], [13, 3, 0, 4], [11, 5, 12, 2]],
       [[5, 6, 4, 12], [11, 14, 9, 1], [0, 3, 8, 15], [10, 7, 2, 13]],
        [[14, 2, 8, 1], [7, 10, 4, 0], [6, 15, 11, 5], [9, 3, 13, 12]]
         [[11, 3, 1, 7],[4, 6 ,8 ,2],[15 ,9 ,10, 13],[14, 12, 5 ,0]],  # Dif_IDA_Starresult.txt
         [[14, 10, 6, 0],[4, 9 ,1 ,8],[2, 3, 5 ,11],[12, 13, 7 ,15]],
         [[0, 5, 15, 14],[7, 9, 6 ,13],[1, 2 ,12, 10],[8, 11, 4, 3]],
        [[6 ,10, 3, 15],[14, 8, 7, 11],[5, 1, 0, 2],[13, 12, 9 ,4]]
          [[1 , 3 ,10 , 4],[5 , 2  ,6 , 8], [14, 11 ,12 , 0], [7, 13,  9 ,15]]
   ]


# 方向数组
dx = [0, -1, 0, 1]
dy = [1, 0, -1, 0]

OPEN = []

CLOSE = set() # close表，用于判重

path = []

def print_path(node):
    if node.parent != None:
        print_path(node.parent)
    path.append(node.state)
    return path

# 状态结点
class Node(object):
    def __init__(self, gn=0, hn=0, state=None, hash_value=None, parent=None):
       self.gn = gn
       self.hn = hn
       self.fn = self.gn + self.hn
       self.state = state
       self.hash_value = hash_value
       self.parent = parent

    def __lt__(self, node): # heapq的比较函数
        if self.fn == node.fn:
            return self.gn > node.gn
        return self.fn < node.fn


def manhattan(state):
    M = 0
    for i in range(4):
        for j in range(4):
            if state[i][j] == end_state[i][j] or state[i][j] == 0:
                continue
            else:
                x = (state[i][j] - 1) // 4   # 最终坐标
                y = state[i][j] - 4 * x -1 
                M += (abs(x - i) + abs(y - j))
    return M


def misplaced(state):
    sum = 0
    for i in range(4):
        for j in range(4):
            if state[i][j] == 0:
                continue
            if state[i][j] != end_state[i][j]:
                sum += 1
    return sum


def A_star(start, Fx):
    root = Node(0, 0, start, hash(str(start)), None) # gn, hn, state hash_value, parent
    OPEN.append(root)
    heapq.heapify(OPEN)
    CLOSE.add(root.hash_value)
    while len(OPEN) != 0:
        top = heapq.heappop(OPEN)
        global node_num 
        node_num += 1
        if top.state == end_state:
            return print_path(top)
        for i in range(4):
            for j in range(4):
                if top.state[i][j] != 0:
                    continue
                for d in range(4):
                    new_x = i + dx[d]
                    new_y = j + dy[d]
                    if 0 <= new_x <= 3 and 0 <= new_y <= 3:
                        state = copy.deepcopy(top.state)
                        state[i][j], state[new_x][new_y] = state[new_x][new_y], state[i][j]
                        h = hash(str(state))
                        if h in CLOSE:
                            continue
                        CLOSE.add(h)
                        child = Node(top.gn+1, Fx(state), state, h ,top)
                        heapq.heappush(OPEN, child)


if __name__ == '__main__':
    f = open('AStar_Misplaced.txt','w')
    for idx, test in enumerate(init_state):
        time1 = time.time()
        PATH = np.asarray(A_star(test, manhattan))
        time2 = time.time()

        test = np.asarray(test)

        for i, p in enumerate(PATH):  #路径打印
            if i == 0:
                 print("15-Puzzle initial state:")
                 print(p)
                 f.write("15-Puzzle initial state:\n")
                 f.write('%s\n\n' %(str(p)))
            else:
                print('Move: %d' %(i))
                print(p)
                f.write('Move: %d \n' %(i))
                f.write("%s \n" %(str(p)))

        print('Test %d, Total Step %d' %(idx+1, len(path)-1))
        print("Used Time %f" %(time2-time1), "sec")
        print("Expanded %d nodes" %(node_num))

        f.write('Test %d, Total Step %d \n' %(idx+1, len(path)-1))
        f.write("Used Time %f sec\n" %(time2-time1))
        f.write("Expanded %d nodes\n\n" %(node_num))

        OPEN.clear()
        CLOSE.clear()
        path.clear()
    
