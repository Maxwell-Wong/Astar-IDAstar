import numpy as np
import time 
import copy
# 最终状态
end_state = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 0]]
node_num = 0
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

    ]


CLOSE = {}

# 方向数组
dx = [0, -1, 0, 1]
dy = [1, 0, -1, 0]
def is_goal(node):
	index = 1
	for row in node:
		for col in row:
			if(index != col):
				break
			index += 1
	return index == 16
# 曼哈顿距离（注意：不需要计算‘0’的曼哈顿值，否则不满足Admittable)
def manhattan(state): 
    M = 0 
    for i in range(4):
        for j in range(4):
            if state[i][j] == end_state[i][j] or state[i][j] == 0:
                continue
            # if state[i][j] == 0:
            #     x, y = 3, 3 
            else:
                x = int((state[i][j] - 1) / 4)   # 最终坐标
                y =  state[i][j] - 4 * x - 1
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

def generateChild(state, Hx):  # 生成子结点
    child = [] # 用于保存生成的子结点
    for i in range(4):       
            for j in range(4):
                if state[i][j] != 0:
                    continue
                for d in range(4):
                    new_x = i + dx[d]
                    new_y = j + dy[d]
                    if 0 <= new_x <= 3 and 0 <= new_y <= 3:
                        new_state = copy.deepcopy(state)
                        new_state[i][j], new_state[new_x][new_y] = new_state[new_x][new_y], new_state[i][j]
                        child.append(new_state)
                
    return sorted(child, key=lambda x: Hx(x))  # 将子结点进行排序，以H(n)递增排序


def IDAsearch(path, g, Hx, bound): # 
    global node_num
    node = path[-1]  # 取出path中的start结点,这里相当于如果迭代超过了bound，回来继续迭代的开始结点，也就是path中的最后一个结点
    node_num +=1
    f = g + Hx(node) 
    if f > bound:      # 如果f(n)的值大于bound则返回f(n)
        return f
    if node == end_state: # 目标检测，以0表示找到目标
        return 0  

    Min = 99999
    CLOSE[str(node)] = g
    for c in generateChild(node, Hx): # 遍历该结点所扩展的孩子结点
            path.append(c)
            t = IDAsearch(path, g+1, Hx, bound)
            if t == 0: # 如果得到的返回值为0，表示找到目标，迭代结束
                return 0
            if t < Min: # 如果返回值不是0，说明f>bound，这时对Min进行更新，取值最小的返回值作为Min
                Min = t
            path.pop() # 深搜回溯

    return Min

def IDAstar(start, Hx):
    bound = Hx(start) # IDA*迭代限制
    path = [start] # 路径集合, 视为栈
    CLOSE[str(start)] = 0

    while(True):
        ans = IDAsearch(path, 0, Hx,bound) # path, g, Hx, bound
        if(ans == 0):
            return (path,bound)
        if ans == -1:
            return None
        bound = ans # 此处对bound进行更新


if __name__ == '__main__':
    f = open('IDAStar_Difresult.txt','w')
    for idx, test in enumerate(init_state):
        time1 = time.time()
        PATH,BOUND = IDAstar(test, misplaced)
        time2 = time.time()
        PATH = np.asarray(PATH)

        test = np.asarray(test)

        for i, p in enumerate(PATH):  #路径打印
            if i == 0:
                 print("15-Puzzle initial state:")
                 f.write("15-Puzzle initial state:\n")
                
                 p = np.asarray(p)
                 print(p)
                 f.write('%s\n\n' %(str(p)))
            else:
                print('Move: %d' %(i))
                
                f.write('Move: %d \n' %(i))
                p = np.asarray(p)
                print(p)
                f.write("%s \n" %(str(p)))

        print('Test %d, Total Step %d' %(idx+1, len(PATH)-1))
        print("Used Time %f" %(time2-time1), "sec")
        print("Expanded %d nodes" %(node_num))
        print("Bound: %d" %(BOUND))

        f.write('Test %d, Total Step %d \n' %(idx+1, len(PATH)-1))
        f.write("Used Time %f sec\n" %(time2-time1))
        f.write("Expanded %d nodes\n" %(node_num))
        f.write("Bound: %d\n\n" %(BOUND))

