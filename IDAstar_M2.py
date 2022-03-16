import numpy as np
import time 
import copy
# 最终状态
end_state = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0)
node_num = 0
# 初始状态测例集
init_state = [
     (1, 15, 7, 10,9, 14, 4, 11, 8, 5,0 , 6, 13, 3, 2, 12),
     (1, 7, 8, 10,6, 9, 15, 14, 13, 3, 0, 4, 11, 5, 12, 2),
     (5, 6, 4, 12, 11, 14, 9, 1, 0, 3, 8, 15, 10, 7, 2, 13),
     (14, 2, 8, 1, 7, 10, 4, 0, 6, 15, 11, 5, 9, 3, 13, 12),

   (11, 3, 1, 7,4, 6 ,8 ,2,15 ,9 ,10, 13,14, 12, 5 ,0),  # Dif_IDA_Starresult.txt
     (14, 10, 6, 0,4, 9 ,1 ,8,2, 3, 5 ,11,12, 13, 7 ,15),
     (0, 5, 15, 14,7, 9, 6 ,13,1, 2 ,12, 10,8, 11, 4, 3),
     (6 ,10, 3, 15,14, 8, 7, 11,5, 1, 0, 2,13, 12, 9 ,4)

    ]


CLOSE = set() #  用于判重
path = []

# 方向数组
dx = [0, -1, 0, 1]
dy = [1, 0, -1, 0]

# 曼哈顿距离（注意：不需要计算‘0’的曼哈顿值，否则不满足Admittable)
def manhattan(state): 
    M = 0 
    for t in range(16):
            if state[t] == end_state[t] or state[t]== 0:
                continue
            else:
                x =  (state[t] - 1) // 4   # 最终坐标
                y =  state[t] - 4 * x - 1
                dx = t // 4 # 实际坐标
                dy = t % 4
                M += (abs(x - dx) + abs(y - dy))
    return M


def generateChild():  # 生成子结点
   movetable = [] # 针对数码矩阵上每一个可能的位置，生成其能够移动的方向列表
   for i in range(16):
       x, y = i % 4, i // 4
       moves = []
       if x > 0: moves.append(-1)    # 左移
       if x < 3: moves.append(+1)    # 右移
       if y > 0: moves.append(-4)    # 上移
       if y < 3: moves.append(+4)    # 下移
       movetable.append(moves) 
   def children(state):
       idxz = state.index(0) # 寻找数码矩阵上0的坐标
       l = list(state) # 将元组转换成list，方便进行元素修改
       for m in movetable[idxz]:
           l[idxz] = l[idxz + m] # 数码交换位置
           l[idxz + m] = 0
           yield(1, tuple(l)) # 临时返回
           l[idxz + m] = l[idxz]
           l[idxz] = 0
   return children




def IDAsearch(g, Hx, bound): 
    global node_num
    node = path[-1]  # 取出path中的start结点,这里相当于如果迭代超过了bound，回来继续迭代的开始结点，也就是path中的最后一个结点
    node_num +=1
    f = g + Hx(node) 
    if f > bound:      # 如果f(n)的值大于bound则返回f(n)
        return f
    if node == end_state: # 目标检测，以0表示找到目标
        return 0  
    
    Min = 99999 # 保存子结点中返回的最小的f值，作为下次迭代的bound
    generator = generateChild() # 获取generateChild()中生成的child
    for cost, state in generator(node):
        if state in CLOSE: continue
        path.append(state)
        CLOSE.add(state) # 利用set查找的优势，进行路径检测
        t = IDAsearch(g+1,Hx,bound)

        if t == 0: return 0
        if t < Min: Min = t

        path.pop() # 回溯
        CLOSE.remove(state) 
    return Min
   

def IDAstar(start, Hx):
    global CLOSE
    global path
    bound = Hx(start) # IDA*迭代限制
    path = [start] # 路径集合, 视为栈
    CLOSE = {start}


    while(True):
        ans = IDAsearch(0, Hx,bound) # path, g, Hx, bound
        if(ans == 0):
            return (path,bound)
        if ans == -1:
            return None
        bound = ans # 此处对bound进行更新


if __name__ == '__main__':
    f = open('DifIDAStar_result_M.txt','w')
    for idx, test in enumerate(init_state):
        time1 = time.time()
        PATH,BOUND = IDAstar(test, manhattan)
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
