# -- coding: utf-8 --
# 本函数route_plan(qizi)是路径规划出绝对路径，需要qizi(一个二维列表变量)作为输入，输出绝对路径
# 例子：
# [0,   1,  0,  0,  0,  0,  0,  0]转置之后为第0列 从左往右是从上往下
import copy
import numpy as np
miaomiaona = [3,9,15,21,27,33,39,45,51,57,63,69,75,81,87,93]
miaomiaofang = (np.array(miaomiaona)+3).tolist()
qizi = [[0,   1,  0,  0,  0,  0,  0,  0],
        [1,   0,  1,  1,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  1,  0,  0],
        [0,   0,  0,  0,  1,  0,  1,  1],
        [0,   0,  0,  0,  0,  0,  0,  0]]
# qizi = [[0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0],
#         [1,   1,  1,  0,  0,  0,  0,  0],
#         [0,   1,  0,  0,  0,  0,  0,  0],
#         [0,   0,  0,  0,  0,  0,  0,  0]]
# from rowclass import row已将类定义移植到本处，因此不需要导入了

class row(object):
    def __init__(self, label = -1,qizinum = -1,bestsolution = -1,qizi = [],top = -1):
        self.label = label               #该列的列标号
        self.qizinum = qizinum           #该列所拥有棋子数目
        self.bestsolution = bestsolution #最优解的位置
        self.qizi = qizi                 #棋子数组 比如[0,0,0,0,0,0,1,0]
        self.top = top                   #该列中最先出现的棋子的下标,-1则是没有


def route_plan(qizi):
    #allrow是所有的列对象化之后的集合，即对象数组
    allrow = [row(),row(),row(),row(),row(),row(),row(),row()]
    i = 0
    # 所有列对象进行初始化，列坐标，top棋子(没有则为-1)，列所拥有棋子数，最优解位置，棋子数
    for temp in copy.deepcopy(qizi):
        allrow[i].qizi = temp
        allrow[i].label = i
        allrow[i].qizinum = allrow[i].qizi.count(1)
        allrow[i].bestsolution = i
        if max(allrow[i].qizi) == 1:
            allrow[i].top = allrow[i].qizi.index(1)
        i = i+1
    del i
    del temp

    # feikongrow是所有的有棋子的列的对象集合
    feikongrow = []
    # feikongrownotop是所有有棋子的列去掉top棋子之后的对象集合
    feikongrownotop = []
    # kongrow是所有空列的对象集合
    kongrow = []
    for temp in allrow:
        if temp.qizinum != 0:
            feikongrow.append(copy.deepcopy(temp))
            feikongrownotop.append(copy.deepcopy(temp))
        else:
            kongrow.append(copy.deepcopy(temp))
    del temp

    for temp in feikongrownotop:
        temp.qizi[temp.top] = 0
    del temp
    # qizicopy是棋子矩阵去掉top棋子的棋子矩阵
    qizinotop = copy.deepcopy(qizi)
    for temp in qizinotop:
        if max(temp) == 1:
            temp[temp.index(1)] = 0
    del temp
    # yaobandeqizi是去掉top棋子的要搬的棋子,是一个列表的列表，从qizicopy(无top棋子)得到
    yaobandeqizi = []
    hang1 = 0
    for hang in qizinotop:
        single1 = 0
        for single in hang[::-1]:
            if single == 1:
                yaobandeqizi.append(copy.deepcopy([hang1,7-single1]))
            single1 = single1 + 1
        hang1 = hang1 + 1
    del hang1
    del hang
    del single1
    del single
    # 默认初始位置为[0,0]，剩下的是他要去的绝对位置
    location = [[0,0]]
    qiziloc = 0
    try:
        for temp5 in kongrow:
            location.append(copy.deepcopy([yaobandeqizi[qiziloc][0],7-yaobandeqizi[qiziloc][1]]))
            location.append(copy.deepcopy([temp5.label,7-temp5.bestsolution]))
            qiziloc = qiziloc + 1
        del temp5
    except:
        pass

    for temp6 in feikongrow:
        if temp6.top == temp6.bestsolution:
            continue
        location.append(copy.deepcopy([temp6.label,7-temp6.top]))
        location.append(copy.deepcopy([temp6.label, 7-temp6.bestsolution]))
    del temp6
    return location

# print(route_plan(qizi))最初始的绝对坐标
def trans(location):
    sth = []
    j = 0
    # for what in location:
    #     sth.append([what[0],0])
    #     sth.append(what)
    #     sth.append([what[0],0 ])
    sth1 = np.array(location[:-1])
    sth2 = np.array((location[1:]))
    sth3 = sth2 - sth1
    # print(sth3.tolist())简化后的四方向坐标
    last = []
    for temp in sth3:
        # if temp.tolist() == [0,0]:
        #     continue
        last.append(temp.tolist())
    # print(last)
    del temp
    # last1 = []
    # for temp in last:
    #     last1.append(temp)
    #     if j in miaomiaona:
    #         last1.append([1])
    #     if j in miaomiaofang:
    #         last1.append([0])
    #     j = j + 1
    return [y for y in last if y != [0,0]]
chushiroute = route_plan(qizi)
print(chushiroute)
# print(trans(route_plan(qizi)))
# print(qizi)
from implementation import *
import numpy as np

def breadth_first_search_3(graph, start, goal):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current

    return came_from
allroute = []
def mya_star(qizi = qizi,start = (),goal = ()):
    g = SquareGrid(8,8)
    g.walls =np.argwhere(np.array(qizi) == 1).tolist()
    parents = breadth_first_search_3(g, start, goal)
    temproute = []
    temp = goal
    while (1):
        if temp == None:
            break
        allroute.append([list(temp)[0],7-list(temp)[1]])
        temproute.append([list(temp)[0],7-list(temp)[1]])
        temp = parents[temp]
    return temproute[::-1]
route = []
qizicopy = qizi
mm = 0
# print(route_plan(qizi))
temproute = []
for (wtf1,wtf2) in zip(route_plan(qizi)[0:-1], route_plan(qizi)[1:]):
    wtf1 = [wtf1[0],7-wtf1[1]]
    wtf2 = [wtf2[0], 7 - wtf2[1]]
    temproute = mya_star(qizicopy, tuple(wtf1), tuple(wtf2))
    temproute.insert(-1, [6,6,6])#长度为三 代表要减速了
    route.extend(temproute)
    qizicopy[wtf1[0]][wtf1[1]] = 0
    qizicopy[wtf2[0]][wtf2[1]] = 1
    if mm % 2 == 0:
        route.append([1])
    else:
        route.append([0])
    mm = mm + 1
# print(route)
templist = []
nn = 0
linshi = [0,0]
for temp in chushiroute :
    templist.append(abs(temp[0]-linshi[0])+abs(temp[1]-linshi[1]))
    linshi = [temp[0],temp[1]]
print(route)
# transroute = [kong for kong in trans([x for x in route if len(x) == 2]) if kong != [0,0]]
# print(transroute)
# final_last = []
# ww = 0
# jixiebi = 1
# www = 1
# for temp2 in transroute:
#     if ww == templist[www]:
#         final_last.append([jixiebi])
#         jixiebi = 1 - jixiebi
#         www = www + 1
#         ww = 0
#     else:
#         final_last.append(temp2)
#         ww = ww + 1
# print(final_last)
# print(templist)


# import time
# DIR = 0 #代表方向 1前 2后 3左 4右
# nowstep = 0 #当前步所需步数
# start = [0,0]
# for step in route[1:]:
#     if len(step) == 2:
#         linshi = [step[0]-start[0],step[1]-start[1]]
#         start = [step[0],step[1]]
#         step = linshi
#         if step == [0,0]:
#             continue
#         stickin = time.time()
#         if step[0] == 0:
#             nowstep = abs(step[1])
#             if step[1] > 0:
#                 DIR = 1
#             else:
#                 DIR = 2
#         else:
#             nowstep = abs(step[0])
#             if step[0] > 0:
#                 DIR = 4
#             else:
#                 DIR = 3
