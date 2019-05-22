class node(object):
    def __init__(self,father = -1,walkable = -1,Gvalue = -1 , Hvalue = -1,
                 up = 1,down = 1,left = 1,right = 1,loc = [0,0],
                 leftG= 64 ,upG= 64,downG= 64 ,rightG = 64,
                 leftH=64, upH=64, downH=64, rightH=64):
        self.father = father
        self.walkable = walkable
        self.Gvalue = Gvalue
        self.Hvalue = Hvalue
        self.up = up
        self.right = right
        self.down = down
        self.left = left
        self.loc = loc
        self.leftG = leftG
        self.rightG=rightG
        self.downG=downG
        self.upG=upG
        self.leftH = leftH
        self.rightH = rightH
        self.downH = downH
        self.upH = upH
qizi = [[0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  0,  0,  0],
        [0,   0,  0,  0,  0,  1,  0,  0],
        [0,   0,  0,  1,  1,  1,  0,  1],
        [0,   0,  0,  0,  0,  1,  1,  1],
        [0,   0,  0,  0,  0,  0,  0,  0]]
qizicopy = [[1,   0,  0,  0,  0,  0,  0,  0],
        [0,   1,  0,  0,  0,  0,  0,  0],
        [0,   0,  1,  0,  0,  0,  0,  0],
        [0,   0,  0,  1,  0,  0,  0,  0],
        [0,   0,  0,  0,  1,  0,  0,  0],
        [0,   0,  0,  0,  0,  1,  0,  0],
        [0,   0,  0,  0,  0,  0,  1,  1],
        [0,   0,  0,  0,  0,  0,  0,  0]]
allnode = [[],[],[],[],[],[],[],[]]
i = 0
for M in range(8):
    for N in range(8):
        allnode[M].append(node())
for T in qizi:
    qizi[i] = T[::-1]
    i = i + 1
for j in range(8):
    for i in range(8):
        for j in range(8):
            allnode[i][j].loc = [i,j]
            if i-1 >= 0 :
                allnode[i][j].up = qizi[i-1][j]
            if i+1 <= 7 :
                allnode[i][j].down = qizi[i+1][j]
            if j-1 >= 0 :
                allnode[i][j].left = qizi[i][j - 1]
            if j+1 <= 7 :
                allnode[i][j].right = qizi[i][j+1]
    for i in range(8):
        for j in range(8):
            if allnode[i][j].up+allnode[i][j].down+allnode[i][j].left+allnode[i][j].right >= 3:
                qizi[i][j] = 1
for T in qizi:
    print(T)
def jueduiroute(start = [2,3], over = [5,4],qizi = qizi):
    kaishi = allnode[start[0]][start[1]]
    nownode = kaishi
    nownodeloc = start
    openlist = []
    closelist = []
    if nownode.left == 0:
        nownode.leftG = abs(nownode.loc[0]-start[0])+abs(nownode.loc[1]-start[1])
        nownode.leftH = abs(nownode.loc[0]-over[0])+abs(nownode.loc[1]-over[1])
    if nownode.right == 0:
        nownode.rightG = abs(nownode.loc[0]-start[0])+abs(nownode.loc[1]-start[1])
        nownode.rightH = abs(nownode.loc[0]-over[0])+abs(nownode.loc[1]-over[1])
    if nownode.up == 0:
        nownode.upG = abs(nownode.loc[0]-start[0])+abs(nownode.loc[1]-start[1])
        nownode.upH = abs(nownode.loc[0]-over[0])+abs(nownode.loc[1]-over[1])
    if nownode.down == 0:
        nownode.downG = abs(nownode.loc[0]-start[0])+abs(nownode.loc[1]-start[1])
        nownode.downH = abs(nownode.loc[0]-over[0])+abs(nownode.loc[1]-over[1])
