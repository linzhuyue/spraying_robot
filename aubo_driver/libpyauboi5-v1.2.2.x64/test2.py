def Caculate_Path_point(Sector_Nums, Sector_Width, Sector_Length, pose, Left_Right_Flag):
    Res_Left = []
    Res_Right = []
    Last_Queue = []
    for i in range(Sector_Nums):
        Res_Left.append({i: (pose[0], pose[1] - Sector_Length / 2.0, pose[2] - i * Sector_Width)})
        Res_Right.append({i: (pose[0], pose[1] + Sector_Length / 2.0, pose[2] - i * Sector_Width)})
    if Left_Right_Flag:
        for i in range(len(Res_Left)):
            if i % 2 == 0:  # Even
                Last_Queue.append(Res_Left[i][i])
                Last_Queue.append(Res_Right[i][i])
            else:
                Last_Queue.append(Res_Right[i][i])
                Last_Queue.append(Res_Left[i][i])
    else:
        if i % 2 != 0:  # Even
            Last_Queue.append(Res_Left[i][i])
            Last_Queue.append(Res_Right[i][i])
        else:
            Last_Queue.append(Res_Right[i][i])
            Last_Queue.append(Res_Left[i][i])
    return Res_Left, Res_Right, Last_Queue
k,j,p=Caculate_Path_point(3,1,1,(1,2,3),1)
print k
print j
print p