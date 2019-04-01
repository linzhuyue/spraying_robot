#!/usr/bin/env python
def getpi(listb):
    lista=[]
    listcc=[]
    for i in listb:
        temp=i/180*3.14
        lista.append((temp,i))
        listcc.append(temp)
    return lista
def getangle(listb):
    lista=[]
    for i in listb:
        temp=i/3.14*180
        lista.append((i,temp))
    return lista
def getangle_new(listb):
    lista=[]
    for i in listb:
        temp=i/3.14*180
        lista.append(temp)
    return lista
def display(listc):
    listpi=[]
    listangle=[]
    for i in listc:
        listpi.append(i[0])
        listangle.append(i[1])
        #print(i)
    print("pi====:",listpi)
    print("angle==:",listangle)
    return listpi
def getpi_for_py(listc):
    listpi=[]
    listangle=[]
    for i in listc:
        listpi.append(i[0])
        listangle.append(i[1])
        #print(i)
    #print("pi====:",listpi)
    #print("angle==:",listangle)
    return listpi
if __name__=="__main__":
    #joint_positions_inpi = [-1.565429989491598, -1.6473701635943812, 0.05049753189086914, -1.4097726980792444,-1.14049534956561487, -0.8895475069154912]
    #kk=getangle(joint_positions_inpi)
    #joint_position_inangle=[-89.73802487531454, -94.43523230795815, 2.894762974635811, -80.81499543129426, -65.37871430630913, -50.993169186238354]
    #aa=getpi(joint_position_inangle)
    #display(kk)
    reslut=[]
    #display(aa)
    Q0=[-0.69,-100.70,102.06,-174.24,-90.16,-45.35]
    Q1=[-0.69,-91.22,62.51,-151.14,-90.16,-45.36]
    #Q0=[-14.49,-49.08,69.91,-203.91,-75.70,-65.06]
    #Q1=[-0.12,-50.14,69.91,-199.52,-89.52,-65.07]
    Q2=[14.27,-44.13,32.03,-165.34,-100.79,-65.07]
    #Q23 = [8.24, -40.93, 15.51, -155.41, -97.56, -65.07]
    Q3=[8.24,-40.93,6.51,-146.41,-97.56,-65.07]
    Q4=[-0.55,-41.01,6.27,-151.46,-93.72,-65.07]
    Q5=[-15.41,-42.01,6.29,-144.16,-73.85,-65.07]
    Q6=[-16.84,-54.61,56.91,-186.31,-73.86,-65.08]
    Q7=[-3.04,-52.20,49.33,-180.78,-84.35,-65.07]
    Q8=[84.81, -124.65, -78.10, 99.59, -96.62, 89.99]
    Q9=[1.4794633333333334, -2.17445, -1.3624111111111112, 1.7372922222222222, -1.6854822222222223, 1.5698255555555556]
    display(getangle(Q9))
    #display(getpi(Q0))
    #reslut.append(display(getpi(Q8)))
    #display(getpi(Q1))
    #reslut.append(display(getpi(Q1)))
    # display(getpi(Q2))
    # reslut.append(display(getpi(Q2)))
    # #display(getpi(Q23))
    # #reslut.append(display(getpi(Q23)))
    # display(getpi(Q3))
    # reslut.append(display(getpi(Q3)))
    # display(getpi(Q4))
    # reslut.append(display(getpi(Q4)))
    # display(getpi(Q5))
    # reslut.append(display(getpi(Q5)))
    # display(getpi(Q6))
    # reslut.append(display(getpi(Q6)))
    # display(getpi(Q7))
    # reslut.append(display(getpi(Q7)))
    print(reslut)

