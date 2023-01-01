def CalculateDistance(Points):
    # list of tuples (x,y,z)
    temp = []
    for i in Points:
        #Points = [((x1,y1,z1),(x2,y2,z2))]
        x1 = i[0][0]
        y1 = i[0][1]
        z1 = i[0][2]
        x2 = i[1][0]
        y2 = i[1][1]
        z2 = i[1][2]

        distance = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5

        temp.append((((x1,y1,z1),(x2,y2,z2)),distance))
        
    return temp


if __name__ == "__main__":

    POINTS = []

    while True:
        points = input("Enter the points in format x1 y1 z1 x2 y2 z2 : ")
        
        if points == "q":
            break

        P = list(map(int,points.split()))
        POINTS.append(((P[0],P[1],P[2]),(P[3],P[4],P[5])))

    
    # print(POINTS)

    # for i,j in CalculateDistance(POINTS).items():
    #     print(i,"----->",j)

    # print(CalculateDistance(POINTS))

    for res in CalculateDistance(POINTS):
        print(res[0][0],res[0][1],"--->",res[1])
