import math

x = 0;
y = 0;
angle = 0;
distance = 0;
A = [x,y];

def compute_paths(A,angle,distance):
    angle = angle %360;

    AB = compute_AB(A, angle);
    B = compute_B(A,distance,angle);
    B_wall = compute_wall_B(B,AB);
    #A_para_wall = [compute_wall_para_A(A,AB)];
    print(AB)
    print(B)
    print(B_wall)
    return [B,B_wall];

def compute_AB(A, angle):
    if angle == 0 or angle == 180:
        mAB = 0;
        nAB = A[1];
    elif angle == 90 or angle == 270:
        mAB = "undef"
        nAB = A[0] #this represents the line x = A[0]
    else:
        mAB = math.tan(math.radians(angle));
        nAB = A[1] - mAB * A[0];
    return [mAB,nAB];

def compute_B(A, distance,angle):
    #x = (AB[1]-B_wall[1])/(B_wall[0]-AB[0]);
    #y = AB[0]*x+AB[1];
    x=A[0] + distance*math.cos(math.radians(angle));
    y=A[1] + distance*math.sin(math.radians(angle));
    return [x,y];

def compute_wall_B(B, AB):
    if AB[0] == 0:
        mB = "undef";
        nB= B[0]; #this represents the line x = B[0]
    elif AB[0] == "undef":
        mB =0;
        nB=B[1];
    else:
        mB = -1/AB[0];
        nB = B[1] - mB * B[0];
    #aux = distance * math.sqrt(math.pow(AB[0], 2)+1);
    #|AB[1]-nB| = aux;
    #nB=?
    return [mB,nB];


def compute_wall_para_A(A, AB):
    mA = -1/AB[0];
    nA = A[1] - mA * A[0];
    return [mA, nA];


#compute_paths([-4,4],45,math.sqrt(2));
#compute_paths([-4,4],180,1);
#compute_paths([-4,4],90,1);
