import numpy as nm

#Link Lengths (L)
L1 = 2.5
L2 = 10.5
L3 = 10
L4 = 2

#Link Offset (d)
d1 = 0
d6 = 17



rad = nm.pi/180

a = [0, 0, L2, L3, L4, 0]
alpha = [0, 90 , 0 , 0 , -90  , 0]
d = [d1, 0, 0, 0, 0, d6]


theta = [0,0,0,0,0,0]
add_theta = [5,-2,-3,0,0,0]  #Additional theta[i] from X[i-1] to X[i]
#add_theta = [0,0,0,0,0,0]

for i in range(6):
    if i == 3:
        theta[i] = (-(float(input(f"Theta {i+1} = "))) + add_theta[i])
    else:
        theta[i] = (float(input(f"Theta {i+1} = "))+add_theta[i])
print("",end="\n")

matrix = []
for j in range(6):
    c_theta = nm.cos(theta[j]*rad)
    s_theta = nm.sin(theta[j]*rad)
    
    c_alpha = nm.cos(alpha[j]*rad)
    s_alpha = nm.sin(alpha[j]*rad)
    
    matrix1 = nm.array([[       c_theta         ,       -(s_theta)        ,        0       ,         a[j]      ],
                        [ (s_theta)*(c_alpha)   ,   (c_theta)*(c_alpha)   ,   -(s_alpha)   ,   -(s_alpha)*d[j] ],
                        [ (s_theta)*(s_alpha)   ,   (c_theta)*(s_alpha)   ,     c_alpha    ,    (c_alpha)*d[j] ],
                        [          0            ,       0             ,       0            ,           1       ]
                        ])

    matrix.append(matrix1)

transformed_point = (nm.dot(nm.dot(nm.dot(nm.dot(nm.dot(matrix[0] , matrix[1]) , matrix[2]) , matrix[3]) , matrix[4]) , matrix[5]))
x = transformed_point[0,3]
y = transformed_point[1,3]
z = transformed_point[2,3]

if z <= 2:
    print("Workspace Restriction !!!")
elif x>=-5 and x<=5 and y>=-5 and y<=5 and z<=15:
    print("Workspace Restriction !!!")
    print(f"[ {x} {y} {z} ]")
else:
    print(f"(x , y , z) = ({x}, {y}, {z})")
#print(transformed_point)














