import numpy as np
#from tqdm import tqdm
import sys

L2 = 10.5
L3 = 9
L4 = 2.3
L5 = 16

r = 17
#r = np.sqrt(L4**2 + L5**2)
limit_theta0 = 181
limit_theta1 = 181
limit_theta2 = 156
limit_theta3 = 171

theta = [0,0,0,0,0,0]

def cos(angle):
    cosine = np.cos(angle)
    return cosine
def sin(angle):
    sine = np.sin(angle)
    return sine
def tan(angle):
    tangent = np.tan(angle)
    return tangent

x = float(input("Enter the value of x: "))
y = float(input("Enter the value of y: "))
z = float(input("Enter the value of z: "))

r1 = np.sqrt(x**2 + y**2)

if r1 < 7.2 and z < 10:
    print("Self Collision !!!")
    sys.exit()
if z < -8:
    print("End effector is trying to move below surface !!!")
    sys.exit()
    
def cubical_boundary():        
    if x > 20 or x < -20 or z > 20 or y > 20 or y < -20:
        print("Cubical Workspace Restriction !!!")
        sys.exit()
    return 0

def spherical_boundary():
    if r1 < 7.2 or r1 > 32:
        print("Spherical Workspace Restriction !!!")
        sys.exit()

#cubical_boundary()
spherical_boundary()    

theta[0] = np.arctan2(y,x) 

def defected_area():
    s = False
    for j in range(0,limit_theta2):
        if s == False:
            for i in range(0,limit_theta3):
                
                theta[3] = np.deg2rad(i)
                theta[2] = np.deg2rad(j)
                
                fwd_x = cos(theta[0])*(L2*cos(theta[1]) + L3*cos(theta[1] - theta[2]) + r*cos(theta[1] - theta[2] - theta[3]))
                fwd_y = sin(theta[0])*(L2*cos(theta[1]) + L3*cos(theta[1] - theta[2]) + r*cos(theta[1] - theta[2] - theta[3]))
                fwd_z = L2*sin(theta[1]) + L3*sin(theta[1] - theta[2]) + r*sin(theta[1] - theta[2] - theta[3])
                
                error_x = np.abs(fwd_x - x)
                error_y = np.abs(fwd_y - y)
                error_z = np.abs(fwd_z - z)
                
                if error_x < 1 and error_y < 1 and error_z < 1:
                    theta[0] = np.rad2deg(theta[0])
                    theta[1] = np.rad2deg(theta[1])
                    theta[2] = np.rad2deg(theta[2])
                    theta[3] = np.rad2deg(theta[3])
                    
                    #print(f"theta1: {theta[0]}\ntheta2: {theta[1]}\ntheta3: {theta[2]}\ntheta4: {theta[3]}")
                    print(f"\n{round(theta[0])},{round(theta[1])},{round(theta[2])},{round(theta[3])}")
                    print(f"cal_x: {fwd_x}; cal_y: {fwd_y}; cal_z: {fwd_z}")
                    print(f"error_x: {error_x}; error_y: {error_y}; error_z: {error_z}")
                    s = True
                    sys.exit()
                    break
                
        else:
            break

if r1 >= 5 and r1 <= 17 and z >= -8 and z <= 15:
    theta[1] = np.deg2rad(111)
    defected_area()

elif r1 > 17 and r1 <= 26 and z >= -8 and z <= 15:
    theta[1] = np.deg2rad(65)
    defected_area()

def thetas():
    s = False
    for j in range(0,limit_theta1):
        if s == False:
            for k in range(0,limit_theta2):
                theta[1] = np.deg2rad(j)
                theta[2] = np.deg2rad(k)
                
                fwd_x = cos(theta[0])*(L2*cos(theta[1]) + L3*cos(theta[1] - theta[2]) + r*cos(theta[1] - theta[2] - theta[3]))
                fwd_y = sin(theta[0])*(L2*cos(theta[1]) + L3*cos(theta[1] - theta[2]) + r*cos(theta[1] - theta[2] - theta[3]))
                fwd_z = L2*sin(theta[1]) + L3*sin(theta[1] - theta[2]) + r*sin(theta[1] - theta[2] - theta[3])
                
                error_x = np.abs(fwd_x - x)
                error_y = np.abs(fwd_y - y)
                error_z = np.abs(fwd_z - z)
                
                if error_x < 1 and error_y < 1 and error_z < 1:
                    theta[0] = np.rad2deg(theta[0])
                    theta[1] = j
                    theta[2] = k
                    theta[3] = np.rad2deg(theta[3])
                    
                    #print(f"theta1: {theta[0]}\ntheta2: {theta[1]}\ntheta3: {theta[2]}\ntheta4: {theta[3]}")
                    print(f"\n{round(theta[0])},{round(theta[1])},{round(theta[2])},{round(theta[3])}")
                    print(f"error_x: {error_x}; error_y: {error_y}; error_z: {error_z};")
                    s = True
                    break
        else:
            break
            

for i in range(0,limit_theta3):
    i = np.deg2rad(i)
    
    theta[3] = i
    
    # for theta [2]:
    a2 = 2*L2*L3 + 2*r*L2*cos(theta[3])
    b2 = -2*r*L2*sin(theta[3])
    c2 = x**2 + y**2 + z**2 - L2**2 - L3**2 - r**2 - 2*r*L3*cos(theta[3])
    
    sqr_root_2_value = a2**2 + b2**2 - c2**2
    
    if sqr_root_2_value > 0:
        theta[2] = np.arctan2(c2 , np.sqrt(sqr_root_2_value)) - np.arctan2(a2,b2)
        
    else:
        continue
    
    # for theta[1]
    
    a1 = -L3*sin(theta[2]) -r*sin(theta[2] + theta[3])
    b1 = L2 + r*cos(theta[2] + theta[3])
    c1 = z
    
    sqr_root_1_value = a1**2 + b1**2 - c1**2 
    
    if sqr_root_1_value > 0:
        theta[1] = np.arctan2(c1 , np.sqrt(sqr_root_1_value)) - np.arctan2(a1,b1)
        
    else:
        continue
    
    fwd_x = cos(theta[0])*(L2*cos(theta[1]) + L3*cos(theta[1] - theta[2]) + r*cos(theta[1] - theta[2] - theta[3]))
    fwd_y = sin(theta[0])*(L2*cos(theta[1]) + L3*cos(theta[1] - theta[2]) + r*cos(theta[1] - theta[2] - theta[3]))
    fwd_z = L2*sin(theta[1]) + L3*sin(theta[1] - theta[2]) + r*sin(theta[1] - theta[2] - theta[3])
    
    error_x = np.abs(fwd_x - x)
    error_y = np.abs(fwd_y - y)
    error_z = np.abs(fwd_z - z)
    
    if error_x < 1 and error_y < 1 and error_z < 1 :
        
        thetas()
        break
    else:
        continue