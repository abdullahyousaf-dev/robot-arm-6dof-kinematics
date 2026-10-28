import numpy as np

l2 = 10.4
l3 = 15

theta = [0,0,0]

t0 = [0,0,0]
tf = [0,0,0]
new_t = [0,0,0]

size = 100

pt1 = [0,10,0]
pt2 = [0,18,0]
pt3 = [5,10,0]
pt4 = [5,18,0]


(x,y,z) = (5,18,0)

for i in range(1,4):
    initial = i
    final = i+1
    
    if initial == 1:
        x0 = pt1[0]
        y0 = pt1[1]
        z0 = pt1[2]
        
    elif initial == 2:
        x0 = pt2[0]
        y0 = pt2[1]
        z0 = pt2[2]
        
    elif initial == 3:
        x0 = pt3[0]
        y0 = pt3[1]
        z0 = pt3[2]
        
    elif initial == 4:
        x0 = pt4[0]
        y0 = pt4[1]
        z0 = pt4[2]

    if final == 1:
        xf = pt1[0]
        yf = pt1[1]
        zf = pt1[2]
        
    elif final == 2:
        xf = pt2[0]
        yf = pt2[1]
        zf = pt2[2]
        
    elif final == 3:
        xf = pt3[0]
        yf = pt3[1]
        zf = pt3[2]
        
    elif final == 4:
        xf = pt4[0]
        yf = pt4[1]
        zf = pt4[2]
    

    #Initial Thetas
    t0[0] = np.arctan2(y0,x0)

    cos02 = (x0**2 + y0**2 + z0**2 - l2**2 - l3**2)/(2*l2*l3)
    sin02 = np.sqrt(1-cos02**2)

    t0[2] = np.arctan2(sin02,cos02)

    a01 = -1*l3*np.sin(t0[2])
    b01 = l2 + l3*np.cos(t0[2])
    c01 = z0

    sq_root = np.sqrt(a01**2 + b01**2 - c01**2)

    t0[1] = (np.arctan2((c01),(sq_root))) - (np.arctan2(a01,b01))

    fwd_x0 = np.cos(t0[0])*(l2*np.cos(t0[1]) + l3*np.cos(t0[1] - t0[2]))
    fwd_y0 = np.sin(t0[0])*(l2*np.cos(t0[1]) + l3*np.cos(t0[1] - t0[2]))
    fwd_z0 = l2*np.sin(t0[1]) + l3*np.sin(t0[1] - t0[2])
    
    
    #Final Thetas
    tf[0] = np.arctan2(yf,xf)

    cosf2 = (xf**2 + yf**2 + zf**2 - l2**2 - l3**2)/(2*l2*l3)
    sinf2 = np.sqrt(1-cosf2**2)

    tf[2] = np.arctan2(sinf2,cosf2)

    af1 = -1*l3*np.sin(tf[2])
    bf1 = l2 + l3*np.cos(tf[2])
    cf1 = zf

    sq_root = np.sqrt(af1**2 + bf1**2 - cf1**2)

    tf[1] = (np.arctan2((cf1),(sq_root))) - (np.arctan2(af1,bf1))


    fwd_xf = np.cos(tf[0])*(l2*np.cos(tf[1]) + l3*np.cos(tf[1] - tf[2]))
    fwd_yf = np.sin(tf[0])*(l2*np.cos(tf[1]) + l3*np.cos(tf[1] - tf[2]))
    fwd_zf = l2*np.sin(tf[1]) + l3*np.sin(tf[1] - tf[2])
    
    motor_theta = [t0[0],t0[1],t0[2]]
    print(initial, final)
    
    for j in range(size):

        motor_theta[0] = ((tf[0] - t0[0])/size) + motor_theta[0]
        motor_theta[1] = ((tf[1] - t0[1])/size) + motor_theta[1]
        motor_theta[2] = ((tf[2] - t0[2])/size) + motor_theta[2]
        
        print(np.rad2deg(motor_theta[0]))
    print(np.rad2deg(t0[0]),np.rad2deg(tf[0]))
    
    print(x0,y0,z0,xf,yf,zf)
    print(fwd_x0, fwd_y0, fwd_z0)
    print(fwd_xf, fwd_yf, fwd_zf)

