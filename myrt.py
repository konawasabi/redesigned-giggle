import math
from PIL import Image

def sign(val): #符号を返す
    return math.copysign(1.0,val)

def inner_product(a, b): #3次元ベクトルの内積
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def vec_subtract(a, b):
    result =[]
    for i in [0,1,2]:
        result.append(float(a[i]) - float(b[i]))
    return result

def vec_sum(a, b):
    result =[]
    for i in [0,1,2]:
        result.append(float(a[i]) + float(b[i]))
    return result

def vec_scale(a, sc):
    result =[]
    for i in [0,1,2]:
        result.append(float(a[i]) * sc)
    return result

def vec_length(a):
    result =0
    for i in [0,1,2]:
        result+=float(a[i])**2
    return math.sqrt(result)
    
def rot_x(vec, theta):
    return [vec[0],\
           vec[1]*math.cos(theta) - vec[2]*math.sin(theta),\
           vec[1]*math.sin(theta) + vec[2]*math.cos(theta)]

def rot_y(vec, theta):
    return [vec[0]*math.cos(theta) + vec[2]*math.sin(theta),\
           vec[1],\
           -1.0*vec[0]*math.sin(theta) + vec[2]*math.cos(theta)]

def rot_z(vec, theta):
    return [vec[0]*math.cos(theta) - vec[1]*math.sin(theta),\
            vec[0]*math.sin(theta) + vec[1]*math.cos(theta),\
            vec[2]]

def rot_x_2nd(vec, theta):
    return [vec[0],\
           vec[1]*math.cos(theta) + vec[2]*math.sin(theta),\
           -1.0*vec[1]*math.sin(theta) + vec[2]*math.cos(theta)]

def rot_y_2nd(vec, theta):
    return [vec[0]*math.cos(theta) - vec[2]*math.sin(theta),\
           vec[1],\
           vec[0]*math.sin(theta) + vec[2]*math.cos(theta)]

def rot_z_2nd(vec, theta):
    return [vec[0]*math.cos(theta) + vec[1]*math.sin(theta),\
           -1.0*vec[0]*math.sin(theta) + vec[1]*math.cos(theta),\
            vec[2]]

def rot_x_1st(vec, theta):
    return [vec[0],\
           vec[1]*math.cos(theta) - vec[2]*math.sin(theta),\
           -1.0*vec[1]*math.sin(theta) + vec[2]*math.cos(theta)]

def rot_y_1st(vec, theta):
    return [vec[0]*math.cos(theta) - vec[2]*math.sin(theta),\
           vec[1],\
           vec[0]*math.sin(theta) + vec[2]*math.cos(theta)]

def rot_z_1st(vec, theta):
    return [vec[0]*math.cos(theta) + vec[1]*math.sin(theta),\
           -1.0*vec[0]*math.sin(theta) + vec[1]*math.cos(theta),\
            vec[2]]

def cross_distance(prim,e_view,r_view): #prim: 距離を求めるプリミティプ、e_view: 視線の単位ベクトル、r_view: 視点の位置ベクトル
    #from IPython.core.debugger import Pdb; Pdb().set_trace()
    cross = False
    dist = 1e15
    r_prim = [prim['pX'],prim['pY'],prim['pZ']]
    
    if (prim['pP'] == 1): # 直方体
        #tmp_cross = False
        #tmp_dist = dist
        tmp_vec_list = [[[1, 0, 0], [prim['pa'], 0, 0]],\
                         [[0, 1, 0], [0, prim['pb'], 0]],\
                         [[0, 0, 1], [0, 0, prim['pc']]],\
                         [[-1, 0, 0], [-1.0*prim['pa'], 0, 0]],\
                         [[0, -1, 0], [0, -1.0*prim['pb'], 0]],\
                         [[0, 0, -1], [0, 0, -1.0*prim['pc']]]]# tmp_vecs[0]: 各面の法線ベクトル、[1]: 法線ベクトルの位置
        for tmp_vecs in tmp_vec_list:
            r_vp_prim = vec_subtract(vec_sum(r_prim, tmp_vecs[1]), r_view)
            tmp = inner_product(tmp_vecs[0] ,e_view) * sign(prim["pSG"])
            if (tmp < 0): # 内積が<0なら、平面の表側から入射
                tmp_dist_tmp = inner_product(tmp_vecs[0], r_vp_prim) / tmp
                crosspoint = vec_sum(vec_scale(e_view, tmp_dist_tmp), r_view) #交点の座標
                if (is_contain(prim,crosspoint) == True):
                    #tmp_cross = True
                    #tmp_dist = tmp_dist_tmp
                    #if(tmp_dist_tmp < tmp_dist):
                    #    tmp_dist = tmp_dist_tmp
                    cross = True
                    dist = tmp_dist_tmp
                    break
        #if(tmp_cross == True):
            #cross = True
            #dist = tmp_dist
        
    elif(prim['pP'] == 2): # 平面
        norm = [prim['pa'],prim['pb'],prim['pc']]
        norm = vec_scale(norm, 1/vec_length(norm))
        r_vp_prim = vec_subtract(r_prim,r_view) # 視点に対するプリミティブの位置
        tmp = inner_product(norm, e_view) * sign(prim["pSG"])
        if (tmp < 0): # 内積が<0なら、平面の表側から入射
            dist = inner_product(norm, r_vp_prim) / tmp
            cross = True
            
    elif(prim['pP'] == 3 or prim['pP'] == 4): # 二次曲面 or 錐
        parameter = []
        r_prim_vp = vec_subtract(r_view, r_prim)
        if(prim['pP'] == 3):
            for tmp_p in [prim['pa'],prim['pb'],prim['pc']]:
                if(tmp_p != 0):
                    parameter.append(sign(tmp_p)/(tmp_p**2))
                else:
                    parameter.append(0.0)
        else:
            for tmp_p in [prim['pa'],prim['pb'],prim['pc']]:
                parameter.append(tmp_p)
        A=0.0
        B=0.0
        C=-1.0
        if(prim['pP'] == 4):
            C=0.0
        for i in [0,1,2]:
            A+=(e_view[i]**2)*parameter[i]
            B+=(e_view[i]*r_prim_vp[i])*parameter[i]
            C+=(r_prim_vp[i]**2)*parameter[i]
        tmp = (B**2-A*C)
        if((tmp >= 0.0) and (A != 0.0)):
            if (prim["pSG"] > 0):
                dist = (-math.sqrt(tmp)-B)/A #符号をどう決定するか？
            else:
                dist = (math.sqrt(tmp)-B)/A
            cross = True
    #from IPython.core.debugger import Pdb; Pdb().set_trace()
    return cross, dist

def is_contain(prim, r_cross):
    result = False
    
    r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
    rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
    
    if (prim['pP'] == 1): # 直方体
        if(abs(rel_cross[0]) <= prim['pa'] and\
        abs(rel_cross[1]) <= prim['pb'] and\
        abs(rel_cross[2]) <= prim['pc'] and prim['pSG']>0):
            result = True
        elif(abs(rel_cross[0]) > prim['pa'] and\
        abs(rel_cross[1]) > prim['pb'] and\
        abs(rel_cross[2]) > prim['pc'] and prim['pSG']<0):
            result = True
        #if(rel_cross[0] <= prim['pa'] and rel_cross[1] <= prim['pb'] and rel_cross[2] <= prim['pc'] and\
        #rel_cross[0] >= -1.0*prim['pa'] and rel_cross[1] >= -1.0*prim['pb'] and rel_cross[2] >= -1.0*prim['pc']):
            #if(prim['pSG']>0):
                #result = True
        #elif(rel_cross[0] >= prim['pa'] and rel_cross[1] >= prim['pb'] and rel_cross[2] >= prim['pc'] and\
        #rel_cross[0] <= -1.0*prim['pa'] and rel_cross[1] <= -1.0*prim['pb'] and rel_cross[2] <= -1.0*prim['pc']):
            #if(prim['pSG']<0):
                #result = True
                
    elif(prim['pP'] == 2): # 平面
        norm = [prim['pa'],prim['pb'],prim['pc']]
        norm = vec_scale(norm, 1/vec_length(norm))
        tmp = inner_product(norm, rel_cross)
        if(prim['pSG']>0 and tmp <= 0):
            result = True
        elif(prim['pSG']<0 and tmp >= 0):
            result = True
        
    elif(prim['pP'] == 3 or prim['pP'] == 4): # 二次曲面 or 錐
        tmp = 0.0
        tmp_p = [prim['pa'],prim['pb'],prim['pc']]
        if(prim['pP'] == 3):
            for ix in [0,1,2]:
                if(tmp_p[ix]!=0):
                    tmp+= sign(tmp_p[ix])/(tmp_p[ix]**2) * (rel_cross[ix]**2)
                else:
                    tmp += 0.0
            if (tmp<=1.0 and prim['pSG']>0):
                result = True
            elif (tmp>=1.0 and prim['pSG']<0):
                result = True
        elif(prim['pP'] == 4):
            for ix in [0,1,2]:
                tmp += (tmp_p[ix]) * (rel_cross[ix]**2)
            if (tmp<=0.0 and prim['pSG']>0):
                result = True
            elif (tmp>=0.0 and prim['pSG']<0):
                result = True
    return result

def trace(e_view,r_view):
    #from IPython.core.debugger import Pdb; Pdb().set_trace()
    min_dist = 1e15
    result = False
    prim_id = -1

    for or_ids in prims_OR:
        if(or_ids[0] < 99):
            cross, dist = cross_distance(primitives[or_ids[0]],e_view,r_view)
            if((cross == False) or (min_dist <= dist)):
                continue
        for or_index in or_ids[1:]:
            for and_index in prims_AND[or_index]: # AND定義に含まれるプリミティブで、視点から最も近いものをさがす
                cross, dist = cross_distance(primitives[and_index],e_view,r_view)
                if(cross == False): #対象のプリミティブと交わらない
                    if(primitives[and_index]['pSG'] > 0): #プリミティブの極性が正 = 視点はプリミティブの外側
                        break #上記条件を満たすプリミティブが存在 = 対象のAND定義とは交わらない
                    else:
                        continue
                if((min_dist <= dist) or (dist < 0)): #すでに別のプリミティブと交差 or 視点の背後で交差
                    continue
                
                crosspoint = vec_sum(vec_scale(e_view, dist), r_view) #交点の座標
                    
                tmp_min_dist = min_dist
                tmp_prim_id = prim_id
                
                min_dist = dist
                prim_id = and_index
                for contain_index in prims_AND[or_index]: #交点が別のプリミティブの内側にあるか
                    if (contain_index != and_index):
                        if(is_contain(primitives[contain_index],crosspoint) == False): #1つでも交点を含まないプリミティブがある
                            min_dist = tmp_min_dist
                            prim_id = tmp_prim_id
                            break
    if((min_dist > 1e14) or (min_dist < 0)):
        result = False
    else:
        result = True
    return result, min_dist, prim_id
    
def mainloop():
    #from IPython.core.debugger import Pdb; Pdb().set_trace()
    screen_width = 256
    screen_height = 256
    width_offset = screen_width / 2.0
    height_offset = screen_height / 2.0
    dist_screen_viewpoint = -200.0

    img = Image.new('RGB', (screen_width, screen_height))

    dx = 128.0 / screen_width
    dy = 128.0 / screen_height

    rel_viewpoint = [0.0, 0.0, dist_screen_viewpoint]
    rel_viewpoint = rot_x(rel_viewpoint,math.radians(viewangle[0]))
    rel_viewpoint = rot_y(rel_viewpoint,math.radians(viewangle[1]))
    abs_viewpoint = vec_sum(rel_viewpoint,viewplane)

    for scr_y in range(0,screen_height):
        rel_screenpoint = [0.0, 0.0, 0.0]
        rel_screenpoint[1] = (height_offset - scr_y) * dy
        for scr_x in range(0,screen_width):
            rel_screenpoint[0] = (scr_x - width_offset) * dx
            
            tmp_relsc = rot_x(rel_screenpoint,math.radians(viewangle[0]))
            tmp_relsc = rot_y(tmp_relsc,math.radians(viewangle[1]))
            abs_screenpoint = vec_sum(tmp_relsc,viewplane)
            
            tmp_elem = vec_subtract(abs_screenpoint, abs_viewpoint)
            elem_view = vec_scale(tmp_elem, 1.0/vec_length(tmp_elem))
            
            result, min_dist, prim_id = trace(elem_view,abs_viewpoint)
            if(result == True):
                img.putpixel((scr_x,scr_y), (int(primitives[prim_id]['pR']), int(primitives[prim_id]['pG']), int(primitives[prim_id]['pB'])))
            else:
                img.putpixel((scr_x,scr_y), (0, 0, 0))

    img.save('result.png')


f = open("shuttle_tube.sld")
buffer = f.read().split()
f.close()

index=0
viewplane=[]
viewplane.append(float(buffer[index]))
index=index+1
viewplane.append(float(buffer[index]))
index=index+1
viewplane.append(float(buffer[index]))

viewangle=[]
index=index+1
viewangle.append(float(buffer[index]))
index=index+1
viewangle.append(float(buffer[index]))

index=index+1
ls_num = float(buffer[index])

ls_vec=[]
ls_vec.append(0)
index=index+1
ls_vec.append(float(buffer[index]))
index=index+1
ls_vec.append(float(buffer[index]))

index=index+1
beam_higlight = float(buffer[index])

primitives=[]
prim_keys_int=[]
prim_keys_int.append("pTX")
prim_keys_int.append("pP")
prim_keys_int.append("pSF")
prim_keys_int.append("pRT")
prim_keys=[]
prim_keys.append("pa")
prim_keys.append("pb")
prim_keys.append("pc")
prim_keys.append("pX")
prim_keys.append("pY")
prim_keys.append("pZ")
prim_keys.append("pSG")
prim_keys.append("pREF")
prim_keys.append("pHL")
prim_keys.append("pR")
prim_keys.append("pG")
prim_keys.append("pB")

index=index+1
while True:
    prim_temp = {}
    for key in prim_keys_int:
        prim_temp[key]=int(buffer[index])
        index=index+1
    for key in prim_keys:
        prim_temp[key]=float(buffer[index])
        index=index+1
    if(prim_temp['pRT']==1):
        for key in ['pRoX','pRoY','pRoZ']:
            prim_temp[key]=float(buffer[index])
            index=index+1
    primitives.append(prim_temp)
    
    if(buffer[index] == '-1'):
        break

prims_AND = []
index=index+1

while True:
    if(buffer[index] == '-1'):
        index=index+1
        break
    prAND_temp = []
    while True:
        if(buffer[index] == '-1'):
            index=index+1
            break
        prAND_temp.append(int(buffer[index]))
        index=index+1
    prims_AND.append(prAND_temp)
    
prims_OR = []

while True:
    if(buffer[index] == '-1'):
        index=index+1
        break
    temp = []
    while True:
        if(buffer[index] == '-1'):
            index=index+1
            break
        temp.append(int(buffer[index]))
        index=index+1
    prims_OR.append(temp)

mainloop()
