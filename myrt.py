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
    back_nc = False
    dist = 1e15
    r_prim = [prim['pX'],prim['pY'],prim['pZ']]
    
    if (prim['pP'] == 1): # 直方体
        tmp_vec_list = [ [[1.0, 0, 0],  [prim['pa'], 0, 0],      [0, 1, 1]],\
                         [[0, 1.0, 0],  [0, prim['pb'], 0],      [1, 0, 1]],\
                         [[0, 0, 1.0],  [0, 0, prim['pc']],      [1, 1, 0]],\
                         [[-1.0, 0, 0], [-1.0*prim['pa'], 0, 0], [0, 1, 1]],\
                         [[0, -1.0, 0], [0, -1.0*prim['pb'], 0], [1, 0, 1]],\
                         [[0, 0, -1.0], [0, 0, -1.0*prim['pc']], [1, 1, 0]]]# tmp_vecs[0]: 各面の法線ベクトル、[1]: 法線ベクトルの位置
        for tmp_vecs in tmp_vec_list:
            r_vp_prim = vec_subtract(vec_sum(r_prim, tmp_vecs[1]), r_view)
            tmp = inner_product(tmp_vecs[0] ,e_view)
            if ((tmp < -0.001 and sign(prim["pSG"]) > 0) or (tmp > 0.001 and sign(prim["pSG"]) < 0)): # 内積が<0なら、平面の表側から入射
                tmp_dist_tmp = inner_product(tmp_vecs[0], r_vp_prim) / tmp
                crosspoint = vec_sum(vec_scale(e_view, tmp_dist_tmp), r_view) #交点の座標
                rel_cross = vec_subtract(crosspoint, r_prim)
                tmp_param = [prim['pa'],prim['pb'],prim['pc']]
                tmp_contain = True
                for i in [0, 1, 2]:
                    if(tmp_vecs[2][i] == 1 and abs(rel_cross[i]) > tmp_param[i]):
                        tmp_contain = False
                        break
                if (tmp_contain == True):
                    cross = True
                    dist = tmp_dist_tmp
                    break
        
    elif(prim['pP'] == 2): # 平面
        norm = [prim['pa'],prim['pb'],prim['pc']]
        norm = vec_scale(norm, 1/vec_length(norm))
        r_vp_prim = vec_subtract(r_prim,r_view) # 視点に対するプリミティブの位置
        tmp = inner_product(norm, e_view)
        if ((tmp < -1e-3 and sign(prim["pSG"]) > 0) or (tmp > 1e-3 and sign(prim["pSG"]) < 0)): # 内積が<0なら、平面の表側から入射
            dist = inner_product(norm, r_vp_prim) / tmp
            cross = True
        elif((tmp > 1e-3 and sign(prim["pSG"]) > 0) or (tmp < -1e-3 and sign(prim["pSG"]) < 0)): # 内積が>0なら、平面の表側から入射
            back_nc = True
            
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
    return cross, dist, back_nc

def is_contain(prim, r_cross):
    result = False
    
    r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
    rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
    
    if (prim['pP'] == 1): # 直方体
        if(abs(rel_cross[0]) <= prim['pa'] and\
        abs(rel_cross[1]) <= prim['pb'] and\
        abs(rel_cross[2]) <= prim['pc'] and prim['pSG']>0):
            result = True
        elif((abs(rel_cross[0]) > prim['pa'] or\
        abs(rel_cross[1]) > prim['pb'] or\
        abs(rel_cross[2]) > prim['pc']) and prim['pSG']<0):
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

def trace(e_view,r_view,ref_prim):
    #from IPython.core.debugger import Pdb; Pdb().set_trace()
    min_dist = 1e15
    result = False
    prim_id = -1
    ref_prim = -1

    for or_ids in prims_OR:
        #if(or_ids[0] == ref_prim):
        #    continue
        if(or_ids[0] < 99):
            cross, dist, back_nc = cross_distance(primitives[or_ids[0]],e_view,r_view)
            if((cross == False) or (min_dist <= dist)):
                continue
        for or_index in or_ids[1:]:
            for and_index in prims_AND[or_index]: # AND定義に含まれるプリミティブで、視点から最も近いものをさがす
                #if(and_index == ref_prim):
                #    break
                cross, dist, back_nc = cross_distance(primitives[and_index],e_view,r_view)
                if(primitives[and_index]['pP'] == 2): #対象となるプリミティブが平面
                    if(cross == False):
                        if(back_nc == True): # 視線が平面の裏から入射
                            continue
                        else: #視線が法線と平行
                            break
                else: #それ以外
                    if(cross == False): #対象のプリミティブと交わらない
                        if(primitives[and_index]['pSG'] > 0): #プリミティブの極性が正 = 視点はプリミティブの外側
                            break #上記条件を満たすプリミティブが存在 = 対象のAND定義とは交わらない
                        else:
                            continue
                if((min_dist <= dist) or (dist < 0)): #すでに別のプリミティブと交差 or 視点の背後で交差
                    continue
                
                #dist+=0.01
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
    
def texture(prim, r_cross):
    color = (int(prim['pR']), int(prim['pG']), int(prim['pB'])) #pTX == 0: 単色 の場合
    if (prim['pTX'] == 1): # x-z平面のチェッカー
        r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
        rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
        tmp = True
        if(10 < rel_cross[0] % 20.0):
            tmp = not tmp
        if(10 < rel_cross[2] % 20.0):
            tmp = not tmp
        if(tmp == True):
            color = (int(prim['pR']), int(255.0), int(prim['pB']))
        else:
            color = (int(prim['pR']), int(0), int(prim['pB']))
            
    elif (prim['pTX'] == 2): # y軸方向のストライプ
        tmp = (math.sin(r_cross[1] * 0.25))**2
        color = (int(255*tmp), int(255*(1-tmp)), int(prim['pB']))
        
    elif (prim['pTX'] == 3): # x-z平面の同心円
        r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
        rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
        
        tmp = math.sqrt(rel_cross[0]**2 + rel_cross[2]**2)*0.1
        tmp1 = math.pi * ( (tmp - math.floor(tmp)) )
        tmp2 = (math.cos(tmp1))**2
        color = (int(prim['pR']), int(255*tmp2), int(255*(1-tmp2)))
    
    elif (prim['pTX'] == 4): # 球面上の班点
        r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
        rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
        t_param = [prim['pa'],prim['pb'],prim['pc']]
        tmp0 = []
        for i in [0,1,2]:
            tmp0.append(rel_cross[i] / t_param[i]) # プリミティブを単位球に見立てる(?)
        tmp1 = math.sqrt(tmp0[0]**2 + tmp0[2]**2) # y= tmp0[1]面で切断した時の断面の半径
        if(0.0001 < abs(tmp0[0])):
            tmp2 = math.atan(abs(tmp0[2]/tmp0[0])) * 9.549296585514
        else:
            tmp2 = 15.0
        if(0.0001 < abs(tmp1)):
            tmp3 = math.atan(abs(tmp0[1]/tmp1)) * 9.549296585514
        else:
            tmp3 = 15.0
        tmp4 = 0.15 - ((0.5 - (tmp2 - math.floor(tmp2)))**2 + (0.5 - (tmp3 - math.floor(tmp3)))**2)
        if(tmp4 <= 0):
            color = (int(prim['pR']), int(prim['pG']), int(0))
        else:
            color = (int(prim['pR']), int(prim['pG']), int(256/0.3 * tmp4))
        
    elif (prim['pTX'] == 5): # 10x10x10のチェッカー
        r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
        rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
        tmp = True
        if(10.0 < rel_cross[0] % 20.0):
            tmp = not tmp
        if(10.0 < rel_cross[1] % 20.0):
            tmp = not tmp
        if(10.0 < rel_cross[2] % 20.0):
            tmp = not tmp
        if(tmp == True):
            color = (int(prim['pR']), int(255.0), int(prim['pB']))
        else:
            color = (int(prim['pR']), int(0), int(prim['pB']))
    return color
    
def normal_vector(prim, r_cross, e_view):
    result = [0,0,0]
    tmp_param = [prim['pa'],prim['pb'],prim['pc']]
    r_prim = [prim['pX'],prim['pY'],prim['pZ']] # プリミティブの中心座標
    rel_cross = vec_subtract(r_cross, r_prim) # プリミティブ中心に対する交点
    if(prim['pP'] == 1):
        tmp_vec_list = [ [[1.0, 0, 0],  [prim['pa'], 0, 0],      [0, 1, 1]],\
                         [[0, 1.0, 0],  [0, prim['pb'], 0],      [1, 0, 1]],\
                         [[0, 0, 1.0],  [0, 0, prim['pc']],      [1, 1, 0]],\
                         [[-1.0, 0, 0], [-1.0*prim['pa'], 0, 0], [0, 1, 1]],\
                         [[0, -1.0, 0], [0, -1.0*prim['pb'], 0], [1, 0, 1]],\
                         [[0, 0, -1.0], [0, 0, -1.0*prim['pc']], [1, 1, 0]]]# tmp_vecs[0]: 各面の法線ベクトル、[1]: 法線ベクトルの位置
        for tmp_vecs in tmp_vec_list:
            tmp = inner_product(tmp_vecs[0], vec_subtract(r_cross, vec_sum(r_prim, tmp_vecs[1]))) # 対象となる法線ペクトルの起点に対する交点の位置ベクトルと、法線ベクトルの内積
            if(abs(tmp) < 0.001): #交点が対象となる面上にあるなら、上記のベクトルは直交
                result = vec_scale(tmp_vecs[0], sign(prim["pSG"]))
                break
    elif(prim['pP'] == 2):
        result = vec_scale(tmp_param, sign(prim["pSG"]))
    elif(prim['pP'] == 3):
        for i in [0,1,2]:
            if (tmp_param[i] != 0):
                result[i] = rel_cross[i] / (tmp_param[i])**2 * sign(tmp_param[i])
            else:
                result[i] = 0
        result = vec_scale(result, 1.0/vec_length(result)*sign(prim['pSG']))
    elif(prim['pP'] == 4):
        for i in [0,1,2]:
            result[i] = rel_cross[i] * tmp_param[i]
        result = vec_scale(result, 1.0/vec_length(result)*sign(prim['pSG']))
        
    return result
    
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
            abs_viewpoint = vec_sum(rel_viewpoint,viewplane)
            rel_screenpoint[0] = (scr_x - width_offset) * dx
            
            tmp_relsc = rot_x(rel_screenpoint,math.radians(viewangle[0]))
            tmp_relsc = rot_y(tmp_relsc,math.radians(viewangle[1]))
            abs_screenpoint = vec_sum(tmp_relsc,viewplane)
            
            tmp_elem = vec_subtract(abs_screenpoint, abs_viewpoint)
            elem_view = vec_scale(tmp_elem, 1.0/vec_length(tmp_elem))
            
            color = [0, 0, 0]
            energy = 1.0
            reflex = 0
            ref_prim_id = -1
            while True:
                result, min_dist, prim_id = trace(elem_view,abs_viewpoint,ref_prim_id)
                
                if(result == False):
                    if(reflex != 0):
                        cos_highlight = -1.0*inner_product(elem_view, ls_vec)
                        if(cos_highlight < 0):
                            cos_highlight = 0.0
                        highlight = cos_highlight**3 * energy * beam_higlight
                        color[0] += int(highlight)
                        color[1] += int(highlight)
                        color[2] += int(highlight)
                    break
                
                ref_prim_id = prim_id
                crosspoint = vec_sum(vec_scale(elem_view, min_dist), abs_viewpoint)
                crosspoint_texture =vec_sum(vec_scale(elem_view, min_dist+0.01), abs_viewpoint) #テクスチャの評価用 距離を+0.01
                vec_normal = normal_vector(primitives[prim_id], crosspoint,elem_view)
                
                abs_viewpoint = crosspoint
                
                tmp_bright1 = inner_product(vec_normal, ls_vec)
                if(tmp_bright1 > 0):
                    tmp_bright1 = 0.0
                brightness = (0.2 - tmp_bright1) * energy * primitives[prim_id]['pREF']
                
                # シャドウの評価
                
                tmp_color = [primitives[prim_id]['pR'], primitives[prim_id]['pG'], primitives[prim_id]['pB']]
                if(brightness != 0.0):
                    #tmp_color = texture(primitives[prim_id], crosspoint)
                    tmp_color = texture(primitives[prim_id], crosspoint_texture)
                
                #brightness=1
                for i in [0,1,2]:
                    color[i] += int(math.floor(brightness * tmp_color[i]))
                
                if(energy < 0.1 or reflex > 4):
                    break
                
                if(primitives[prim_id]['pSF'] == 1): #乱反射
                    if(primitives[prim_id]['pHL'] > 0):
                        tmp_new_eview = -2.0 * inner_product(elem_view,vec_normal)
                        elem_view = vec_sum(elem_view, vec_scale(vec_normal, tmp_new_eview))
                        cos_highlight = -1.0*inner_product(elem_view, ls_vec)
                        if(cos_highlight < 0):
                            cos_highlight = 0.0
                        highlight = cos_highlight**4 * energy * brightness * primitives[prim_id]['pHL']
                        color[0] += int(highlight)
                        color[1] += int(highlight)
                        color[2] += int(highlight)
                    break
                elif(primitives[prim_id]['pSF'] == 2): #鏡面
                    energy *= (1.0 - primitives[prim_id]['pREF'])
                    tmp_new_eview = -2.0 * inner_product(elem_view,vec_normal)
                    elem_view = vec_sum(elem_view, vec_scale(vec_normal, tmp_new_eview))
                else:
                    break
                reflex +=1
                    
            for i in [0,1,2]:
                if(color[i] > 255):
                    color[i] = 255
                else:
                    color[i] = int(color[i])
            img.putpixel((scr_x,scr_y), (color[0], color[1], color[2]))
    img.save('result.png')


viewplane=[]
viewangle=[]
ls_num = 0
ls_vec=[]
beam_higlight = 0
primitives=[]
prims_AND = []
prims_OR = []
    
def load_data(filename):
    global viewplane
    global viewangle
    global ls_num
    global ls_vec
    global beam_higlight
    global primitives
    global prims_AND
    global prims_OR
    
    f = open(filename)
    buffer = f.read().split()
    f.close()

    index=0

    viewplane.append(float(buffer[index]))
    index=index+1
    viewplane.append(float(buffer[index]))
    index=index+1
    viewplane.append(float(buffer[index]))

    index=index+1
    viewangle.append(float(buffer[index]))
    index=index+1
    viewangle.append(float(buffer[index]))

    index=index+1
    ls_num = float(buffer[index])

    ls_tmp = []
    index=index+1
    ls_tmp.append(math.radians(float(buffer[index])))
    index=index+1
    ls_tmp.append(math.radians(float(buffer[index])))
    
    ls_vec.append(math.sin(ls_tmp[1]) * math.cos(ls_tmp[0]))
    ls_vec.append(-math.sin(ls_tmp[0]))
    ls_vec.append(math.cos(ls_tmp[1]) * math.cos(ls_tmp[0]))

    index=index+1
    beam_higlight = float(buffer[index])

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


if (__name__ == '__main__'):
    load_data("sld_orig/tron.sld")
    #load_data("tron_yuka.sld")
    mainloop()
