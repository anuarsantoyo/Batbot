
stroke=0 #上下行程判断，0为下1为上
m_value=15 #最大/最小值，用于辅助判断行程
m_value_changed=False
@micropython.native
def stroke_estimate(wing_angle):
    global stroke,m_value,m_value_changed
    if stroke==1: #上行程
        if wing_angle>m_value:
            m_value=wing_angle
            m_value_changed=True
        elif m_value-wing_angle>2 and m_value_changed:
            stroke=0
            m_value=15
            m_value_changed=False
    else:       #下行程
        if wing_angle<m_value:
            m_value=wing_angle
            m_value_changed=True
        elif wing_angle-m_value>2 and m_value_changed:
            stroke=1
            m_value=15
            m_value_changed=False
    return stroke


def fit_func(x,a,b,c,d,e,f,g):
    return a*x**6+b*x**5+c*x**4+d*x**3+e*x**2+f*x+g

@micropython.native
def calc_wingSpanAngle(wing_angle):
    #print(stroke,wing_angle,m_value)#test
    norm_WAng=wing_angle/40
    if stroke==1: #上行程
        #return 120 if wing_angle<25 else 80
        norm_Wspan=fit_func(norm_WAng,2.19011746e+00,-5.07366669e+00,2.46254020e+00,
                            7.74043387e-01,-2.00814360e-02,1.62535593e-02,3.58224145e-03)
    else:       #下行程
        #return 80 if wing_angle>5 else 120
        norm_Wspan=fit_func(norm_WAng,-1.00877782e+01,4.02595675e+01,-5.00380932e+01,
                            1.69617564e+01,3.57086848e+00,-3.11379855e-01,4.28113352e-03)
    return 90*norm_Wspan+70
    
def calc_swingAngle(wing_angle):
    if stroke==1: #上行程
        return 160 if wing_angle<25 else 80
    else:       #下行程
        return 80 if wing_angle>5 else 160


def calc_LlegYawAngle(wing_angle):
    return 50

def calc_LlegPitchAngle(wing_angle):
    return 20

def calc_RlegYawAngle(wing_angle):
    return 50

def calc_RlegPitchAngle(wing_angle):
    return 20