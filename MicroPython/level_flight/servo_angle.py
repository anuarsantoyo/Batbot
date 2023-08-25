
stroke=0 #上下行程判断，0为下1为上
m_value=15 #最大/最小值，用于辅助判断行程
m_value_changed=False
def calc_wingSpanAngle(wing_angle):
    global stroke,m_value,m_value_changed
    #print(stroke,wing_angle,m_value)#test
    if stroke==1: #上行程
        if wing_angle>m_value:
            m_value=wing_angle
            m_value_changed=True
        elif m_value-wing_angle>2 and m_value_changed:
            stroke=0
            m_value=15
            m_value_changed=False
        return 20 if wing_angle<25 else -70
    else:       #下行程
        if wing_angle<m_value:
            m_value=wing_angle
            m_value_changed=True
        elif wing_angle-m_value>2 and m_value_changed:
            stroke=1
            m_value=15
            m_value_changed=False
        return -70 if wing_angle>5 else 20


def calc_LlegYawAngle(wing_angle):
    return 50

def calc_LlegPitchAngle(wing_angle):
    return 20

def calc_RlegYawAngle(wing_angle):
    return 50

def calc_RlegPitchAngle(wing_angle):
    return 20