# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
from FRA333_HW3_6528_6540 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1. ธรรมภณ_6528
2. พงศกร_6540
3.
'''

# กำหนดค่าตัวแปร
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

q_singulality = [0, -np.pi/2, -0.1]
q_init = [0, 0, 0] 
w = np.array([1, 1, 1, 1, 1, 1]) 

# สร้างโมเดล
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0, a=0, d=d_1, offset=np.pi), 
        rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0, offset=0),
        rtb.RevoluteMDH(alpha=0, a=-a_2,  d=0, offset=0)
    ], 
    tool = SE3(
    [
        [0, 0, -1, -(a_3 + d_6)],
        [0, 1, 0, -d_5],
        [1, 0, 0, d_4],
        [0, 0, 0, 1]
    ]),
    name = "3DOF_Robot")

#===========================================<ตรวจคำตอบข้อ 1>====================================================#
def endEffectorJacobianRTB(q:list[float])->list[float]:
    # คำนวณ Jacobian อิงจาก Base
    jacob_rtb = robot.jacob0(q)
    jacob = endEffectorJacobianHW3(q_init)
    # หา Error
    diff = jacob_rtb - jacob
    # กำหนด threshold
    threshold = 1e-10
    # แสดงผล และเปรียบเทียบผลลัพธ์
    print("-------------------Jacobian Check-------------------")
    print(f"Jacobian HW3: \n {jacob}")
    print(f"Jacobian Robotic toolbox: \n {jacob_rtb}")
    if np.linalg.norm(diff) < threshold:
        # ถ้า Error น้อยกว่า Threshold จะถือว่า Error เป็น 0 ไปเลย
        diff = np.where(np.abs(diff) < threshold, 0.0, diff)
        print(f"Error: \n {diff}")
        print("Answer: CORRECT\n")
    else:
        print(f"Error: \n {diff}\n")
        print("Answer: INCORRECT\n")
    return jacob_rtb
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
def checksingularityRTB(q:list[float])->bool:
    epsilon = 0.001  # ค่าเกณฑ์สำหรับการตรวจสอบ Singular
    
    # คำนวณ Jacobian ใช้ Robotic Toolbox
    J = robot.jacob0(q)  # Jacobian at the base frame
    
    # ตรวจสอบขนาดของ Jacobian
    if J.shape[0] < 3 or J.shape[1] < 3:
        raise ValueError("Jacobian must have at least shape (3, 3) for determinant calculation.")
    
    # เลือก Jacobian ที่ต้องการคำนวณ determinant (เลือกแค่แถวแรก)
    J_star = J[:3, :]  # หรือ J[:, :3] ขึ้นอยู่กับว่าอยากเลือกแถวหรือคอลัมน์
    
    # คำนวณ Determinant ของ Jacobian ที่เลือก
    det_J = np.linalg.det(J_star)
    # แสดงผล และเปรียบเทียบผลลัพธ์
    print("------------------Singularity Check-----------------")
    print("Singularity Check HW3:")
    checkSingularityHW3(q)
    print("Singularity Check Robotic toolbox:")
    # ตรวจสอบว่าค่า Determinant น้อยกว่า epsilon หรือไม่
    if abs(det_J) < epsilon:
        flag = 1  # อยู่ในสภาวะ Singular
        print("- Robot IS IN SINGULARITY\n")
    else:
        flag = 0  # อยู่ในสภาวะปกติ
        print("- Robot is NOT in singularity\n")

    
    return flag
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
def computeEffortRTB(q: list[float],w:list[float])->list[float]:
    # คำนวณ Jacobian
    J = robot.jacob0(q)
    # คำนวณ effort
    tau_rtb = robot.pay(w, J=J, frame=0)
    tau = computeEffortHW3(q,w)
    # หา Error
    diff = tau_rtb - tau
    # กำหนด threshold
    threshold = 1e-10
    # แสดงผล และเปรียบเทียบผลลัพธ์
    print("--------------------Effort Check--------------------")
    print(f"Effort for each joint HW3: \n {tau}")
    print(f"Effort for each joint Robotic toolbox: \n {tau_rtb}")
    if np.linalg.norm(diff) < threshold:
        # ถ้า Error น้อยกว่า Threshold จะถือว่า Error เป็น 0 ไปเลย
        diff = np.where(np.abs(diff) < threshold, 0.0, diff)
        print(f"Error: \n {diff}")
        print("Answer: CORRECT\n")
    else:
        print(f"Error: \n {diff}\n")
        print("Answer: INCORRECT\n")
    return tau_rtb
#==============================================================================================================#

endEffectorJacobianRTB(q_init)
checksingularityRTB(q_singulality)
computeEffortRTB(q_init, w)