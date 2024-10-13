from HW3_utils import FKHW3
import numpy as np
# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. ธรรมภณ_6528
2. พงศกร_6540
3.
'''
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # เรียกใช้ FKHW3 จาก HW3_utils.py
    R, P, R_e, p_e = FKHW3(q)

    # ขนาดของ Jacobian
    J_v = np.zeros((3, 3))  # Jacobian เชิงเส้น
    J_omega = np.zeros((3, 3))  # Jacobian เชิงมุม
    
    # คำนวณ J_v และ J_omega
    for i in range(3):
        J_v[:, i] = np.cross(R[:, :, i][:, 2], (p_e - P[:, i]))
        J_omega[:, i] = R[:, :, i][:, 2]

    J = np.vstack((J_v, J_omega))
    return J
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    epsilon = 0.001  # ค่าเกณฑ์สำหรับการตรวจสอบ Singular

    # คำนวณ Jacobian
    J = endEffectorJacobianHW3(q)

    # ตรวจสอบขนาดของ Jacobian
    if J.shape[0] < 3 or J.shape[1] < 3:
        raise ValueError("Jacobian must have at least shape (3, 3) for determinant calculation.")

    # เลือก Jacobian ที่ต้องการคำนวณ determinant
    J_star = J[:3, :]  # หรือ J[:, :3] ขึ้นอยู่กับความหมายของจาโคเบียนในกรณีนี้

    # คำนวณ Determinant ของ Jacobian ที่เลือก
    det_J = np.linalg.det(J_star)

    # ตรวจสอบว่าค่า Determinant น้อยกว่า epsilon หรือไม่
    if abs(det_J) < epsilon:
        flag = 1  # อยู่ในสภาวะ Singular
        print("- Robot IS IN SINGULARITY")
    else:
        flag = 0  # อยู่ในสภาวะปกติ
        print("- Robot is NOT in singularity")

    return flag
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    # คำนวณ Jacobian
    J = endEffectorJacobianHW3(q)

    # แยกค่า force และ moment จาก wrench w
    f = w[:3]  # แรง (force)
    n = w[3:]  # โมเมนต์ (moment)

    # สร้าง wrench เวกเตอร์รวม (force + moment)
    wrench = np.concatenate((f, n))

    # คำนวณ effort ของแต่ละข้อต่อ
    tau = np.dot(-J.T, wrench)

    return tau
#==============================================================================================================#