# FRA333 Homework Assignment 3: Static Force
การบ้านนี้ถูกออกแบบมาเพื่อให้ผู้เรียนได้ประยุกต์ใช้ความรู้การหาจลนศาสตร์เชิงอนุพันธ์ (Differential Kinematics) ของหุ่นยนต์แขนกล 3 แกน (3-DOF Manipulator) โดยใช้ฟังก์ชัน Forward Kinematics ที่กำหนดในไฟล์ `HW3_utils.py` ซึ่งจะต้องสร้าง Function เพื่อหา Jacobian, การตรวจสอบสถานะ Singularity, และคำนวณ Effort ของข้อต่อของหุ่นยนต์ RRR ตามที่ระบุในโจทย์
## Getting Started

### Prerequisites
ตรวจสอบว่าได้มีการติดตั้ง Library ดังต่อไปนี้:
- `numpy` (Recommend: NumPy version 1.24)
- `roboticstoolbox`
- `spatialmath`
- `math`

### Installation
Clone the repository:

```bash
git clone https://github.com/ch4dum/FRA333_HW3_6528_6540.git
```

## Finding Answer
### 1. การคำนวณ Jacobian Matrix
ในการหาคำตอบของข้อที่ 1 เราต้องการคำนวณที่จะหา Jacobian Matrix ซึ่งเป็นเมทริกซ์ที่อธิบายความสัมพันธ์ระหว่างความเร็วเชิงมุมของข้อต่อกับความเร็วเชิงเส้นและความเร็วเชิงมุมของจุดสิ้นสุดของหุ่นยนต์ (End-Effector) โดยมีขั้นตอนการคำนวณดังนี้:

**การคำนวณ Jacobian เชิงเส้น:** ใช้ Cross product เพื่อหาค่าของแต่ละคอลัมน์ใน $J_v$

$$
J_v[:, i] = R[:, :, i][:, 2] \times (p_e - P[:, i])
$$

โดยที่:
- $R[:,:,i][:,2]$ คือแกนการหมุน (axis of rotation) ของข้อต่อที่ $i$
- $(P_e−P[:,i])$ คือเวกเตอร์ระยะทางจากข้อต่อนั้นถึงจุดสิ้นสุด

**การคำนวณ Jacobian เชิงมุม:** คำนวณจากแกนการหมุนของแต่ละข้อต่อสำหรับ Jacobian เชิงมุม $J_\omega$

$$
J_\omega[:, i] = R[:, :, i][:, 2]
$$

**การรวมเมทริกซ์ Jacobian:** สุดท้ายจะทำการรวม $J_\omega$ และ $J_v$ เข้าด้วยกันเพื่อสร้าง Jacobian ขนาด $6\times3$

$$
J = 
\begin{bmatrix}
J_v \\
J_\omega
\end{bmatrix}
$$

จากวิธีการคำนวณข้างต้นจะทำให้สามารถสร้าง Function `endEffectorJacobianHW3` ได้ดัง Code ด้านล่าง
```python
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
```

### 2. การตรวจสอบ Singularity
ในการหาคำตอบของข้อที่ 2 เราต้องการตรวจสอบว่าหุ่นยนต์อยู่ในของตำแหน่ง Singularity หรือไม่ โดยมีขั้นตอนการคำนวณดังนี้:

**การคำนวณ Determinant:** ทำการคำนวณหาค่า ของ Determinant $J_v$ ที่เป็น Jacobian เชิงเส้น

$$
det(J_v)
$$

โดยที่: $J_v = J[0:3,:]$ (3 แถวแรกของ Jacobian Matrix)

**การประเมินสถานะ Singularity:** เปรียบเทียบกับค่า Epsilon ($\epsilon$)
โดยถ้า
- $|det(J_v)| < \epsilon$ แสดงว่าหุ่นยนต์อยู่ในสถานะ Singular
- $|det(J_v)| > \epsilon$ แสดงว่าหุ่นยนต์ไม่ได้อยู่ในสถานะ Singular

จากวิธีการคำนวณและประเมิณข้างต้นจะทำให้สามารถสร้าง Function `checkSingularityHW3` ได้ดัง Code ด้านล่าง
```python
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
```

### 3. การคำนวณแรงบิดที่ข้อต่อ (Joint Torque)
ในการหาคำตอบของข้อที่ 3 เป็นการคำนวณแรงบิดที่ข้อต่อของหุ่นยนต์ ซึ่งเกิดขึ้นเมื่อมีแรงกระทำที่จุดสิ้นสุดของหุ่นยนต์ (End-Effector) สามารถคำนวณได้โดยใช้ Jacobian Matrix ร่วมกับแรงที่กระทำหรือที่เรียกว่า Wrench ซึ่งเป็นแรงและโมเมนต์ที่กระทำต่อ End-Effector โดยมีวิธีการคำนวณดังนี้:

**การคำนวณแรงบิดที่ข้อต่อ:**

$$
\tau = J^T \cdot w
$$

โดยที่:
- $\tau$ คือเวกเตอร์ของแรงบิดที่ข้อต่อ
- $J^T$ คือ Jacobian Matrix ที่ผ่านการ Transpose (การสลับแถวเป็นคอลัมน์)
- $w$ คือ Wrench หรือแรงที่กระทำที่จุดสิ้นสุดของหุ่นยนต์

จากวิธีการคำนวณข้างต้นจะทำให้สามารถสร้าง Function `computeEffortHW3` ได้ดัง Code ด้านล่าง
```python
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
```
## Checking Answer
### Setup and Configuration
เริ่มประกาศ หรือกำหนดตัวแปรสำหรับการทำโมเดลของหุ่นยนต์ RRR

```python
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

q_singulality = [0, -np.pi/2, -0.1]
q_init = [0, 0, 0] 
w = np.array([1, 1, 1, 1, 1, 1]) 
```

โดยจะมีตัวแปร `q_singulality`, `q_init` และ `w` ที่เป็นตัวแปรที่สามารถปรับเปลี่ยนค่าได้สำหรับการ Check คำตอบ

ทำการ**สร้างโมเดลของหุ่นยนต์** RRR โดยใช้ Robotic Toolbox for Python ร่วมกับ Modified Denavit-Hartenberg (MDH) parameters

```python
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
```
### 1. Checking Jacobian Matrix

ใช้งาน Function `jacob0` ที่เป็น Function ใน Robotic Toolbox เพื่อใช้ในการคำนวณ Jacobian matrix โดยอิงจากจุดเริ่มต้นของหุ่นยนต์ (Base frame หรือ Frame 0) ซึ่งจะนำค่าที่ได้เปรียบเทียบกับ Function การหาคำตอบจากการคำนวณ (`endEffectorJacobianHW3`)

```python
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
        diff = np.where(np.abs(diff) < threshold, 0.0, diff)
        print(f"Error: \n {diff}")
        print("Answer: CORRECT\n")
    else:
        print(f"Error: \n {diff}\n")
        print("Answer: INCORRECT\n")
    return jacob_rtb
```
