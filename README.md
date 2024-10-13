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
