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
### Jacobian Matrix
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
