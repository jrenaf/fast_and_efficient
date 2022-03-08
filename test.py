from scipy.spatial.transform import Rotation 
from scipy.spatial.transform import Slerp
q_1 = [0.0, 0.0, 0.0, 1.0]
q_2 = [1.0, 0.0, 0.0, 0.0]
r_1=  Rotation.from_quat(q_1)
r_2 = Rotation.from_quat(q_2)
slerp = Slerp([1,2], [r_1, r_2])
r_interp = slerp(0.5)  
q_interp = Rotation.as_quat(r_interp)
print(q_interp)