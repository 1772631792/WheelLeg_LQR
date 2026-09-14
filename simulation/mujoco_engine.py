"""MuJoCo plant -> reference C controller -> six actual motor torques."""
import time
import numpy as np
from simulation.mujoco_model import create,angles
from simulation.mujoco_bootstrap import mujoco
from simulation.firmware_native import Firmware


class Arena:
    def __init__(self,height=.18,terrain=True,substeps=1):
        if substeps not in (1,2,4):raise ValueError('substeps must be 1, 2 or 4')
        self.model,self.data=create(height,terrain);self.control_dt=.001;self.substeps=substeps
        self.model.opt.timestep=self.control_dt/substeps;self.firmware=Firmware(self.control_dt)
        self.start_angles=angles(height);self.distance=0.;self.height=height
        self.body=self.model.body('chassis').id
        self.qidx={name:self.model.joint(name).qposadr[0] for name in ('LA','LE','RA','RE')}
        self.vidx={name:self.model.joint(name).dofadr[0] for name in self.qidx}
        self.tires=[self.model.geom(name+'tire').id for name in ('L','R')]
        self.sensor=np.zeros(18);self.command=np.zeros(6);self.command[2]=height
        self.normal=np.zeros(2);self.contact_buffer=np.zeros(6)
        self.elapsed_physics=0.;self.steps=0

    def observe(self):
        d=self.data;matrix=d.xmat[self.body].reshape(3,3)
        pitch=np.arctan2(-matrix[2,0],np.hypot(matrix[0,0],matrix[1,0]))
        roll=np.arctan2(matrix[2,1],matrix[2,2]);yaw=np.arctan2(matrix[1,0],matrix[0,0])
        # Rotational free-joint qvel is body-local; map pitch to source's sign convention.
        self.sensor[:8]=[-pitch,-d.qvel[4],roll,d.qvel[3],yaw,d.qvel[5],self.distance,
                         np.dot(d.qvel[:3],[np.cos(yaw),np.sin(yaw),0])]
        self.normal[:]=0
        for i in range(d.ncon):
            contact=d.contact[i]
            for side,gid in enumerate(self.tires):
                if gid in (contact.geom1,contact.geom2):
                    mujoco.mj_contactForce(self.model,d,i,self.contact_buffer)
                    self.normal[side]+=abs(contact.frame[2])*self.contact_buffer[0]
        for side,prefix in enumerate(('L','R')):
            self.sensor[8+5*side:13+5*side]=[self.start_angles[0]+d.qpos[self.qidx[prefix+'A']],
                self.start_angles[3]+d.qpos[self.qidx[prefix+'E']],d.qvel[self.vidx[prefix+'A']],d.qvel[self.vidx[prefix+'E']],self.normal[side]]
        return self.sensor

    def step(self,speed=0,yaw_rate=0,height=None,jump=False,zero=False):
        start=time.perf_counter()
        self.observe();self.command[:]=[speed,yaw_rate,height if height is not None else self.height,jump,zero,self.data.time]
        output=self.firmware.update(self.sensor,self.command)
        self.data.ctrl[:]=output[[0,2,3,1,4,5]]
        for _ in range(self.substeps):mujoco.mj_step(self.model,self.data)
        self.distance+=self.sensor[7]*self.control_dt
        self.elapsed_physics+=time.perf_counter()-start;self.steps+=1
        return self.sensor

    def snapshot(self):
        return dict(time=float(self.data.time),state=self.sensor.copy(),telemetry=self.firmware.log.copy(),
                    motors=self.firmware.output.copy(),position=self.data.xpos[self.body].copy(),
                    rtf=self.steps*self.control_dt/max(self.elapsed_physics,1e-9))


if __name__=='__main__':
    arena=Arena(terrain=False)
    for i in range(5000):
        arena.step()
        if i%500==0:print(i,arena.sensor[:8],arena.firmware.log[[0,1,8,9]],arena.firmware.output)
    print('RTF',arena.snapshot()['rtf'])
