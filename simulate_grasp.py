"""Physical friction grasp of a free cube: no weld, attachment or object teleport."""
import argparse
import json
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares

from manipulation.demo_model import load_robot, OUT, fmt

DURATION=17.


def smooth(t,a,b):
    u=np.clip((t-a)/(b-a),0.,1.)
    return float(10*u**3-15*u**4+6*u**5)


def phase(t, language='it'):
    for end,label in [(1,'Assestamento'),(5,'Abbassamento e avvicinamento'),(7,'Allineamento alla presa'),(8,'Chiusura pinza'),(10,'Sollevamento oggetto'),(14,'Riallungamento gambe'),(18,'Oggetto sospeso')]:
        if t<end:
            if language=='en':
                return {'Assestamento':'Initial settling','Abbassamento e avvicinamento':'Lowering and approach','Allineamento alla presa':'Grasp alignment','Chiusura pinza':'Closing gripper','Sollevamento oggetto':'Lifting object','Riallungamento gambe':'Extending legs','Oggetto sospeso':'Holding object suspended'}[label]
            return label


def build():
    root,old,old_data=load_robot()
    option=root.find('option');option.set('cone','elliptic');option.set('impratio','100');option.set('solver','Newton')
    # Reuse the real CAD and inertials, with contact pads fitted to the existing
    # flat inner faces (CAD cross-sections x=40..60 mm, y=0..3.1, z=±7.5).
    for side,sign in [('left',1),('right',-1)]:
        body=root.find(".//body[@name='omx_gripper_%s_link']"%side)
        ET.SubElement(body,'geom',name='grasp_pad_'+side,type='box',pos=fmt([.0495,sign*.00155,0]),size='.0105 .00155 .0075',
            group='3',rgba='.2 .2 .2 1',contype='8',conaffinity='4',condim='4',friction='1 .005 .0001',solref='.004 1',solimp='.99 .999 .001',mass='0')
    world=root.find('worldbody')
    for g in list(world.findall('geom')):
        if g.get('name')=='target_cube_visual':world.remove(g)
        if g.get('type')=='plane':g.set('conaffinity','7')
    cube=ET.SubElement(world,'body',name='grasp_object',pos='.35 0 .015')
    ET.SubElement(cube,'freejoint',name='object_free')
    ET.SubElement(cube,'inertial',pos='0 0 0',mass='.05',diaginertia='.0000075 .0000075 .0000075')
    ET.SubElement(cube,'geom',name='object_collision',type='box',size='.015 .015 .015',rgba='.85 .12 .06 1',
        contype='4',conaffinity='9',condim='4',friction='1 .005 .0001',solref='.004 1',solimp='.99 .999 .001')
    tool=root.find(".//body[@name='omx_end_effector_link']")
    ET.SubElement(tool,'site',name='grasp_center',pos='.01 0 0',size='.002')
    root.remove(root.find('keyframe'))
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model)
    data.qpos[:old.nq]=old_data.qpos
    mujoco.mj_forward(model,data)
    ET.SubElement(ET.SubElement(root,'keyframe'),'key',name='start',qpos=fmt(data.qpos))
    assert model.neq==0, 'No equality attachment is permitted'
    assert np.count_nonzero(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)==2
    return root,model,data


class ArmIK:
    def __init__(self,model):
        self.model=model;self.data=mujoco.MjData(model)
        self.site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'grasp_center')
        js=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'omx_joint'+str(i)) for i in range(1,5)]
        self.qa=model.jnt_qposadr[js];self.lo=model.jnt_range[js,0];self.hi=model.jnt_range[js,1]
    def solve(self,data,goal,guess,orientation_weight=.08):
        self.data.qpos[:]=data.qpos
        def residual(q):
            self.data.qpos[self.qa]=q;mujoco.mj_forward(self.model,self.data)
            # Four-DoF arm: prioritize center position, then downward approach.
            axis=self.data.site_xmat[self.site].reshape(3,3)[:,0]
            return np.r_[self.data.site_xpos[self.site]-goal,orientation_weight*axis[:2],orientation_weight*(axis[2]+1)*.3]
        fit=least_squares(residual,np.clip(guess,self.lo+1e-6,self.hi-1e-6),bounds=(self.lo,self.hi),max_nfev=35,ftol=1e-6,xtol=1e-6,gtol=1e-6)
        return fit.x


def run(viewer=False):
    folder=OUT/('grasp_pickup_interactive' if viewer else 'grasp_pickup');folder.mkdir(parents=True,exist_ok=True)
    root,model,data=build();ET.ElementTree(root).write(folder/'dynamic_scene.xml',encoding='unicode')
    joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(j)) for j in joints]
    arm=np.array([n.startswith('omx_joint') for n in names]);grip=np.array(['gripper' in n for n in names]);legs=~(arm|grip)
    nominal=data.qpos[qa].copy();crouch=nominal.copy()
    for i,n in enumerate(names):
        if not legs[i]:continue
        if n.endswith('HFE'):crouch[i]=1.1 if n.startswith(('FL','FR')) else -1.1
        if n.endswith('KFE'):crouch[i]=-2.2 if n.startswith(('FL','FR')) else 2.2
    target=nominal.copy();kp=np.where(arm,3.,np.where(grip,500.,6.));kd=np.where(arm,.08,np.where(grip,.03,.4))
    limits=model.actuator_ctrlrange[:,1].copy();limits[grip]=2.
    object_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'grasp_object')
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    ik=ArmIK(model);start_arm=target[arm].copy();pre_arm=None;goal=np.array([.35,0,.10]);grasp_xy=None
    window=None;start=time.monotonic()
    if viewer:
        from mujoco import viewer as mv
        window=mv.launch_passive(model,data);window.cam.lookat[:]=[.06,0,.25];window.cam.distance=1.2;window.cam.azimuth=135;window.cam.elevation=-22;window.opt.geomgroup[3]=0
    rows={k:[] for k in ('time','qpos','qvel','torque','target','object_pos','grasp_pos','pad_force','object_floor_force','height','tilt')}
    for step in range(round(DURATION/model.opt.timestep)):
        if window is not None and not window.is_running():break
        t=data.time
        lower=smooth(t,1,5);stand=smooth(t,10,14)
        target[legs]=((1-lower)*nominal+lower*crouch)[legs]
        target[legs]=(1-stand)*target[legs]+stand*nominal[legs]
        target[grip]=.012+smooth(t,7,8)*(-.010-.012)
        if t<1:target[arm]=start_arm
        elif t<5:
            # Joint reference opens the arm above the object while legs lower.
            if pre_arm is None:
                probe=mujoco.MjData(model);probe.qpos[:]=data.qpos
                probe.qpos[2]=.135
                pre_arm=ik.solve(probe,np.array([.35,0,.105]),np.array([0,.9,-.5,1.1]))
            target[arm]=(1-lower)*start_arm+lower*pre_arm
        elif step%20==0:
            if grasp_xy is None:grasp_xy=data.xpos[object_id,:2].copy()
            z=.105+smooth(t,5,7)*(.017-.105)
            z+=smooth(t,8,10)*.08
            z+=smooth(t,10,14)*.13
            goal=np.r_[grasp_xy,z]
            target[arm]=ik.solve(data,goal,target[arm])
        raw=kp*(target-data.qpos[qa])-kd*data.qvel[va]
        raw[arm|grip]+=data.qfrc_bias[va[arm|grip]]
        data.ctrl[:]=np.clip(raw,-limits,limits)
        mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all():raise RuntimeError('Nonfinite state')
        if step%10==0:
            mujoco.mj_forward(model,data)
            pads=np.zeros(2);floor=0.
            for ci,c in enumerate(data.contact):
                gs=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in (c.geom1,c.geom2)]
                if 'object_collision' not in gs:continue
                force=np.zeros(6);mujoco.mj_contactForce(model,data,ci,force)
                for k,side in enumerate(('left','right')):
                    if 'grasp_pad_'+side in gs:pads[k]+=force[0]
                if any(model.geom_bodyid[g]==0 for g in (c.geom1,c.geom2)):floor+=force[0]
            values=dict(time=data.time,qpos=data.qpos.copy(),qvel=data.qvel.copy(),torque=data.ctrl.copy(),target=target.copy(),object_pos=data.xpos[object_id].copy(),grasp_pos=data.site_xpos[ik.site].copy(),pad_force=pads,object_floor_force=floor,height=data.xpos[base,2],tilt=np.arccos(np.clip(data.xmat[base].reshape(3,3)[2,2],-1,1)))
            for key,value in values.items():rows[key].append(value)
        if window is not None:
            window.sync();delay=data.time-(time.monotonic()-start)
            if delay>0:time.sleep(delay)
        if step%1000==0:print('t',round(data.time,1),'cube z',round(data.xpos[object_id,2],3),'grasp',np.round(data.site_xpos[ik.site],3),'pads',np.round(rows['pad_force'][-1],2),flush=True)
    if window is not None:window.close()
    rows={k:np.array(v) for k,v in rows.items()};np.savez_compressed(folder/'trajectory.npz',**rows)
    completed=bool(data.time>=DURATION-model.opt.timestep/2)
    final=rows['time']>max(0.,data.time-2.)
    report=dict(duration_s=float(data.time),completed=completed,success=bool(completed and np.all(rows['object_pos'][final,2]>.12) and np.all(rows['pad_force'][final].min(axis=1)>.01)),
        object_mass_kg=.05,object_side_m=.03,final_object_center_m=rows['object_pos'][-1].tolist(),
        minimum_final_object_height_m=float(rows['object_pos'][final,2].min()),final_pad_normal_forces_N=rows['pad_force'][-1].tolist(),
        final_object_floor_force_N=float(rows['object_floor_force'][-1]),maximum_base_tilt_deg=float(np.rad2deg(rows['tilt'].max())),
        fall_detected=bool(rows['height'].min()<.07 or rows['tilt'].max()>np.deg2rad(50)),
        contact_solver={'cone':'elliptic','impratio':100,'solver':'Newton','noslip_iterations':0},equality_constraints=int(model.neq),object_free_joint=True,object_pose_prescribed_after_initialization=False,
        limitations=['Rigid ideal bracket; no structural or bolt verification.','Flat contact pads approximate the CAD inner finger faces; friction coefficient 1 is assumed.','No grasp weld or kinematic object attachment.','Arm/self/robot collision response is not enabled; reference and actual trajectory require geometric screening.'])
    (folder/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    return folder,report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--viewer',action='store_true');p.add_argument('--render',action='store_true');p.add_argument('--language',choices=['en','it'],default='en');args=p.parse_args()
    folder,report=run(args.viewer)
    if args.render:
        from manipulation.animation import transition_gif
        transition_gif(folder,phase_function=lambda t:phase(t,args.language),output_name='grasp_pickup_en.gif' if args.language=='en' else 'grasp_pickup.gif')


if __name__=='__main__':main()
