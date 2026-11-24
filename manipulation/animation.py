"""Replay recorded states into a GIF without rerunning the dynamics."""
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import font_manager


def transition_gif(folder, phase_function=None, output_name='transition.gif'):
    folder=Path(folder)
    model=mujoco.MjModel.from_xml_path(str(folder/'dynamic_scene.xml'));data=mujoco.MjData(model)
    saved=np.load(folder/'trajectory.npz')
    option=mujoco.MjvOption();option.geomgroup[3]=0;option.sitegroup[:]=0
    cam=mujoco.MjvCamera();cam.lookat[:]=[.02,0,.32];cam.distance=1.35;cam.azimuth=135;cam.elevation=-22
    font=ImageFont.truetype(font_manager.findfont('DejaVu Sans'),18)
    frames=[]
    with mujoco.Renderer(model,height=600,width=800) as renderer:
      for i in range(0,len(saved['time']),5):
        data.qpos[:]=saved['qpos'][i];mujoco.mj_forward(model,data)
        renderer.update_scene(data,camera=cam,scene_option=option)
        image=Image.fromarray(renderer.render().copy())
        t=float(saved['time'][i]);draw=ImageDraw.Draw(image)
        phase=phase_function(t) if phase_function else 'Assestamento iniziale' if t<1 else 'Discesa coordinata' if t<7 else 'Mantenimento finale'
        draw.rectangle((0,0,800,34),fill=(30,34,40));draw.text((12,7),f'{phase} | t = {t:.2f} s',font=font,fill='white')
        frames.append(image)
    # Explicit independent full frames avoid optimized disposal/transparency artifacts.
    palette_sample=Image.new('RGB',(800,600*3))
    for i,frame in enumerate((frames[0],frames[len(frames)//2],frames[-1])):palette_sample.paste(frame,(0,600*i))
    palette=palette_sample.quantize(colors=256)
    paletted=[f.quantize(palette=palette,dither=Image.Dither.NONE) for f in frames]
    paletted[0].save(folder/output_name,save_all=True,append_images=paletted[1:],duration=50,loop=0,optimize=False,disposal=2)
    frames[0].save(folder/'start.png');frames[-1].save(folder/'end.png')
    print('Saved',folder/output_name,len(frames),'frames',flush=True)
