"""Exercise real Tk widgets and worker lifecycle without requiring user interaction."""
import time
import json
import sys
from types import SimpleNamespace
import tkinter as tk
import numpy as np
from simulation.host_app import HostApp,matrix_values,matrix_initializer
from simulation.parameter_guide import PARAMETERS,signal_value
from simulation.chassis_sim import ROOT


def main():
    root=tk.Tk();root.withdraw();app=HostApp(root)
    try:
        app.fields['duration'].set('5')
        app.compute()
        deadline=time.perf_counter()+30
        while app.result is None and time.perf_counter()<deadline:
            root.update();time.sleep(.01)
        assert app.result is not None,'Background calculation did not finish'
        for tab in app.pages.values():
            app.tabs.select(tab);root.update();app.render();root.update()
        app.seek('2.5');assert app.current==2.5
        app.toggle_play();deadline=time.perf_counter()+.2
        while time.perf_counter()<deadline:root.update();time.sleep(.01)
        assert app.current>2.5
        app.toggle_play();app.export_to(ROOT/'outputs'/'host_verified')
        # A delayed frame may cross more than one loop; preserve the remainder.
        app.speed.set('1');app.loop.set(True);app.playing=True
        app.anchor_time=4.8;app.anchor_wall=100.;app.advance_playback(110.5)
        assert abs(app.current-.3)<1e-10 and app.playing
        app.loop.set(False);app.anchor_time=4.8;app.anchor_wall=100.;app.advance_playback(100.5)
        assert app.current==5 and not app.playing
        # Clipboard contains only completed-run matrix data, not edited settings/logs.
        for key in ('A','B','Ad','Bd','Q','R','P','K'):
            app.matrix_key.set(key);app.refresh_matrix();app.copy_matrix()
            assert root.clipboard_get()==matrix_initializer(matrix_values(app.result,1,key))
        app.fields['r'].set('99');app.copy_all_matrices()
        copied=json.loads(root.clipboard_get())
        assert set(copied)=={'A','B','Ad','Bd','Q','R','P','K'} and copied['R']==[[.1]]
        app.fields['r'].set('.1')
        app.matrix_node.current(0);app.matrix_key.set('K');app.copy_matrix()
        assert root.clipboard_get()==matrix_initializer(matrix_values(app.result,0,'K'))
        # Exercise diagram hit testing, hover, click-to-pin, and curve navigation.
        app.tabs.select(app.pages['参数导览']);root.update();app.seek('2.5');root.update()
        for key in PARAMETERS:
            x,y=app.guide.label_centers[key];event=SimpleNamespace(x=x,y=y)
            assert app.guide.key_at(event)==key, f'Callout hit test failed: {key}'
            app.guide.motion(event);assert app.guide.hovered==key
            app.guide.click(event);assert app.guide.selected==key and app.guide.pinned
            app.guide.jump();tab=PARAMETERS[key][2]
            assert app.tabs.tab(app.tabs.select(),'text')==tab
            ax=app.figures[tab].axes[PARAMETERS[key][3]]
            assert ax.get_facecolor()[2]<.95
            app.tabs.select(app.pages['参数导览']);root.update()
        state=app.result['states'][2500]
        assert signal_value('left_height',state,None)==f'{state[4]+.18*state[6]:.4f} m'
        app.guide.leave();app.guide.unpin();app.rewind();assert app.current==0 and not app.playing
        if '--screenshots' in sys.argv:
            # Capture only this test application's HWND, never the full desktop.
            import ctypes
            from PIL import ImageGrab
            root.deiconify();root.update()
            hwnd=ctypes.windll.user32.GetParent(root.winfo_id())
            app.guide.selected='pitch';app.guide.pinned=True
            app.tabs.select(app.pages['参数导览']);app.seek('0');root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'parameter_guide_ui.png')
            app.tabs.select(app.pages['C算法与增益']);app.refresh_matrix();root.update()
            ImageGrab.grab(window=hwnd).save(ROOT/'outputs'/'host_verified'/'matrix_ui.png')
            root.withdraw()
        previous=app.result
        app.fields['duration'].set('20');app.fields['physics_dt'].set('0.0001')
        app.compute();app.cancel.set()
        deadline=time.perf_counter()+10
        while app.worker.is_alive() and time.perf_counter()<deadline:root.update();time.sleep(.01)
        root.update();assert not app.worker.is_alive();assert app.result is previous
        print('PASS: Tk startup, worker, eight tabs, seek/play/loop/end, matrix-only clipboard,')
        print('      11 diagram hover/pin/curve links, export and cancellation.')
    finally:
        app.close()


if __name__=='__main__':main()
