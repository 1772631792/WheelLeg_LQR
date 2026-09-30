"""Tk page for exporting the portable production controller."""
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk,filedialog,messagebox
from simulation.chassis_sim import ROOT


class ProductionExportPanel(ttk.Frame):
    def __init__(self,parent,arena_panel):
        super().__init__(parent);self.arena_panel=arena_panel;self.events=queue.Queue();self.worker=None;self.closed=False
        self.profile=tk.StringVar(value='wheelleg-reference')
        self.output=tk.StringVar(value=str(ROOT/'outputs'/'production_code'/'wheelleg_control'))
        self.gain_source=tk.StringVar(value='参考固件默认系数');self.compiler=tk.StringVar(value='gcc')
        title=ttk.Label(self,text='可移植生产代码导出',font=('Microsoft YaHei UI',16,'bold'));title.pack(anchor='w')
        ttk.Label(self,text='生成 MATLAB/Simulink Coder 风格的可编辑 C 模型：wheel_leg_U / Y / B / DW / P 与 initialize / step / terminate。自定义 PID 放在独立 user 文件中；导出后自动做依赖扫描、独立编译和自测。',wraplength=1150).pack(anchor='w',pady=(4,12))
        form=ttk.LabelFrame(self,text='导出配置',padding=10);form.pack(fill='x')
        ttk.Label(form,text='配置名称').grid(row=0,column=0,sticky='w',padx=4,pady=5);ttk.Entry(form,textvariable=self.profile,width=28).grid(row=0,column=1,sticky='ew',padx=4)
        ttk.Label(form,text='增益来源').grid(row=0,column=2,sticky='w',padx=(18,4));ttk.Combobox(form,textvariable=self.gain_source,values=['参考固件默认系数','MuJoCo 页最近重算系数'],state='readonly',width=25).grid(row=0,column=3,sticky='w')
        ttk.Label(form,text='C 编译器').grid(row=0,column=4,sticky='w',padx=(18,4));ttk.Entry(form,textvariable=self.compiler,width=12).grid(row=0,column=5,sticky='w')
        ttk.Label(form,text='输出目录').grid(row=1,column=0,sticky='w',padx=4,pady=5);ttk.Entry(form,textvariable=self.output).grid(row=1,column=1,columnspan=4,sticky='ew',padx=4)
        ttk.Button(form,text='选择…',command=self.choose).grid(row=1,column=5,padx=4)
        form.columnconfigure(1,weight=1);form.columnconfigure(3,weight=1)
        parameters=ttk.LabelFrame(self,text='生成参数（SI 单位）',padding=8);parameters.pack(fill='x',pady=(8,0))
        defaults=[('控制周期 s','sample_time_s','.001'),('机身质量 kg','body_mass_kg','7.645'),('大腿 m','thigh_length_m','.135'),('小腿 m','calf_length_m','.24'),
                  ('髋间距 m','joint_distance_m','.12'),('轮距 m','wheel_distance_m','.52'),('轮力矩 Nm','wheel_torque_limit_nm','8'),('关节力矩 Nm','joint_torque_limit_nm','35'),
                  ('最短腿 m','min_leg_length_m','.15'),('最长腿 m','max_leg_length_m','.33'),('离地阈值 N','airborne_force_n','20'),('最大加速度 m/s²','max_acceleration_m_s2','2.5')]
        self.parameters={}
        for index,(label,key,value) in enumerate(defaults):
            row=index//4;column=(index%4)*2;var=tk.StringVar(value=value);self.parameters[key]=var
            ttk.Label(parameters,text=label).grid(row=row,column=column,sticky='e',padx=(6,2),pady=3)
            ttk.Entry(parameters,textvariable=var,width=9).grid(row=row,column=column+1,sticky='w',padx=(2,10))
        actions=ttk.Frame(self);actions.pack(fill='x',pady=10)
        self.button=ttk.Button(actions,text='生成并验证生产代码包',command=self.start);self.button.pack(side='left')
        ttk.Button(actions,text='打开方案文档',command=self.open_doc).pack(side='left',padx=8)
        self.state=tk.StringVar(value='就绪：默认导出参考固件系数。若选择重算系数，请先在 MuJoCo 页完成一次 C 重算。')
        ttk.Label(actions,textvariable=self.state).pack(side='left',padx=12)
        box=ttk.LabelFrame(self,text='导出与验证日志',padding=5);box.pack(fill='both',expand=True)
        self.log=tk.Text(box,font=('Consolas',10),wrap='word',state='disabled');scroll=ttk.Scrollbar(box,command=self.log.yview);scroll.pack(side='right',fill='y');self.log.configure(yscrollcommand=scroll.set);self.log.pack(fill='both',expand=True)
        self.write('导出内容：wheel_leg 模型入口、U/Y/B/DW/P 结构、可编辑 user hooks、PID 模块、数值核心、生成参数、中文接入与 PID 指南、测试与 manifest。\n')
        self.after(80,self.poll)
    def write(self,text):
        self.log.configure(state='normal');self.log.insert('end',text);self.log.see('end');self.log.configure(state='disabled')
    def choose(self):
        path=filedialog.askdirectory(title='选择生产代码包输出目录')
        if path:self.output.set(str(Path(path)/'wheelleg_control'))
    def open_doc(self):
        import os
        path=ROOT/'docs'/'portable_control_core_plan.md'
        try:os.startfile(path)
        except OSError as exc:messagebox.showerror('无法打开',str(exc))
    def start(self):
        if self.worker and self.worker.is_alive():return
        target=self.output.get().strip();profile=self.profile.get().strip();compiler=self.compiler.get().strip()
        if not target or not profile or not compiler:messagebox.showerror('参数错误','配置名称、输出目录和编译器不能为空。');return
        try:parameters={key:float(var.get()) for key,var in self.parameters.items()}
        except ValueError:messagebox.showerror('参数错误','所有生成参数必须是数字。');return
        report=self.arena_panel.report if self.gain_source.get().startswith('MuJoCo') else None
        if self.gain_source.get().startswith('MuJoCo') and report is None:messagebox.showerror('尚无重算参数','请先到 MuJoCo 实景 → 替代 MATLAB 页面完成一次 C 重算。');return
        self.button.configure(state='disabled');self.state.set('正在生成、扫描依赖、独立编译并运行自测…');self.write('\n开始导出：'+target+'\n')
        def work():
            try:
                from simulation.production_export import export_package
                manifest=export_package(target,profile,report['lqr_coefficients'] if report else None,report['mpc_coefficients'] if report else None,
                                        sample_time=parameters['sample_time_s'],compiler=compiler,parameters=parameters)
                self.events.put(('done',(target,manifest)))
            except Exception as exc:self.events.put(('error',str(exc)))
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()
    def poll(self):
        if self.closed:return
        try:
            while True:
                kind,value=self.events.get_nowait();self.button.configure(state='normal')
                if kind=='done':
                    target,manifest=value;self.state.set('导出完成并通过独立编译/自测。')
                    self.write('PASS 依赖扫描：无 HAL / RTOS / 动态内存依赖\nPASS '+manifest['self_test']+'\n参数 SHA-256：'+manifest['parameter_sha256']+'\n输出：'+target+'\n')
                else:self.state.set('导出失败。');self.write('FAIL '+value+'\n');messagebox.showerror('生产代码导出失败',value)
        except queue.Empty:pass
        self.after(80,self.poll)
    def close(self):self.closed=True
