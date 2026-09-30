"""A restrained, macOS-inspired light theme for the Tk desktop UI."""
import tkinter as tk
from tkinter import ttk


COLORS={
    'window':'#f5f5f7','surface':'#ffffff','sidebar':'#fbfbfd','text':'#1d1d1f',
    'secondary':'#6e6e73','separator':'#d9d9de','field':'#ffffff',
    'accent':'#0071e3','accent_pressed':'#0062c3','selection':'#e8f2ff',
    'danger':'#d70015','code':'#f7f8fa','success':'#248a3d',
}


def apply_theme(root):
    root.configure(bg=COLORS['window'])
    root.option_add('*Font','{Microsoft YaHei UI} 10')
    root.option_add('*Text.Font','Consolas 10')
    root.option_add('*Text.Background',COLORS['code'])
    root.option_add('*Text.Foreground',COLORS['text'])
    root.option_add('*Text.InsertBackground',COLORS['text'])
    root.option_add('*Text.BorderWidth',0)
    root.option_add('*Text.Relief','flat')
    style=ttk.Style(root);style.theme_use('clam')
    style.configure('.',background=COLORS['window'],foreground=COLORS['text'],font=('Microsoft YaHei UI',10))
    style.configure('TFrame',background=COLORS['surface'])
    style.configure('Surface.TFrame',background=COLORS['surface'])
    style.configure('Toolbar.TFrame',background=COLORS['surface'])
    style.configure('Sidebar.TFrame',background=COLORS['sidebar'])
    style.configure('TLabel',background=COLORS['surface'],foreground=COLORS['text'])
    style.configure('Surface.TLabel',background=COLORS['surface'])
    style.configure('Sidebar.TLabel',background=COLORS['sidebar'])
    style.configure('Title.TLabel',background=COLORS['surface'],foreground=COLORS['text'],font=('Microsoft YaHei UI',15,'bold'))
    style.configure('PageTitle.TLabel',foreground=COLORS['text'],font=('Microsoft YaHei UI',18,'bold'))
    style.configure('Section.TLabel',foreground=COLORS['text'],font=('Microsoft YaHei UI',11,'bold'))
    style.configure('Secondary.TLabel',foreground=COLORS['secondary'])
    style.configure('SidebarHeading.TLabel',background=COLORS['sidebar'],foreground='#86868b',font=('Microsoft YaHei UI',9,'bold'))
    style.configure('Status.TLabel',background=COLORS['surface'],foreground=COLORS['secondary'],padding=(14,7))
    style.configure('TButton',padding=(12,7),background='#ececef',foreground=COLORS['text'],borderwidth=0,relief='flat')
    style.map('TButton',background=[('pressed','#dcdce1'),('active','#e2e2e7'),('disabled','#f1f1f3')],foreground=[('disabled','#aeaeb2')])
    style.configure('Primary.TButton',padding=(16,8),background=COLORS['accent'],foreground='white',font=('Microsoft YaHei UI',10,'bold'))
    style.map('Primary.TButton',background=[('pressed',COLORS['accent_pressed']),('active','#147ce5'),('disabled','#a9cdf3')],foreground=[('disabled','white')])
    style.configure('Danger.TButton',background='#fff0f1',foreground=COLORS['danger'])
    style.map('Danger.TButton',background=[('active','#ffe2e5'),('pressed','#ffd5da')])
    style.configure('Compact.TButton',padding=(10,6),background='#ececef',foreground=COLORS['text'],borderwidth=0,relief='flat')
    style.map('Compact.TButton',background=[('pressed','#dcdce1'),('active','#e2e2e7')])
    style.configure('Nav.TButton',anchor='w',padding=(13,8),background=COLORS['sidebar'],foreground='#3a3a3c',borderwidth=0)
    style.map('Nav.TButton',background=[('active','#f0f0f3')])
    style.configure('NavSelected.TButton',anchor='w',padding=(13,8),background=COLORS['selection'],foreground=COLORS['accent'],font=('Microsoft YaHei UI',10,'bold'),borderwidth=0)
    style.map('NavSelected.TButton',background=[('active',COLORS['selection'])])
    style.configure('TEntry',padding=(8,6),fieldbackground=COLORS['field'],bordercolor=COLORS['separator'],lightcolor=COLORS['separator'],darkcolor=COLORS['separator'],borderwidth=1)
    style.map('TEntry',bordercolor=[('focus',COLORS['accent'])],lightcolor=[('focus',COLORS['accent'])],darkcolor=[('focus',COLORS['accent'])])
    style.configure('TCombobox',padding=(8,5),fieldbackground=COLORS['field'],background=COLORS['field'],bordercolor=COLORS['separator'],arrowcolor=COLORS['secondary'])
    style.map('TCombobox',bordercolor=[('focus',COLORS['accent'])],fieldbackground=[('readonly',COLORS['field'])],selectbackground=[('readonly',COLORS['field'])],selectforeground=[('readonly',COLORS['text'])])
    style.configure('TCheckbutton',background=COLORS['surface'],foreground=COLORS['text'],padding=4)
    style.configure('Surface.TCheckbutton',background=COLORS['surface'])
    style.configure('TLabelframe',background=COLORS['surface'],bordercolor='#e5e5ea',borderwidth=1,relief='solid',padding=14)
    style.configure('TLabelframe.Label',background=COLORS['surface'],foreground=COLORS['text'],font=('Microsoft YaHei UI',11,'bold'))
    style.configure('Hidden.TNotebook',background=COLORS['surface'],borderwidth=0,tabmargins=0)
    style.layout('Hidden.TNotebook',[('Notebook.client',{'sticky':'nswe'})])
    style.layout('Hidden.TNotebook.Tab',[])
    style.configure('TNotebook',background=COLORS['window'],borderwidth=0)
    style.configure('TNotebook.Tab',padding=(14,8),background='#ececef',borderwidth=0)
    style.map('TNotebook.Tab',background=[('selected',COLORS['surface']),('active','#e2e2e7')],foreground=[('selected',COLORS['accent'])])
    style.configure('Horizontal.TProgressbar',background=COLORS['accent'],troughcolor='#e5e5ea',borderwidth=0,thickness=5)
    style.configure('TScale',background=COLORS['surface'],troughcolor='#d8d8dc')
    style.configure('Vertical.TScrollbar',background='#d1d1d6',troughcolor=COLORS['code'],borderwidth=0,arrowcolor=COLORS['secondary'])
    style.configure('Horizontal.TScrollbar',background='#d1d1d6',troughcolor=COLORS['code'],borderwidth=0,arrowcolor=COLORS['secondary'])
    return style


def page_header(parent,title,subtitle):
    header=ttk.Frame(parent);header.pack(fill='x',pady=(2,14))
    ttk.Label(header,text=title,style='PageTitle.TLabel').pack(anchor='w')
    ttk.Label(header,text=subtitle,style='Secondary.TLabel').pack(anchor='w',pady=(3,0))
    return header
