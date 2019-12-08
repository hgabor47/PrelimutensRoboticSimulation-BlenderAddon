#########
#
# Prelimutens Robotic
#
####

bl_info = {
    "name": "Prelimutens Robotic",
    "author": "Gabor Horvath",
    "version": (0, 32),
    "blender": (2, 80, 0),
    "location": "View3D > Pre.Rob ",
    "description": "Robotic simulation interface between a device (or service) and Blender systems",
    "warning": "",
    "category": "Mesh"}

import sys
import datetime
import requests
import threading
import functools
import queue
import math
import bmesh
from mathutils import Vector, Euler, Matrix

import ast
import json
import time
from collections import namedtuple

import bpy
from bpy.app.handlers import persistent
from bpy.props import (BoolProperty,EnumProperty,
                       FloatProperty,
                       FloatVectorProperty,
                       IntProperty,
                       PointerProperty,
                       StringProperty)

msg = dict()
msg['noip'] = 'No IP address for device/service!'
msg['badparam'] = 'Bad params from device/service!'
msg['noanswer'] = 'Bad answer from device/service!'
msg['norigidbodies'] = 'Please check it: need least one rigid body and device/service connected to network'
msg['becauseblender'] = 'because Blender behaviours'
msg['needempty'] = 'Need only one selected and empty type object for reference!'
msg['driverdesc'] = 'Not used if empty. Use it instead of BL-driver if it does not working properly.Example data for input from distance sensor: bpy.data.objects["distsensor.008"]["distance"]. For output simple example: bpy.data.objects["motor"].rigid_body_constraint.motor_ang_target_velocity. And you can use $x what is mean "output value" for example: bpy.data.objects["Motor"].rigid_body_constraint.motor_ang_target_velocity=3*$x'
msg['to']='>>'
msg['from']='<<'

ipdefault='http://localhost:82/'
IDdefault ='values2'
Paramspath = '.params'
Valuespath = ''
killthread=False  #thread
execution_queue = queue.Queue()
northpole=0
compass=0
runWithoutService = False
servospeed=15 # 5 mean fast ... 33 mean slow positioning
servostart=0.30
_servostart=1-servostart
servodeadzone=1 # in degree

def ShowMessageBox(message = "", title = "Message Box", icon = 'INFO'):

    def draw(self, context):
        self.layout.scale_x=2
        self.layout.label(text=message)
        print(self.layout)

    bpy.context.window_manager.popup_menu(draw, title = title, icon = icon)

def removekey(d,key):
    r = dict(d)
    del r[key]
    return r

def pos360(v,l,b):
    #value,last,base
    if (math.copysign(1,v)!=math.copysign(1,l)):
        if ((l>90) and (v<-90)):
            print('+360')
            return b+360
        if ((l<-90) and (v>90)):
            print('360')
            return b-360
    return b   # return base ...,-720,-360,0,360,720... need to add to V value 

def min_dist(camera_name, rays_n=10):
    scene = bpy.context.scene
    camo = bpy.data.objects[camera_name]
    cam = camo.data #bpy.data.cameras[camera_name]
    #camlo = camo.location 
    camlo = camo.matrix_world.translation
    #r = camo.rotation_euler
    r = camo.matrix_world.to_euler('XYZ')
    view_layer = bpy.context.view_layer
    z = -cam.lens * 0.027778
    objs, ds = [], []
    for i in range(rays_n + 1):
        for j in range(rays_n + 1):
            x = -0.5 + i / rays_n
            y = 0.5 - j / rays_n
            v = Vector((x, y, z))    
            v.rotate(r)                                      
            result, location, normal, index, object, matrix = scene.ray_cast(view_layer, camlo , v) #camo.location originaly
            if result:
                d = Vector(camlo) - Vector(location)
                d = d.length
                if object.name in objs:
                    if d < ds[objs.index(object.name)]:
                        ds[objs.index(object.name)] = d
                else:
                    objs.append(object.name)
                    ds.append(d)
    if len(ds) > 0:
        return min(ds), objs[ds.index(min(ds))]
    else:
        return None, None
    
def lightsensor(camera_name, pixel = 4):    
    #https://ammous88.wordpress.com/2015/01/16/blender-access-render-results-pixels-directly-from-python-2/
    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.render.resolution_x = pixel
    bpy.context.scene.render.resolution_y = pixel
    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.camera = bpy.data.objects[camera_name]
    bpy.ops.render.render()        
    pixels = list(bpy.data.images['Viewer Node'].pixels)
    
    r=0.0
    g=0.0
    b=0.0
    for i in range(0, len(pixels), 4):
        r=r+pixels[i]
        g=g+pixels[i+1]
        b=b+pixels[i+2]
    pixnum = len(pixels)/4
    r = r / pixnum
    g=g/pixnum
    b=b/pixnum
    Lumi = (0.2126*r) + (0.7152*g) + (0.0722*b)    
    return Lumi,[r,g,b]

def diffangle(ob1,ob2):
    axis = ob1.matrix_world.to_quaternion()
    q = ob2.matrix_world.to_quaternion()
    rotdif = axis.rotation_difference(q)
    #print(rotdif)
    return math.degrees(rotdif.to_euler()[2])   #Z axes diff

def thread_function(name):
    global killthread
    killthread=False
    maxi=100000
    scene = bpy.data.scenes["Scene"]
    ip=scene.prelisim_ip
    ippath=scene.prelisim_ippath
    #ips='http://'+ip+':82'+ippath+'values'
    ips = ip+'/'+ippath
    print(ips)
    while (maxi>0) and (not killthread):
        time.sleep(0.4) 
        maxi-=1    
        payload={}

        #TEST driver
        #scene.prelisim_0003 = scene['distsensor'][0]['distance']
        #scene.prelisim_0004 = scene['distsensor'][1]['distance']
        #ENDTEST

        for item in bpy.types.Scene.prelisim:
            type=item["type"]
            name=item["name"]
            if name[0]=='I':
                #name=name[1:99]
                #Processing the attached drivers
                exec('bpy.types.Scene.prelisimdrv=scene.{0}drv'.format(item["var_name"]))
                if (bpy.types.Scene.prelisimdrv!=''):
                    s='scene.{0}={1}'.format(item["var_name"],bpy.types.Scene.prelisimdrv)
                    exec(s) 
                #End Processing the attached drivers

                if type == "boolean":          
                    exec('bpy.types.Scene.prelisimv=scene.{0}'.format(item["var_name"]))      
                    if bpy.types.Scene.prelisimv:
                        payload[item["name"]]=1
                    else:
                        payload[item["name"]]=0
                if type == "string":          
                    exec('payload[item["name"]]=scene.{0}'.format(item["var_name"]))      
                if type == "float":   
                    s='bpy.types.Scene.prelisimv=round(scene.{0},4)'.format(item["var_name"])
                    exec(s)   
                    #print(bpy.types.Scene.prelisimv)
                    payload[item["name"]]=bpy.types.Scene.prelisimv
                #TODO !!! folytatni kell a többi típussal   
                    
                
        #print(payload)
        r = requests.get(ips,params=payload)
        y = json.loads(r.text)
        #print(y)
        
        #logging.info(y["motL"])
        #bpy.data.objects["E.1M"].rigid_body_constraint.motor_ang_target_velocity=y["motL"]*speed
        #bpy.data.objects["E.2M"].rigid_body_constraint.motor_ang_target_velocity=y["motR"]*speed
        for item in bpy.types.Scene.prelisim:
            type=item["type"]
            name=item["name"]
            if name[0]=='O':
                #name=name[1:99]
                #Processing the attached drivers
                exec('bpy.types.Scene.prelisimdrv=scene.{0}drv'.format(item["var_name"]))
                if (bpy.types.Scene.prelisimdrv!=''):
                    s = bpy.types.Scene.prelisimdrv
                    A = s.split('=')
                    if (len(A)>1):
                        s=A[0]+'='+A[1].replace('$x','scene.'+item["var_name"])
                    else:
                        s='{1}=scene.{0}'.format(item["var_name"],A[0])
                    exec(s) 
                #End Processing the attached drivers
                try:
                    exec('bpy.types.Scene.prelisimv=y["'+name+'"]')
                    s='bpy.data.scenes["Scene"].{0}=bpy.types.Scene.prelisimv'.format(item["var_name"])
                    exec(s)
                except:
                    print("ERR:"+name)
                
        
    print('Thread End')

def clearcache(context):  
    v=''
    try:
        v=context.selected_objects[0]
    except:
        print('NoSelectedObj')
    bpy.ops.mesh.primitive_cube_add(size=2, enter_editmode=False, location=(1000, 0, 0))
    bpy.ops.rigidbody.object_add()
    bpy.ops.object.delete(use_global=False)
    if v:
        v.select_set(True)
        bpy.context.view_layer.objects.active=v
    bpy.ops.ptcache.free_bake_all()

stopcycle=0
class ModalTimerOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    
    idx=0
    def modal(self, context, event):        
        if event.type == 'TIMER':
            self.idx+=1
            print(self.idx)
            if bpy.context.scene.frame_current>5:
                bpy.context.scene.frame_current=1
            #if bpy.context.scene.frame_current>2 and context.scene.frame_start==1:                
                if context.screen.is_animation_playing:
                    bpy.ops.screen.animation_play()                
                self.cancel(context)
                return {'FINISHED'}
            if bpy.context.scene.frame_current>3:
                if context.scene.frame_start!=1: 
                    context.scene.frame_start=1  
                    context.scene.rigidbody_world.solver_iterations=15
                    
            
        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        try:
            bpy.types.Scene.prelisim_timer_stop
        except:
            bpy.types.Scene.prelisim_timer_stop=None                    
        if bpy.types.Scene.prelisim_timer_stop==None:
            bpy.types.Scene.prelisim_timer_stop = wm.event_timer_add(0.01, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):        
        global stopcycle
        wm = context.window_manager
        if bpy.types.Scene.prelisim_timer_stop!=None:
            wm.event_timer_remove(bpy.types.Scene.prelisim_timer_stop)
        bpy.types.Scene.prelisim_timer_stop=None
        stopcycle+=1
        if (stopcycle<2):
            #print('RESTOP'+msg['becauseblender']) 
            bpy.context.scene.frame_current=1
            play(context)
            bpy.ops.wm.modal_timer_operator()
        
class ModalTimerRenderOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.modal_rendertimer_operator"
    bl_label = "Modal Timer Render Operator"
    
    idx=0
    def modal(self, context, event):
        if event.type == 'TIMER':
            self.idx+=1
            if context.screen.is_animation_playing:
                try:                            
                    for o in context.scene['lightsensor']:     
                        try:                                                         
                            l,rgb = lightsensor(o.name)
                            o['luminance']=l
                            o['R']=rgb[0]
                            o['G']=rgb[1]
                            o['B']=rgb[2]
                        except:
                            pass
                except:
                    pass    
                    
            else:      
                self.cancel(context)
                return {'FINISHED'}
        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        try:
            bpy.types.Scene.prelisim_rendertimer_stop
        except:
            bpy.types.Scene.prelisim_rendertimer_stop=None                    
        if bpy.types.Scene.prelisim_rendertimer_stop==None:
            bpy.types.Scene.prelisim_rendertimer_stop = wm.event_timer_add(1, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        if bpy.types.Scene.prelisim_rendertimer_stop!=None:
            wm.event_timer_remove(bpy.types.Scene.prelisim_rendertimer_stop)
        bpy.types.Scene.prelisim_rendertimer_stop=None

def play(context):
    if not context.screen.is_animation_playing:
        bpy.ops.screen.animation_play()   
        bpy.ops.wm.modal_rendertimer_operator() 

def stop(context):
    global stopcycle
    stopcycle=0
    bpy.context.scene.frame_current=1
    play(context)
    bpy.ops.wm.modal_timer_operator()
  
def setInfinity(context): 
    context.scene.frame_current=1
    context.scene.frame_start=1
    context.scene.rigidbody_world.solver_iterations=15
    context.scene.frame_current=2
    context.scene.frame_current=3
    context.scene.frame_start=2
    context.scene.rigidbody_world.solver_iterations=14
    play(context)

def add_variable():
    var = "prelisim_" + str(bpy.context.scene.prelisim_count_total).zfill(4)
    bpy.types.Scene.prelisim_count_total +=1
    return var

class prelisim_stop(bpy.types.Operator):
    bl_idname = "scene.prelisim_stop"
    bl_label = "Stop Simulation"

    def execute(self, context):
        global killthread
        killthread=True
        #ShowMessageBox("Simulation stopped!") 
        stop(context)
        '''try:   
            bpy.app.handlers.frame_change_pre.remove(eventpoke)
        except:
            pass '''
        try:
            #bpy.app.handlers.frame_change_post.remove(eventpeek)
            bpy.app.handlers.frame_change_post.remove(eventframe)
        except:
            pass 
           
        return {'FINISHED'}



# {"V1": ["1.56","4.67",9.84],"Power":"6.6", "Style":"1", "Styleaa":"perrito", "Enabled": "true" }
# {"V1": [1.56,4.67,9.84],"Power":"6.6", "Style":"1", "Enabled": true }
# { "Power":"6.6", "Style":"1", "Enabled" : "true" }
class prelisim_generator(bpy.types.Operator):
    bl_idname = "scene.prelisim_generator"
    bl_label = "Connect and load params"

    def execute(self, context):
        global runWithoutService
        bpy.types.Scene.prelisim_count_total =0
        
        if context.scene.prelisim_ip == "":
            ShowMessageBox(msg['noip']) 
            return {'FINISHED'}
        if context.scene.prelisim_ippath=='':
            context.scene.prelisim_ippath=IDdefault

        try:
            ips = context.scene.prelisim_ip+'/'+context.scene.prelisim_ippath+Paramspath            
            print(ips)
            r = requests.get(ips)
            runWithoutService=False
            if r.status_code!=200:
                runWithoutService=True
                ShowMessageBox(msg['badparam']+str(r.status_code)) 
                return {'CANCELLED'}
        except:
            runWithoutService=True
            ShowMessageBox("Network error!")             
            return {'CANCELLED'}
        
        data = r.text
        print(r.text)
        #data = '{ "title" : "string","address" : "string","V1" : "vector","V2" : "vector","V3" : "vector","Power":"float", "Style":{"0":"XYZ","1":"Red","2":"Fruit"}, "Enabled": "boolean" }'
        script = json.loads(data)

        jkeys = []
        jvalues = []
        list_var_name = []
        ldic = locals()
        
        for k, v in zip(script.keys(), script.values()):
            if type(v) == str:
                s = v.lower()
                if s == "string" or s == "vector" or s == "float" or s == "boolean":
                    jkeys.append(k)
                    jvalues.append(v)
                    list_var_name.append(add_variable())                      
            elif type(v) == dict:
                jkeys.append(k)
                jvalues.append(v)
                list_var_name.append(add_variable())
        bpy.types.Scene.prelisim=[]
        
        for var_name, var, value in zip(list_var_name, jkeys, jvalues):
            #var=var[1:99]
            dir=var[0:1]
            if type(value) == str:
                if value == "boolean":
                    exec("bpy.types.Scene.{0}=BoolProperty(name='{1}')".format(var_name, var))  
                    exec("bpy.types.Scene.{0}drv=StringProperty(description='{2}',name='{3}')".format(var_name, var,msg['driverdesc'],msg['from'] if dir=='I' else msg['to'])) 

                elif value == "vector":
                    t = (0.0, 0.0, 0.0)
                    exec("bpy.types.Scene.{0}=FloatVectorProperty(name='{1}', default={2})".format(var_name, var,t))  
                    exec("bpy.types.Scene.{0}drv=StringProperty(description='{2}',name='{3}')".format(var_name, var,msg['driverdesc'],msg['from'] if dir=='I' else msg['to'])) 

                elif value == "float":
                    exec("bpy.types.Scene.{0}=FloatProperty(name='{1}')".format(var_name, var)) 
                    exec("bpy.types.Scene.{0}drv=StringProperty(description='{2}',name='{3}')".format(var_name, var,msg['driverdesc'],msg['from'] if dir=='I' else msg['to'])) 
                    print(dir)

                elif value == "string":
                    exec("bpy.types.Scene.{0}=StringProperty(name='{1}')".format(var_name, var))  
                    exec("bpy.types.Scene.{0}drv=StringProperty(description='{2}',name='{3}')".format(var_name, var,msg['driverdesc'],msg['from'] if dir=='I' else msg['to'])) 

                bpy.types.Scene.prelisim.append({"var_name":var_name,"name":var,"type":value})

            elif type(value) == dict:
                l = []
                for v, k in zip(value.keys(), value.values()):
                    l.append((v, k, ""))
                exec("bpy.types.Scene.{0}=EnumProperty(items={1}, name='{2}')".format(var_name, l, var))
        bpy.ops.wm.prelisim_timer_operator()    
        return {'FINISHED'}

class prelisim_start(bpy.types.Operator):
    bl_idname="scene.prelisim_start"
    bl_label="Start Simulation"
    

    def execute(self,context):
        global x
        global runWithoutService
        print("Start")
        if context.scene.prelisim_ip == "":
            ShowMessageBox(msg['noip']) 
            return {'FINISHED'}
        
        #bpy.app.handlers.frame_change_pre.append(eventpeek)
        #bpy.app.handlers.frame_change_post.append(eventpoke)
        initialize()
        bpy.app.handlers.frame_change_post.append(eventframe)
        
        context.scene.frame_current = 1
        try:
            if (not runWithoutService):
                if context.scene.prelisim_ippath=='':
                    context.scene.prelisim_ippath=IDdefault
                r = requests.get(context.scene.prelisim_ip+'/'+context.scene.prelisim_ippath+Valuespath)
                if r.status_code!=200:
                    runWithoutService=True
                    ShowMessageBox(msg['noanswer']) 
                    #return {'CANCELLED'}
                #clearcache(context)
                setInfinity(context)
                if (not runWithoutService):                    
                    x = threading.Thread(target=thread_function, args=(1,))
                    x.start()
            else:
                setInfinity(context)
        except:
            ShowMessageBox(msg['norigidbodies']) 
            print('No rigid bodies or http error')   
                             
        print('OK')
        return {'FINISHED'}
     
def eventframe(scene):
    #print('eventfr')    TUDAS:
    #bpy.context.view_layer.update() 
    #bpy.context.scene.update()
    #bpy.ops.anim.update_animated_transform_constraints(True)
    try:
        for o in scene['switches']:
            v = o.matrix_local.to_euler()[2] / ( math.pi /180)
            o['angle']=(0.8*o['angle'])+(0.2*v)
            v = min(math.floor(max(v-o['limitangle'],0)),1)
            o['boolvalue']=v
    except:
        pass
    
    try:        
        for o in scene['distsensor']:     
            try: 
                v = min_dist(o.name)
                o['distance']=v[0]
                o['object']=v[1]
            except:
                removekey(scene['distsensor'],o)
                print('Dist error:'+o.name)
                pass
    except:
        print('Distance measure error:')
        pass

    try:              
        if ((scene.frame_current % 8)==0):
            i=0              
            #print('---')  
            for o in scene['servo']: 
                if o is None:
                    removekey(scene['servo'],o.name)
                    continue
                if (o.name in bpy.data.collections['RigidBodyWorld'].objects):
                    esgen=o['servoobj'][1]
                    esmotor=o['servoobj'][2]
                    opos = o['servoposition']                    
                    olast = o['_servolast']                      
                    obase = o['_servobase']
                    oangle = o['servoangle']                    
                    #print('Servo:'+o.name)               
                    i=i+1
                    if ((opos%360)==0.0):
                        if abs(oangle-(olast+obase))<servodeadzone:
                            #print('dead')
                            continue
                        if (abs(esmotor.rigid_body_constraint.motor_ang_target_velocity)<0.01):
                            #print('speed up')
                            esmotor.rigid_body_constraint.motor_ang_target_velocity=0.1 # because if no speed than stucked in this pos        
                        #print('no') 
                        continue #0.0 mean Blender error so no calculated values randomly :(   
                    o['_servobase'] = pos360(opos,olast,obase)
                    v = opos+o['_servobase']   #actual position
                    o['_servolast'] = v
                    dif = oangle-v
                    
                    #print(dif)
                    if (abs(dif)<servodeadzone):
                        esmotor.rigid_body_constraint.motor_ang_target_velocity=0
                    else:                        
                        vs=esmotor.rigid_body_constraint.motor_ang_target_velocity
                        if (dif<0):
                            #o.rigid_body_constraint.motor_ang_target_velocity=-min(3,(-dif/servospeed))                    
                            esmotor.rigid_body_constraint.motor_ang_target_velocity=(vs*servostart)+((-min(o['servomaxspeed'],(-dif/servospeed)))*_servostart)                    
                        else:
                            #o.rigid_body_constraint.motor_ang_target_velocity=min(3,dif/servospeed)
                            esmotor.rigid_body_constraint.motor_ang_target_velocity=(vs*servostart)+((min(o['servomaxspeed'],(dif/servospeed)))*_servostart)                    
    except:
        print(sys.exc_info()[0])
        pass    

        

'''   # Commented out 
    def eventpoke(scene):
        if scene.frame_current == scene.frame_end:
            print('-poke')
            scene.frame_set(scene.frame_start+1)
            #poke(bpy.context.view_layer.layer_collection)
            
    def eventpeek(scene):
        if scene.frame_current == scene.frame_start:
            print('-peek')
            #peek(bpy.context.view_layer.layer_collection)    
    def poke(layerColl): #poke(bpy.context.view_layer.layer_collection)
        found = None
        #print('-'+layerColl.collection.name)
        for obj in layerColl.collection.objects:
            try:
                copy=obj.matrix_world.copy()
                copyl=obj.matrix_local.copy()
                obj['prelisim_mwc']=copy
                obj['prelisim_mlc']=copyl
                print(obj.name)
            except:
                pass
        for layer in layerColl.children:
            poke(layer)
    def peek(layerColl):
        found = None
        #print('-'+layerColl.collection.name)
        for obj in layerColl.collection.objects:
            try:
                print(obj.name)
                copy=Matrix(obj['prelisim_mwc'])            
                copyl=Matrix(obj['prelisim_mwc'])            
                if copy:
                    obj.matrix_world=copy
                    obj.matrix_local=copyl
            except:
                print('Err')
                pass
        for layer in layerColl.children: 
            poke(layer)
'''
def recurLayerCollection(layerColl, collName):
    found = None
    if (layerColl.name == collName):
        return layerColl
    for layer in layerColl.children:
        found = recurLayerCollection(layer, collName)
        if found:
            return found
            
def find_collection(context, item):
    collections = item.users_collection
    if len(collections) > 0:
        return collections[0]
    return context.scene.collection

def make_collection(collection_name, parent_collection):
    if collection_name in bpy.data.collections: # Does the collection already exist?
        return bpy.data.collections[collection_name]
    else:
        new_collection = bpy.data.collections.new(collection_name)
        parent_collection.children.link(new_collection) # Add the new collection under a parent
        return new_collection

#can exit with bpy.types.Scene.prelisim_timer_exit
class PrelisimTimerOperator(bpy.types.Operator):
    """Prelimutens Robotic Heart Tick"""
    bl_idname = "wm.prelisim_timer_operator"
    bl_label = "Prelisim Timer Operator"
    _timer = None
    def modal(self, context, event):
        try:
            bpy.types.Scene.prelisim_timer_exit
        except:
            bpy.types.Scene.prelisim_timer_exit=False
        if bpy.types.Scene.prelisim_timer_exit:
            self.cancel(context)
            return {'FINISHED'}    
        if event.type == 'TIMER':
            try:
                scene = bpy.data.scenes["Scene"]                
                euler=scene.prelisim_compass.matrix_local.decompose()[1].to_euler()
                euler[2]=euler[2]*(180/math.pi)
                scene.prelisim_compassdir=euler[2]
            except:
                euler=0

            # try:                                    
            #     for o in scene['servo']:                            
            #         v = o.matrix_local.to_euler()                    
            #         v0 = round( v[0] / ( math.pi /180) ,1 )
            #         #if (v0!=0.0):
            #         print('\t\t'+o.name+':'+str(v0))
            #         '''
            #         dif = (o['servoangle']-v)            
            #         if (abs(dif)<4):
            #             print('servo0 '+str(v)+'..'+str(dif))
            #             o.rigid_body_constraint.motor_ang_target_velocity=0
            #         else:
            #             if (dif<0):
            #                 print('servo-1 '+str(v)+'..'+str(dif))
            #                 o.rigid_body_constraint.motor_ang_target_velocity=-min(1,(-dif/4))                    
            #             else:
            #                 print('servo1 '+str(v)+'..'+str(dif))
            #                 o.rigid_body_constraint.motor_ang_target_velocity=min(1,dif/4)
            #         '''                            
            # except:
            #     print(sys.exc_info()[0])
            #     pass

        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        try:
            bpy.types.Scene.prelisim_timer
        except:
            bpy.types.Scene.prelisim_timer=None
        if bpy.types.Scene.prelisim_timer==None:
            bpy.types.Scene.prelisim_timer = wm.event_timer_add(time_step=0.1, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):        
        wm = context.window_manager
        if bpy.types.Scene.prelisim_timer!=None:
            wm.event_timer_remove(bpy.types.Scene.prelisim_timer)
        bpy.types.Scene.prelisim_timer=None

class prelisim_addhelper(bpy.types.Operator):
    bl_idname="scene.prelisim_addhelper"
    bl_label="Add helper to Scene"    

    def execute(self,context):
        global northpole
        global compass
        id=int(bpy.data.scenes['Scene'].prelisim_helper)
        cr = datetime.datetime.now() 
        nametag=str(cr.day)+str(cr.hour)+str(cr.minute)+str(cr.second)
        if id==0: #CollisionSwitch
 ######COLLISIONSWITCH
            
            layerColl = recurLayerCollection(bpy.context.view_layer.layer_collection, 'Master Collection')
            bpy.context.view_layer.active_layer_collection = layerColl
            
            #bpy.ops.object.select_all(action='SELECT')
            #bpy.ops.object.delete(use_global=True)
            bpy.ops.mesh.primitive_cube_add(size=2, enter_editmode=False, location=(0, 0, 0))
            #bpy.ops.object.move_to_collection(collection_index=0, is_new=True, new_collection_name="switch1")
            context.scene.cursor.location = [1,1,0]
            bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
            bpy.ops.object.location_clear(clear_delta=False)
            context.object.name = "base"
            base=bpy.data.objects[context.object.name]
            bpy.ops.rigidbody.object_add()
            context.object.rigid_body.type = 'PASSIVE'
            context.object.rigid_body.kinematic = True
            
            
            #layer=bpy.data.collections['switch1']


            bpy.ops.mesh.primitive_cube_add(size=2, enter_editmode=False, location=(0, 0, 0))
            #bpy.ops.object.move_to_collection(collection_index=0)
            bpy.context.scene.cursor.location = [-1,1,0]
            bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
            bpy.ops.object.location_clear(clear_delta=False)
            bpy.context.object.name = "arm"
            arm=bpy.data.objects[context.object.name]
            context.object['limitangle']=30
            context.object['boolvalue']=0            
            context.object['angle']=0            
            bpy.ops.rigidbody.object_add()
            bpy.context.object.rigid_body.mass = 0.1
            bpy.context.object.rigid_body.mesh_source = 'BASE'
            bpy.context.object.rigid_body.friction = 1
            bpy.context.object.rigid_body.use_margin = True
            bpy.context.object.rigid_body.collision_margin = 0
            bpy.ops.transform.resize(value=(2, 1, 1), orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(True, False, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False)
            


            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0,0 ,0))
            #bpy.ops.object.move_to_collection(collection_index=1)  
            context.object.empty_display_size = 2
            context.object.name = "E.1"
            e1=bpy.data.objects[context.object.name]
            bpy.context.object.rotation_euler[1] = 1.5708
            bpy.ops.rigidbody.constraint_add()
            bpy.context.object.rigid_body_constraint.type = 'MOTOR'
            bpy.context.object.rigid_body_constraint.use_motor_ang = True
            bpy.context.object.rigid_body_constraint.motor_ang_target_velocity = 10
            bpy.context.object.rigid_body_constraint.motor_ang_max_impulse = 0.2
            bpy.context.object.rigid_body_constraint.use_override_solver_iterations = True
            bpy.context.object.rigid_body_constraint.solver_iterations = 40
            bpy.context.object.rigid_body_constraint.object1 = arm
            bpy.context.object.rigid_body_constraint.object2 = base



            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0,0 ,0))
            #bpy.ops.object.move_to_collection(collection_index=1)
            bpy.context.object.empty_display_size = 2
            bpy.context.object.name = "E.2"
            e2=bpy.data.objects[bpy.context.object.name]
            bpy.ops.rigidbody.constraint_add()
            bpy.context.object.rigid_body_constraint.type = 'GENERIC'
            bpy.context.object.rigid_body_constraint.disable_collisions = False
            bpy.context.object.rigid_body_constraint.use_override_solver_iterations = True
            bpy.context.object.rigid_body_constraint.solver_iterations = 40
            bpy.context.object.rigid_body_constraint.object1 = arm
            bpy.context.object.rigid_body_constraint.object2 = base
            bpy.context.object.rigid_body_constraint.use_limit_ang_x = True
            bpy.context.object.rigid_body_constraint.use_limit_ang_y = True
            bpy.context.object.rigid_body_constraint.use_limit_ang_z = True
            bpy.context.object.rigid_body_constraint.limit_ang_x_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_x_upper = 0
            bpy.context.object.rigid_body_constraint.limit_ang_y_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_y_upper = 0
            bpy.context.object.rigid_body_constraint.limit_ang_z_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_z_upper = 1.22173

            bpy.context.object.rigid_body_constraint.use_limit_lin_x = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_y = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_z = True
            bpy.context.object.rigid_body_constraint.limit_lin_x_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_x_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_y_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_y_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_z_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_z_upper = 0


            bpy.ops.object.select_all(action='DESELECT')
            arm.select_set(True)
            e2.select_set(True)
            e1.select_set(True)
            base.select_set(True)
            bpy.context.view_layer.objects.active=base
            bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)

            new_collection = make_collection("Switch"+nametag, context.scene.collection)
            new_collection.objects.link(base)
            new_collection.objects.link(e2)
            new_collection.objects.link(e1)
            new_collection.objects.link(arm)
            context.scene.collection.objects.unlink(e1)
            context.scene.collection.objects.unlink(e2)
            context.scene.collection.objects.unlink(arm)
            context.scene.collection.objects.unlink(base)
            
            
            

        if id==1:
 ###### MotorWheel
            
            layerColl = recurLayerCollection(bpy.context.view_layer.layer_collection, 'Master Collection')
            bpy.context.view_layer.active_layer_collection = layerColl

            bpy.context.scene.cursor.location = [-0.5,0,0]

            bpy.ops.mesh.primitive_cube_add(size=0.5, enter_editmode=False, location=(0, 0, 0))
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
            bpy.context.object.name = "wheelbase"
            wheelbase=bpy.data.objects[bpy.context.object.name]
            bpy.ops.rigidbody.object_add()
            bpy.context.object.rigid_body.mass = 0.5
            bpy.context.object.rigid_body.mesh_source = 'BASE'
            bpy.context.object.rigid_body.friction = 1
            bpy.context.object.rigid_body.use_margin = True
            bpy.context.object.rigid_body.collision_margin = 0
            
            bpy.ops.mesh.primitive_cylinder_add(depth=0.2, enter_editmode=False, align='WORLD', location=(0, 0, 0), rotation=(0, 1.5708, 0))
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
            bpy.ops.transform.translate(value=(-0.5, 0, 0), orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(True, False, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False)
            bpy.context.object.name = "wheel"
            wheel=bpy.data.objects[bpy.context.object.name]
            bpy.ops.rigidbody.object_add()
            bpy.context.object.rigid_body.mass = 0.2
            bpy.context.object.rigid_body.friction = 1
            bpy.context.object.rigid_body.use_margin = True
            bpy.context.object.rigid_body.collision_margin = 0

            bpy.ops.object.empty_add(type='ARROWS', location=(0,0,0))
            context.object.empty_display_size = 2
            context.object.name = "E.W2base"
            esbase=bpy.data.objects[bpy.context.object.name]           

            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0,0,0))
            context.object.empty_display_size = 2
            context.object.name = "E.W2Motor"
            esmotor=bpy.data.objects[bpy.context.object.name]             
            bpy.ops.rigidbody.constraint_add()
            bpy.context.object.rigid_body_constraint.type = 'MOTOR'
            bpy.context.object.rigid_body_constraint.use_motor_ang = True
            bpy.context.object.rigid_body_constraint.motor_ang_target_velocity = -0.1
            bpy.context.object.rigid_body_constraint.motor_ang_max_impulse = 0.5
            bpy.context.object.rigid_body_constraint.use_override_solver_iterations = True
            bpy.context.object.rigid_body_constraint.solver_iterations = 40
            bpy.context.object.rigid_body_constraint.object1 = wheelbase
            bpy.context.object.rigid_body_constraint.object2 = wheel

            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0,0 ,0))
            #bpy.ops.object.move_to_collection(collection_index=1)
            bpy.context.object.empty_display_size = 2
            bpy.context.object.name = "E.3"
            e3=bpy.data.objects[bpy.context.object.name]
            bpy.ops.rigidbody.constraint_add()
            bpy.context.object.rigid_body_constraint.type = 'GENERIC'
            bpy.context.object.rigid_body_constraint.disable_collisions = False
            bpy.context.object.rigid_body_constraint.use_override_solver_iterations = True
            bpy.context.object.rigid_body_constraint.solver_iterations = 40
            bpy.context.object.rigid_body_constraint.use_limit_ang_x = False
            bpy.context.object.rigid_body_constraint.use_limit_ang_y = True
            bpy.context.object.rigid_body_constraint.use_limit_ang_z = True
            bpy.context.object.rigid_body_constraint.limit_ang_x_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_x_upper = 0
            bpy.context.object.rigid_body_constraint.limit_ang_y_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_y_upper = 0
            bpy.context.object.rigid_body_constraint.limit_ang_z_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_z_upper = 0
            bpy.context.object.rigid_body_constraint.use_limit_lin_x = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_y = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_z = True
            bpy.context.object.rigid_body_constraint.limit_lin_x_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_x_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_y_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_y_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_z_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_z_upper = 0
            bpy.context.object.rigid_body_constraint.object1 = wheel
            bpy.context.object.rigid_body_constraint.object2 = wheelbase            
            #bpy.context.object.hide_viewport = True

            bpy.ops.object.select_all(action='DESELECT')
            wheel.select_set(True)
            wheelbase.select_set(True)
            esmotor.select_set(True)
            e3.select_set(True)
            esbase.select_set(True)
            bpy.context.view_layer.objects.active=esbase
            bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)

            new_collection = make_collection("MotorWheel"+nametag, context.scene.collection)
            new_collection.objects.link(wheelbase)
            new_collection.objects.link(wheel)
            new_collection.objects.link(esmotor)
            new_collection.objects.link(esbase)
            new_collection.objects.link(e3)
            context.scene.collection.objects.unlink(e3)
            context.scene.collection.objects.unlink(esbase)
            context.scene.collection.objects.unlink(esmotor)
            context.scene.collection.objects.unlink(wheel)
            context.scene.collection.objects.unlink(wheelbase)
          
        if id==2: 
 ###### World with compass
            
            layerColl = recurLayerCollection(bpy.context.view_layer.layer_collection, 'Master Collection')
            bpy.context.view_layer.active_layer_collection = layerColl

            bpy.ops.mesh.primitive_plane_add(size=20, enter_editmode=False, location=(0, 0, -1.1))
            bpy.ops.rigidbody.object_add()
            context.object.rigid_body.type = 'PASSIVE'
            context.object.rigid_body.kinematic = True
            context.object.rigid_body.friction = 1
            bpy.context.object.rigid_body.use_margin = True
            bpy.context.object.rigid_body.collision_margin = 0.04
            
            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0, 100, 0))
            bpy.context.object.empty_display_size = 2
            bpy.context.object.name = "NORTHPOLE"
            northpole=bpy.data.objects[bpy.context.object.name]
            
            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0, 0, 3))
            bpy.context.object.empty_display_size = 2
            bpy.context.object.name = "COMPASS"
            compass=bpy.data.objects[bpy.context.object.name]
        
            bpy.ops.object.constraint_add(type='TRACK_TO')
            compass.constraints['Track To'].target=northpole
            compass.constraints['Track To'].track_axis = 'TRACK_NEGATIVE_Y'
            compass.constraints['Track To'].up_axis = 'UP_Z'
            
            bpy.ops.object.constraint_add(type='LIMIT_ROTATION')
            compass.constraints["Limit Rotation"].use_limit_x = True
            compass.constraints["Limit Rotation"].use_limit_y = True
            context.scene.prelisim_compass=compass
                            
            
        if id==3:
 ####### DISTANCESENSOR
            
            bpy.ops.object.camera_add(enter_editmode=False, align='VIEW', location=(0, 0, 0), rotation=(1.5708, 0,0))
            bpy.context.object.name = "distsensor"+nametag
            distsensor=bpy.data.objects[bpy.context.object.name]
            #l= len(bpy.data.cameras)-1            
            #bpy.data.cameras[l].name = bpy.context.object.name
            distsensor.data.name=bpy.context.object.name
            context.object['distance']=0.0            
            context.object['object']=''   
            bpy.context.object.data.lens = 150
            bpy.context.object.data.show_sensor = True
            bpy.context.object.hide_render = True

        if id==4:
 ####### AIRENGINE
            fps = context.scene.render.fps
            
            layerColl = recurLayerCollection(bpy.context.view_layer.layer_collection, 'Master Collection')
            bpy.context.view_layer.active_layer_collection = layerColl

            bpy.context.scene.cursor.location = [-0,0,0]
            bpy.ops.object.empty_add(type='ARROWS', location=(0,0,0))
            context.object.empty_display_size = 2
            context.object.name = "E.S2base"
            esbase=bpy.data.objects[bpy.context.object.name]   

            bpy.ops.mesh.primitive_cube_add(size=2, enter_editmode=False, location=(0, 0, 0.2))
            bpy.ops.transform.resize(value=(1, 1, 0.2))
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
            bpy.context.object.name = "airbase"
            airbase=bpy.data.objects[bpy.context.object.name]
            bpy.ops.rigidbody.object_add()
            bpy.context.object.rigid_body.mass = 0.1            

            bpy.ops.object.effector_add(type='WIND', enter_editmode=False, location=(0, 0, 0))
            bpy.context.object.name = "airup"
            airup=bpy.data.objects[bpy.context.object.name]
            bpy.context.object.field.shape = 'LINE'
            bpy.context.object.field.strength = fps+1  #not exactly why but it is a good approach
            bpy.context.object.field.flow = 0
            bpy.context.object.field.falloff_type = 'SPHERE'
            bpy.context.object.field.z_direction = 'POSITIVE'
            bpy.context.object.field.falloff_power = 10
            bpy.context.object.field.use_min_distance = False
            bpy.context.object.field.use_max_distance = True
            bpy.context.object.field.distance_max = 2

                        
            bpy.ops.object.effector_add(type='WIND',  enter_editmode=False, location=(0, 0, 0))
            bpy.ops.transform.rotate(value=3.14159, orient_axis='X', orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(True, False, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False)
            bpy.context.object.name = "airdown"
            airdown=bpy.data.objects[bpy.context.object.name]
            bpy.context.object.field.shape = 'LINE'
            bpy.context.object.field.strength = 1
            bpy.context.object.field.flow = 0
            bpy.context.object.field.falloff_type = 'SPHERE'
            bpy.context.object.field.z_direction = 'POSITIVE'
            bpy.context.object.field.falloff_power = 3
            bpy.context.object.field.use_min_distance = True
            bpy.context.object.field.distance_min = 2
            bpy.context.object.field.use_max_distance = True
            bpy.context.object.field.distance_max = 3            

            
            bpy.ops.object.select_all(action='DESELECT')
            airup.select_set(True)
            airdown.select_set(True)
            esbase.select_set(True)
            bpy.context.view_layer.objects.active=esbase
            bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)

            bpy.ops.object.select_all(action='DESELECT')
            esbase.select_set(True)
            airbase.select_set(True)
            bpy.context.view_layer.objects.active=airbase
            bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)


            new_collection = make_collection("AirWheel"+nametag, context.scene.collection)
            new_collection.objects.link(airbase)
            new_collection.objects.link(airup)
            new_collection.objects.link(airdown)
            new_collection.objects.link(esbase)
            context.scene.collection.objects.unlink(airup)
            context.scene.collection.objects.unlink(airdown)
            context.scene.collection.objects.unlink(airbase)
            context.scene.collection.objects.unlink(esbase)
        
        if id == 5 :
 #print('TODO:LightSensor')
            bpy.ops.mesh.primitive_plane_add(size=1, enter_editmode=False, location=(0, 0, -0.02))
            bpy.context.object.name = "LightDetector"
            context.object['lightdetector']=True  
            lightdetector=bpy.data.objects[bpy.context.object.name]            
            bpy.ops.object.camera_add(enter_editmode=False, align='VIEW', location=(0,0,0), rotation=(0,0,0))
            bpy.context.object.name = "LightSensor"
            context.object['lightsensor']=True  
            context.object['R']=1.0
            context.object['G']=1.0
            context.object['B']=1.0
            lightsensor=bpy.data.objects[bpy.context.object.name]    
            bpy.context.object.data.type = 'ORTHO'
            bpy.context.object.data.ortho_scale = 1
            bpy.context.object.data.clip_start = 0.01
            bpy.context.object.data.clip_end = 0.03
            bpy.context.object.data.display_size = 0.01

            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0, 0, 1))
            bpy.context.object.rotation_euler[0] = 3.14159
            context.object['lightsensordir']=True  
            lightsensordir=bpy.data.objects[bpy.context.object.name]    

            #NODE for image pixel extracting
            bpy.context.scene.use_nodes = True
            tree = bpy.context.scene.node_tree
            links = tree.links
            for n in tree.nodes:
                tree.nodes.remove(n)
            rl = tree.nodes.new('CompositorNodeRLayers')      
            rl.location = 100,100
            v = tree.nodes.new('CompositorNodeViewer')   
            v.location = 500,100
            v.use_alpha = False
            links.new(rl.outputs[0], v.inputs[0])  # link Image output to Viewer input
            bpy.context.scene.render.engine = 'CYCLES'
            bpy.context.scene.render.resolution_x = 4
            bpy.context.scene.render.resolution_y = 4
            bpy.context.scene.render.resolution_percentage = 100            

            bpy.ops.object.select_all(action='DESELECT')
            lightdetector.select_set(True)
            lightsensordir.select_set(True)
            lightsensor.select_set(True)
            bpy.context.view_layer.objects.active=lightsensor
            bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)

        if id == 6 :
            print('TODO:GyroSensor')

        if id == 7:
 ##### SERVO            
            print("TODO Servo")
            # sel = bpy.context.selected_objects
            # if ((sel==[]) or (sel[0].type!='EMPTY')):
            #     ShowMessageBox(msg['needempty']) 
            #     return {'FINISHED'}
            
            
            layerColl = recurLayerCollection(bpy.context.view_layer.layer_collection, 'Master Collection')
            bpy.context.view_layer.active_layer_collection = layerColl
            
            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0,0,0))
            context.object.empty_display_size = 2
            context.object.name = "E.S1Generic"                        
            esgen=bpy.data.objects[bpy.context.object.name]

            bpy.ops.object.empty_add(type='SINGLE_ARROW', location=(0,0,0))
            context.object.empty_display_size = 2
            context.object.name = "E.S2Motor"
            
            esmotor=bpy.data.objects[bpy.context.object.name]           

            bpy.ops.object.empty_add(type='ARROWS', location=(0,0,0))
            context.object.empty_display_size = 2
            context.object.name = "E.S2base"
            esbase=bpy.data.objects[bpy.context.object.name]           

            bpy.context.scene.cursor.location = [-0.5,0,0]
            
            bpy.ops.mesh.primitive_cube_add(size=0.5, enter_editmode=False, location=(0, 0, 0))
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
            bpy.context.object.name = "servobase"
            servobase=bpy.data.objects[bpy.context.object.name]
            
            bpy.ops.mesh.primitive_cube_add(size=0.5, enter_editmode=False, location=(0, 0, 0))
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
            bpy.ops.transform.translate(value=(-0.5, 0, 0), orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(True, False, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False)
            bpy.context.object.name = "servo"
            servo=bpy.data.objects[bpy.context.object.name]

            bpy.ops.object.select_all(action='DESELECT')
            servo.select_set(True)
            servobase.select_set(True)
            esmotor.select_set(True)
            esgen.select_set(True)
            esbase.select_set(True)
            bpy.context.view_layer.objects.active=esbase #servobase
            bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)
            #servobase.rotation_euler=sel[0].rotation_euler
            #servobase.location=sel[0].location

            bpy.ops.object.select_all(action='DESELECT')
            servobase.select_set(True)
            bpy.context.view_layer.objects.active = servobase
            #servobase
            bpy.ops.rigidbody.object_add()
            bpy.context.object.rigid_body.type = 'PASSIVE'
            bpy.context.object.rigid_body.mass = 1
            bpy.context.object.rigid_body.mesh_source = 'BASE'
            bpy.context.object.rigid_body.friction = 1
            bpy.context.object.rigid_body.use_margin = True
            bpy.context.object.rigid_body.collision_margin = 0

            bpy.ops.object.select_all(action='DESELECT')
            esgen.select_set(True)
            bpy.context.view_layer.objects.active = esgen
            bpy.ops.rigidbody.constraint_add()
            bpy.context.object.rigid_body_constraint.type = 'GENERIC'
            bpy.context.object.rigid_body_constraint.use_limit_ang_x = True #False is OK but... softlimiter
            bpy.context.object.rigid_body_constraint.use_limit_ang_y = True
            bpy.context.object.rigid_body_constraint.use_limit_ang_z = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_x = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_y = True
            bpy.context.object.rigid_body_constraint.use_limit_lin_z = True
            bpy.context.object.rigid_body_constraint.limit_ang_x_lower = 0 # not working realtime and nor with driver
            bpy.context.object.rigid_body_constraint.limit_ang_x_upper = 0
            bpy.context.object.rigid_body_constraint.limit_ang_y_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_y_upper = 0
            bpy.context.object.rigid_body_constraint.limit_ang_z_lower = 0
            bpy.context.object.rigid_body_constraint.limit_ang_z_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_x_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_x_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_y_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_y_upper = 0
            bpy.context.object.rigid_body_constraint.limit_lin_z_lower = 0
            bpy.context.object.rigid_body_constraint.limit_lin_z_upper = 0
            bpy.context.object.rigid_body_constraint.use_override_solver_iterations = True
            bpy.context.object.rigid_body_constraint.solver_iterations = 40
            bpy.context.object.rigid_body_constraint.object1 = servobase
            bpy.context.object.rigid_body_constraint.object2 = servo            
            #end servobase

            bpy.ops.object.select_all(action='DESELECT')
            servo.select_set(True)
            bpy.context.view_layer.objects.active = servo
            #servo
            bpy.ops.rigidbody.object_add()
            bpy.context.object.rigid_body.mass = 0.5
            bpy.context.object.rigid_body.friction = 1
            bpy.context.object.rigid_body.use_margin = True
            bpy.context.object.rigid_body.collision_margin = 0
            bpy.context.object['servolimitmin']=0 
            bpy.context.object['servolimitmax']=120
            bpy.context.object['servoangle']=30   #need hold this position (run to this pos)
            bpy.context.object['servoposition']=0.0 #actual pos
            bpy.context.object['servomaxspeed']=2.0 #max
            bpy.context.object['_servolast']=0.0 # calculated last pos 
            bpy.context.object['_servobase']=0.0 # calculated 0, 360, 720 ... -360 -720 ...
            bpy.ops.object.select_all(action='DESELECT')
            esmotor.select_set(True)
            bpy.context.view_layer.objects.active = esmotor
            bpy.ops.rigidbody.constraint_add()
            bpy.context.object.rigid_body_constraint.type = 'MOTOR'
            bpy.context.object.rigid_body_constraint.use_motor_ang = True
            bpy.context.object.rigid_body_constraint.motor_ang_target_velocity = 0
            bpy.context.object.rigid_body_constraint.motor_ang_max_impulse = 4
            bpy.context.object.rigid_body_constraint.use_override_solver_iterations = True
            bpy.context.object.rigid_body_constraint.solver_iterations = 40
            bpy.context.object.rigid_body_constraint.object1 = servobase
            bpy.context.object.rigid_body_constraint.object2 = servo
            #end servo

            v = servo.driver_add('["servoposition"]')            
            v.driver.variables.new()
            v.driver.variables[0].type='ROTATION_DIFF' #no negative value :( 
            v.driver.expression='degrees(var)'
            v.driver.variables[0].targets[0].id=servo
            v.driver.variables[0].targets[1].id=servobase

            v = esgen.driver_add('rigid_body_constraint.limit_ang_x_lower')          
            v.driver.variables.new()
            v.driver.variables[0].type='SINGLE_PROP' 
            v.driver.expression='radians(var)'
            v.driver.variables[0].targets[0].id=servo
            v.driver.variables[0].targets[0].data_path='["servolimitmin"]'

            v = esgen.driver_add('rigid_body_constraint.limit_ang_x_upper')          
            v.driver.variables.new()
            v.driver.variables[0].type='SINGLE_PROP' 
            v.driver.expression='radians(var)'
            v.driver.variables[0].targets[0].id=servo
            v.driver.variables[0].targets[0].data_path='["servolimitmax"]'

            servo['servoobj']=[servobase,esgen,esmotor]

            new_collection = make_collection("Servo"+nametag, context.scene.collection)
            new_collection.objects.link(servobase)
            new_collection.objects.link(servo)
            new_collection.objects.link(esbase)
            new_collection.objects.link(esgen)
            new_collection.objects.link(esmotor)
            context.scene.collection.objects.unlink(servo)
            context.scene.collection.objects.unlink(servobase)
            context.scene.collection.objects.unlink(esbase)
            context.scene.collection.objects.unlink(esgen)
            context.scene.collection.objects.unlink(esmotor)

            

        print(id)
        return {'FINISHED'}

class VIEW3D_PT_prelisim_panel_creator(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Prelimutens Robotic Simulation"
    bl_category = "PreliSim"
    bl_context     = "objectmode"

    @classmethod
    def poll(cls, context):
        return True #context.object

    def draw(self, context):
        layout = self.layout
        
        column = layout.column(align=False)
        #column.prop(context.scene, "prelisim_script")
        column.prop(context.scene, "prelisim_ip")
        column.prop(context.scene, "prelisim_ippath")
        column.operator("scene.prelisim_generator")

        column.prop(context.scene, "prelisim_helper")
        column.operator("scene.prelisim_addhelper")

        for x in range(0, bpy.types.Scene.prelisim_count_total):
            row = column.row(align = True)
            row.prop(context.scene,"prelisim_" + str(x).zfill(4))
            row.prop(context.scene,"prelisim_" + str(x).zfill(4)+"drv")
        
        column.operator("scene.prelisim_start")
        column.operator("scene.prelisim_stop")
        if bpy.types.Scene.prelisim_count_total>0:
            column.prop(context.scene, "prelisim_compass")
            column.prop(context.scene, "prelisim_compassdir")
        
def initialize():    
    sw=[]
    dst=[]
    lig=[]
    ser=[]
    bpy.context.scene['switches']=sw
    bpy.context.scene['distsensor']=dst
    bpy.context.scene['lightsensor']=lig
    bpy.context.scene['servo']=ser
    for o in bpy.context.scene.objects: #bpy.data.objects:        
        if 'limitangle' in o:
            print('Switch: '+o.name)
            sw.append(o)
        else: 
            if 'distance' in o:
                try: 
                    v = min_dist(o.name)
                    dst.append(o)
                    print('DistanceMeter tested: '+o.name)
                except:
                    pass
            else:
                if 'lightsensor' in o:
                    print('LightSensor:'+o.name)
                    o['lightsensor']=len(lig)
                    lig.append(o)
                else:
                    if 'servoangle' in o:
                        print('Servo:'+o.name)
                        o['_servolast']=0
                        o['_servobase']=0
                        o['servo']=len(ser)
                        try: ##version change : need to exists
                            o['servomaxspeed']=o['servomaxspeed']
                        except:
                            o['servomaxspeed']=2.0
                        print('.')
                        o['servoobj'][2].rigid_body_constraint.motor_ang_target_velocity=0.0
                        print(':')
                        ser.append(o)

    bpy.context.scene['switches']=sw
    bpy.context.scene['distsensor']=dst
    bpy.context.scene['lightsensor']=lig
    bpy.context.scene['servo']=ser
    print(lig)
    print('initialize')

def register():
    bpy.types.Scene.prelisim_text = PointerProperty(type=bpy.types.Text)
    bpy.types.Scene.prelisim_count_total = 0 #IntProperty(default=0, min=0, soft_min=0)
    bpy.types.Scene.prelisim_var_panel = StringProperty(default="")
    bpy.types.Scene.prelisim_ip = StringProperty(name="LogicAddress", default=ipdefault)
    bpy.types.Scene.prelisim_ippath = StringProperty(name="ID", default=IDdefault)
    bpy.types.Scene.prelisim_helper = EnumProperty(items=[('0', 'CollisionSwitch', ''), ('1', 'MotorWheel', ''), ('2', 'World', ''),('3','DistanceSensor',''),('4','AirEngine',''),('5','LightSensor',''),('6','GyroSensor',''),('7','Servo','')], name='Helper')
    bpy.types.Scene.prelisim_compass = PointerProperty(type=bpy.types.Object,name="Compass",description='Need a parent for the COMPASS to calc the relative Direction')
    bpy.types.Scene.prelisim_compassdir = FloatProperty(name="Compass Direction")
    #bpy.types.Scene.prelisim_script = StringProperty(name="Script")
    bpy.utils.register_class(prelisim_generator)
    bpy.utils.register_class(prelisim_addhelper)
    bpy.utils.register_class(prelisim_start)
    bpy.utils.register_class(prelisim_stop)
    bpy.utils.register_class(VIEW3D_PT_prelisim_panel_creator)
    bpy.utils.register_class(ModalTimerOperator)
    bpy.utils.register_class(ModalTimerRenderOperator)    
    bpy.utils.register_class(PrelisimTimerOperator)
    

def unregister():
    del bpy.types.Scene.prelisim_text
    del bpy.types.Scene.prelisim_count_total
    del bpy.types.Scene.prelisim_var_panel
    del bpy.types.Scene.prelisim_ip
    del bpy.types.Scene.prelisim_ippath
    del bpy.types.Scene.prelisim_helper
    del bpy.types.Scene.prelisim_compass
    del bpy.types.Scene.prelisim_compassdir
    #del bpy.types.Scene.prelisim_script
    bpy.utils.unregister_class(prelisim_generator)
    bpy.utils.unregister_class(prelisim_addhelper)
    bpy.utils.unregister_class(prelisim_start)
    bpy.utils.unregister_class(prelisim_stop)
    bpy.utils.unregister_class(VIEW3D_PT_prelisim_panel_creator)
    bpy.utils.unregister_class(ModalTimerOperator)
    bpy.utils.register_class(ModalTimerOperator)    
    bpy.utils.unregister_class(PrelisimTimerOperator)

if __name__ == "__main__":
    register()  
    
