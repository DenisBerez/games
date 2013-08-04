# -*- coding: utf-8 -*-
import math
import pyglet
from pyglet import clock
from pyglet.image import AnimationFrame
from pyglet import gl
from pyglet.window import key
from Box2D import (
    b2Vec2, b2PolygonDef, b2World,
    b2BodyDef, b2AABB, b2CircleDef, b2RevoluteJointDef, b2ContactListener,
    b2DistanceJointDef, b2BuoyancyControllerDef, b2Contact, b2ContactResult
)


FPS = 60
TIMESTEP = 1.0 / FPS
W = 100
H = 72
FLOOR = 5
#FLOOR1 = 15

SCALE = 0.1    # World units - screen units conversion factor

world = None  # let's keep world as a global for now

''' Создаем наш мир.
            world =  new b2World(
                new b2Vec2(0, 10)   # Вектор гравитации.
               ,true                          # doSleep флаг.
             )'''
def load_image_centered(filename):
    """Load an image and set its anchor point to the middle."""
    im = pyglet.image.load(filename)
    im.anchor_x = im.width // 2
    im.anchor_y = im.height // 2
    return im

def load_image_ending(filename):
    im = pyglet.image.load(filename)
    im.anchor_x = im.width
    im.anchor_y = im.height
    return im


# Load sprites
tank_body_image1 = load_image_centered('textures/tank-body.png')
tank_body_image2 = load_image_centered('textures/tank-body2.png')
tank_barrel_image1 = pyglet.image.load('textures/tank-barrel.png')
tank_barrel_image2 = load_image_ending('textures/tank-barrel2.png')
earth_tex = pyglet.image.load('textures/earth-tex.png').get_mipmapped_texture()
#flames = pyglet.image.load('textures/ogon1.gif')
tank_body_image1_1 = load_image_centered('textures/1.png')
tank_body_image1_2 = load_image_centered('textures/2.png')
tank_body_image1_3 = load_image_centered('textures/3.png')
tank_body_image1_4 = load_image_centered('textures/4.png')
tank_body_image2_1 = load_image_centered('textures/tank2.1l.png')
tank_body_image2_2 = load_image_centered('textures/tank2.2l.png')
tank_body_image2_3 = load_image_centered('textures/tank2.3l.png')
tank_body_image2_4 = load_image_centered('textures/tank2.4l.png')



def screen_to_world(pos):
    sx, sy = pos
    return b2Vec2(sx * SCALE, sy * SCALE)


def world_to_screen(pos):
    wx, wy = pos
    return (wx / SCALE, wy / SCALE)


def sprite_scale(self):
    return 0.1 / SCALE


def setup_world():
    world_bounds = b2AABB()
    world_bounds.lowerBound = (-200, -1000)
    world_bounds.upperBound = (200, 200)
    world = b2World(
        world_bounds,
        b2Vec2(0, -30),  # Gravity vector
        True  # Use "sleep" optimisation
    )

    wallsdef = b2BodyDef()
    walls = world.CreateBody(wallsdef)
    walls.userData = 'Blocks'

    WALLS = [
        (W, FLOOR * 0.5, (W / 2, FLOOR * 0.5), 0),  # floor
        #(W / 2, 1, (W / 2, H + 1), 0),  # ceiling
        #(1, 600, (-1, -500), 0),  # left wall
        #(1, 600, (W + 1, -500), 0),  # right wall
    ]

    for wall in WALLS:
        shape = b2PolygonDef()
        shape.SetAsBox(*wall)
        walls.CreateShape(shape)

    return world


class GraphicalObject(object):
    # Load image here
    IMAGE = None

    def __init__(self, pos):
        self.create_sprite(pos)

    def create_sprite(self, pos):
        x, y = world_to_screen(pos)
        self.sprite = pyglet.sprite.Sprite(self.IMAGE, x, y, batch=batch)

    def update(self, dt):
        pass

    def destroy(self):
        objects.remove(self)


def box_def(w, h,
        center=(0, 0),
        angle=0,
        density=1,
        restitution=0.1,
        friction=2,
        groupindex=0):
    s = b2PolygonDef()
    s.SetAsBox(w * 0.5, h * 0.5, b2Vec2(*center), angle)
    s.density = density
    s.restitution = restitution
    s.friction = friction
    s.filter.groupIndex = groupindex
    return s


def circle_def(radius,
        center=(0, 0),
        density=1,
        restitution=0.1,
        friction=2,
        groupindex=0):
    s = b2CircleDef()
    s.radius = radius
    s.localPosition = b2Vec2(*center)
    s.density = density
    s.restitution = restitution
    s.friction = friction
    s.filter.groupIndex = groupindex
    return s


class PhysicalObject(GraphicalObject):
    BULLET = False
    SHAPEDEFS = []

    def __init__(self, pos):
        super(PhysicalObject, self).__init__(pos)
        self.create_body(pos)

    def create_body(self, pos):
        if not self.SHAPEDEFS:
            return

        bodydef = b2BodyDef()
        bodydef.position = b2Vec2(*pos)
        body = world.CreateBody(bodydef)
        for shape in self.SHAPEDEFS:
            body.CreateShape(shape)
        body.SetBullet(self.BULLET)
        body.SetMassFromShapes()
        self.body = body
        #buoyancy.AddBody(body)
        body.userData = self

    def update(self, dt):
        self.sprite.position = world_to_screen(self.body.position)
        self.sprite.rotation = -math.degrees(self.body.angle)

    def destroy(self):
        world.DestroyBody(self.body)
        objects.remove(self)


class Brick(PhysicalObject):
    """Just another brick in the wall."""
    W = 4.7
    H = 2
    IMAGE = load_image_centered('textures/brick.png')

    def create_body(self, pos):
        bodydef = b2BodyDef()
        bodydef.position = b2Vec2(*pos)
        body = world.CreateBody(bodydef)
        shape = b2PolygonDef()
        shape.SetAsBox(self.W * 0.5, self.H * 0.5, (0, 0), 0)
        shape.density = 0.5
        shape.restitution = 0.1
        shape.friction = 0.5
        body.CreateShape(shape)
        body.SetMassFromShapes()
        self.body = body
        self.body.userData = self


class HalfBrick(Brick):
    W = 2.2
    H = 2
    IMAGE = load_image_centered('textures/half-brick.png')


class Cannonball(PhysicalObject):
    RADIUS = 0.8
    IMAGE = load_image_centered('textures/cannonball.png')
    exploding = False

    @classmethod
    def fire(self, pos, velocity):
        c = Cannonball(pos)
        c.body.SetLinearVelocity(b2Vec2(*velocity))
        objects.append(c)
        return c

    def create_body(self, pos):
        bodydef = b2BodyDef()
        bodydef.position = b2Vec2(*pos)
        body = world.CreateBody(bodydef)
        cdef = b2CircleDef()
        cdef.radius = self.RADIUS
        cdef.density = 1.0  #вес тела
        cdef.restitution = 0.1 # прыгучесть, отскок
        cdef.friction = 0.5  # сила трения
        body.CreateShape(cdef)
        self.body = body
        body.SetBullet(True)
        body.SetMassFromShapes()
        self.body.userData = self
        
    def explode(self):
        self.exploding = True
        #explosion_sound.play()

    def update(self, dt):
        #import ipdb; ipdb.set_trace()
        if self.exploding and self.body:
            self.destroy()
            objects.append(Explosion(self.body.position))
        else:
            super(Cannonball, self).update(dt)
        
        
        
class Explosion(GraphicalObject):
    IMAGE = load_image_centered('textures/explosion.png')
    ANGULAR_VELOCITY = 720
    MAX_AGE = 0.4
    age = 0
    #import ipdb; ipdb.set_trace()
    def create_sprite(self, pos):
        super(Explosion, self).create_sprite(pos)
        self.sprite.scale = 0.5

    def update(self, dt):
        self.age += dt
        if self.age > self.MAX_AGE:
            self.destroy()
            return
        self.sprite.scale *= 500.0 ** dt  # grow
        self.sprite.rotation += self.ANGULAR_VELOCITY * dt  # spin
        self.sprite.opacity = 255.0 * (1.0 - self.age / self.MAX_AGE)  # fade
        
        
        

    
'''class Flames(GraphicalObject):
    #import ipdb; ipdb.set_trace()
    IMAGE =pyglet.image.load('textures/fire.png')
    def create_sprite(self, pos):
        super(Flames, self).create_sprite(pos)
        self.sprite.scale = 1'''

class ContactListener(b2ContactListener):
    impairment = 0
    def Add(self, point):
        ob1 = point.shape1.GetBody().userData
        ob2 = point.shape2.GetBody().userData
        #ContactResult = b2ContactResult
        
        if isinstance(ob1, Cannonball):
            ob1.explode()
        if isinstance(ob2, Cannonball):
            ob2.explode()
        if isinstance(ob1, Tank) or isinstance(ob2, Tank):
            #import ipdb; ipdb.set_trace()
            if isinstance(ob1, Tank):
                tank = ob1
                body = ob2
            else:
                tank = ob2
                body = ob1
                
            tank.damage(body, self.impairment)
            
    def Result(self, point):
                impairment = point.normalImpulse
                if impairment <= 40:
                    self.impairment = 0
                else:
                    self.impairment = int(impairment/40)



class Wheel(PhysicalObject):
    IMAGE = load_image_centered('textures/wheel.png')
    SHAPEDEFS = [
        circle_def(1.0, friction=50.0)
    ]
        

class Tank(PhysicalObject):
    IMAGE = None
    LIFE = 100
    SHAPEDEFS = [
        box_def(5.3, 3.5, (0.0, -0.4),
                density=6,
                restitution=1.0,
                friction=5,
                 groupindex= -1),
        box_def(4.3, 3.5, (0.0, -2.7),
                density=30,
                restitution=1.0,
                friction=5,
                groupindex= -1),
    ]

    WHEEL_POSITIONS = [
        b2Vec2(-4.3, -4),#расстояние между колесами
        b2Vec2(3.8, -4),#
    ]

    # Angular acceleration
    ACCEL = 20
    motoraccel = 0
    motorspeed = 0

    def create_body(self, pos):
        pos = b2Vec2(*pos)
        super(Tank, self).create_body(pos)
        self.wheels = []
        self.joints = []
        for i, p in enumerate(self.WHEEL_POSITIONS):
            p = pos + p
            w = Wheel(p)
            jdef = b2RevoluteJointDef()
            jdef.enableMotor = i < 2
            jdef.motorSpeed = 0
            jdef.maxMotorTorque = 6000
            jdef.Initialize(self.body, w.body, p)
            j = world.CreateJoint(jdef).asRevoluteJoint()
            self.joints.append(j)
            self.wheels.append(w)

    def left(self):
        self.motoraccel = 1

    def right(self):
        self.motoraccel = -1
        
    def damage(self, body, impairment):
        if isinstance(body, Cannonball):
            self.LIFE = self.LIFE - 50
            print self.LIFE
        if isinstance(body, HalfBrick) or isinstance(body, Brick):
            self.LIFE = self.LIFE - impairment
            print self.LIFE
        if self.LIFE < 75 :
            if self is tank:
#                 import ipdb; ipdb.set_trace()
                self.sprite.image = tank_body_image1_1
                tank_barrel.image = pyglet.image.load('textures/barrel1.png')
            else:
                self.sprite.image = tank_body_image2_1
                tank_barrel2.image = load_image_ending('textures/barrel1l.png')
        if self.LIFE < 50 :
            if self is tank:
                self.sprite.image = tank_body_image1_2
                tank_barrel.image = pyglet.image.load('textures/barrel2.png')
            else:
                self.sprite.image = tank_body_image2_2
                tank_barrel2.image = load_image_ending('textures/barrel2l.png')
        if self.LIFE < 25 :
            if self is tank:
                self.sprite.image = tank_body_image1_3
                tank_barrel.image = pyglet.image.load('textures/barrel3.png')
            else:
                self.sprite.image = tank_body_image2_3
                tank_barrel2.image = load_image_ending('textures/barrel3l.png')
        if self.LIFE <= 0:
            damage_animation = Animation()
            '''pyglet.clock.schedule( damage_animation.tank_animation(self))
            pyglet.app.run()'''
            #clock.set_fps_limit (10)
            damage_animation.tank_animation(self)
            #pyglet.image.AnimationFrame

    def update(self, dt):
        self.motorspeed += self.motoraccel * self.ACCEL ** dt
        self.motorspeed *= 0.01 ** dt
        self.motoraccel = 0
        for j in self.joints[:2]:
            j.SetMotorSpeed(self.motorspeed)
        super(Tank, self).update(dt)
        for w in self.wheels:
            w.update(dt)

    def destroy(self):
        super(Tank, self).destroy()
        for w in self.wheels:
            w.destroy()
        self.wheels[:] = []
        self.joints[:] = []
        
        
class Animation(object):
    def animation_tank(self, tank, image):
        #import pdb;pdb.set_trace()
        tank_damage = map(lambda img: load_image_centered(img),image)
        animation = pyglet.image.Animation.from_image_sequence(tank_damage, 0.3)
        tank.sprite.image = animation
        world.DestroyBody(tank.body)
        
    def tank_animation(self, tank):
        image = []
        if tank == tank:
            for img in range(4, 13):
                image.append('textures/%d.png' %img)
            self.animation_tank(tank, image)
        else:
            for img in range (4, 13):
                image.append('textures/tank2.%dl.png' %img)
            self.animation_tank(tank, image)
        
        '''for image in l:
            #clock.schedule_once(i, 1/10)
            tank_damage = AnimationFrame(image, 1)'''
        #import pdb;pdb.set_trace()
        
        #tank.sprite.on_animation_end
        '''tank.sprite.image = load_image_centered('textures/5.png')
        tank.sprite.image = load_image_centered('textures/6.png')
        tank.sprite.image = load_image_centered('textures/7.png')
        tank.sprite.image = load_image_centered('textures/8.png')'''
#Tank.sprite.on_animation_end()


def clamp(val, minimum, maximum):
    return max(minimum, min(maximum, val))

batch = None
tank = None
tank2 = None
tank_barrel = None
tank_barrel2 = None
objects = []
slowmo = False

'''controlling = tank

camera = b2Vec2(W * 0.5, H * 0.5)



def on_key_press(symbol, modifiers):
    global controlling
    keyboard.on_key_press(symbol, modifiers)
    if symbol == key.TAB:
        if controlling is tank:
            controlling = tank2
        else:
            controlling = tank

def tank_controlled(dt):
    global camera
    if controlling and controlling.body:
        p = controlling.body.position   
        cx, cy = camera + (p - camera) * (1.0 - 0.1 ** dt)
        cx = max(cx, W * 0.5)
        cy = max(cy, H * 0.5)
        camera = b2Vec2(cx, cy)
        
def on_mouse_press(x, y, button, modifiers):
    """Fire in the hole!"""
    if controlling is tank:
        angle = math.radians(-tank_barrel.rotation)
        v = b2Vec2(math.cos(angle), math.sin(angle))
        pos = controlling.body.position + b2Vec2(1.7, 2) + v * 3.2
        Cannonball.fire(pos, v * 100)
    else:
        angle = math.radians(tank_barrel2.rotation)
        v = b2Vec2(math.cos(angle), math.sin(angle))    
        pos = controlling.body.position + b2Vec2(-6, 2) + - v * 3
        Cannonball.fire((pos), v * 100)
    
def on_mouse_motion(x, y, dx, dy):
    global barrel_angle, barrel_angle2
    p = screen_to_world((x, y))
    dx, dy = p - controlling.body.position
    if controlling is tank:
        barrel_angle = math.degrees(math.atan2(dy, dx))
        barrel_angle = clamp(barrel_angle, 0, 80)
    else:
        barrel_angle2 = math.degrees(math.atan2(dy, -dx))
        barrel_angle2 = clamp(barrel_angle2, 0, 80)'''
 
def on_key_press(symbol, modifiers):
    keyboard.on_key_press(symbol, modifiers)
    if symbol == key.E:
        angle = math.radians(-tank_barrel.rotation)
        v = b2Vec2(math.cos(angle), math.sin(angle))
        pos = tank.body.position + b2Vec2(1.7, 2) + v * 3.2
        Cannonball.fire(pos, v * 100)
    if symbol == key.INSERT:
        angle = math.radians(tank_barrel2.rotation)
        v = b2Vec2(math.cos(-angle), math.sin(-angle))    
        pos = tank2.body.position + b2Vec2(-2, 2) + (-v) * 3
        Cannonball.fire((pos), (-v) * 100)


def update(dt):
    world.Step(TIMESTEP, 20, 16)
    tank_barrel.position = world_to_screen(tank.body.position + b2Vec2(1, 0.6))
    #tank_barrel2.rotation = barrel_angle2   
    tank_barrel2.position = world_to_screen(tank2.body.position + b2Vec2(-0.5, 1.6))
    for b in objects:
        b.update(dt)
    if keyboard[key.D]:
        tank.right()        
    if keyboard[key.A]:
        tank.left()
    if keyboard[key.W]:
        if tank_barrel.rotation <= -60.0:
            None
        else:
            tank_barrel.rotation -= 1
            
    if keyboard[key.S]:
        if tank_barrel.rotation == 0:
            None
        else:
            tank_barrel.rotation += 1
        
            return
    if keyboard[key.RIGHT]:
        tank2.right()
    if keyboard[key.UP]:
        if tank_barrel2.rotation >= 60:
            None
        else:
            tank_barrel2.rotation += 1
    if keyboard[key.DOWN]:
        if tank_barrel2.rotation == 0:
            None
        else:
            tank_barrel2.rotation -= 1 
    elif keyboard[key.LEFT]:    
        tank2.left()

#ank_controlled(dt)



def build_wall(x, y, layers):
    MORTAR = 0.0
    for i in xrange(layers):
        cy = y + i * Brick.H
        if i % 2:
            objects.extend([
                HalfBrick((x - (HalfBrick.W + Brick.W) * 0.5 - MORTAR, cy)),
                Brick((x, cy)),
                HalfBrick((x + (HalfBrick.W + Brick.W) * 0.5 + MORTAR, cy)),
            ])
        else:
            objects.extend([
                Brick((x - Brick.W * 0.5 - MORTAR, cy)),
                Brick((x + Brick.W * 0.5 + MORTAR, cy)),
            ])


def setup_scene():
    global batch, tank, tank2, tank_barrel, tank_barrel2
    batch = pyglet.graphics.Batch()

    # Gradient sky
    l, b = world_to_screen((0, FLOOR))
    r, t = world_to_screen((W, H))
    horizon = 177 / 255.0, 202 / 255.0, 1.0
    zenith = 68 / 255.0, 0.5, 1.0
    batch.add(4, gl.GL_QUADS, None,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('c3f', sum([horizon, zenith, zenith, horizon], ())),
    )

    # Create the ground
    #ground = pyglet.sprite.Sprite(earth_tex, 0, 0, batch=batch)
    group = pyglet.sprite.SpriteGroup(earth_tex, gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    l, b = world_to_screen((0, 0))
    r, t = world_to_screen((W, FLOOR))
    batch.add(4, gl.GL_QUADS, group,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('t2f', [0, 0.5, 0, 1, 10, 1, 10, 0.5]),
    )
    #flames = pyglet.sprite.Sprite(flames, batch=batch)
    tank_barrel = pyglet.sprite.Sprite(tank_barrel_image1, batch=batch)
    tank_barrel.position = world_to_screen(b2Vec2(10, FLOOR + 2.5) + b2Vec2(1, 0.6))
    tank_barrel2 = pyglet.sprite.Sprite(tank_barrel_image2, batch=batch)
    tank_barrel2.position = world_to_screen(b2Vec2(90, FLOOR + 2.5) + b2Vec2(-0.7, 4.2))
    
    

    # Create tank sprite
    Tank.IMAGE = tank_body_image1
    tank = Tank((10, FLOOR + 2.5))
    objects.append(tank)
    Tank.IMAGE = tank_body_image2
    tank2 = Tank((90, FLOOR + 2.5))
    #tank1.sprite._set_rotation(180)
    objects.append(tank2)
    
    #build_wall(50, FLOOR, 20)


def on_draw():
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    batch.draw()


if __name__ == '__main__':
    world = setup_world()
    contact_listener = ContactListener()
    world.SetContactListener(contact_listener)
    setup_scene()

    # Warm up the wall physics
    for i in range(200):
        world.Step(0.01, 20, 16)
    # Then freeze the wall in place


    window = pyglet.window.Window(
        width=int(W / SCALE),
        height=int(H / SCALE)
    )
    keyboard = key.KeyStateHandler()
    window.push_handlers(keyboard)
    
   #window.event(on_mouse_press)
    #window.event(on_mouse_motion)
    window.event(on_draw)
    window.event(on_key_press)

    pyglet.clock.schedule(update)
    pyglet.app.run()