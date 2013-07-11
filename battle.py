# -*- coding: utf-8 -*-
import math
import pyglet
from pyglet import gl
from pyglet.window import key
from Box2D import (
    b2Vec2, b2PolygonDef, b2World,
    b2BodyDef, b2AABB, b2CircleDef,
    b2MouseJointDef, b2RevoluteJointDef, b2ContactListener,
    b2DistanceJointDef, b2BuoyancyControllerDef
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


# Load sprites
tank_body_image = load_image_centered('textures/tank-body.png')
#tank_body_image1 = load_image_centered('textures/tank-body.png')
tank_barrel_image = pyglet.image.load('textures/tank-barrel.png')
earth_tex = pyglet.image.load('textures/earth-tex.png').get_mipmapped_texture()


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


class HalfBrick(Brick):
    W = 2.2
    H = 2
    IMAGE = load_image_centered('textures/half-brick.png')


class Cannonball(PhysicalObject):
    RADIUS = 0.8
    IMAGE = load_image_centered('textures/cannonball.png')

    @classmethod
    def fire(self, pos, velocity):
        c = Cannonball(pos)
        c.body.SetLinearVelocity(b2Vec2(*velocity))
        #import ipdb; ipdb.set_trace()
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
        
    '''def destroy(self):
        import ipdb; ipdb.set_trace()
        world.DestroyBody(self.body)
        objects.remove(self)'''
        
class Wheel(PhysicalObject):
    IMAGE = load_image_centered('textures/wheel.png')
    SHAPEDEFS = [
        circle_def(1.0, friction=50.0)
    ]
        
        
class Tank(PhysicalObject):
    IMAGE = load_image_centered('textures/tank-body.png')
    
    SHAPEDEFS = [
        box_def(5.3, 6.1, (6.0, -0.4)),
        box_def(17.3, 1.4, (0.0, -2.7), friction=0.5),
        
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

    def update(self, dt):
        self.motorspeed += self.motoraccel * self.ACCEL ** dt
        self.motorspeed *= 0.01 ** dt
        self.motoraccel = 0
        
        super(Tank, self).update(dt)
        for w in self.wheels:
            w.update(dt)

    def destroy(self):
        super(Tank, self).destroy()
        for w in self.wheels:
            w.destroy()
        self.wheels[:] = []
        self.joints[:] = []
        
        
"""class ContactListener(b2ContactListener):
    def Add(self, point):
        o1 = point.shape1.GetBody().userData
        o2 = point.shape2.GetBody().userData

        for o, a in zip((o1, o2), (o2, o1)):
            if isinstance(o, Hook) and isinstance(a, Crate):
                o.set_pick_up(a.body, point.position)

    def Remove(self, point):
        o1 = point.shape1.GetBody().userData
        o2 = point.shape2.GetBody().userData
        for o, a in zip((o1, o2), (o2, o1)):
            if isinstance(o, Hook) and isinstance(a, Crate):
                o.cancel_pick_up()"""


def clamp(val, minimum, maximum):
    return max(minimum, min(maximum, val))


tank_pos = b2Vec2(10, FLOOR + 2.5)
tank_pos2 = b2Vec2(90, FLOOR + 2.5)
barrel_angle = 0


def on_mouse_press(x, y, button, modifiers):
    """Fire in the hole!"""
    angle = math.radians(barrel_angle)
    v = b2Vec2(math.cos(angle), math.sin(angle))
    pos = tank_pos + b2Vec2(0, 2) + v * 3
    Cannonball.fire(pos, v * 100)


def on_mouse_motion(x, y, dx, dy):
    global barrel_angle
    p = screen_to_world((x, y))
    dx, dy = p - tank_pos
    barrel_angle = math.degrees(math.atan2(dy, dx))
    barrel_angle = clamp(barrel_angle, 0, 80)


'''def on_key_press(symbol, modifiers):
    global slowmo
    if symbol == pyglet.window.key.S:
        slowmo = not slowmo'''
def on_key_press(symbol, modifiers):
    global controlling
    keyboard.on_key_press(symbol, modifiers)
    if symbol == key.TAB:
        if controlling is tank:
            controlling = tank1
        else:
            controlling = tank


batch = None
tank = None
tank1 = None
tank_barrel = None
objects = []
slowmo = False

controlling = None

camera = b2Vec2(W * 0.5, H * 0.5)

def tank_controlled(dt):
    global camera
    if controlling and controlling.body:
        p = controlling.body.position
        cx, cy = camera + (p - camera) * (1.0 - 0.1 ** dt)
        cx = max(cx, W * 0.5)
        cy = max(cy, H * 0.5)
        camera = b2Vec2(cx, cy)


def update(dt):
    world.Step(TIMESTEP * 0.2 if slowmo else TIMESTEP, 20, 16)
    tank_barrel.rotation = -barrel_angle
    for b in objects:
        b.update(dt)
        
    '''if controlling is heli:
        if keyboard[key.UP]:
            heli.up()
        if keyboard[key.DOWN]:
            hook.drop()
        if keyboard[key.RIGHT]:
            heli.right()
        elif keyboard[key.LEFT]:
            heli.left()'''
    if controlling is tank:
        if keyboard[key.RIGHT]:
            #import ipdb; ipdb.set_trace()
            tank.right()
        elif keyboard[key.LEFT]:
            tank.left()
    elif controlling is tank1:
        if keyboard[key.RIGHT]:
            #import ipdb; ipdb.set_trace()
            tank.right()
        elif keyboard[key.LEFT]:
            tank.left()

    tank_controlled(dt)



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
    global batch, tank, tank1, tank_barrel, tank_barel1
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

    # Create tank barrel sprite
    tank_barrel = pyglet.sprite.Sprite(tank_barrel_image, batch=batch)
    tank_barrel.position = world_to_screen(tank_pos + b2Vec2(0, 2.6))

    # Create tank sprite
    tank = Tank((10, FLOOR + 2.5))
    objects.append(tank)
    tank1 = Tank((90, FLOOR + 2.5))
    #import ipdb; ipdb.set_trace()
    #tank1.sprite._set_rotation(180)
    tank1.sprite._set_rotation(230) 
    objects.append(tank1)
    #Tank.sprite.rotation
    #tank = pyglet.sprite.Sprite(tank_body_image, batch=batch)
    #tank._set_rotation(180)
    #tank.position = world_to_screen(tank_pos)
    
    #tank1 = pyglet.sprite.Sprite(tank_body_image, batch=batch)
    #tank1.position = world_to_screen(tank_pos2)

    build_wall(50, FLOOR, 20)


def on_draw():
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    batch.draw()


if __name__ == '__main__':
    world = setup_world()
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
    
    window.event(on_mouse_press)
    window.event(on_mouse_motion)
    window.event(on_draw)
    window.event(on_key_press)

    pyglet.clock.schedule(update)
    pyglet.app.run()