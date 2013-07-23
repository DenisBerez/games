# -*- coding: utf-8 -*-
import math
import pyglet
from pyglet import gl
from pyglet.window import key
from Box2D import *

FPS = 60
TIMESTEP = 1.0 / FPS
W = 100
H = 72
FLOOR = 5

SCALE = 0.1    # World units - screen units conversion factor

world = None  # let's keep world as a global for now
mouse_joint = None

def load_image_centered(filename):
    """Load an image and set its anchor point to the middle."""
    im = pyglet.image.load(filename)
    im.anchor_x = im.width // 2
    im.anchor_y = im.height // 2
    return im
# Load sprites
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
        (1, 600, (-1, -500), 0),  # left wall
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
        friction=2):
    s = b2CircleDef()
    s.radius = radius
    s.localPosition = b2Vec2(*center)
    s.density = density
    s.restitution = restitution
    s.friction = friction
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
        body.userData = self

    def update(self, dt):
        self.sprite.position = world_to_screen(self.body.position)
        self.sprite.rotation = -math.degrees(self.body.angle)

    def destroy(self):
        world.DestroyBody(self.body)
        objects.remove(self)
        
class Wheel(PhysicalObject):
    IMAGE = load_image_centered('textures/wheel.png')
    SHAPEDEFS = [
        circle_def(1.6, friction=50.0)
    ]
    
    
class Cannonball(PhysicalObject):
    RADIUS = 0.8
    IMAGE = load_image_centered('textures/cannonball.png')

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
        cdef.density = 1.0
        cdef.restitution = 0.1
        cdef.friction = 0.5
        body.CreateShape(cdef)
        self.body = body
        body.SetBullet(True)
        body.SetMassFromShapes()


class TankBarrel(PhysicalObject):
    IMAGE = load_image_centered('textures/tank-barrel.png')
    SHAPEDEFS = [
        box_def(2, 0.5, density=1, groupindex= -1)
    ]
    

class Tank(PhysicalObject):
    IMAGE = load_image_centered('textures/tank-body.png')
    SHAPEDEFS = [
        box_def(5.3, 6.1, (6.0, -0.4), groupindex= -1),
        box_def(7.3, 1.4, (0.0, -2.7), density=10, groupindex= -1),
    ]

    WHEEL_POSITIONS = [
        b2Vec2(-4.3, -4),
        b2Vec2(3.8, -4),
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
        
def clamp(val, minimum, maximum):
    return max(minimum, min(maximum, val))

tank_pos = b2Vec2(10, FLOOR + 5)
barrel_angle = 0


def on_mouse_press(x, y, button, modifiers):
    """Fire in the hole!"""
    angle = math.radians(barrel_angle)
    v = b2Vec2(math.cos(angle), math.sin(angle))
    pos = tank_pos + b2Vec2(0, 2) + v * 3
    Cannonball.fire(pos, v * 100)

    p = screen_to_world((x, y))

    # Create a mouse joint on the selected body (assuming it's dynamic)
'''
    # Make a small box.
    aabb = b2AABB()
    aabb.lowerBound = p - (0.001, 0.001)
    aabb.upperBound = p + (0.001, 0.001)


    # Query the world for overlapping shapes.
    body = None
    k_maxCount = 10  # maximum amount of shapes to return

    (count, shapes) = world.Query(aabb, k_maxCount)
    for shape in shapes:
        shapeBody = shape.GetBody()
        if not shapeBody.IsStatic() and shapeBody.GetMass() > 0.0:
            if shape.TestPoint(shapeBody.GetXForm(), p):  # is it inside?
                body = shapeBody
                break

    if body:
        print "Block clicked"
        # A body was selected, create the mouse joint
        md = b2MouseJointDef()
        md.body1 = world.GetGroundBody()
        md.body2 = body
        md.target = p
        md.maxForce = 100000.0
        mouse_joint = world.CreateJoint(md).getAsType()
        body.WakeUp()'''


def on_mouse_release(x, y, button, modifiers):
    global mouse_joint
    if mouse_joint:
        world.DestroyJoint(mouse_joint)
        mouse_joint = None


def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    global mouse_joint
    if mouse_joint:
        p = screen_to_world((x, y))
        mouse_joint.SetTarget(p)


batch = None
tank = None
objects = []


def setup_scene():
    global batch, tank
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
    group = pyglet.sprite.SpriteGroup(
        earth_tex, gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA
    )
    l, b = world_to_screen((0, 0))
    r, t = world_to_screen((W, FLOOR))
    batch.add(4, gl.GL_QUADS, group,
        ('v2f', [l, b, l, t, r, t, r, b]),
        ('t2f', [0, 0.5, 0, 1, 10, 1, 10, 0.5]),
    )
    
    

    tank = Tank((5, FLOOR + 1))
    barrel = TankBarrel((8, FLOOR + 4))
    
    jd = b2RevoluteJointDef()
    #fb = b2Fixture()
    jd.Initialize(tank.body, barrel.body, (11, FLOOR + 7))
    tank.barrel_joint = world.CreateJoint(jd).getAsType()
    
    
    #objects.append()
    objects.append(tank)
    objects.append(barrel)



def on_draw():
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    batch.draw()
    
    
def update(dt):
    world.Step(TIMESTEP, 20, 16)
    for b in objects:
        b.update(dt)
    if keyboard[key.RIGHT]:
        #import ipdb; ipdb.set_trace()
        tank.right()
    elif keyboard[key.LEFT]:
        tank.left()



if __name__ == '__main__':
    world = setup_world()
    setup_scene()

    window = pyglet.window.Window(
        width=int(W / SCALE),
        height=int(H / SCALE)
    )
    keyboard = key.KeyStateHandler()
    window.push_handlers(keyboard)

    window.event(on_draw)
    window.event(on_mouse_press)
    window.event(on_mouse_release)
    window.event(on_mouse_drag)

    pyglet.clock.schedule(update)
    pyglet.app.run()

