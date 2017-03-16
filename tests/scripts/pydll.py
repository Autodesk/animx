from ctypes import *
import maya.cmds as cmds
import maya.OpenMaya as om
import random as rand

"""
Script to test the functionality of the curve animation dynamic library.
Instead of relying on a plugin, this script dynamically loads both the curve library and
the plugin library. Plugin library exposes callable functions that create curve wrapper
objects that could be passed as arguments to the animation library functions.
"""

animDll = cdll.LoadLibrary("/Path/to/libAnimX.dylib")          # path to libAnimX.dylib or AnimX.dll
pluginDll = cdll.LoadLibrary("/Path/to/AnimXPlugin.bundle")    # path to AnimXPlugin.bundle or AnimXPlugin.mll
precision = 5                                                          # value comparison precision

"""Class representing a quaternion return value from the library function"""
class Quaternion(Structure):
    _fields_=[
        ("x", c_double), 
        ("y", c_double),
        ("z", c_double),
        ("w", c_double)
    ]

"""Library function that constructs new CurveAPI object"""
newFunc = getattr(pluginDll, "CreateParamCurveAccessor")
newFunc.restype = c_void_p
newFunc.argtypes = [c_char_p]

"""Library function that deletes the CurveAPI object"""
deleteFunc = getattr(pluginDll, "DeleteParamCurveAccessor")
deleteFunc.argtypes = [c_void_p]

"""Library function to evaluate a single curve"""
evalFunc = getattr(animDll, "evaluateCurve")
evalFunc.restype = c_double
evalFunc.argtypes = [c_double, c_void_p]

"""Library function to evaluate a trio of rotation curves"""
evalQuatFunc = getattr(animDll, "evaluateQuaternionCurve")
evalQuatFunc.restype = Quaternion
evalQuatFunc.argtypes = [c_double, c_void_p, c_void_p, c_void_p, c_int]


"""Class that manages the lifespan of a CurveAPI object created by the library"""
class ParamCurveWrapper(object):
    ptr = None
    def __init__(self, curveName):
        self.ptr = newFunc( curveName.encode('utf-8') )

    def __del__(self):
        deleteFunc(self.ptr)
    

def frange(start, end=None, inc=None):
    """A range function, that does accept float increments..."""
    if end == None:
        end = start + 0.0
        start = 0.0

    if inc == None:
        inc = 1.0

    L = []
    while 1:
        next = start + len(L) * inc
        if inc > 0 and next >= end:
            break
        elif inc < 0 and next <= end:
            break
        L.append(next)
        
    return L


def checkValue(i, api, scene, curve):
    """Utility function to compare two curve values within a certain precision and print an error message if there is a mismatch"""
    v1 = round(api, precision)
    v2 = round(scene, precision)
    if v1 != v2:
        print "Error. Mismatch for {}[{}]. API:{} vs. Maya:{}".format(curve, int(i), v1, v2)
        return 1
    return 0


def verifyCurve(curve):
    """Test if a single animation curve evaluates to the same values using the library as it does in Maya"""
    if not cmds.objectType(curve, isAType='animCurve'):
        print curve + ' is not a param curve'
        return False

    # find out the time range of the curve
    count = cmds.keyframe(curve, q=1, keyframeCount=1)
    last  = float( cmds.keyframe(curve, q=1, index=(count-1,count-1), timeChange=1)[0] )
    first = float( cmds.keyframe(curve, q=1, index=(0,0), timeChange=1)[0] )

    extends = 10
    last += extends
    first -= extends
    n = last - first
    steps = n
    step = 1
    errors = 0

    curveUtf = curve.encode('utf-8')
    ptr = newFunc(curveUtf)
    if ptr == 0:
        return False

    isRotation = cmds.objectType(curve, isType='animCurveTA') or cmds.objectType(curve, isType='animCurveUA') 

    for i in frange(first, last, step):
        cmds.currentTime(i)
        sceneValue = cmds.getAttr(curve+".output")
        apiValue = evalFunc(i/24.0, ptr)

        # rotation values need to be converted back to Maya's UI units before comparing
        if isRotation:
            apiValue = om.MAngle.internalToUI(apiValue)

        errors += checkValue(i, apiValue, sceneValue, curve)

    deleteFunc(ptr)
    return errors


def rotationInterpolationMethod(curve): 
    """Convert curve interpolation method to an enum value accepted by the library"""
    method = cmds.rotationInterpolation(curve, c=True, q=True)
    switcher = {
        "none": 0,
        "euler": 1,
        "quaternionSlerp": 2,       
        "quaternion": 3,
        "quaternionSquad": 4
    }
    return switcher.get(method, 0)


def verifyCurveQuaternion(curveX, curveY, curveZ):
    curves = [curveX, curveY, curveZ]
    for c in curves:
        if not cmds.objectType(c, isAType='animCurve'):
            print c + ' is not a param curve'
            return 1

    # find out the time range of the curve
    count = cmds.keyframe(curveX, q=1, keyframeCount=1)
    last  = float( cmds.keyframe(curveX, q=1, index=(count-1,count-1), timeChange=1)[0] )
    first = float( cmds.keyframe(curveX, q=1, index=(0,0), timeChange=1)[0] )

    extends = 10
    last += extends
    first -= extends
    n = last - first
    steps = n
    step = 1
    errors = 0

    cX = ParamCurveWrapper(curveX)
    cY = ParamCurveWrapper(curveY)
    cZ = ParamCurveWrapper(curveZ)

    if cX.ptr == 0 or cY.ptr == 0 or cZ.ptr == 0:
        return 1

    interpMethod = rotationInterpolationMethod(curveX)

    for i in frange(first, last, step):
        cmds.currentTime(i)
        sceneValues = [ 
            cmds.getAttr(curveX+".output"),
            cmds.getAttr(curveY+".output"),
            cmds.getAttr(curveZ+".output")
        ]
        apiValuesQuaternion = evalQuatFunc(i/24.0, cX.ptr, cY.ptr, cZ.ptr, interpMethod)
        apiValues = om.MQuaternion(apiValuesQuaternion.x, apiValuesQuaternion.y, apiValuesQuaternion.z, apiValuesQuaternion.w).asEulerRotation()

        errors += checkValue(i, om.MAngle.internalToUI(apiValues.x), sceneValues[0], curves[0])
        errors += checkValue(i, om.MAngle.internalToUI(apiValues.y), sceneValues[1], curves[1])
        errors += checkValue(i, om.MAngle.internalToUI(apiValues.z), sceneValues[2], curves[2])

    return errors


def testCurves():
    """ Testing suite function that will generate a random animation for a cube and run
    checks whether the values evaluated by the animation library match Maya's. The test
    will iterate over all infinity and tangent types as well as interpolation methods
    for rotation curves"""

    cmds.file(f=True, new=True)
    name = 'cube'

    # generate random animation for the cube
    cmds.polyCube(name=name)

    cmds.currentTime(-10)
    cmds.move(rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), name, a=True)
    cmds.rotate(rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0),name, a=True)
    cmds.setKeyframe(name)
    
    cmds.currentTime(0)
    cmds.move(rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), name, a=True)
    cmds.rotate(rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0),name, a=True)
    cmds.setKeyframe(name)

    cmds.currentTime(10)
    cmds.move(rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), name, a=True)
    cmds.rotate(rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0),name, a=True)
    cmds.setKeyframe(name)

    cmds.currentTime(20)
    cmds.move(rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), name, a=True)
    cmds.rotate(rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0),name, a=True)
    cmds.setKeyframe(name)

    cmds.currentTime(30)
    cmds.move(rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), rand.uniform(-100.0, 100.0), name, a=True)
    cmds.rotate(rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0), rand.uniform(-360.0, 360.0),name, a=True)
    cmds.setKeyframe(name)

    

    curves = [ 
        name + '_translateX', 
        name + '_rotateX',
        name + '_rotateY',
        name + '_rotateZ',
        name + '_visibility'
    ]

    rotCurves = [ 
        name + '_rotateX',
        name + '_rotateY',
        name + '_rotateZ',
    ]

    infinities = [
        'constant',
        'linear',
        'cycle',
        'cycleRelative',
        'oscillate'
    ]

    tangents = [
        'linear',
        'fast',
        'slow',
        'flat',
        'step',
        'stepnext',
        'fixed',
        'clamped',
        'plateau',
        'spline'
    ]

    interpolation = [
        'none',
        'euler',
        'quaternion',
        'quaternionSlerp',
        'quaternionSquad'
    ]

    errors = 0

    print '\nTESTING INFINITY MODES\n'

    for inf in infinities:  
        print "### Checking infinity mode: {}".format(inf)
        for curve in curves:            
            cmds.setInfinity(curve, pri=inf, poi=inf)               
            errors += verifyCurve(curve)

    print '\nTESTING TANGENT TYPES\n'

    for tan in tangents:    
        print "### Checking tangent: {}".format(tan)
        cmds.keyTangent(curves, edit=True, time=(':',), itt=tan, ott=tan)
        for curve in curves:
            errors += verifyCurve(curve)
    
    print '\nTESTING INTERPOLATION MODES\n'

    for int in interpolation:   
        print "### Checking interpolation mode: {}".format(int) 
        for curve in rotCurves:
            cmds.rotationInterpolation(curve, convert=int)

        if int == "none" or int == "euler":
            for curve in rotCurves:
                errors += verifyCurve(curve)
        else:
            errors += verifyCurveQuaternion(rotCurves[0], rotCurves[1], rotCurves[2])

    print "\nTotal errors: {}".format(errors)



# testCurves()



