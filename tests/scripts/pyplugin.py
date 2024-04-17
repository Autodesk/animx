import maya.cmds as cmds
import random as rand
import string

"""
Script to test the functionality of the curve animation library Maya plugin. It makes use of
commands exported by the plugin that evaluate the curve and verify the results with Maya scene.
"""

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


def testCurves(precision=13, measureError=True):
    """
    Testing suite function that will generate a random animation for a cube and run
    checks whether the values evaluated by the animation library match Maya's. The test
    will iterate over all infinity and tangent types as well as interpolation methods
    for rotation curves
    """

    def setRandomKey(name, time):
        '''
            Add random translation and rotation keys to object name at the specified time.
            Translation and rotation values are samples from the uniform distribution 
            between [-100, 100] m and [-360, 360] degrees respectively.
        '''

        t = (rand.uniform(-100., 100.), rand.uniform(-100., 100.), rand.uniform(-100., 100.))
        r = (rand.uniform(-360., 360.), rand.uniform(-360., 360.), rand.uniform(-360., 360.))

        cmds.currentTime(time)
        cmds.move  (t[0], t[1], t[2], name, a=True)
        cmds.rotate(r[0], r[1], r[2], name, a=True)
        cmds.setKeyframe(name)

    def testInfinities(curves):
        print ('\nTESTING INFINITY MODES\n')
        errors = 0
        for inf in ['constant', 'linear', 'cycle', 'cycleRelative', 'oscillate']:
            print ("### Checking {} infinity mode".format(inf.upper()))
            for curve in curves:
                cmds.setInfinity(curve, pri=inf, poi=inf)               
                errors += verifyCurve(curve, precision=precision, measureError=measureError)
        return errors

    def testTangents(curves):
        print ('\nTESTING TANGENT TYPES\n')
        errors = 0
        for tan in ['fixed', 'auto', 'autoease', 'automix', 'linear','fast','slow','flat','step','stepnext','fixed','clamped','plateau','spline']:
            print ("### Checking {} tangent mode (non-weighted)".format(tan.upper()))
            cmds.keyTangent(curves, edit=True, time=(':',), itt=tan, ott=tan)
            cmds.keyTangent(curves, edit=True, weightedTangents=False)   
            # Note: we specify to recalculate tangents to test that autoTangents are working correctly
            for curve in curves:
                errors += verifyCurve(curve, precision=precision, measureError=measureError, recalcTangents=True)
        
            print ("### Checking {} tangent mode (weighted)".format(tan.upper()))
            cmds.keyTangent(curves, edit=True, weightedTangents=True)   
            for curve in curves:
                errors += verifyCurve(curve, precision=precision, measureError=measureError, recalcTangents=True)
        cmds.keyTangent(curves, edit=True, weightedTangents=False)  
        return errors

    def testInterpolation(curves):
        print ('\nTESTING INTERPOLATION MODES\n')
        errors = 0
        for interp in ['none', 'euler', 'quaternion', 'quaternionSlerp', 'quaternionSquad']:
            print ("### Checking {} interpolation mode".format(interp.upper()))
            for curve in curves:
                cmds.rotationInterpolation(curve, convert=interp)              
                errors += verifyCurve(curve, precision=precision, measureError=measureError, recalcTangents=False)
        return errors

    # Create a new scene and add a cube with random simulation
    cmds.file(f=True, new=True)
    name = 'cube'
    cmds.polyCube(name=name)
    rand.seed(567)
    for t in range(-10, 40, 10):
        setRandomKey(name, t)

    rotCurves = [ '{}_rotate{}'.format(name,r) for r in ['X', 'Y', 'Z']]

    curves = rotCurves + [name + '_translateX', name + '_visibility']

    errors = 0
    print (f'\nTESTING CURVES, precision = {precision}\n')
    errors += testInfinities(curves)
    errors += testTangents(curves)
    errors += testInterpolation(rotCurves)
    print ("\nTotal errors: {}".format(errors))



def verifyCurve(curve, precision=12, measureError=True, recalcTangents=False):
    if not cmds.objectType(curve, isAType='animCurve'):
        print ('{} is not a param curve'.format(curve))
        return False

    # find out the time range of the curve
    count = cmds.keyframe(curve, q=1, keyframeCount=1)
    last  = float( cmds.keyframe(curve, q=1, index=(count-1,count-1), timeChange=1)[0] )
    first = float( cmds.keyframe(curve, q=1, index=(0,0), timeChange=1)[0] )

    extends = 10
    last += extends
    first -= extends
    step = 1
    apiValues = cmds.AnimX(curve, start=first, stop=last-0.0001, step=step, mer=measureError, rtn=recalcTangents)

    index = 0
    errors = 0
        
    if measureError:
        errTol = pow(10.0, -precision)
        for i in frange(first, last, step):
            if (abs(apiValues[index]) > errTol):
                print ("Error. Mismatch for {}[{}]. difference: {}".format(curve, int(i), apiValues[index]))
                errors += 1
            index += 1
    else:
        sceneValues = []
        for i in frange(first, last, step):
            cmds.currentTime(i)
            v = cmds.getAttr(curve+".output")
            sceneValues.append( v )

            v1 = round(apiValues[index], precision)
            v2 = round(v, precision)
            if v1 != v2:
                print ("Error. Mismatch for {}[{}]. API: {} vs. Maya: {}".format(curve, int(i), v1, v2))
                errors += 1
            index += 1

        if len(apiValues) < len(sceneValues):
            print ("Error. {}: Values count for api ({}) does not equal that of the scene ({})".format(curve, len(apiValues), len(sceneValues)))
            return False

    return errors

# Example usage to test specific curve in a specific time range:
# cmds.AnimX('cube_translateX', start=1, stop=1, step=1)
#
# Or run an automated test:
# testCurves()