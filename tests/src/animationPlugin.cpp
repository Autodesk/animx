#include <maya/MSimple.h>
#include <maya/MGlobal.h>
#include <maya/MSyntax.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MSelectionList.h>
#include <maya/MQuaternion.h>
#include <maya/MCommandResult.h>
#include <maya/MArgDatabase.h>
#include <maya/MEulerRotation.h>
#include <maya/MItSelectionList.h>
#include <animx.h>
#include <unordered_map>

using namespace adsk;

#define kStartFlag     "-s"
#define kStartFlagLong "-start"
#define kStopFlag      "-e"
#define kStopFlagLong  "-stop"
#define kStepFlag      "-i"
#define kStepFlagLong  "-step"

#ifdef WIN32
#define PLUGIN_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define PLUGIN_DLL_EXPORT extern "C"
#endif


class AnimXCommand : public MPxCommand 
{
public:
    AnimXCommand() 
    {};

    virtual MStatus doIt(const MArgList&);

    static void*    creator()
    {
        return new AnimXCommand;
    }
    
    static MSyntax  newSyntax()
    {
        MSyntax syntax;
        syntax.addFlag(kStartFlag, kStartFlagLong, MSyntax::kTime);
        syntax.addFlag(kStopFlag, kStopFlagLong, MSyntax::kTime);
        syntax.addFlag(kStepFlag, kStepFlagLong, MSyntax::kTime);
        syntax.addArg(MSyntax::kString);
        return syntax;
    }

private:
    MStatus parseArgs(const MArgList &args, MTime &start, MTime &stop, MTime &step)
    {
        MStatus status;
        MArgDatabase argData(syntax(), args, &status);
        if (argData.isFlagSet(kStartFlag))
            argData.getFlagArgument(kStartFlag, 0, start);
        if (argData.isFlagSet(kStopFlag))
            argData.getFlagArgument(kStopFlag, 0, stop);
        if (argData.isFlagSet(kStepFlag))
            argData.getFlagArgument(kStepFlag, 0, step);
        return status;
    }
};                                  


MStatus initializePlugin(MObject _obj)
{
    MFnPlugin   plugin(_obj, "Autodesk","0.1");
    MStatus     stat;
    stat = plugin.registerCommand("AnimX", 
        AnimXCommand::creator,
        AnimXCommand::newSyntax);
    if (!stat)
        stat.perror("registerCommand");
    return stat;
}

MStatus uninitializePlugin(MObject _obj)
{
    MFnPlugin   plugin(_obj);
    MStatus     stat;
    stat = plugin.deregisterCommand("AnimX");
    if (!stat)
        stat.perror("deregisterCommand");
    return stat;
}


/*!
    Implementation of the curve adapter class for Maya MFnAnimCurve objects.
*/
class TparamCurveAccessor : public ICurve
{
public:
    TparamCurveAccessor(MString curveName) 
    { 
        MSelectionList list;
        list.add(curveName);
        MItSelectionList iter(list);
        iter.getDependNode(fCurve);

		// Workaround for a bug in Maya 2014 and 2015, where the "quaternionW" attribute cannot be reliably queried
		// unless it was "networked". Temporarily connecting it and disconnecting resolves the problem
		if (curveName.length() > 0 && isRotation())
		{
			MString cmd = MString("cmds.connectAttr('" + curveName + ".quaternionW', '" + curveName + ".outStippleThreshold')");
			MGlobal::executePythonCommand(cmd);

			cmd = MString("cmds.disconnectAttr('" + curveName + ".quaternionW', '" + curveName + ".outStippleThreshold')");
			MGlobal::executePythonCommand(cmd);
		}
    };

    bool keyframe(double time, Keyframe &key) const override
    {
        MFnAnimCurve curve(fCurve);

        if (curve.numKeys() == 0)
            return false;

        unsigned int index = curve.findClosest( MTime(time, MTime::kSeconds) );
        if (index == curve.numKeys())
            return last(key);

        // if found key lies before the requested time, choose next key instead
        if (curve.time(index).as(MTime::kSeconds) < time)
        {
            if (index != curve.numKeys() - 1)
                index++;
            else
                return last(key);
        }

        return keyframeAtIndex(index, key);
    }

    bool keyframeAtIndex(int index, Keyframe &key) const override
    {
        MFnAnimCurve curve(fCurve);

        if (index < 0 || index >= (int)curve.numKeys())
            return false;

        key.index = index;
        key.time = curve.time(index).as(MTime::kSeconds);
        key.value = curve.value(index);
        
        // fetch tangent values using python command instead OpenMaya interface because
        // the latter casts down the values to float which loses precision in 64 bit time mode
        MStringArray results;
        MString cmd = MString("cmds.keyTangent('" + curve.name() + "', q=1, index=(" + index + ",), ix=1, ox=1, iy=1, oy=1)");
        MGlobal::executePythonCommand(cmd, results);

        key.tanIn.type = tangentType(curve.inTangentType(index));
        key.tanIn.x = (seconds)results[0].asDouble();
        key.tanIn.y = (seconds)results[2].asDouble();

        key.tanOut.type = tangentType(curve.outTangentType(index));
        key.tanOut.x = (seconds)results[1].asDouble();
        key.tanOut.y = (seconds)results[3].asDouble();

        key.linearInterpolation = false;
        key.quaternionW = 1.0;

        // for auto tangents, calculate the value on the fly based on the previous and the next key
        if (key.tanIn.type == TangentType::Auto || key.tanOut.type == TangentType::Auto)
        {			
            bool hasPrev = index > 0;
            bool hasNext = index < (int)curve.numKeys() - 1;
            Keyframe prev, next;
            if (hasPrev)
            {
                prev.time = curve.time(index - 1).as(MTime::kSeconds);
                prev.value = curve.value(index - 1);
            }
            if (hasNext)
            {
                next.time = curve.time(index + 1).as(MTime::kSeconds);
                next.value = curve.value(index + 1);
            }

            CurveInterpolatorMethod interp = key.curveInterpolationMethod(isWeighted());

            if (key.tanIn.type == TangentType::Auto)
                autoTangent(true, key, hasPrev ? &prev : nullptr, hasNext ? &next : nullptr, interp, key.tanIn.x, key.tanIn.y);
            
            if (key.tanOut.type == TangentType::Auto)
                autoTangent(false, key, hasPrev ? &prev : nullptr, hasNext ? &next : nullptr, interp, key.tanOut.x, key.tanOut.y);
        }
            

        MStatus status;

        if (key.tanOut.type == TangentType::Linear &&       
            (tangentType(curve.inTangentType(index + 1, &status)) == TangentType::Linear) &&
            status == MS::kSuccess)
        {
            key.linearInterpolation = true;             
        }

        if (key.tanIn.type == TangentType::Linear &&
            (tangentType(curve.outTangentType(index - 1, &status)) == TangentType::Linear) &&
                status == MS::kSuccess)
        {
            key.linearInterpolation = true;
        }

        if (isRotation())
        {
			double time = MTime(key.time, MTime::kSeconds).as(MTime::uiUnit());
            auto it = quaternionWcache.find(time);
            if (it != quaternionWcache.end())
            {
                key.quaternionW = it->second;
            }
            else
            {
                // there appears to be a bug when trying to fetch quaternionW of a curve node
                // using getAttr -time at a specific frame. Sometimes, the quaternion sign is
                // flipped, so instead we change the time globally to have the curve evaluate
                // properly
                MString result;
                MString cmd = MString("cmds.currentTime(") + time + ")";
                MGlobal::executePythonCommand(cmd);
                cmd = "cmds.getAttr('" + curve.name() + ".quaternionW')";
                MGlobal::executePythonCommand(cmd, result);
                quaternionWcache[time] = key.quaternionW = result.asDouble();
            }
        }

        return true;
    }

    bool first(Keyframe &key) const override
    {
        return keyframeAtIndex(0, key);
    }

    bool last(Keyframe &key) const override
    {
        return (keyframeCount() > 0) ?
            keyframeAtIndex(keyframeCount() - 1, key) :
            false;
    }

    InfinityType preInfinityType() const override
    {
        MFnAnimCurve curve(fCurve);
        return infinityType(curve.preInfinityType());
    }

    InfinityType postInfinityType() const override
    {
        MFnAnimCurve curve(fCurve);
        return infinityType(curve.postInfinityType());
    }

    bool isWeighted() const override
    {
        MFnAnimCurve curve(fCurve);
        return curve.isWeighted();
    }

    bool isStatic() const override
    {
        MFnAnimCurve curve(fCurve);
        return curve.isStatic();
    }

    unsigned int keyframeCount() const override
    {
        MFnAnimCurve curve(fCurve);
        return curve.numKeys();
    }

    bool isRotation() const
    {
        MFnAnimCurve curve(fCurve);
        CurveRotationInterpolationMethod method = rotationInterpolationMethod(curve.name());
        return
            method == CurveRotationInterpolationMethod::Quaternion ||
            method == CurveRotationInterpolationMethod::Slerp ||
            method == CurveRotationInterpolationMethod::Squad;
    }

public:
    static TangentType tangentType(MFnAnimCurve::TangentType type)
    {
        switch (type)
        {
        case MFnAnimCurve::kTangentGlobal: return TangentType::Global;
        case MFnAnimCurve::kTangentFixed: return TangentType::Fixed;
        case MFnAnimCurve::kTangentLinear: return TangentType::Linear;
        case MFnAnimCurve::kTangentFlat: return TangentType::Flat;
        case MFnAnimCurve::kTangentStep: return TangentType::Step;
        case MFnAnimCurve::kTangentSlow: return TangentType::Slow;
        case MFnAnimCurve::kTangentFast: return TangentType::Fast;
        case MFnAnimCurve::kTangentSmooth: return TangentType::Smooth;
        case MFnAnimCurve::kTangentClamped: return TangentType::Clamped;
        case MFnAnimCurve::kTangentPlateau: return TangentType::Plateau;
        case MFnAnimCurve::kTangentStepNext: return TangentType::StepNext;
        case MFnAnimCurve::kTangentAuto: return TangentType::Auto;
        default:
            return TangentType::Clamped;
        }
    }

    static InfinityType infinityType(MFnAnimCurve::InfinityType type)
    {
        switch (type)
        {
        case MFnAnimCurve::kConstant: return InfinityType::Constant;
        case MFnAnimCurve::kCycle: return InfinityType::Cycle;
        case MFnAnimCurve::kCycleRelative: return InfinityType::CycleRelative;
        case MFnAnimCurve::kOscillate: return InfinityType::Oscillate;
        case MFnAnimCurve::kLinear: return InfinityType::Linear;
        default:
            return InfinityType::Constant;
        }
    }

    static CurveRotationInterpolationMethod rotationInterpolationMethod(MString curveNode)
    {
        MStringArray result;
        MStatus status = MGlobal::executeCommand(MString("rotationInterpolation -q -c ") + curveNode, result);

        if (status != MS::kSuccess)
            return CurveRotationInterpolationMethod::None;

        MString method = result[0];
        if (method == "none")
            return CurveRotationInterpolationMethod::None;

        if (method == "euler")
            return CurveRotationInterpolationMethod::Euler;

        if (method == "quaternionSlerp")
            return CurveRotationInterpolationMethod::Slerp;

        if (method == "quaternionSquad")
            return CurveRotationInterpolationMethod::Squad;

        if (method == "quaternion")
            return CurveRotationInterpolationMethod::Quaternion;

        return CurveRotationInterpolationMethod::None;      
    }

protected:
    mutable std::unordered_map<double, double>  quaternionWcache;
    MObject fCurve;
};

/*!
    Find a trio of rotation curves for X, Y, Z channels given a rotation curve.
    When calculating quaternion interpolation for rotation curves, we need to operate on all three simultaneously.
*/
bool findSiblingCurves(const MFnAnimCurve &curve, MObject &pcX, MObject &pcY, MObject &pcZ)
{
    MStringArray result;
    MStatus status = MGlobal::executeCommand(
        MString("listConnections -s 0 -d 1 -p 1 ") + curve.name(), 
        result);

    if (status == MS::kFailure || result.length() == 0)
        return false;

    MString target = result[0];
    MStringArray targetSplit;
    status = target.split('.', targetSplit);

    if (status == MS::kFailure || targetSplit.length() < 2)
        return false;
    
    MString rotAttrs[] = { ".rotateX", ".rotateY", ".rotateZ" };
    for (int i = 0; i < 3; i++)
    {
        result.clear();
        status = MGlobal::executeCommand(MString("listConnections -s 1 -d 0 ") + targetSplit[0] + rotAttrs[i], result);
        if (status == MS::kFailure || result.length() == 0)
            return false;

        MSelectionList list;
        list.add(result[0]);
        MObject node;
        status = list.getDependNode(0, node);

        if (status == MS::kFailure)
            return false;

        switch (i)
        {
            case 0: pcX = node; break;
            case 1: pcY = node; break;
            case 2: pcZ = node; break;
        }

        if (status == MS::kFailure)
            return false;
    }

    return true;
}



MStatus AnimXCommand::doIt(const MArgList &args)
{
    MTime start(0.0), end(30.0), step(1.0);
    MStatus status = parseArgs(args, start, end, step);

    if (status != MS::kSuccess)
        return status;
    
    MSelectionList list;
    MString arg;    
    
    args.get(0, arg);
    list.add(arg);
    
    MObject node;
    MItSelectionList iter(list);
    iter.getDependNode(node);

    MFnAnimCurve curve(node, &status);
    if (status != MS::kSuccess)
    {
        displayError("Argument is not a param curve.");
        return MS::kFailure;
    }

    TparamCurveAccessor curveAPI(curve.name());
    double value = 0.0;
    bool isRotation = false;

    MObject pcX, pcY, pcZ;
    if (curveAPI.isRotation() &&
        findSiblingCurves(curve, pcX, pcY, pcZ))
    {
        isRotation = true;
    }
        
    TparamCurveAccessor x(MFnDependencyNode(pcX).name()), y(MFnDependencyNode(pcY).name()), z(MFnDependencyNode(pcZ).name());

    for (MTime t = start; t <= end; t += step)
    {
        if (isRotation) 
        {
            CurveRotationInterpolationMethod interpMethod = TparamCurveAccessor::rotationInterpolationMethod(curve.name());
            Quaternion q = adsk::evaluateQuaternionCurve(t.as(MTime::kSeconds), x, y, z, interpMethod);
            MEulerRotation euler = MQuaternion(q.x, q.y, q.z, q.w).asEulerRotation();
            if (pcX == curve.object())
                value = euler.x;
            else if (pcY == curve.object())
                value = euler.y;
            else if (pcZ == curve.object())
                value = euler.z;
        }
        else
        {
            value = adsk::evaluateCurve(t.as(MTime::kSeconds), curveAPI);
        }

        if (curve.animCurveType() == MFnAnimCurve::kAnimCurveUA ||
            curve.animCurveType() == MFnAnimCurve::kAnimCurveTA)
        {
            value = MAngle::internalToUI(value);
        }

        appendToResult(value);
    }   

    return MS::kSuccess;
}


PLUGIN_DLL_EXPORT TparamCurveAccessor* CreateParamCurveAccessor(const char *curveName)
{   
    return new TparamCurveAccessor(curveName);
}


PLUGIN_DLL_EXPORT void DeleteParamCurveAccessor(TparamCurveAccessor *curve)
{
    delete curve;
}