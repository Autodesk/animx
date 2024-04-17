#include <cassert>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include "animx.h"

#include "internal/math.h"
#include "internal/interpolators.h"

#define _SIGN(x) (((x)<0.0)?-1:1)

namespace adsk
{
    //! Utility functions to perform interpolation between two keys
    double interpolateLinear(double time, double keyTime1, double keyValue1, double keyTime2, double keyValue2)
    {
        assert(keyTime2 >= keyTime1);
        assert(time >= keyTime1);
        assert(time <= keyTime2);

        if (keyTime1 == keyTime2)
            return keyValue1;

        double t = (time - keyTime1) / (keyTime2 - keyTime1);
        return lerp(t, keyValue1, keyValue2);
    }


    double inline interpolateStep(double time, double keyTime1, double keyValue1, double keyTime2, double keyValue2)
    {
        return keyValue1;
    }


    inline double interpolateStepNext(double time, double keyTime1, double keyValue1, double keyTime2, double keyValue2)
    {
        return keyValue2;
    }


    //! Utility function to compute tangent angle given its X and Y components
    double tangentAngle(
        const ICurve &curve,
        unsigned int index,
        bool inTangent,
        double xScale = 1.0,
        double yScale = 1.0
    )
    {
        Keyframe key;
        curve.keyframeAtIndex(index, key);
        double tanTime, tanValue;

        if (inTangent)
        {
            tanTime = key.tanIn.x;
            tanValue = key.tanIn.y;
        }
        else
        {
            tanTime = key.tanOut.x;
            tanValue = key.tanOut.y;
        }

        if (!equivalent(tanTime, 0.0))
        {
            double angleVal = atan((tanValue * yScale) / (tanTime * xScale));
            return angleVal;
        }

        double angle(
            equivalent(tanValue, 0.0) ? 0.0 : sign(tanValue) * 90.0
        );
        return angle * kPI / 180.0;
    }


    //! Evaluate a single curve at a given time
    double evaluateCurve(double time, const ICurve &curve)
    {
        Keyframe first, last;
        if (!curve.first(first) || !curve.last(last))
            return 0.0;

        if (curve.preInfinityType() != InfinityType::Constant && time < first.time)
        {
            return evaluateInfinity(time, curve, Infinity::Pre);
        }
        else if (curve.postInfinityType() != InfinityType::Constant && time > last.time)
        {
            return evaluateInfinity(time, curve, Infinity::Post);
        }

        if (curve.isStatic())
        {
            if (curve.keyframeCount() > 0)
                return first.value;
            return 0.0;
        }

        Keyframe prev, next;
        if (!curve.keyframe(time, next))
            return 0.0;
        if (equivalent(next.time, time) || (next.time < time && next.index == 0))
            return next.value;
        if (time >= last.time && next.index == last.index)
            return last.value;

        if (!curve.keyframeAtIndex(next.index - 1, prev))
            return next.value;

        static const double oneThird = 1.0 / 3.0;

        // for Bezier interpolation we might need to drop the precision
        // in case not using 64-bit time precision
        if (prev.spanInterpolationMethod() == SpanInterpolationMethod::Curve)
        {
            time = (seconds)time;
            prev.time = (seconds)prev.time;
            next.time = (seconds)next.time;
        }

        return evaluateCurveSegment(
            prev.spanInterpolationMethod(),
            prev.curveInterpolationMethod(curve.isWeighted()),
            time,
            prev.time, prev.value,
            prev.time + prev.tanOut.x * oneThird, prev.value + prev.tanOut.y * oneThird,
            next.time - next.tanIn.x  * oneThird, next.value - next.tanIn.y  * oneThird,
            next.time, next.value);
    }


    //! Evaluate an individual curve segment
    double evaluateCurveSegment(
        SpanInterpolationMethod interpolationMethod,
        CurveInterpolatorMethod curveInterpolatorMethod,
        double time,
        double startX, double startY,
        double x1, double y1,
        double x2, double y2,
        double endX, double endY)
    {
        double result = 0.0;
        switch (interpolationMethod)
        {
        case SpanInterpolationMethod::Curve:
            switch (curveInterpolatorMethod)
            {
            case CurveInterpolatorMethod::Bezier:
                result = CurveInterpolators::bezier(startX, startY, x1, y1, x2, y2, endX, endY, time);
                break;

            case CurveInterpolatorMethod::Hermite:
                result = CurveInterpolators::hermite(startX, startY, x1, y1, x2, y2, endX, endY, time);
                break;

            case CurveInterpolatorMethod::Sine:
                result = CurveInterpolators::sine(startX, startY, x1, y1, x2, y2, endX, endY, time);
                break;

            case CurveInterpolatorMethod::Parabolic:
                result = CurveInterpolators::parabolic(startX, startY, x1, y1, x2, y2, endX, endY, time);
                break;

            case CurveInterpolatorMethod::TangentLog:
                result = CurveInterpolators::log(startX, startY, x1, y1, x2, y2, endX, endY, time);
                break;
            }
            break;

        case SpanInterpolationMethod::Linear:
            result = interpolateLinear(time, startX, startY, endX, endY);
            break;

        case SpanInterpolationMethod::Step:
            result = interpolateStep(time, startX, startY, endX, endY);
            break;

        case SpanInterpolationMethod::StepNext:
            result = interpolateStepNext(time, startX, startY, endX, endY);
            break;
        }
        return result;
    }

    //! Evaluate rotation infinities using quaternion interpolation
    bool evaluateQuaternionInfinity(
        double &time,
        double firstTime, Quaternion firstValue,
        double lastTime, Quaternion lastValue,
        InfinityType preInfinityType,
        InfinityType postInfinityType,
        Quaternion &qOffset,
        Quaternion &qStart,
        bool &inverse)
    {
        qOffset = Quaternion{ 0.0, 0.0, 0.0, 1.0 };
        qStart = Quaternion{ 0.0, 0.0, 0.0, 1.0 };
        inverse = false;

        seconds start = (seconds)firstTime;
        seconds end = (seconds)lastTime;
        seconds range = (seconds)(end - start);
        unsigned int cycles = 0;
        bool needsPostProcessing = false;

        if (time < start)
        {
            switch (preInfinityType)
            {
            case InfinityType::Constant:
                time = start;
                break;

            case InfinityType::Cycle:
            case InfinityType::CycleRelative:
            case InfinityType::Oscillate:
            {
                seconds diff = (seconds)(start - time);
                time = range - fmod(diff, range);
                if (preInfinityType == InfinityType::Oscillate)
                {
                    seconds mod2 = fmod(diff, 2 * range);
                    if (mod2 < range)
                        time = range - time;
                }
                else if (preInfinityType == InfinityType::CycleRelative)
                {
                    cycles = (unsigned int)std::floor(diff / range);
                    time = range - time;
                    inverse = true;
                    needsPostProcessing = true;
                }

                time += start;
            }
            break;

            case InfinityType::Linear:
                time = start;
                return false;
            }
        }
        else if (time > end)
        {
            switch (postInfinityType)
            {
            case InfinityType::Constant:
                time = end;
                break;

            case InfinityType::Cycle:
            case InfinityType::CycleRelative:
            case InfinityType::Oscillate:
            {
                seconds diff = (seconds)(time - end);
                time = fmod(diff, range);
                if (postInfinityType == InfinityType::Oscillate)
                {
                    seconds mod2 = fmod(diff, 2 * range);
                    if (mod2 < range)
                        time = range - time;
                }
                else if (postInfinityType == InfinityType::CycleRelative)
                {
                    cycles = (unsigned int)std::floor(diff / range) + 1;
                    needsPostProcessing = true;
                }

                time += start;
            }
            break;

            case InfinityType::Linear:
                time = end;
                return false;
            }
        }
        else
        {
            return false;
        }

        qStart = firstValue;

        if (cycles > 0)
        {
            Tquaternion qEnd(lastValue);
            Tquaternion qStart(firstValue);
            qOffset = (qStart.conjugate() * qEnd).pow((double)cycles).normalizeIt();
        }

        return needsPostProcessing;
    }

    //! Given three rotation curves RX, Ry, RZ, find the closest key forward or backward in time
    //  common to all three curves
    Tquaternion findClosestKeyframe(
        double time,
        const ICurve &pcX, const ICurve &pcY, const ICurve &pcZ,
        bool forward,
        bool atStart,
        Keyframe *keyframe)
    {
        int nX = pcX.keyframeCount();
        int nY = pcY.keyframeCount();
        int nZ = pcZ.keyframeCount();
        int iX = 0;
        Keyframe key;

        if (forward)
        {
            for (; iX < nX; ++iX)
            {
                pcX.keyframeAtIndex(iX, key);
                double t = key.time;
                if (t >= time)
                {
                    if (t > time && atStart)
                        --iX;
                    break;
                }
            }
        }
        else
        {
            for (iX = nX - 1; iX >= 0; --iX)
            {
                pcX.keyframeAtIndex(iX, key);
                if (key.time <= time)
                    break;
            }
        }

        double tX, tY, tZ;
        if (iX >= 0 && iX < nX)
        {
            Keyframe kx, ky, kz;
            pcX.keyframeAtIndex(iX, kx);
            pcY.keyframeAtIndex(iX, ky);
            pcZ.keyframeAtIndex(iX, kz);
            tX = kx.time;
            if ((iX < nY && equivalent(tX, ky.time)) &&
                (iX < nZ && equivalent(tX, kz.time)))
            {
                if (keyframe)
                    *keyframe = kz;

                return Tquaternion(
                    kx.value,
                    ky.value,
                    kz.value,
                    (float)kz.quaternionW /* deliberate cast to float */);
            }
        }
        else if (iX < 0)
        {
            Keyframe kx, ky, kz;
            pcX.first(kx);
            pcY.first(ky);
            pcZ.first(kz);
            tX = kx.time;
            tY = ky.time;
            tZ = kz.time;
            if (tY >= time && tZ >= time)
            {
                if (keyframe)
                    *keyframe = kz;

                return Tquaternion(
                    kx.value,
                    ky.value,
                    kz.value,
                    (float)kz.quaternionW /* deliberate cast to float */);
            }
        }
        else
        {
            Keyframe kx, ky, kz;
            pcX.last(kx);
            pcY.last(ky);
            pcZ.last(kz);
            tX = kx.time;
            tY = ky.time;
            tZ = kz.time;
            if (tY <= time && tZ <= time)
            {
                if (keyframe)
                    *keyframe = kz;

                return Tquaternion(
                    kx.value,
                    ky.value,
                    kz.value,
                    (float)kz.quaternionW /* deliberate cast to float */);
            }
        }

        iX = iX >= 0 ? iX : 0;
        int iY = clamp(iX, 0, nY - 1);
        int iZ = clamp(iX, 0, nZ - 1);

        Keyframe kx, ky, kz;
        pcX.keyframeAtIndex(iX, kx);
        pcY.keyframeAtIndex(iY, ky);
        pcZ.keyframeAtIndex(iZ, kz);

        if (keyframe)
            *keyframe = kx;

        return Tquaternion(
            kx.value,
            ky.value,
            kz.value,
            (float)kz.quaternionW /* deliberate cast to float */);
    }


    //! Evaluate rotation curves using quaternion interpolation
    Quaternion evaluateQuaternionCurve(
        double time,
        const ICurve &pcX, const ICurve &pcY, const ICurve &pcZ,
        CurveRotationInterpolationMethod interpolationMethod)
    {
        Keyframe keyX, keyY, keyZ;
        if (pcX.keyframe(time, keyX) && keyX.time == time)
        {
            if (pcY.keyframe(time, keyY) && keyY.time == time)
            {
                if (pcZ.keyframe(time, keyZ) && keyZ.time == time)
                {
                    return Tquaternion(keyX.value, keyY.value, keyZ.value, (float)keyZ.quaternionW /* deliberate cast to float */);
                }
            }
        }

        Keyframe start, end;
        if (!pcX.first(start) || !pcX.last(end))
            return Tquaternion();

        Keyframe first, last;
        Tquaternion firstQ = findClosestKeyframe(start.time, pcX, pcY, pcZ, true, true, &first);
        Tquaternion lastQ = findClosestKeyframe(end.time, pcX, pcY, pcZ, true, false, &last);

        bool isInfInversed;
        Quaternion qInfOffsetQ, qInfStartQ;
        const bool needsInfPostProcessing = evaluateQuaternionInfinity(
            time,
            first.time, firstQ,
            last.time, lastQ,
            pcZ.preInfinityType(),
            pcZ.postInfinityType(),
            qInfOffsetQ, qInfStartQ, isInfInversed);

        Tquaternion qInfOffset = qInfOffsetQ, qInfStart = qInfStartQ;

        static seconds oneTickInSeconds = (seconds)(1.0 / kTicksPerSecondInTicks);

        Keyframe prev, next;
        Tquaternion q1 = findClosestKeyframe(time, pcX, pcY, pcZ, true, true, &start);
        Tquaternion q2 = findClosestKeyframe(time + oneTickInSeconds, pcX, pcY, pcZ, true, false, &end);

        Tquaternion q0 = findClosestKeyframe(start.time - oneTickInSeconds, pcX, pcY, pcZ, false, false, &prev);
        Tquaternion q3 = findClosestKeyframe(end.time + oneTickInSeconds, pcX, pcY, pcZ, true, false, &next);

        Quaternion _q = evaluateQuaternion(
            (seconds)time,
            interpolationMethod,
            start.spanInterpolationMethod(),
            (seconds)start.time, q1,
            (seconds)end.time, q2,
            start.tanOut.type,
            q0,
            q3);


        if (needsInfPostProcessing)
        {
            Tquaternion q(_q.x, _q.y, _q.z, _q.w);
            q *= qInfOffset;

            if (isInfInversed) {
                q *= qInfStart.conjugate();
                q.invertIt();
                q *= qInfStart;
            }
            return Quaternion{ q.x, q.y, q.z, q.w };
        }

        return _q;
    }


    //! Evaluate an individual rotation curve segment using quaternion interpolation
    Quaternion evaluateQuaternion(
        seconds time,
        CurveRotationInterpolationMethod interpolationMethod,
        SpanInterpolationMethod spanInterpolationMethod,
        seconds startTime, Quaternion startValue,
        seconds endTime, Quaternion endValue,
        TangentType tangentType,
        Quaternion prevValue,
        Quaternion nextValue)
    {
        Tquaternion q1(startValue);
        Tquaternion q2(endValue);
        Tquaternion q;
        bool notEvaluated = false;
        double param = equivalent(startTime, endTime) ?
            1.0 :
            (time - startTime) / (double)(endTime - startTime);

        if (interpolationMethod == CurveRotationInterpolationMethod::Slerp)
        {
            q = slerp(q1, q2, param);
        }
        else if (interpolationMethod == CurveRotationInterpolationMethod::Quaternion)
        {
            if (time < startTime || spanInterpolationMethod == SpanInterpolationMethod::Step || tangentType == TangentType::Step)
            {
                q = q1;
            }
            else if (spanInterpolationMethod == SpanInterpolationMethod::Linear || tangentType == TangentType::Linear)
            {
                q = slerp(q1, q2, param);
            }
            else
            {
                notEvaluated = true;
            }
        }
        else
        {
            notEvaluated = true;
        }

        if (notEvaluated)
        {
            Tquaternion q0(prevValue);
            Tquaternion q3(nextValue);
            Tquaternion ctrl0 = bezierPt(q0, q1, q2, true);
            Tquaternion ctrl1 = bezierPt(q1, q2, q3, false);
            q = bezier(q1, ctrl0, ctrl1, q2, param);
        }

        return q;
    }


    //! Evaluate infinities of a single curve
    double evaluateInfinity(
        double time,
        const ICurve &curve,
        Infinity infinity)
    {
        Keyframe first, last;
        if (!curve.first(first) || !curve.last(last))
            return 0.0;

        InfinityType preInfinityType = curve.preInfinityType();
        InfinityType postInfinityType = curve.postInfinityType();

        double value = 0.0;
        double range = last.time - first.time;
        double remainder, numCycles, notUsed, ratio;

        if (range == 0)
        {
            return value;
        }
        else if (time > last.time)
        {
            double diff = time - last.time;
            ratio = diff / range;
            remainder = fabs(modf(ratio, &numCycles));
        }
        else
        {
            double diff = time - first.time;
            ratio = diff / range;
            remainder = fabs(modf(ratio, &numCycles));
        }

        double factoredTime = range * remainder;
        numCycles = fabs(numCycles) + 1;

        if (infinity == Infinity::Pre)
        {
            if (preInfinityType == InfinityType::Oscillate)
            {
                if ((remainder = modf((numCycles / 2), &notUsed)) != 0.0)
                    factoredTime = first.time + factoredTime;
                else
                    factoredTime = last.time - factoredTime;
            }
            else if (
                preInfinityType == InfinityType::Cycle ||
                preInfinityType == InfinityType::CycleRelative)
            {
                factoredTime = last.time - factoredTime;
            }
            else if (preInfinityType == InfinityType::Linear)
            {
                double inc = first.time - time;
                double angle = tangentAngle(curve, 0, true);
                value = first.value - inc * tan(angle);
                return value;
            }
        }
        else
        {
            if (postInfinityType == InfinityType::Oscillate)
            {
                if ((remainder = modf((numCycles / 2), &notUsed)) != 0.0)
                    factoredTime = last.time - factoredTime;
                else
                    factoredTime = first.time + factoredTime;
            }
            else if (
                postInfinityType == InfinityType::Cycle ||
                postInfinityType == InfinityType::CycleRelative)
            {
                factoredTime = first.time + factoredTime;
            }
            else if (postInfinityType == InfinityType::Linear)
            {
                double inc = time - last.time;
                double angle = tangentAngle(curve, last.index, false);
                value = last.value + inc * tan(angle);
                return value;
            }
        }

        value = evaluateCurve(toTickDoubleTime(factoredTime), curve);

        if (infinity == Infinity::Pre && preInfinityType == InfinityType::CycleRelative)
        {
            double range = last.value - first.value;
            value -= numCycles * range;
        }
        else if (infinity == Infinity::Post && postInfinityType == InfinityType::CycleRelative)
        {
            double range = last.value - first.value;
            value += numCycles * range;
        }
        return value;
    }

    void validateTangent(CurveInterpolatorMethod curveInterpolationMethod,
                          seconds &newTanX, seconds &newTanY)
    {
        if (curveInterpolationMethod == CurveInterpolatorMethod::Hermite) {
            // non-weighted tangents are normalized
            seconds length = sqrt(newTanX*newTanX + newTanY * newTanY);
            if (length > 0)
            {
                newTanX /= length;
                newTanY /= length;
            }
        } else if (curveInterpolationMethod == CurveInterpolatorMethod::Bezier) {
            if (newTanX < 0)
                newTanX = 0;
        }
    }   

    void
    _autoTangent(
                seconds & newTanX, 
                seconds& newTanY,
                bool calculateInTangent,
                seconds x, seconds px, seconds nx,
                seconds y, seconds py, seconds ny,
                TangentType tangentType)
    {
        // prevSlope3 and nextSlope3 are respectively the slopes to the left and right keys multiplied by 3.
        // Target slope needs to be adjusted to fit between these 2 last slope values to ensure that the
        // control points are not outside of the Y range defined by the prev and next keys
        constexpr static const float kEaseCoefficient = -0.5f;

        double targetSlope = 0.0;
        double prevSlope3 = 3.0 * (  y - py ) / (  x - px );
        double nextSlope3 = 3.0 * ( ny -  y ) / ( nx -  x );

        // Default is ease
        //
        if( tangentType == TangentType::Auto ){
            // Legacy auto tangents - Target slope is the default spline slope.
            //
            targetSlope = ( ny - py ) /  ( nx - px );
        } else {
            // Interpolated auto tangent
            //
            double prevSlope = (  y - py ) / (  x - px );
            double nextSlope = ( ny -  y ) / ( nx -  x );
            double f = (x-px)/(nx-px);
            if( tangentType == TangentType::AutoMix ){
                // Linear interpolation
                //
                targetSlope = (1-f)*prevSlope + f*nextSlope;
            } else { // tangentType == TangentType::AutoEase 
                // Cubic interpolation
                //
                // 
                // This is a family of cubic functions g with c in [-1/2,1], and
                // f in [0,1]
                //
                // g(c,f) = 0.5 + (f-0.5)*(1-c+4*c*(f-0.5)^2)
                //
                float c = kEaseCoefficient;
                // Ease interpolation - this is the cubic interpolation that
                // uses the coefficient that gives the strongest influence to
                // its closest neighbor.
                //
                double f_minus_half = f - 0.5;
                double f_minus_half_squared = f_minus_half*f_minus_half;

                // Map f using g:
                //
                double g = 0.5 + f_minus_half*(1-c+4*c*f_minus_half_squared);

                // Now use g instead of f to interpolate
                //
                targetSlope = (1-g)*prevSlope + g*nextSlope;
            }
        }

        // Clamp
        //
        if ( _SIGN( prevSlope3 ) != _SIGN( nextSlope3) ||
            _SIGN( targetSlope ) != _SIGN( nextSlope3 ) )
        {
            targetSlope = 0.0;
        }
        else if ( nextSlope3 >= 0 )
        {
            targetSlope = std::min(std::min(targetSlope, nextSlope3), prevSlope3);
        }
        else
        {
            targetSlope = std::max(std::max(targetSlope, nextSlope3), prevSlope3);
        }

        // Find the x tangent value
        // No need to divide by 3 since the interpolator code does this already
        if ( calculateInTangent )
            newTanX = ( x - px );
        else
            newTanX = ( nx - x );
    
        // Find the y tangent value
        newTanY = targetSlope * newTanX;
    }

    void flatTangent(bool calculateInTangent, 
                     KeyTimeValue key, const KeyTimeValue *prevKey, const KeyTimeValue *nextKey,
                     CurveInterpolatorMethod curveInterpolationMethod,
                     seconds &newTanX, seconds &newTanY)
    {
        seconds tanInx = 0.0, tanIny = 0.0;
        seconds tanOutx = 0.0, tanOuty = 0.0;
        seconds x = (seconds)key.time;

        if (prevKey != nullptr)
            tanInx = x - seconds(prevKey->time);
        
        if (nextKey != nullptr)
            tanOutx = seconds(nextKey->time) - x;
        
        if (prevKey == nullptr)
            tanInx = tanOutx;
        
        if (nextKey == nullptr)
            tanOutx = tanInx;
        
        if (calculateInTangent)
        {
            newTanX = tanInx;
            newTanY = tanIny;
        }
        else
        {
            newTanX = tanOutx;
            newTanY = tanOuty;
        }

        validateTangent(curveInterpolationMethod, newTanX, newTanY);
    }

    //! Compute tangent values for a key with Auto tangent type
    void autoTangent(bool calculateInTangent,
                    TangentType tangentType,
                    KeyTimeValue key, KeyTimeValue *prevKey, KeyTimeValue *nextKey, 
                    CurveInterpolatorMethod curveInterpolationMethod,
                    seconds &newTanX, seconds &newTanY)
    {
        if (prevKey == nullptr || nextKey == nullptr)
        {
            // calculate flat tangent
            flatTangent(calculateInTangent, key, prevKey, nextKey, curveInterpolationMethod, newTanX, newTanY);
        }
        else
        {
            seconds x = (seconds)key.time;
            double y = key.value;
            seconds px = (seconds)prevKey->time;
            seconds nx = (seconds)nextKey->time;
            double py = prevKey->value;
            double ny = nextKey->value;

            _autoTangent(newTanX, newTanY, calculateInTangent, x, px, nx, y, py, ny, tangentType);
            validateTangent(curveInterpolationMethod, newTanX, newTanY);
        }
    }
}



