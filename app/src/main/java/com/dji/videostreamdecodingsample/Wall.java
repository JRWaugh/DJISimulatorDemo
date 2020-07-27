package com.dji.videostreamdecodingsample;

import android.graphics.PointF;

import static dji.common.flightcontroller.virtualstick.Limits.YAW_CONTROL_MAX_ANGLE;
import static dji.common.flightcontroller.virtualstick.Limits.YAW_CONTROL_MIN_ANGLE;

public class Wall {
    private enum State {
        New, WaxingCrescent, FirstQuarter, WaxingGibbous, Full, WaningGibbous, ThirdQuarter, WaningCrescent,
        Fraudulent
    }

    public Wall(final PointF originPoint, final float heading) {
        mOriginPoint = originPoint;
        mHeading = heading;
        mPreferredHeading = 0;

        mPortHeading = mHeading - 90.0f;
        if (mPortHeading < YAW_CONTROL_MIN_ANGLE)
            mPortHeading = YAW_CONTROL_MAX_ANGLE + (mPortHeading - YAW_CONTROL_MIN_ANGLE);

        mStarboardHeading = mHeading + 90.0f;
        if (mStarboardHeading > YAW_CONTROL_MAX_ANGLE)
            mStarboardHeading = YAW_CONTROL_MIN_ANGLE + (mStarboardHeading - YAW_CONTROL_MAX_ANGLE);

    }

    public float getHeading() {
        return mHeading;
    }

    public float getPortHeading() {
        return mPortHeading;
    }

    public float getStarboardHeading() {
        return mStarboardHeading;
    }

    public void setTargetPoint(final PointF point) {
        mTargetPoint = point;
    }

    public void setTargetPoint(final float displacement, final float direction) {
        mTargetPoint = new PointF(mOriginPoint.x + (float) Math.cos(direction) * displacement, mOriginPoint.y + (float) Math.sin(direction) * displacement);
    }

    public void setPreferredHeading(final int preferredHeading) {
        mPreferredHeading = preferredHeading;
    }

    public int getPreferredHeading() {
        return mPreferredHeading;
    }

    public PointF getTargetPoint() {
        return mTargetPoint;
    }

    private int mPreferredHeading;
    private final float mHeading;
    private float mPortHeading, mStarboardHeading;
    private final PointF mOriginPoint;
    private PointF mTargetPoint;
}

